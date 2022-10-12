from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from zenoh_flow.types import Context
from typing import Dict, Any


from collections import deque
import numpy as np


from stunt.types import (
    Pose,
    EgoInfo,
    Transform,
    Rotation,
    Location,
    Waypoints,
    RoadOption,
    Trajectory,
    BehaviorPlannerState,
)
from stunt.map import HDMap

from stunt.types.planner import cost_overtake

DEFAULT_LOCATION_GOAL = (0.0, 0.0, 0.0)
DEFAULT_INITIAL_STATE = BehaviorPlannerState.FOLLOW_WAYPOINTS
DEFAULT_MIN_MOVING_SPEED = 0.7


class BehaviourPlanning(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ):
        configuration = configuration if configuration is not None else {}

        self.map_file = configuration.get("map", None)
        if self.map_file is None:
            raise ValueError("BehaviourPlanning cannot proceed without a map!")

        self.goal_location = Location(
            *configuration.get("goal", DEFAULT_LOCATION_GOAL)
        )

        # Getting carla map
        with open(self.map_file) as f:
            opendrive = f.read()
        self.map = HDMap.from_opendrive(opendrive)

        self.state = DEFAULT_INITIAL_STATE
        self.cost_functions = [cost_overtake]
        self.function_weights = [1]

        self.ego_info = EgoInfo()

        # initialize the route with the given goal
        self.route = Waypoints(
            deque([Transform(self.goal_location, Rotation())]),
            road_options=deque([RoadOption.LANE_FOLLOW]),
        )

        self.is_first = True

        self.pose_input = inputs.get("Pose", None)
        self.output = outputs.get("Trajectory", None)

    async def iteration(self):

        # wait for location data
        data_msg = await self.pose_input.recv()
        pose = Pose.deserialize(data_msg.data)

        ego_transform = pose.transform
        # forward_speed = pose.forward_speed

        # When we get the position for the first time we compute the waypoints
        if self.is_first is True:
            waypoints = self.map.compute_waypoints(
                ego_transform.location, self.goal_location
            )
            road_options = deque(
                [RoadOption.LANE_FOLLOW for _ in range(len(waypoints))]
            )
            self.route = Waypoints(waypoints, road_options=road_options)
            self.is_first = False
            return None

        # update ego information
        self.ego_info.update(pose)

        old_state = self.state
        # check if we the car can change its behaviour
        self.state = self.best_state_transition()

        if (
            self.state != old_state
            and self.state == BehaviorPlannerState.OVERTAKE
        ):
            self.route.remove_waypoint_if_close(ego_transform.location, 10)
        else:
            if not self.map.is_intersection(ego_transform.location):
                self.route.remove_waypoint_if_close(ego_transform.location, 10)
            else:
                self.route.remove_waypoint_if_close(ego_transform.location, 3)

        new_goal_location = self.get_goal_location(ego_transform)

        if new_goal_location != self.goal_location:
            self.goal_location = new_goal_location
            waypoints = self.map.compute_waypoints(
                ego_transform.location, self.goal_location
            )
            road_options = deque(
                [RoadOption.LANE_FOLLOW for _ in range(len(waypoints))]
            )
            waypoints = Waypoints(waypoints, road_options=road_options)

            if waypoints.is_empty():
                # No more waypoints stop the car
                waypoints = Waypoints(
                    deque([ego_transform]),
                    deque([0]),
                    deque(RoadOption.LANE_FOLLOW),
                )

            trajectory = Trajectory(waypoints, self.state)

            await self.output.send(trajectory.serialize())
        elif old_state != self.state:
            trajectory = Trajectory(self.route, self.state)
            await self.output.send(trajectory.serialize())
        return None

    def finalize(self) -> None:
        return None

    def successor_states(self):
        """Returns possible state transitions from current state."""
        if self.state == BehaviorPlannerState.FOLLOW_WAYPOINTS:
            # Can transition to OVERTAKE if the ego vehicle has been stuck
            # behind an obstacle for a while.
            return [
                BehaviorPlannerState.FOLLOW_WAYPOINTS,
                BehaviorPlannerState.OVERTAKE,
            ]
        elif self.state == BehaviorPlannerState.OVERTAKE:
            return [
                BehaviorPlannerState.OVERTAKE,
                BehaviorPlannerState.FOLLOW_WAYPOINTS,
            ]
        elif self.state == BehaviorPlannerState.KEEP_LANE:
            # 1) keep_lane -> prepare_lane_change_left
            # 2) keep_lane -> prepare_lane_change_right
            return [
                BehaviorPlannerState.KEEP_LANE,
                BehaviorPlannerState.PREPARE_LANE_CHANGE_LEFT,
                BehaviorPlannerState.PREPARE_LANE_CHANGE_RIGHT,
            ]
        elif self.state == BehaviorPlannerState.PREPARE_LANE_CHANGE_LEFT:
            # 1) prepare_lane_change_left -> keep_lane
            # 2) prepare_lane_change_left -> lange_change_left
            return [
                BehaviorPlannerState.KEEP_LANE,
                BehaviorPlannerState.PREPARE_LANE_CHANGE_LEFT,
                BehaviorPlannerState.LANE_CHANGE_LEFT,
            ]
        elif self.state == BehaviorPlannerState.LANE_CHANGE_LEFT:
            # 1) lange_change_left -> keep_lane
            return [
                BehaviorPlannerState.KEEP_LANE,
                BehaviorPlannerState.LANE_CHANGE_LEFT,
            ]
        elif self.state == BehaviorPlannerState.PREPARE_LANE_CHANGE_RIGHT:
            # 1) prepare_lane_change_right -> keep_lane
            # 2) prepare_lane_change_right -> lange_change_right
            return [
                BehaviorPlannerState.KEEP_LANE,
                BehaviorPlannerState.PREPARE_LANE_CHANGE_RIGHT,
                BehaviorPlannerState.LANE_CHANGE_RIGHT,
            ]
        elif self.state == BehaviorPlannerState.LANE_CHANGE_RIGHT:
            # 1) lane_change_right -> keep_lane
            return [
                BehaviorPlannerState.KEEP_LANE,
                BehaviorPlannerState.LANE_CHANGE_RIGHT,
            ]
        else:
            raise ValueError("Unexpected vehicle state {}".format(self.state))

    def best_state_transition(self):
        """Computes most likely state transition from current state."""
        # Get possible next state machine states.
        possible_next_states = self.successor_states()
        best_next_state = None
        min_state_cost = np.infty
        for state in possible_next_states:
            state_cost = 0
            # Compute the cost of the trajectory.
            for i in range(len(self.cost_functions)):
                cost_func = self.cost_functions[i](
                    self.state, state, self.ego_info
                )
                state_cost += self.function_weights[i] * cost_func
            # Check if it's the best trajectory.
            if state_cost < min_state_cost:
                best_next_state = state
                min_state_cost = state_cost
        return best_next_state

    def get_goal_location(self, ego_transform):
        if len(self.route.waypoints) > 1:
            dist = ego_transform.location.distance(
                self.route.waypoints[0].location
            )
            if dist < 5:
                new_goal_location = self.route.waypoints[2].location
            else:
                new_goal_location = self.route.waypoints[1].location
        elif len(self.route.waypoints) == 1:
            new_goal_location = self.route.waypoints[0].location
        else:
            new_goal_location = ego_transform.location
        return new_goal_location


def register():
    return BehaviourPlanning
