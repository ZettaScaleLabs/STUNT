from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from typing import Dict, Any, Callable
import time
import asyncio
import enum

from collections import deque
import carla
import numpy as np


class BehaviorPlannerState(enum.Enum):
    """States in which the FSM behavior planner can be in."""

    FOLLOW_WAYPOINTS = 0
    READY = 1
    KEEP_LANE = 2
    PREPARE_LANE_CHANGE_LEFT = 3
    LANGE_CHANGE_LEFT = 4
    PREPARE_LANE_CHANGE_RIGHT = 5
    LANE_CHANGE_RIGHT = 6
    OVERTAKE = 7


DEFAULT_CARLA_HOST = "localhost"
DEFAULT_CARLA_PORT = 2000
DEFAULT_LOCATION_GOAL = (0.0, 0.0, 0.0)
DEFAULT_INITIAL_STATE = BehaviorPlannerState.FOLLOW_WAYPOINTS
DEFAULT_MIN_MOVING_SPEED = 0.7


class State(object):
    def __init__(self, configuration):
        configuration = configuration if configuration is not None else {}

        self.carla_port = int(configuration.get("port", DEFAULT_CARLA_PORT))
        self.carla_host = configuration.get("host", DEFAULT_CARLA_HOST)
        self.goal_location = carla.Location(
            *configuration.get("goal", DEFAULT_LOCATION_GOAL)
        )

        self.carla_world = None
        self.carla_client = None

        # Connecting to CARLA to get the map
        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        # Getting carla map
        self.map = self.carla_world.get_map().to_opendrive()

        self.state = DEFAULT_INITIAL_STATE
        self.cost_functions = []
        self.function_weights = []

        self.ego_info = {
            "last_time_moving": 0,
            "last_time_stopped": 0,
            "current_time": 0,
        }

        # initialize the route with the given goal
        self.route = carla.Waypoints(
            deque([carla.Transform(self.goal_location, carla.Rotation())])
        )


class BehaviourPlanning(Operator):
    def __init__(self, state, pose_input, output):
        self.state = state
        self.pose_input = pose_input
        self.output = output

    async def run(self):

        # wait for location data
        data_msg = await self.pose_input.recv()
        pose = json.loads(data_msg.data.decode("utf-8"))

        # update ego information
        self.state.ego_info["current_time"] = pose["localization_time"]
        if pose["forward_speed"] > DEFAULT_MIN_MOVING_SPEED:
            self.state.ego_info["last_time_moving"] = pose["localization_time"]
        else:
            self.state.ego_info["last_time_stopped"] = pose["localization_time"]

        # check if we the car can change its behaviour
        new_state = self.best_state_transition()

        await self.output.send(json.dumps({'state':new_state}).encode("utf-8"))

        return None

    def setup(
        self,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ) -> Callable[[], Any]:
        state = State(configuration)

        pose_input = inputs.get("Pose", None)

        output = outputs.get("Pose", None)

        l = BehaviourPlanning(state, pose_input, output)
        return l.run

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
                cost_func = self.cost_functions[i](self.state, state, ego_info)
                state_cost += self.function_weights[i] * cost_func
            # Check if it's the best trajectory.
            if state_cost < min_state_cost:
                best_next_state = state
                min_state_cost = state_cost
        return best_next_state

    def get_goal_location(self, ego_transform):
        if len(self.route.waypoints) > 1:
            dist = ego_transform.location.distance(self.route.waypoints[0].location)
            if dist < 5:
                new_goal_location = self.route.waypoints[1].location
            else:
                new_goal_location = self.route.waypoints[0].location
        elif len(self.route.waypoints) == 1:
            new_goal_location = self.route.waypoints[0].location
        else:
            new_goal_location = ego_transform.location
        return new_goal_location


def register():
    return BehaviourPlanning
