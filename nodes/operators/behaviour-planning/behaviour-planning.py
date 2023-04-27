from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any
import asyncio

from collections import deque
import numpy as np

from stunt.map import HDMap
from stunt.types import (
    Pose,
    EgoInfo,
    Waypoints,
    RoadOption,
    Trajectory,
    BehaviorPlannerState,
)

from stunt.types.planner import cost_overtake
import logging

DEFAULT_INITIAL_STATE = BehaviorPlannerState.FOLLOW_WAYPOINTS
DEFAULT_MIN_MOVING_SPEED = 0.7


class BehaviourPlanning(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):

        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
        configuration = configuration if configuration is not None else {}

        self.map_file = configuration.get("map", None)
        if self.map_file is None:
            raise ValueError("BehaviourPlanning cannot proceed without a map!")

        # Getting carla map
        with open(self.map_file) as f:
            opendrive = f.read()
        self.map = HDMap.from_opendrive(opendrive)

        self.goal_location = None

        self.state = DEFAULT_INITIAL_STATE
        self.cost_functions = [cost_overtake]
        self.function_weights = [1]

        self.ego_info = EgoInfo()

        self.route = None

        self.is_first = True

        self.pose_input = inputs.get("Pose", None)
        if self.pose_input is None:
            raise ValueError("Input 'Pose' not found")
        self.route_input = inputs.get("Route")
        if self.route_input is None:
            raise ValueError("Input 'Route' not found")
        self.output = outputs.get("Trajectory", None)
        if self.output is None:
            raise ValueError("Output 'Trajectory' not found")

        self.pending = []

    async def wait_pose(self):
        data_msg = await self.pose_input.recv()
        return ("Pose", data_msg)

    async def wait_route(self):
        data_msg = await self.route_input.recv()
        return ("Route", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "Route" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_route(), name="Route")
            )

        if not any(t.get_name() == "Pose" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_pose(), name="Pose")
            )
        return task_list

    async def iteration(self):
        try:
            (done, pending) = await asyncio.wait(
                self.create_task_list(),
                return_when=asyncio.FIRST_COMPLETED,
            )

            self.pending = list(pending)
            for d in done:
                (who, data_msg) = d.result()
                # logging.debug(f"[BehaviourPlanning] Received from input {who}")

                if who == "Route":
                    # Storing the route and the goal.
                    self.route = Waypoints.deserialize(data_msg.data)
                    self.goal_location = self.route.waypoints[-1].location
                    # logging.debug(f"[BehaviourPlanning] Route to destination: {self.goal_location} ( {len(self.route.waypoints)} waypoints)")

                elif who == "Pose":

                    pose = Pose.deserialize(data_msg.data)

                    ego_transform = pose.transform
                    # print(
                    #     f"BehaviourPlanning received pose {ego_transform.location}"
                    # )
                    # forward_speed = pose.forward_speed

                    # In order to compute we need the route.
                    if self.route is None:
                        # logging.warn("[BehaviourPlanning] route is none!")
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
                        self.route.remove_waypoint_if_close(
                            ego_transform.location, 10
                        )
                    else:
                        if not self.map.is_intersection(ego_transform.location):
                            self.route.remove_waypoint_if_close(
                                ego_transform.location, 10
                            )
                        else:
                            self.route.remove_waypoint_if_close(
                                ego_transform.location, 3
                            )

                    new_goal_location = self.get_goal_location(ego_transform)

                    distance_to_new = ego_transform.location.distance(new_goal_location)
                    distance_to_old = ego_transform.location.distance(self.goal_location)
                    # logging.debug(f"[BehaviourPlanning] Previous goal location: {self.goal_location} new goal location: {new_goal_location}")
                    # logging.debug(f"[BehaviourPlanning] Distance between pose and old goal location is {distance_to_old}")
                    # logging.debug(f"[BehaviourPlanning] Distance between pose and new goal location is {distance_to_new}")


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
                            logging.debug("[BehaviourPlanning] No more waypoints stop the car")

                            # No more waypoints stop the car
                            waypoints = Waypoints(
                                deque([ego_transform]),
                                deque([0]),
                                deque(RoadOption.LANE_FOLLOW),
                            )

                        trajectory = Trajectory(waypoints, self.state)

                        await self.output.send(trajectory.serialize())
                        # print(f"BehaviourPlanning sending trajectory {trajectory}")
                    elif old_state != self.state:
                        trajectory = Trajectory(self.route, self.state)
                        await self.output.send(trajectory.serialize())
                        # print(f"BehaviourPlanning sending trajectory {trajectory}")
        except Exception as e:
            logging.warning(f"[BehaviourPlanning] got error {e}")
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
        if len(self.route.waypoints) > 2:
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
