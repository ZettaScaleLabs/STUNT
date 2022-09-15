from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from typing import Dict, Any, Callable
import time
import asyncio

import json
import numpy as np
import math

import carla
from collections import deque

from stunt.types import (
    Transform,
    Quaternion,
    Pose,
    Obstacle,
    ObstacleTrajectory,
    ObstaclePrediction,
    BehaviorPlannerState,
    TimeToDecision,
    Trajectory,
    TrafficLight,
    World,
)
from stunt.map import HDMap

from stunt import DEFAULT_CARLA_HOST, DEFAULT_CARLA_PORT


DEFAULT_STOP_FOR_TRAFFIC_LIGHTS = True
DEFAULT_STOP_FOR_PEOPLE = True
DEFAULT_STOP_FOR_VEHICLES = True
DEFAULT_STOP_FOR_UNCONTROLLED_JUNCTIONS = True

DEFAULT_PREDICTION_TTD = 500
DEFAULT_TARGET_SPEED = 6.0


class WaypointPlanner(Operator):
    def __init__(
        self,
        configuration,
        trajectory_input,
        pose_input,
        traffic_lights_input,
        obstacles_input,
        ttd_input,
        output,
    ):
        configuration = configuration if configuration is not None else {}

        self.carla_port = int(configuration.get("port", DEFAULT_CARLA_PORT))
        self.carla_host = configuration.get("host", DEFAULT_CARLA_HOST)

        self.pending = []

        self.traffic_lights_input = traffic_lights_input
        self.trajectory_input = trajectory_input
        self.pose_input = pose_input
        self.obstacles_input = obstacles_input
        self.ttd_input = ttd_input
        self.output = output

        self.start_time = 0
        self.world = World(configuration)

        self.obstacle_trajectories = []
        self.trajectory = None
        self.ttd = TimeToDecision(500)
        self.traffic_lights = []

        self.target_speed = DEFAULT_TARGET_SPEED
        self.state = BehaviorPlannerState.FOLLOW_WAYPOINTS

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()
        self.map = HDMap(self.carla_world.get_map())

    def get_predictions(self, prediction_msg, ego_transform):
        """Extracts obstacle predictions out of the message.

        This method is useful to build obstacle predictions when
        the operator directly receives detections instead of predictions.
        The method assumes that the obstacles are static.

        (PS: [0] is added because the prediction msg is a list)
        """
        predictions = []
        if prediction_msg:
            # if it's not an empty list
            # TODO: check why the prediction msg is empty
            if isinstance(prediction_msg[0], Obstacle):
                # Transform the obstacle into a prediction.
                predictions = []
                for obstacle in prediction_msg.obstacles:
                    obstacle_trajectory = ObstacleTrajectory(obstacle, [])
                    prediction = ObstaclePrediction(
                        obstacle_trajectory,
                        obstacle.transform,
                        1.0,
                        [ego_transform.inverse_transform() * obstacle.transform],
                    )
                    predictions.append(prediction)
            elif isinstance(prediction_msg[0], ObstaclePrediction):
                # TODO: check if we should use this instead
                predictions = prediction_msg
                # predictions = prediction_msg[0]
            else:
                raise ValueError(
                    "Unexpected obstacles msg type {}".format(type(prediction_msg[0]))
                )
        return predictions

    async def wait_obstacles(self):
        data_msg = await self.obstacles_input.recv()
        return ("ObstaclePredictions", data_msg)

    async def wait_ttd(self):
        data_msg = await self.ttd_input.recv()
        return ("TTD", data_msg)

    async def wait_pose(self):
        data_msg = await self.pose_input.recv()
        return ("Pose", data_msg)

    async def wait_trajectory(self):
        data_msg = await self.trajectory_input.recv()
        return ("Trajectory", data_msg)

    async def wait_traffic_lights(self):
        data_msg = await self.traffic_lights_input.recv()
        return ("TrafficLights", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "TTD" for t in task_list):
            task_list.append(asyncio.create_task(self.wait_ttd(), name="TTD"))

        if not any(t.get_name() == "ObstaclePredictions" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_obstacles(), name="ObstaclePredictions")
            )

        if not any(t.get_name() == "Pose" for t in task_list):
            task_list.append(asyncio.create_task(self.wait_pose(), name="Pose"))

        if not any(t.get_name() == "Trajectory" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_trajectory(), name="Trajectory")
            )

        if not any(t.get_name() == "TrafficLights" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_traffic_lights(), name="TrafficLights")
            )
        return task_list

    async def run(self):

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )

        self.pending = list(pending)

        for d in done:
            (who, data_msg) = d.result()

            if who == "TTD":
                self.ttd = TimeToDecision.deserialize(data_msg.data)
            elif who == "Trajectory":
                # getting the trajectory from the behaviour planner
                # it generates the kind of GPS waypoints towards the
                # destination
                self.trajectory = Trajectory.deserialize(data_msg.data)

                self.state = self.trajectory.state

                if (
                    self.trajectory.waypoints is not None
                    and len(self.trajectory.waypoints.waypoints) > 0
                ):

                    self.world.update_waypoints(
                        self.trajectory.waypoints.waypoints[-1].location,
                        self.trajectory.waypoints,
                    )

            elif who == "ObstaclePredictions":
                predictions_list = json.loads(data_msg.data.decode("utf-8"))
                self.obstacle_trajectories = []
                for p in predictions_list:
                    self.obstacle_trajectories.append(ObstaclePrediction.from_dict(p))

            elif who == "TrafficLights":
                traffic_lights = json.loads(data_msg.data.decode("utf-8"))
                self.traffic_lights = []
                for tl in traffic_lights:
                    self.traffic_lights.append(TrafficLight.from_dict(tl))

            elif who == "Pose":
                pose = Pose.deserialize(data_msg.data)

                predictions = self.get_predictions(
                    self.obstacle_trajectories, pose.transform
                )

                self.world.update(
                    pose.localization_time,
                    pose,
                    predictions,
                    self.traffic_lights,
                    self.map,
                    None,
                )

                (
                    speed_factor,
                    _,
                    _,
                    speed_factor_tl,
                    speed_factor_stop,
                ) = self.world.stop_for_agents(pose.localization_time)

                target_speed = speed_factor * self.target_speed

                output_wps = self.world.follow_waypoints(target_speed)

                # remove waypoints that are too close (we already reach them)
                output_wps.remove_waypoint_if_close(pose.transform.location)

                await self.output.send(output_wps.serialize())

        return None

    def setup(
        self,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ) -> Callable[[], Any]:

        pose_input = inputs.get("Pose", None)
        trajectory_input = inputs.get("Trajectory", None)
        traffic_lights_input = inputs.get("TrafficLights", None)
        obstacles_input = inputs.get("ObstaclePredictions", None)
        ttd_input = inputs.get("TTD", None)
        output = outputs.get("Waypoints", None)

        l = WaypointPlanner(
            configuration,
            trajectory_input,
            pose_input,
            traffic_lights_input,
            obstacles_input,
            ttd_input,
            output,
        )
        return l.run

    def finalize(self) -> None:
        return None


def register():
    return WaypointPlanner
