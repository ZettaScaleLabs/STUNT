from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from zenoh_flow.types import Context
from typing import Dict, Any, Callable
import time
import asyncio

import json
from collections import defaultdict, deque
from cv2 import transform

from stunt.types import ObstacleTrajectory, Obstacle, Transform, Quaternion, Pose


DEFAULT_PREDICTION_EGO_AGENT = False
DEFAULT_OBSTACLE_DISTANCE_THRESHOLD_M = 30.0
DEFAULT_TRACKING_NUM_STEPS = 10


class PerfectTracker(Operator):
    def __init__(
        self,
        context,
        configuration,
        inputs,
        outputs,
    ):
        configuration = configuration if configuration is not None else {}

        self.pending = []

        self.prediction_ego_agent = configuration.get(
            "prediction_ego_agent", DEFAULT_PREDICTION_EGO_AGENT
        )
        self.dynamic_obstacle_distance_threshold = configuration.get(
            "dynamic_obstacle_distance_threshold", DEFAULT_OBSTACLE_DISTANCE_THRESHOLD_M
        )

        self.tracking_num_steps = configuration.get(
            "tracking_num_steps", DEFAULT_TRACKING_NUM_STEPS
        )

        self._obstacles = defaultdict(lambda: deque(maxlen=self.tracking_num_steps))

        self.obstacles_input = inputs.get("Obstacles", None)
        self.pose_input = inputs.get("Pose", None)
        self.output = outputs.get("ObstacleTrajectories", None)

        self.pose = Pose()
        self.speed_limits = []
        self.stop_signs = []

    async def wait_obstacles(self):
        data_msg = await self.obstacles_input.recv()
        return ("Obstacles", data_msg)

    async def wait_pose(self):
        data_msg = await self.pose_input.recv()
        return ("Pose", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "Pose" for t in task_list):
            task_list.append(asyncio.create_task(self.wait_pose(), name="Pose"))

        if not any(t.get_name() == "Obstacles" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_obstacles(), name="Obstacles")
            )
        return task_list

    async def iteration(self):

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )

        self.pending = list(pending)

        (who, data_msg) = done.pop().result()

        if who == "Pose":
            self.pose = Pose.deserialize(data_msg.data)

        elif who == "Obstacles":

            obstacle_trajectories = []

            obstacles_list = json.loads(data_msg.data.decode("utf-8"))
            for o in obstacles_list:
                obstacle = Obstacle.from_dict(o)

                # skipping the ego vehicle
                if obstacle.detailed_label == "hero":
                    continue

                # skip tracking the object is too far away
                if (
                    self.pose.transform.location.distance(obstacle.transform.location)
                    > self.dynamic_obstacle_distance_threshold
                ):
                    continue

                # storing the obstacle in the hystory
                self._obstacles[obstacle.id].append(obstacle)

                cur_obstacle_trajectory = []
                # Iterate through past frames of this obstacle.
                for past_obstacle_loc in self._obstacles[obstacle.id]:
                    # Get the transform of the center of the obstacle's bounding
                    # box, in relation to the Pose measurement.
                    v_transform = (
                        past_obstacle_loc.transform
                        * past_obstacle_loc.bounding_box.transform
                    )
                    new_transform = (
                        self.pose.transform.inverse_transform() * v_transform
                    )
                    cur_obstacle_trajectory.append(new_transform)

                    obs_traj = ObstacleTrajectory(
                        obstacle, cur_obstacle_trajectory
                    ).to_dict()

                    obstacle_trajectories.append(obs_traj)

                await self.output.send(
                    json.dumps(obstacle_trajectories).encode("utf-8")
                )

        return None

    def finalize(self) -> None:
        return None


def register():
    return PerfectTracker
