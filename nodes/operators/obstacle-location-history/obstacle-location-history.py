from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any
import asyncio
import json

from collections import defaultdict, deque

from stunt.types import (
    Obstacle,
    Transform,
    LidarMeasurement,
    PointCloud,
    Rotation,
    Pose,
    Image,
    ObstacleTrajectory,
)


DEFAULT_OBSTACLE_DISTANCE_THRESHOLD_M = 30.0
DEFAULT_TRACKING_NUM_STEPS = 10


class ObstacleLocationHistory(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        # TODO: commented inputs are not used, they need to be removed
        configuration = {} if configuration is None else configuration

        self.pose_input = inputs.get("Pose", None)
        self.obstacles_input = inputs.get("Obstacles", None)
        # self.lidar_input = inputs.get("LIDAR", None)
        # self.tracker_delay_input = inputs.get("TrackerDelay", None)
        # self.image_input = inputs.get("Image", None)
        self.output = outputs.get("ObstacleTrajectories", None)

        self.pending = []

        self.dynamic_obstacle_distance_threshold = configuration.get(
            "dynamic_obstacle_distance_threshold",
            DEFAULT_OBSTACLE_DISTANCE_THRESHOLD_M,
        )

        self.tracking_num_steps = configuration.get(
            "tracking_num_steps", DEFAULT_TRACKING_NUM_STEPS
        )

        self.pose = None
        self.point_cloud = None
        self.image = None
        self.delay = None
        self.obstacle_history = defaultdict(deque)
        self.timestamp_history = deque()
        self.timestamp_to_id = defaultdict(list)

    async def wait_pose(self):
        data_msg = await self.pose_input.recv()
        return ("Pose", data_msg)

    async def wait_obstacles(self):
        data_msg = await self.obstacles_input.recv()
        return ("Obstacles", data_msg)

    # async def wait_lidar(self):
    #     data_msg = await self.lidar_input.recv()
    #     return ("LIDAR", data_msg)

    # async def wait_delay(self):
    #     data_msg = await self.tracker_delay_input.recv()
    #     return ("TrackerDelay", data_msg)

    # async def wait_image(self):
    #     data_msg = await self.image_input.recv()
    #     return ("Image", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "Pose" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_pose(), name="Pose")
            )

        if not any(t.get_name() == "Obstacles" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_obstacles(), name="Obstacles")
            )

        # if not any(t.get_name() == "LIDAR" for t in task_list):
        #     task_list.append(
        #         asyncio.create_task(self.wait_lidar(), name="LIDAR")
        #     )

        # if not any(t.get_name() == "TrackerDelay" for t in task_list):
        #     task_list.append(
        #         asyncio.create_task(self.wait_delay(), name="TrackerDelay")
        #     )

        # if not any(t.get_name() == "Image" for t in task_list):
        #     task_list.append(
        #         asyncio.create_task(self.wait_image(), name="Image")
        #     )

        return task_list

    async def iteration(self):

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )

        self.pending = list(pending)

        for d in done:
            (who, data_msg) = d.result()

            if who == "Pose":
                self.pose = Pose.deserialize(data_msg.data)
            elif who == "LIDAR":
                lidar_reading = LidarMeasurement.deserialize(data_msg.data)
                self.point_cloud = PointCloud.from_lidar_measurement(
                    lidar_reading
                )
            elif who == "Image":
                self.image = Image.deserialize(data_msg.data)
            elif who == "TrackerDelay":
                pass
            elif who == "Obstacles":
                # We compute on Obstacles
                if self.pose is None:
                    return None

                timestamp = self.pose.localization_time

                # deserializing obstacles
                dict_obstacles = json.loads(data_msg.data.decode("utf-8"))
                # print(f'ObstacleLocationHistory received obstacles {len(dict_obstacles)}')
                obstacles = []
                for o in dict_obstacles:
                    obstacles.append(Obstacle.from_dict(o))

                obstacles_trajectories = []
                ids_current_timestamp = []

                for obstacle in obstacles:
                    if (
                        self.pose.transform.location.distance(
                            obstacle.transform.location
                        )
                        <= self.dynamic_obstacle_distance_threshold
                    ):
                        ids_current_timestamp.append(obstacle.id)
                        self.obstacle_history[obstacle.id].append(obstacle)

                        current_obstacle_trajectory = []
                        for cur_obstacle in self.obstacle_history.get(
                            obstacle.id, []
                        ):
                            new_location = self.pose.transform.inverse_transform_locations(
                                [obstacle.transform.location]
                            )[
                                0
                            ]
                            # FIXME: need a function that can inverse transform
                            # one location

                            current_obstacle_trajectory.append(
                                Transform(new_location, Rotation())
                            )
                        obstacles_trajectories.append(
                            ObstacleTrajectory(
                                obstacle, current_obstacle_trajectory
                            ).to_dict()
                        )

                self.timestamp_history.append(timestamp)
                self.timestamp_to_id[timestamp] = ids_current_timestamp

                # removing older positions and objects
                if len(self.timestamp_history) >= self.tracking_num_steps:
                    gc_timestamp = self.timestamp_history.popleft()
                    for obstacle_id in self.timestamp_to_id[gc_timestamp]:
                        self.obstacle_history[obstacle_id].popleft()

                        # if the id has no locations, remove it
                        if len(self.obstacle_history[obstacle_id]) == 0:
                            self.obstacle_history.pop(obstacle_id)

                    # remove oldest ids
                    self.timestamp_to_id.pop(gc_timestamp)

                # print(f'ObstacleLocationHistory computed trajectories {len(obstacles_trajectories)}')
                await self.output.send(
                    json.dumps(obstacles_trajectories).encode("utf-8")
                )

                # consuming data
                self.pose = None

        return None

    def finalize(self) -> None:
        return None


def register():
    return ObstacleLocationHistory
