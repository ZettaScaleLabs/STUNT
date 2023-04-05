from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any
import asyncio
import json


from stunt.types import (
    Obstacle,
    Transform,
    LidarMeasurement,
    PointCloud,
    Rotation,
    Location,
    Pose,
    TrafficLight,
)

from stunt import (
    create_camera_unreal_transform,
    create_camera_intrinsic_matrix,
)

DEFAULT_CAMERA_FOV = 90
DEFAULT_CAMERA_WIDTH = 1920
DEFAULT_CAMERA_HEIGHT = 1080
DEFAULT_CAMERA_LOCATION = [1.3, 0.0, 1.8]
DEFAULT_CAMERA_ROTATION = [0.0, 0.0, 0.0]
DEFAULT_CAMERA_TYPE = "sensor.camera.rgb"
DEFAULT_LIDAR_LOCATION = [1.3, 0.0, 1.8]
DEFAULT_LIDAR_ROTATION = [0.0, 0.0, 0.0]
DEFAULT_LIDAR_TYPE = "sensor.lidar.ray_cast"


class ObstacleLocationFinder(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):

        configuration = {} if configuration is None else configuration

        self.pose_input = inputs.get("Pose", None)
        self.obstacles_input = inputs.get("TrackedObstacles", None)
        self.lidar_input = inputs.get("LIDAR", None)
        self.output = outputs.get("Obstacles", None)

        self.pending = []

        self.camera_fov = configuration.get("camera_fov", DEFAULT_CAMERA_FOV)
        self.camera_transform = Transform(
            Location(
                *configuration.get("camera_location", DEFAULT_CAMERA_LOCATION)
            ),
            Rotation(
                *configuration.get("camera_rotation", DEFAULT_CAMERA_ROTATION)
            ),
        )
        self.camera_types = configuration.get(
            "camera_type", DEFAULT_CAMERA_TYPE
        )
        self.camera_width = configuration.get(
            "camera_width", DEFAULT_CAMERA_WIDTH
        )
        self.camera_height = configuration.get(
            "camera_height", DEFAULT_CAMERA_HEIGHT
        )

        self.lidar_transform = Transform(
            Location(
                *configuration.get("lidar_location", DEFAULT_LIDAR_LOCATION)
            ),
            Rotation(
                *configuration.get("lidar_rotation", DEFAULT_LIDAR_ROTATION)
            ),
        )

        self.lidar_type = configuration.get("lidar_type", DEFAULT_LIDAR_TYPE)

        self.pose = None
        self.point_cloud = None

    async def wait_pose(self):
        data_msg = await self.pose_input.recv()
        return ("Pose", data_msg)

    async def wait_obstacles(self):
        data_msg = await self.obstacles_input.recv()
        return ("TrackedObstacles", data_msg)

    async def wait_lidar(self):
        data_msg = await self.lidar_input.recv()
        return ("LIDAR", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "Pose" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_pose(), name="Pose")
            )

        if not any(t.get_name() == "TrackedObstacles" for t in task_list):
            task_list.append(
                asyncio.create_task(
                    self.wait_obstacles(), name="TrackedObstacles"
                )
            )

        if not any(t.get_name() == "LIDAR" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_lidar(), name="LIDAR")
            )

        return task_list

    def get_obstacle_locations(self, obstacles):
        # Get the position of the camera in world frame of reference
        world_camera_transform = self.pose.transform * self.camera_transform
        camera_intrinsic_matrix = create_camera_intrinsic_matrix(
            self.camera_width, self.camera_height, self.camera_fov
        )

        camera_unreal_transform = create_camera_unreal_transform(
            world_camera_transform
        )

        obstacles_with_location = []
        for obstacle in obstacles:
            location = self.point_cloud.get_pixel_location(
                obstacle.bounding_box_2D.get_center_point(),
                camera_intrinsic_matrix,
                camera_unreal_transform,
            )
            if location is not None:
                obstacle.transform = Transform(location, Rotation())
                obstacles_with_location.append(obstacle.to_dict())

        return obstacles_with_location

    async def iteration(self):

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )

        self.pending = list(pending)

        for d in done:
            (who, data_msg) = d.result()

            # We compute on IMU
            if who == "Pose":
                self.pose = Pose.deserialize(data_msg.data)
            elif who == "LIDAR":
                lidar_reading = LidarMeasurement.deserialize(data_msg.data)
                self.point_cloud = PointCloud.from_lidar_measurement(
                    lidar_reading
                )
            elif who == "TrackedObstacles":
                dict_obstacles = json.loads(data_msg.data.decode("utf-8"))
                obstacles = []
                for o in dict_obstacles:
                    obs = Obstacle.from_dict(o)
                    if o.get("state") is not None:
                        obs = TrafficLight.from_dict(o)
                    obstacles.append(obs)

                obstacles_with_location = []
                if (
                    len(obstacles) > 0
                    and self.pose is not None
                    and self.point_cloud is not None
                ):
                    obstacles_with_location = self.get_obstacle_locations(
                        obstacles
                    )

                await self.output.send(
                    json.dumps(obstacles_with_location).encode("utf-8")
                )

                # consuming inputs
                self.pose = None
                self.point_cloud = None

        return None

    def finalize(self) -> None:
        return None


def register():
    return ObstacleLocationFinder
