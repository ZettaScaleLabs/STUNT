from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable
import time
import asyncio

import math
import json
import carla
import array
import numpy as np

DEFAULT_LIDAR_LOCATION = (0.0, 0.0, 2.0)
DEFAULT_LIDAR_ROTATION = (0.0, 0.0, 0.0)
DEFAULT_LIDAR_ROTATION_FREQUENCY = 20
DEFAULT_LIDAR_RANGE_CM = 10000
DEFAULT_LIDAR_CHANNELS = 16
DEFAULT_LIDAR_UPPER_FOV = 15
DEFAULT_LIDAR_LOWER_FOV = 30
DEFAULT_LIDAR_POINTS_PER_SECOND = 500000

DEFAULT_ZERO_INTENSITY = 0.2
DEFAULT_GENERAL_RATE = 0.25
DEFAULT_INTENSITY_LIMIT = 0.6


class CarlaLidarSrcState:
    def __init__(self, configuration):

        self.carla_port = 2000
        self.carla_host = "localhost"
        self.carla_world = None
        self.carla_client = None
        self.player = None

        self.period = 1 / 30
        self.rotation_frequency = DEFAULT_LIDAR_ROTATION_FREQUENCY
        self.lidar_location = DEFAULT_LIDAR_LOCATION
        self.lidar_rotation = DEFAULT_LIDAR_ROTATION
        self.lidar_range_cm = DEFAULT_LIDAR_RANGE_CM
        self.lidar_channels = DEFAULT_LIDAR_CHANNELS
        self.lidar_upper_fov = DEFAULT_LIDAR_UPPER_FOV
        self.lidar_lower_fov = DEFAULT_LIDAR_LOWER_FOV
        self.lidar_pps = DEFAULT_LIDAR_POINTS_PER_SECOND

        self.lidar_reading = None

        if configuration is not None and configuration.get("port") is not None:
            self.carla_port = int(configuration["port"])

        if configuration is not None and configuration.get("host") is not None:
            self.carla_host = configuration["host"]

        if configuration is not None and configuration.get("frequency") is not None:
            self.period = 1 / configuration["frequency"]

        if (
            configuration is not None
            and configuration.get("rotation_frequency") is not None
        ):
            self.rotation_frequency = configuration["rotation_frequency"]

        # Connecting to CARLA
        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        # Waiting EGO vehicle
        while self.player is None:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter("vehicle.*")
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    print("Ego vehicle found")
                    self.player = vehicle
                    break

        #  Configuring the sensor in CARLA
        bp = self.carla_world.get_blueprint_library().find("sensor.lidar.ray_cast")

        bp.set_attribute("channels", str(self.lidar_channels))
        bp.set_attribute("range", str(self.lidar_range_cm))
        bp.set_attribute("points_per_second", str(self.lidar_pps))
        bp.set_attribute("rotation_frequency", str(self.rotation_frequency))
        bp.set_attribute("upper_fov", str(self.lidar_upper_fov))
        bp.set_attribute("lower_fov", str(self.lidar_lower_fov))
        bp.set_attribute("sensor_tick", str(1.0 / self.rotation_frequency))

        bp.set_attribute("dropoff_zero_intensity", str(DEFAULT_ZERO_INTENSITY))
        bp.set_attribute("dropoff_general_rate", str(DEFAULT_GENERAL_RATE))
        bp.set_attribute("dropoff_intensity_limit", str(DEFAULT_INTENSITY_LIMIT))

        lidar_location = carla.Location(*self.lidar_location)
        lidar_rotation = carla.Rotation(*self.lidar_rotation)

        # Attaching the sensor to the vehicle
        self.sensor = self.carla_world.spawn_actor(
            bp, carla.Transform(lidar_location, lidar_rotation), attach_to=self.player
        )

        self.sensor.listen(self.on_sensor_update)

    def on_sensor_update(self, data):
        self.lidar_reading = {
            "channels": data.channels,
            "horizontal_angle": data.horizontal_angle,
            "point_cloud": np.frombuffer(data.raw_data, dtype=np.dtype("f4")).tolist(),
        }


class CarlaLidarSrc(Source):
    def __init__(self, state, output):
        self.state = state
        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.state.period)

        if self.state.lidar_reading is not None:
            await self.output.send(json.dumps(self.state.lidar_reading).encode("utf-8"))

        return None

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        state = CarlaLidarSrcState(configuration)
        output = outputs.get("LIDAR", None)
        l = CarlaLidarSrc(state, output)
        return l.create_data

    def finalize(self) -> None:
        return None


def register():
    return CarlaLidarSrc
