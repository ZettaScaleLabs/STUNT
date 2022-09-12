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

from stunt.types import Image
from stunt import DEFAULT_SAMPLING_FREQUENCY, DEFAULT_CARLA_HOST, DEFAULT_CARLA_PORT

DEFAULT_CAMERA_LOCATION = (0.0, 0.0, 0.0)
DEFAULT_CAMERA_ROTATION = (0.0, 0.0, 0.0)
DEFAULT_CAMERA_TYPE = "sensor.camera.rgb"
DEFAULT_IMAGE_WIDTH = 1920
DEFAULT_IMAGE_HEIGHT = 1080
DEFAULT_CAMERA_FOV = 90


class CarlaCameraSrcState:
    def __init__(self, configuration):

        self.carla_port = DEFAULT_CARLA_PORT
        self.carla_host = DEFAULT_CARLA_HOST
        self.carla_world = None
        self.carla_client = None
        self.player = None

        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )
        self.camera_location = configuration.get("location", DEFAULT_CAMERA_LOCATION)
        self.camera_rotation = configuration.get("rotation", DEFAULT_CAMERA_ROTATION)
        self.camera_type = configuration.get("type", DEFAULT_CAMERA_TYPE)
        self.image_width = configuration.get("width", DEFAULT_IMAGE_WIDTH)
        self.image_height = configuration.get("height", DEFAULT_IMAGE_HEIGHT)
        self.camera_fov = configuration.get("fov", DEFAULT_CAMERA_FOV)

        self.frame = None

        if configuration is not None and configuration.get("port") is not None:
            self.carla_port = int(configuration["port"])

        if configuration is not None and configuration.get("host") is not None:
            self.carla_host = configuration["host"]

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
        bp = self.carla_world.get_blueprint_library().find(self.camera_type)

        bp.set_attribute("image_size_x", str(self.image_width))
        bp.set_attribute("image_size_y", str(self.image_height))
        bp.set_attribute("fov", str(self.camera_fov))
        bp.set_attribute("sensor_tick", str(self.period))

        cam_location = carla.Location(*self.camera_location)
        cam_rotation = carla.Rotation(*self.camera_rotation)

        # Attaching the sensor to the vehicle
        self.sensor = self.carla_world.spawn_actor(
            bp, carla.Transform(cam_location, cam_rotation), attach_to=self.player
        )

        self.sensor.listen(self.on_sensor_update)

    def on_sensor_update(self, data):
        self.frame = Image.from_simulator(data)


class CarlaCameraSrc(Source):
    def __init__(self, state, output):
        self.state = state
        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.state.period)

        if self.state.frame is not None:
            await self.output.send(self.state.frame.serialize())

        return None

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        state = CarlaCameraSrcState(configuration)
        output = outputs.get("Image", None)
        c = CarlaCameraSrc(state, output)
        return c.create_data

    def finalize(self) -> None:
        return None


def register():
    return CarlaCameraSrc
