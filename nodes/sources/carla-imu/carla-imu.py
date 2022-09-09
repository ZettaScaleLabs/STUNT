from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable
import time
import asyncio

import math
import json
import carla


DEFAULT_SAMPLING_FREQUENCY = 30
DEFAULT_CARLA_HOST = "localhost"
DEFAULT_CARLA_PORT = 2000


class CarlaIMUSrcState:
    def __init__(self, configuration):

        self.carla_port = DEFAULT_CARLA_PORT
        self.carla_host = DEFAULT_CARLA_HOST
        self.period = 1 / DEFAULT_SAMPLING_FREQUENCY

        if configuration is not None and configuration.get("port") is not None:
            self.carla_port = int(configuration["port"])

        if configuration is not None and configuration.get("host") is not None:
            self.carla_host = configuration["host"]

        if configuration is not None and configuration.get("frequency") is not None:
            self.period = 1 / configuration["frequency"]

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        self.player = None
        while self.player is None:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter("vehicle.*")
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    print("Ego vehicle found")
                    self.player = vehicle
                    break

        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0

        bp = self.carla_world.get_blueprint_library().find("sensor.other.imu")
        self.sensor = self.carla_world.spawn_actor(
            bp, carla.Transform(), attach_to=self.player
        )

        self.sensor.listen(self.on_sensor_update)

    def on_sensor_update(self, data):
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], data.accelerometer.x)),
            max(limits[0], min(limits[1], data.accelerometer.y)),
            max(limits[0], min(limits[1], data.accelerometer.z)),
        )
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(data.gyroscope.z))),
        )
        self.compass = math.degrees(data.compass)


class CarlaIMUSrc(Source):
    def __init__(self, state, output):
        self.state = state
        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.state.period)

        d = {
            "accelerometer": self.state.accelerometer,
            "gyroscope": self.state.gyroscope,
            "compass": self.state.compass,
        }

        await self.output.send(json.dumps(d).encode("utf-8"))
        return None

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        state = CarlaIMUSrcState(configuration)
        output = outputs.get("IMU", None)
        i = CarlaIMUSrc(state, output)
        return i.create_data

    def finalize(self) -> None:
        return None


def register():
    return CarlaIMUSrc
