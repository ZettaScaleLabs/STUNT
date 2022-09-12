from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable
import time
import asyncio

import json
import carla

from stunt.types import GnssMeasurement
from stunt import DEFAULT_SAMPLING_FREQUENCY, DEFAULT_CARLA_HOST, DEFAULT_CARLA_PORT


class CarlaGNSSSrcState:
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
                    self.player = vehicle
                    break

        self.data = GnssMeasurement()

        bp = self.carla_world.get_blueprint_library().find("sensor.other.gnss")
        self.sensor = self.carla_world.spawn_actor(
            bp, carla.Transform(), attach_to=self.player
        )

        self.sensor.listen(self.on_sensor_update)

    def on_sensor_update(self, data):
        self.data = GnssMeasurement.from_simulator(data)


class CarlaGNSSSrc(Source):
    def __init__(self, state, output):
        self.state = state
        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.state.period)
        await self.output.send(self.state.data.serialize())
        return None

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        state = CarlaGNSSSrcState(configuration)
        output = outputs.get("GNSS", None)
        aself = CarlaGNSSSrc(state, output)

        return aself.create_data

    def finalize(self) -> None:
        return None


def register():
    return CarlaGNSSSrc
