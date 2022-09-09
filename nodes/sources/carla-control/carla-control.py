from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable
import time
import asyncio

import json
import carla

DEFAULT_SAMPLING_FREQUENCY = 30
DEFAULT_CARLA_HOST = "localhost"
DEFAULT_CARLA_PORT = 2000


class CarlaControlSrcState:
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

    def on_world_tick(self, timestamp):
        None


class CarlaControlSrc(Source):
    def __init__(self, state, output):
        self.state = state
        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.state.period)
        control = self.state.player.get_control()
        await self.output.send(self.control_to_bytes(control))
        return None

    def control_to_bytes(self, vc: carla.VehicleControl) -> bytes:
        d = {
            "throttle": vc.throttle,
            "steer": vc.steer,
            "brake": vc.brake,
            "hand_brake": vc.hand_brake,
            "reverse": vc.reverse,
            "manual_gear_shift": vc.manual_gear_shift,
            "gear": vc.gear,
        }

        return json.dumps(d).encode("utf-8")

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        state = CarlaControlSrcState(configuration)
        output = outputs.get("Control", None)
        c = CarlaControlSrc(state, output)
        return c.create_data

    def finalize(self) -> None:
        return None


def register():
    return CarlaControlSrc
