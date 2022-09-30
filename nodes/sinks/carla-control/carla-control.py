from zenoh_flow.interfaces import Sink
from zenoh_flow import DataReceiver
from zenoh_flow.types import Context
from typing import Dict, Any, Callable
import json

import time
import carla
from carla import VehicleControl as CarlaVehicleControl
from stunt.types import VehicleControl
from stunt import (
    DEFAULT_SAMPLING_FREQUENCY,
    DEFAULT_CARLA_HOST,
    DEFAULT_CARLA_PORT,
)


class CtrlCar(Sink):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
    ):

        self.in_stream = inputs.get("Data", None)

        self.carla_port = DEFAULT_CARLA_PORT
        self.carla_host = DEFAULT_CARLA_HOST

        if configuration is not None and configuration.get("port") is not None:
            self.carla_port = int(configuration["port"])

        if configuration is not None and configuration.get("host") is not None:
            self.carla_host = configuration["host"]

        if (
            configuration is not None
            and configuration.get("frequency") is not None
        ):
            self.period = 1 / configuration["frequency"]

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        self.player = None
        while self.player is None:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter(
                "vehicle.*"
            )
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    print("Ego vehicle found")
                    self.player = vehicle
                    break

    def finalize(self):
        return None

    async def iteration(self):
        data_msg = await self.in_stream.recv()
        ctrl = VehicleControl.deserialize(data_msg.data)
        carla_ctrl = CarlaVehicleControl()

        carla_ctrl.throttle = ctrl.throttle
        carla_ctrl.steer = ctrl.steer
        carla_ctrl.brake = ctrl.brake
        self.player.apply_control(carla_ctrl)

        return None


def register():
    return CtrlCar
