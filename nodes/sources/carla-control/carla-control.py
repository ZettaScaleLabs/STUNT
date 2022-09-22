from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable
import time
import asyncio

from stunt.types import VehicleControl
from stunt.simulator.sensors import ControlSensor
from stunt import DEFAULT_SAMPLING_FREQUENCY


class CarlaControl(Source):
    def __init__(self, configuration, output):

        configuration = {} if configuration is None else configuration
        self.period = 1 / configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        self.sensor = ControlSensor(configuration)
        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.period)

        control = VehicleControl.from_simulator(self.sensor.read_data())

        await self.output.send(control.serialize())
        return None

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        output = outputs.get("Control", None)
        c = CarlaControl(configuration, output)
        return c.create_data

    def finalize(self) -> None:
        return None


def register():
    return CarlaControl
