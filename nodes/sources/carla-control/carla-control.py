from zenoh_flow.interfaces import Source
from zenoh_flow import Output
from zenoh_flow.types import Context
from typing import Any, Dict
import asyncio

from stunt.types import VehicleControl
from stunt.simulator.sensors import ControlSensor
from stunt import DEFAULT_SAMPLING_FREQUENCY


class CarlaControl(Source):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        outputs: Dict[str, Output],
    ):

        configuration = {} if configuration is None else configuration
        self.period = 1 / configuration.get(
            "frequency", DEFAULT_SAMPLING_FREQUENCY
        )
        self.sensor = ControlSensor(configuration)
        self.output = outputs.get("Control", None)

    async def iteration(self):
        await asyncio.sleep(self.period)

        control = VehicleControl.from_simulator(self.sensor.read_data())

        await self.output.send(control.serialize())
        return None

    def finalize(self) -> None:
        return None


def register():
    return CarlaControl
