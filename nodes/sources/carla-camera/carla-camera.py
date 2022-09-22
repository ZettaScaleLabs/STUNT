from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable
import time
import asyncio


from stunt.types import Image
from stunt.simulator.sensors.camera import CameraSensor
from stunt import DEFAULT_SAMPLING_FREQUENCY


class CarlaCamera(Source):
    def __init__(self, configuration, output):
        configuration = {} if configuration is None else configuration

        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )
        self.sensor = CameraSensor(configuration, self.on_sensor_update)

        self.frame = None
        self.output = output

    def on_sensor_update(self, data):
        self.frame = Image.from_simulator(data)

    async def create_data(self):
        await asyncio.sleep(self.period)

        if self.frame is not None:
            await self.output.send(self.frame.serialize())

        return None

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:

        output = outputs.get("Image", None)
        c = CarlaCamera(configuration, output)
        return c.create_data

    def finalize(self) -> None:
        return None


def register():
    return CarlaCamera
