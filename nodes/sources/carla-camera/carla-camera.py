from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from zenoh_flow.types import Context
from typing import Any, Dict
import asyncio


from stunt.types import Image
from stunt.simulator.sensors.camera import CameraSensor
from stunt import DEFAULT_SAMPLING_FREQUENCY

DEFAULT_JPEG_QUALITY = 25


class CarlaCamera(Source):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        outputs: Dict[str, DataSender],
    ):
        configuration = {} if configuration is None else configuration

        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )

        self.jpeg_quality = configuration.get(
            "jpeg_quality", DEFAULT_JPEG_QUALITY
        )

        self.sensor = CameraSensor(configuration, self.on_sensor_update)

        self.frame = None
        self.output = outputs.get("Image", None)

    def on_sensor_update(self, data):
        self.frame = Image.from_simulator(data)

    async def iteration(self):
        await asyncio.sleep(self.period)

        if self.frame is not None:
            await self.output.send(self.frame.serialize(self.jpeg_quality))

        return None

    def finalize(self) -> None:
        return None


def register():
    return CarlaCamera
