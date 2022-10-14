from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from zenoh_flow.types import Context
from typing import Any, Dict
import asyncio

from stunt.simulator.ground_truth import Localization
from stunt import DEFAULT_SAMPLING_FREQUENCY


class GroundTruthLocalization(Source):
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

        self.sensor = Localization(configuration, self.on_sensor_update)

        self.pose = None
        self.output = outputs.get("Pose", None)

    def on_sensor_update(self, data):
        self.pose = data

    async def iteration(self):
        await asyncio.sleep(self.period)

        if self.pose is not None:
            await self.output.send(self.pose.serialize())
            self.pose = None

        return None

    def finalize(self) -> None:
        return None


def register():
    return GroundTruthLocalization
