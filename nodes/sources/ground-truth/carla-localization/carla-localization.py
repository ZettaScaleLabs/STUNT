from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable, List
import time
import asyncio

from stunt.simulator.ground_truth import Localization
from stunt import DEFAULT_SAMPLING_FREQUENCY


class GroundTruthLocalization(Source):
    def __init__(self, configuration, output):

        configuration = {} if configuration is None else configuration
        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )

        self.sensor = Localization(configuration, self.on_sensor_update)

        self.pose = None
        self.output = output

    def on_sensor_update(self, data):
        self.pose = data

    async def create_data(self):
        await asyncio.sleep(self.period)

        if self.pose is not None:
            await self.output.send(self.pose.serialize())

        return None

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        output = outputs.get("Pose", None)

        aself = GroundTruthLocalization(configuration, output)
        return aself.create_data

    def finalize(self) -> None:
        return None


def register():
    return GroundTruthLocalization
