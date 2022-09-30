from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from zenoh_flow.types import Context
from typing import Any, Dict
import asyncio
import json


from stunt.simulator.ground_truth import TrafficLights
from stunt import DEFAULT_SAMPLING_FREQUENCY


class GroundTruthTrafficLights(Source):
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

        self.sensor = TrafficLights(configuration, self.on_sensor_update)
        self.traffic_lights = None
        self.output = outputs.get("TrafficLights", None)

    def on_sensor_update(self, data):
        self.traffic_lights = data

    async def iteration(self):
        await asyncio.sleep(self.period)

        if self.traffic_lights is not None:
            await self.output.send(
                json.dumps(self.traffic_lights).encode("utf-8")
            )

        return None

    def finalize(self) -> None:
        return None


def register():
    return GroundTruthTrafficLights
