from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from zenoh_flow.types import Context
from typing import Any, Dict, Callable
import time
import asyncio

import json
import carla

from stunt.types import GnssMeasurement
from stunt.simulator.sensors import GNSSSensor
from stunt import DEFAULT_SAMPLING_FREQUENCY


class CarlaGNSS(Source):
    def __init__(self, context, configuration, outputs):
        configuration = {} if configuration is None else configuration

        self.period = 1 / configuration.get(
            "frequency", DEFAULT_SAMPLING_FREQUENCY
        )
        self.sensor = GNSSSensor(configuration, self.on_sensor_update)

        self.output = outputs.get("GNSS", None)
        self.data = None

    def on_sensor_update(self, data):
        self.data = GnssMeasurement.from_simulator(data)

    async def iteration(self):
        await asyncio.sleep(self.period)

        if self.data is not None:
            await self.output.send(self.data.serialize())
        return None

    def finalize(self) -> None:
        return None


def register():
    return CarlaGNSS
