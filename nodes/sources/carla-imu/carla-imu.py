from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from zenoh_flow.types import Context
from typing import Any, Dict, Callable
import time
import asyncio

import math
import json
import carla

from stunt.types import IMUMeasurement
from stunt.simulator.sensors import IMUSensor
from stunt import DEFAULT_SAMPLING_FREQUENCY


class CarlaIMU(Source):
    def __init__(self, context, configuration, outputs):

        configuration = {} if configuration is None else configuration

        self.sensor = IMUSensor(configuration, self.on_sensor_update)

        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )

        self.data = None

        self.output = outputs.get("IMU", None)

    def on_sensor_update(self, data):
        self.data = IMUMeasurement.from_simulator(data)

    async def iteration(self):
        await asyncio.sleep(self.period)
        if self.data is not None:
            await self.output.send(self.data.serialize())

        return None


    def finalize(self) -> None:
        return None


def register():
    return CarlaIMU
