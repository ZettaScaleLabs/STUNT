from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable, List
import time
import asyncio

import math
import json
import carla
import array
import numpy as np

from stunt.types import Obstacle
from stunt.simulator.ground_truth import Obstacles
from stunt import DEFAULT_SAMPLING_FREQUENCY


class GroundTruthObstacles(Source):
    def __init__(self, configuration, output):
        configuration = {} if configuration is None else configuration

        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )

        self.sensor = Obstacles(configuration, self.on_sensor_update)

        self.obstacles = None
        self.output = output

    def on_sensor_update(self, data):
        self.obstacles = data

    async def create_data(self):
        await asyncio.sleep(self.period)

        if self.obstacles is not None:
            await self.output.send(json.dumps(self.obstacles).encode("utf-8"))

        return None


    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        output = outputs.get("Obstacles", None)
        g = GroundTruthObstacles(configuration, output)
        return g.create_data

    def finalize(self) -> None:
        return None


def register():
    return GroundTruthObstacles
