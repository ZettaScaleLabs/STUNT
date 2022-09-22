from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable
import time
import asyncio

from stunt.types import LidarMeasurement
from stunt.simulator.sensors import LidarSensor
from stunt import DEFAULT_SAMPLING_FREQUENCY


class CarlaLidar(Source):
    def __init__(self, configuration, output):

        configuration = {} if configuration is None else configuration
        self.period = 1 / configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)

        self.lidar_reading = None

        self.sensor = LidarSensor(configuration, self.on_sensor_update)
        self.output = output

    def on_sensor_update(self, data):
        self.lidar_reading = LidarMeasurement.from_simulator(data)

    async def create_data(self):
        await asyncio.sleep(self.period)

        if self.lidar_reading is not None:
            await self.output.send(self.lidar_reading.serialize())

        return None

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        output = outputs.get("LIDAR", None)
        l = CarlaLidar(configuration, output)
        return l.create_data

    def finalize(self) -> None:
        return None


def register():
    return CarlaLidar
