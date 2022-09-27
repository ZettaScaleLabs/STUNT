from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from zenoh_flow.types import Context
from typing import Any, Dict, Callable
import time
import asyncio


from stunt.types import LidarMeasurement
import zenoh
from zenoh import Reliability, SubMode

DEFAULT_ZENOH_LOCATOR = "tcp/127.0.0.1:7447"
DEFAULT_MODE = "peer"
DEFAULT_KE = "/stunt/lidar"


class ZenohLidar(Source):
    def __init__(self,
        context,
        configuration,
        outputs
    ):

        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )

        self.locator = configuration.get("locator", DEFAULT_ZENOH_LOCATOR)
        self.mode = configuration.get("mode", DEFAULT_MODE)
        self.ke = configuration.get("key_expression", DEFAULT_KE)

        self.zconf = zenoh.Config()

        self.zconf.insert_json5(zenoh.config.MODE_KEY, json.dumps(self.mode))
        self.zconf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps([self.locator]))

        self.session = zenoh.open(self.zconf)

        self.sub = self.session.subscribe(
            self.ke,
            self.on_sensor_update,
            reliability=Reliability.Reliable,
            mode=SubMode.Push,
        )

        self.output = outputs.get("LIDAR", None)
        self.lidar = None

    async def iteration(self):
        await asyncio.sleep(self.state.period)

        if self.lidar is not None:
            await self.output.send(self.lidar.serialize())
            self.lidar = None

        return None

    def on_sensor_update(self, sample):
        self.lidar = LidarMeasurement.deserialize(sample.payload)


    def finalize(self) -> None:
        self.sub.close()
        self.session.close()
        return None


def register():
    return ZenohLidar
