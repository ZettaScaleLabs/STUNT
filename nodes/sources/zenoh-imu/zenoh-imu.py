from zenoh_flow.interfaces import Source
from zenoh_flow import Output
from zenoh_flow.types import Context
from typing import Any, Dict
import asyncio

import json
from stunt.types import IMUMeasurement
from stunt import DEFAULT_SAMPLING_FREQUENCY
import zenoh
from zenoh import Reliability

DEFAULT_ZENOH_LOCATOR = "tcp/127.0.0.1:7447"
DEFAULT_MODE = "peer"
DEFAULT_KE = "stunt/imu"


class ZenohIMU(Source):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        outputs: Dict[str, Output],
    ):

        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )

        self.locator = configuration.get("locator", DEFAULT_ZENOH_LOCATOR)
        self.mode = configuration.get("mode", DEFAULT_MODE)
        self.ke = configuration.get("key_expression", DEFAULT_KE)

        self.zconf = zenoh.Config()

        self.zconf.insert_json5(zenoh.config.MODE_KEY, json.dumps(self.mode))
        self.zconf.insert_json5(
            zenoh.config.CONNECT_KEY, json.dumps([self.locator])
        )

        self.session = zenoh.open(self.zconf)

        self.sub = self.session.declare_subscriber(
            self.ke,
            self.on_sensor_update,
            reliability=Reliability.RELIABLE(),
        )

        self.output = outputs.get("IMU", None)
        self.imu = None

    async def iteration(self):
        await asyncio.sleep(self.period)

        if self.imu is not None:
            await self.output.send(self.imu.serialize())
            self.imu = None

        return None

    def on_sensor_update(self, sample):
        self.imu = IMUMeasurement.deserialize(sample.payload)

    def finalize(self) -> None:
        self.sub.undeclare()
        self.session.close()
        return None


def register():
    return ZenohIMU
