from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable
import time
import asyncio


from stunt.types import GnssMeasurement
import zenoh
from zenoh import Reliability, SubMode

DEFAULT_ZENOH_LOCATOR = "tcp/127.0.0.1:7447"
DEFAULT_MODE = "peer"
DEFAULT_KE = "/stunt/gnss"


class ZenohGNSS(Source):
    def __init__(self, configuration, output):

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

        self.output = output
        self.gnss = None

    async def create_data(self):
        await asyncio.sleep(self.state.period)

        if self.gnss is not None:
            await self.output.send(self.gnss.serialize())
            self.gnss = None

        return None

    def on_sensor_update(self, sample):
        self.gnss = GnssMeasurement.deserialize(sample.payload)

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        output = outputs.get("GNSS", None)
        c = ZenohGNSS(configuration, output)
        return c.create_data

    def finalize(self) -> None:
        self.sub.close()
        self.session.close()
        return None


def register():
    return ZenohGNSS
