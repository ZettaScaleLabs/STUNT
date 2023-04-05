from zenoh_flow.interfaces import Sink
from zenoh_flow import Input
from zenoh_flow.types import Context
from typing import Dict, Any
import json
import zenoh

DEFAULT_ZENOH_LOCATOR = "tcp/127.0.0.1:7447"
DEFAULT_MODE = "peer"
DEFAULT_KE = "stunt/control"


class CtrlCarZenoh(Sink):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
    ):
        self.in_stream = inputs.get("Data", None)

        self.locator = configuration.get("locator", DEFAULT_ZENOH_LOCATOR)
        self.mode = configuration.get("mode", DEFAULT_MODE)
        self.ke = configuration.get("key_expression", DEFAULT_KE)

        self.zconf = zenoh.Config()

        self.zconf.insert_json5(zenoh.config.MODE_KEY, json.dumps(self.mode))
        self.zconf.insert_json5(
            zenoh.config.CONNECT_KEY, json.dumps([self.locator])
        )

        self.session = zenoh.open(self.zconf)

    def finalize(self):
        return None

    async def iteration(self):
        data_msg = await self.in_stream.recv()
        self.session.put(self.ke, data_msg.data)

        return None


def register():
    return CtrlCarZenoh
