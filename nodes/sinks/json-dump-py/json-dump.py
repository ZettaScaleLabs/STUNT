from zenoh_flow.interfaces import Sink
from zenoh_flow import DataReceiver
from zenoh_flow.types import Context
from typing import Dict, Any, Callable
import json

import asyncio

DEFAULT_OUTPUT_FILE = "/tmp/data-dump.log"


class JSONDump(Sink):
    def __init__(self, context, configuration, inputs):
        configuration = {} if configuration is None else configuration
        self.output_file = open(
            configuration.get("out_file", DEFAULT_OUTPUT_FILE), "w+"
        )

        self.input_stream = inputs.get("Data", None)

    def finalize(self):
        return None

    async def iteration(self):
        data_msg = await self.input_stream.recv()
        data = data_msg.data.decode("utf-8")
        self.output_file.write(f"{data}\n")
        self.output_file.flush()
        return None


def register():
    return JSONDump
