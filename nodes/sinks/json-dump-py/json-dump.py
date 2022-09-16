from zenoh_flow.interfaces import Sink
from zenoh_flow import DataReceiver
from typing import Dict, Any, Callable
import json

import asyncio

DEFAULT_OUTPUT_FILE = "/tmp/data-dump.log"


class JSONDump(Sink):
    def __init__(self, configuration, input_stream):
        configuration = {} if configuration is None else configuration
        self.output_file = open(
            configuration.get("out_file", DEFAULT_OUTPUT_FILE), "w+"
        )

        self.input_stream = input_stream

    def finalize(self):
        return None

    def setup(
        self, configuration: Dict[str, Any], inputs: Dict[str, DataReceiver]
    ) -> Callable[[], Any]:
        in_stream = inputs.get("Data", None)
        jdump = JSONDump(configuration, in_stream)
        return jdump.run

    async def run(self):
        data_msg = await self.input_stream.recv()
        data = data_msg.data.decode("utf-8")
        self.output_file.write(f"{data}\n")
        self.output_file.flush()
        return None


def register():
    return JSONDump
