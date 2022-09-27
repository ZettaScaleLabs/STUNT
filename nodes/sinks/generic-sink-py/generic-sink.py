from zenoh_flow.interfaces import Sink
from zenoh_flow import DataReceiver
from zenoh_flow.types import Context
from typing import Dict, Any, Callable
import json


class GenericSink(Sink):
    def finalize(self):
        return None

    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
    ):
        self.in_stream = inputs.get("Data", None)


    async def iteration(self):
        data_msg = await self.in_stream.recv()
        data = json.loads(data_msg.data.decode("utf-8"))
        print(f">>> Received {data}")
        return None


def register():
    return GenericSink
