from zenoh_flow.interfaces import Sink
from zenoh_flow import DataReceiver
from typing import Dict, Any, Callable
import json


class GenericSink(Sink):
    def finalize(self):
        return None

    def setup(
        self, configuration: Dict[str, Any], inputs: Dict[str, DataReceiver]
    ) -> Callable[[], Any]:
        in_stream = inputs.get("Data", None)
        return lambda: run(in_stream)


async def run(in_stream):
    data_msg = await in_stream.recv()
    data = json.loads(data_msg.data.decode("utf-8"))
    print(f">>> Received {data}")
    return None


def register():
    return GenericSink
