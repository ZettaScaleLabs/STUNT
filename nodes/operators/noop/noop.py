from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any


class NoOp(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        self.output = outputs.get("out", None)
        self.in_stream = inputs.get("in", None)

        if self.in_stream is None:
            raise ValueError("No input 'in' found")
        if self.output is None:
            raise ValueError("No output 'out' found")

    def finalize(self) -> None:
        return None

    async def iteration(self) -> None:
        data_msg = await self.in_stream.recv()
        await self.output.send(data_msg.data)
        return None


def register():
    return NoOp