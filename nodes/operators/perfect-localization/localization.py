from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from zenoh_flow.types import Context
from typing import Dict, Any


class PerfectLocalization(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ):
        self.input = inputs.get("Pose", None)
        self.output = outputs.get("Pose", None)

    async def iteration(self):

        # wait for one of the input to be available
        data = await self.input.recv()

        # perfect localization, forwards the localization received from the
        # ground truth
        await self.output.send(data.data)

        return None

    def finalize(self) -> None:
        return None


def register():
    return PerfectLocalization
