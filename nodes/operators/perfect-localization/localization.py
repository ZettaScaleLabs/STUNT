from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from typing import Dict, Any, Callable
import time
import asyncio


class PerfectLocalization(Operator):
    def __init__(self, pose_input, output):
        self.input = pose_input
        self.output = output

    async def run(self):

        # wait for one of the input to be available
        data = await self.input.recv()

        # perfect localization, forwards the localization received from the
        # ground truth
        await self.output.send(data.data)

        return None

    def setup(
        self,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ) -> Callable[[], Any]:

        pose_input = inputs.get("Pose", None)

        output = outputs.get("Pose", None)
        l = PerfectLocalization(pose_input, output)
        return l.run

    def finalize(self) -> None:
        return None


def register():
    return PerfectLocalization
