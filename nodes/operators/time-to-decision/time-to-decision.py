from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from typing import Dict, Any, Callable
import time
import asyncio


from stunt.types import TimeToDecision, Pose


DEFAULT_BASE_DEADLINE_SPEED_MS = 10
# this is the deadline when driving at 10 m/s
DEFAULT_BASE_DEADLINE_MS = 400
# it decreases of DEFAULT_DECREASE_MS every 1 m/s of extra speed
DEFAULT_DECREASE_MS = 10


class TTD(Operator):
    def __init__(self, configuration, pose_input, output):
        configuration = configuration if configuration is not None else {}

        self.base_deadline_ms = int(
            configuration.get("base_deadline_ms", DEFAULT_BASE_DEADLINE_MS)
        )
        self.decrease_ms = configuration.get(
            "deadline_decrease_ms", DEFAULT_DECREASE_MS
        )

        self.base_speed = configuration.get(
            "base_deadline_speed_ms", DEFAULT_BASE_DEADLINE_SPEED_MS
        )

        self.pose_input = pose_input
        self.output = output

    async def run(self):

        # wait for location data
        data_msg = await self.pose_input.recv()
        pose = Pose.deserialize(data_msg.data)

        deadline = self.base_deadline_ms - (pose.forward_speed - self.base_speed) * self.decrease_ms

        await self.output.send(TimeToDecision(deadline).serialize())

        return None

    def setup(
        self,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ) -> Callable[[], Any]:

        pose_input = inputs.get("Pose", None)
        output = outputs.get("TTD", None)

        l = TTD(configuration, pose_input, output)
        return l.run

    def finalize(self) -> None:
        return None


def register():
    return TTD
