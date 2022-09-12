from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from typing import Dict, Any, Callable
import time
import asyncio
import json


from stunt.types import Vector3D, VehicleControl


class State(object):
    def __init__(self, configuration):
        configuration = configuration if configuration is not None else {}

        self.sensitivity = configuration.get("sensitivity", 1)


class T2C(Operator):
    def __init__(self, state, twist_input, output):
        self.state = state
        self.twist_input = twist_input
        self.output = output

    async def run(self):

        # wait for one of the input to be available
        data_msg = await self.twist_input.recv()

        deserialized = json.loads(data_msg.data.decode("utf-8"))

        linear = Vector3D.from_dict(deserialized["linear"])
        angular = Vector3D.from_dict(deserialized["angular"])

        throttle = linear.x * self.state.sensitivity if linear.x > 0 else 0.0
        brake = -linear.x * self.state.sensitivity if linear.x < 0 else 0.0
        steer = angular.z * self.state.sensitivity

        ctrl = VehicleControl(
            throttle,
            steer,
            brake,
        )

        await self.output.send(ctrl.serialize())

        return None

    def setup(
        self,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ) -> Callable[[], Any]:
        state = State(configuration)
        twist_input = inputs.get("Twist", None)
        output = outputs.get("Control", None)

        l = T2C(state, twist_input, output)
        return l.run

    def finalize(self) -> None:
        return None


def register():
    return T2C
