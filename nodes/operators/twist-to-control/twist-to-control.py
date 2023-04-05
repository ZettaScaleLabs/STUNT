from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import json


from stunt.types import Vector3D, VehicleControl


class T2C(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        configuration = configuration if configuration is not None else {}

        self.sensitivity = configuration.get("sensitivity", 1)
        self.twist_input = inputs.get("Twist", None)
        self.output = outputs.get("Control", None)

    async def iteration(self):

        # wait for one of the input to be available
        data_msg = await self.twist_input.recv()

        deserialized = json.loads(data_msg.data.decode("utf-8"))

        linear = Vector3D.from_dict(deserialized["linear"])
        angular = Vector3D.from_dict(deserialized["angular"])

        throttle = linear.x * self.sensitivity if linear.x > 0 else 0.0
        brake = -linear.x * self.sensitivity if linear.x < 0 else 0.0
        steer = angular.z * self.sensitivity

        ctrl = VehicleControl(
            throttle,
            steer,
            brake,
        )

        await self.output.send(ctrl.serialize())

        return None

    def finalize(self) -> None:
        return None


def register():
    return T2C
