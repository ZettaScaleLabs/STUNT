from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable
import time
import asyncio
import json
import zenoh
from zenoh import Reliability, SubMode
from pycdr import cdr
from pycdr.types import int8, int32, uint32, float64

from stunt.types import Vector3D


@cdr
class Vector3:
    x: float64
    y: float64
    z: float64


@cdr
class Twist:
    linear: Vector3
    angular: Vector3


class State:
    def __init__(self, configuration):

        self.period = 1 / int(configuration.get("frequency"))
        self.router = configuration.get("router")
        self.key_expr = configuration.get("key_expr")

        self.twist = None

        self.config = zenoh.Config()
        self.config.insert_json5(zenoh.config.MODE_KEY, json.dumps("client"))
        self.config.insert_json5(zenoh.config.CONNECT_KEY, json.dumps([self.router]))
        self.session = zenoh.open(self.config)

        self.subscriber = self.session.subscribe(
            self.key_expr,
            self.on_sensor_update,
            reliability=Reliability.Reliable,
            mode=SubMode.Push,
        )

    def on_sensor_update(self, data):
        self.twist = Twist.deserialize(data.payload)


class TwistSrc(Source):
    def __init__(self, state, output):
        self.state = state
        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.state.period)

        if self.state.twist is not None:

            d = {
                "linear": Vector3D(
                    self.state.twist.linear.x,
                    self.state.twist.linear.y,
                    self.state.twist.linear.z,
                ).to_dict(),
                "angular": Vector3D(
                    self.state.twist.angular.x,
                    self.state.twist.angular.y,
                    self.state.twist.angular.z,
                ).to_dict(),
            }

            await self.output.send(json.dumps(d).encode(("utf-8")))

        return None

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        state = State(configuration)
        output = outputs.get("Twist", None)
        c = TwistSrc(state, output)
        return c.create_data

    def finalize(self) -> None:
        return None


def register():
    return TwistSrc
