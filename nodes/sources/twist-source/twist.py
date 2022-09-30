from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from zenoh_flow.types import Context
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


class TwistSrc(Source):
    def __init__(self, context, configuration, outputs):
        self.output = outputs.get("Twist", None)

        self.period = 1 / int(configuration.get("frequency"))
        self.router = configuration.get("router")
        self.key_expr = configuration.get("key_expr")

        self.twist = None

        self.config = zenoh.Config()
        self.config.insert_json5(zenoh.config.MODE_KEY, json.dumps("client"))
        self.config.insert_json5(
            zenoh.config.CONNECT_KEY, json.dumps([self.router])
        )
        self.session = zenoh.open(self.config)

        self.subscriber = self.session.subscribe(
            self.key_expr,
            self.on_sensor_update,
            reliability=Reliability.Reliable,
            mode=SubMode.Push,
        )

    def on_sensor_update(self, data):
        self.twist = Twist.deserialize(data.payload)

    async def iteration(self):
        await asyncio.sleep(self.period)

        if self.twist is not None:

            d = {
                "linear": Vector3D(
                    self.twist.linear.x,
                    self.twist.linear.y,
                    self.twist.linear.z,
                ).to_dict(),
                "angular": Vector3D(
                    self.twist.angular.x,
                    self.twist.angular.y,
                    self.twist.angular.z,
                ).to_dict(),
            }

            await self.output.send(json.dumps(d).encode(("utf-8")))

        return None

    def finalize(self) -> None:
        return None


def register():
    return TwistSrc
