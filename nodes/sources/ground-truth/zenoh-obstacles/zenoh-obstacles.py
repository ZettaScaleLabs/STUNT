from zenoh_flow.interfaces import Source
from zenoh_flow import Output
from zenoh_flow.types import Context
from typing import Any, Dict
import asyncio
import json
from stunt import DEFAULT_SAMPLING_FREQUENCY
import zenoh
from zenoh import Reliability
from queue import LifoQueue, Empty
import logging

DEFAULT_ZENOH_LOCATOR = "tcp/127.0.0.1:7447"
DEFAULT_MODE = "peer"
DEFAULT_KE = "stunt/obstacles"


class ZenohObstacles(Source):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        outputs: Dict[str, Output],
    ):
        logging.basicConfig(level=logging.DEBUG)
        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )

        self.locator = configuration.get("locator", DEFAULT_ZENOH_LOCATOR)
        self.mode = configuration.get("mode", DEFAULT_MODE)
        self.ke = configuration.get("key_expression", DEFAULT_KE)

        self.zconf = zenoh.Config()

        self.zconf.insert_json5(zenoh.config.MODE_KEY, json.dumps(self.mode))
        self.zconf.insert_json5(
            zenoh.config.CONNECT_KEY, json.dumps([self.locator])
        )

        self.session = zenoh.open(self.zconf)

        self.sub = self.session.declare_subscriber(
            self.ke,
            self.on_sensor_update,
            reliability=Reliability.RELIABLE(),
        )

        self.output = outputs.get("Obstacles", None)
        self.obstacles = LifoQueue()

    async def iteration(self):
        await asyncio.sleep(self.period)
        obstacles = []
        flag = True

        while flag:
            try:
                obstacle = self.obstacles.get_nowait()
                obstacles.append(obstacle)
            except Empty as e:
                logging.warning(f'[ZenohObstacles] no data in the queue: {e}')
                flag = False

        if len(obstacles) > 0:
            await self.output.send(json.dumps(obstacles).encode("utf-8"))
        return None

    def on_sensor_update(self, sample):
        obs = json.loads(sample.payload.decode("utf-8"))
        for o in obs:
            self.obstacles.put_nowait(o)

    def finalize(self) -> None:
        self.sub.undeclare()
        self.session.close()
        return None


def register():
    return ZenohObstacles
