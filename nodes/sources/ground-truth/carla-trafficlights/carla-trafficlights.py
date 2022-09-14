from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable, List
import time
import asyncio

import math
import json
import carla
import array
import numpy as np

from stunt.types import TrafficLight
from stunt import DEFAULT_SAMPLING_FREQUENCY, DEFAULT_CARLA_HOST, DEFAULT_CARLA_PORT

TRAFFIC_LIGTHS_OBSTACLE_TYPE = "traffic.traffic_light*"


class GroundTruthTrafficLights(Source):
    def __init__(self, configuration, output):

        self.carla_port = DEFAULT_CARLA_PORT
        self.carla_host = DEFAULT_CARLA_HOST
        self.carla_world = None
        self.carla_client = None
        self.player = None

        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )
        self.obstacle_type = TRAFFIC_LIGTHS_OBSTACLE_TYPE

        if configuration is not None and configuration.get("port") is not None:
            self.carla_port = int(configuration["port"])

        if configuration is not None and configuration.get("host") is not None:
            self.carla_host = configuration["host"]

        # Connecting to CARLA
        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.period)

        # getting simulator obstacles and filtering them
        sim_obstacles = self.carla_world.get_actors().filter(self.obstacle_type)

        obstacles = self.obstacles_to_bytes(sim_obstacles)
        await self.output.send(obstacles)

        return None

    def obstacles_to_bytes(self, data: List[carla.Actor]) -> bytes:

        obstacles = []

        for obstacle in data:
            obstacle_dict = TrafficLight.from_simulator_actor(obstacle).to_dict()
            obstacles.append(obstacle_dict)

        return json.dumps(obstacles).encode("utf-8")

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        output = outputs.get("TrafficLights", None)
        g = GroundTruthTrafficLights(configuration, output)
        return g.create_data

    def finalize(self) -> None:
        return None


def register():
    return GroundTruthTrafficLights
