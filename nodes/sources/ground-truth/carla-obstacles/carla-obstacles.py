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

from stunt.types import Obstacle
from stunt import DEFAULT_SAMPLING_FREQUENCY, DEFAULT_CARLA_HOST, DEFAULT_CARLA_PORT

DEFAULT_OBSTACLE_TYPES = ["vehicle.*"]


class GTObstaclesState:
    def __init__(self, configuration):

        self.carla_port = DEFAULT_CARLA_PORT
        self.carla_host = DEFAULT_CARLA_HOST
        self.carla_world = None
        self.carla_client = None
        self.player = None

        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )
        self.obstacle_types = configuration.get("types", DEFAULT_OBSTACLE_TYPES)

        self.obstacles = None

        if configuration is not None and configuration.get("port") is not None:
            self.carla_port = int(configuration["port"])

        if configuration is not None and configuration.get("host") is not None:
            self.carla_host = configuration["host"]

        # Connecting to CARLA
        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()


class GroundTruthObstacles(Source):
    def __init__(self, state, output):
        self.state = state
        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.state.period)

        # getting simulator obstacles and filtering them
        sim_obstacles = []

        for obstacle_type in self.obstacles_types:

            current_obstacles = self.state.carla_world.get_actors().filter(
                self.state.obstacle_type
            )

            # removing ego vehicle from the detected obstacles, a car does not
            # detect itself
            current_obstacles = list(
                filter(lambda x: x.attributes["role_name"] != "hero", sim_obstacles)
            )

            sim_obstacles.extend(current_obstacles)

        obstacles = self.obstacles_to_bytes(sim_obstacles)
        await self.output.send(obstacles)

        return None

    def obstacles_to_bytes(self, data: List[carla.Actor]) -> bytes:

        obstacles = []

        for obstacle in data:
            obstacle_dict = Obstacle.from_simulator_actor(obstacle).to_dict()
            obstacles.append(obstacle_dict)

        return json.dumps(obstacles).encode("utf-8")

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        state = GTObstaclesState(configuration)
        output = outputs.get("Obstacles", None)
        g = GroundTruthObstacles(state, output)
        return g.create_data

    def finalize(self) -> None:
        return None


def register():
    return GroundTruthObstacles
