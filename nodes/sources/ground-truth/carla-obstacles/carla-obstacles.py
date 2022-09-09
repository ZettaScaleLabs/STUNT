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

DEFAULT_SAMPLING_FREQUENCY = 30
DEFAULT_CARLA_HOST = "localhost"
DEFAULT_CARLA_PORT = 2000
DEFAULT_OBSTACLE_TYPE = "traffic.traffic_light*"


class GTObstaclesState:
    def __init__(self, configuration):

        self.carla_port = 2000
        self.carla_host = "localhost"
        self.carla_world = None
        self.carla_client = None
        self.player = None

        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )
        self.obstacle_type = configuration.get("type", DEFAULT_OBSTACLE_TYPE)

        self.obstacles = None

        if configuration is not None and configuration.get("port") is not None:
            self.carla_port = int(configuration["port"])

        if configuration is not None and configuration.get("host") is not None:
            self.carla_host = configuration["host"]

        # Connecting to CARLA
        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        # Waiting EGO vehicle
        while self.player is None:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter("vehicle.*")
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    print("Ego vehicle found")
                    self.player = vehicle
                    break


class GroundTruthObstacles(Source):
    def __init__(self, state, output):
        self.state = state
        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.state.period)

        # getting simulator obstacles and filtering them
        sim_obstacles = self.state.carla_world.get_actors().filter(
            self.state.obstacle_type
        )

        obstacles = self.obstacles_to_bytes(sim_obstacles)
        await self.output.send(obstacles)

        return None

    def obstacles_to_bytes(self, data: List[carla.Actor]) -> bytes:

        obstacles = []

        for obstacle in data:
            obst_dict = {
                "attributes": obstacle.attributes,
                "id": obstacle.id,
                "is_alive": obstacle.is_alive,
                "parent": None if obstacle.parent is None else obstacle.parent.id,
                # TODO: bump CARLA version, this crashes in 0.9.10
                # "semantic_tags": obstacle.semantic_tags,
                "type_id": obstacle.type_id,
            }
            obstacles.append(obst_dict)

        d = {
            "obstacles": obstacles,
        }

        return json.dumps(d).encode("utf-8")

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
