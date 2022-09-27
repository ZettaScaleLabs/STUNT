import carla
import time
import numpy as np

from stunt.types import Obstacle, TrafficLight
from stunt import DEFAULT_CARLA_HOST, DEFAULT_CARLA_PORT

S_TO_MS = 1000
DEFAULT_OBSTACLE_TYPES = ["vehicle.*"]

TRAFFIC_LIGTHS_OBSTACLE_TYPE = "traffic.traffic_light*"


class Obstacles:
    def __init__(self, configuration, on_data):
        configuration = {} if configuration is None else configuration

        self.carla_port = int(configuration.get("port", DEFAULT_CARLA_PORT))
        self.carla_host = configuration.get("host", DEFAULT_CARLA_HOST)
        self.obstacles_types = configuration.get("types", DEFAULT_OBSTACLE_TYPES)

        self.on_data = on_data

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        self.carla_world.on_tick(self.on_world_tick)

    def on_world_tick(self, _):
        sim_obstacles = []
        obstacles = []

        for obstacle_type in self.obstacles_types:

            current_obstacles = self.carla_world.get_actors().filter(obstacle_type)
            # removing ego vehicle from the detected obstacles, a car does not
            # detect itself
            current_obstacles = list(
                filter(lambda x: x.attributes["role_name"] != "hero", sim_obstacles)
            )

            sim_obstacles.extend(current_obstacles)

        for sim_obs in sim_obstacles:
            obstacles.append(Obstacle.from_simulator_actor(sim_obs).to_dict())

        self.on_data(obstacles)


class TrafficLights:
    def __init__(self, configuration, on_data):
        configuration = {} if configuration is None else configuration

        self.carla_port = int(configuration.get("port", DEFAULT_CARLA_PORT))
        self.carla_host = configuration.get("host", DEFAULT_CARLA_HOST)
        self.obstacle_type = TRAFFIC_LIGTHS_OBSTACLE_TYPE

        self.on_data = on_data

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        self.carla_world.on_tick(self.on_world_tick)

    def on_world_tick(self, _):
        sim_obstacles = self.carla_world.get_actors().filter(self.obstacle_type)
        obstacles = []
        for sim_obs in sim_obstacles:
            obstacles.append(TrafficLight.from_simulator_actor(sim_obs).to_dict())

        self.on_data(obstacles)
