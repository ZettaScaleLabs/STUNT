from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable, List
import time
import asyncio

import json
import carla
import array
import numpy as np

DEFAULT_SAMPLING_FREQUENCY = 30
DEFAULT_CARLA_HOST = "localhost"
DEFAULT_CARLA_PORT = 2000
S_TO_MS = 1000


class GTLocalizationState:
    def __init__(self, configuration):

        self.carla_port = 2000
        self.carla_host = "localhost"
        self.carla_world = None
        self.carla_client = None
        self.player = None

        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )
        self.pose = None

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

        self.carla_world.on_tick(self.on_world_tick)

    def on_world_tick(self, snapshot):

        vec_transform = self.player.get_transform()
        velocity_vector = self.player.get_velocity()
        forward_speed = np.linalg.norm(
            np.array([velocity_vector.x, velocity_vector.y, velocity_vector.z])
        )

        self.pose = {
            "transform": {
                "location": (
                    vec_transform.location.x,
                    vec_transform.location.y,
                    vec_transform.location.z,
                ),
                "rotation": (
                    vec_transform.rotation.pitch,
                    vec_transform.rotation.yaw,
                    vec_transform.rotation.roll,
                ),
            },
            "forward_speed": forward_speed,
            "velocity_vector": (
                velocity_vector.x,
                velocity_vector.y,
                velocity_vector.z,
            ),
            "localization_time": snapshot.timestamp.elapsed_seconds * S_TO_MS,
        }


class GroundTruthLocalization(Source):
    def __init__(self, state, output):
        self.state = state
        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.state.period)

        if self.state.pose is not None:
            await self.output.send(json.dumps(self.state.pose).encode("utf-8"))

        return None

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        state = GTLocalizationState(configuration)
        output = outputs.get("Pose", None)

        aself = GroundTruthLocalization(state, output)
        return aself.create_data

    def finalize(self) -> None:
        return None


def register():
    return GroundTruthLocalization
