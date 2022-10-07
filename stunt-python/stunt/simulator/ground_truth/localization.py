import carla
import time
import numpy as np

from stunt.types import Transform, Vector3D, Pose
from stunt import DEFAULT_CARLA_HOST, DEFAULT_CARLA_PORT

S_TO_MS = 1000


class Localization:
    def __init__(self, configuration, on_data):
        configuration = {} if configuration is None else configuration

        self.carla_port = int(configuration.get("port", DEFAULT_CARLA_PORT))
        self.carla_host = configuration.get("host", DEFAULT_CARLA_HOST)

        self.on_data = on_data

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        self.player = None
        self.sensor = None

        self.player = None
        while self.player is None:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter(
                "vehicle.*"
            )
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    self.player = vehicle
                    break

        self.carla_world.on_tick(self.on_world_tick)

    def on_world_tick(self, snapshot):
        vec_transform = Transform.from_simulator(self.player.get_transform())
        velocity_vector = Vector3D.from_simulator(self.player.get_velocity())

        forward_speed = np.linalg.norm(
            np.array([velocity_vector.x, velocity_vector.y, velocity_vector.z])
        )

        pose = Pose(
            vec_transform,
            forward_speed,
            velocity_vector,
            snapshot.timestamp.elapsed_seconds * S_TO_MS,
        )
        self.on_data(pose)
