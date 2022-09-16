from zenoh_flow.interfaces import Source
from zenoh_flow import DataSender
from typing import Any, Dict, Callable
import time
import asyncio

import math
import json
import carla

from stunt.types import IMUMeasurement
from stunt import DEFAULT_SAMPLING_FREQUENCY, DEFAULT_CARLA_HOST, DEFAULT_CARLA_PORT


DEFAULT_NOISE_ACC_X_STDDEV = 0.0
DEFAULT_NOISE_ACC_Y_STDDEV = 0.0
DEFAULT_NOISE_ACC_Z_STDDEV = 0.0
DEFAULT_NOISE_GYRO_X_STDDEV = 0.0
DEFAULT_NOISE_GYRO_Y_STDDEV = 0.0
DEFAULT_NOISE_GYRO_Z_STDDEV = 0.0


class CarlaIMUSrcState:
    def __init__(self, configuration):

        configuration = {} if configuration is None else configuration

        self.carla_port = int(configuration.get("port", DEFAULT_CARLA_PORT))
        self.carla_host = configuration.get("host", DEFAULT_CARLA_HOST)
        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )

        self.noise_accel_stddev_x = configuration.get(
            "noise_accel_stddev_x", DEFAULT_NOISE_ACC_X_STDDEV
        )
        self.noise_accel_stddev_y = configuration.get(
            "noise_accel_stddev_y", DEFAULT_NOISE_ACC_Y_STDDEV
        )
        self.noise_accel_stddev_z = configuration.get(
            "noise_accel_stddev_z", DEFAULT_NOISE_ACC_Z_STDDEV
        )
        self.noise_gyro_stddev_x = configuration.get(
            "noise_gyro_stddev_x", DEFAULT_NOISE_GYRO_X_STDDEV
        )
        self.noise_gyro_stddev_y = configuration.get(
            "noise_gyro_stddev_y", DEFAULT_NOISE_GYRO_Y_STDDEV
        )
        self.noise_gyro_stddev_z = configuration.get(
            "noise_gyro_stddev_z", DEFAULT_NOISE_GYRO_Z_STDDEV
        )

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        self.player = None
        while self.player is None:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter("vehicle.*")
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    print("Ego vehicle found")
                    self.player = vehicle
                    break

        self.data = IMUMeasurement()

        bp = self.carla_world.get_blueprint_library().find("sensor.other.imu")

        bp.set_attribute("sensor_tick", str(self.period))

        bp.set_attribute("noise_accel_stddev_x", str(self.noise_accel_stddev_x))
        bp.set_attribute("noise_accel_stddev_y", str(self.noise_accel_stddev_y))
        bp.set_attribute("noise_accel_stddev_z", str(self.noise_accel_stddev_z))
        bp.set_attribute("noise_gyro_stddev_x", str(self.noise_gyro_stddev_x))
        bp.set_attribute("noise_gyro_stddev_y", str(self.noise_gyro_stddev_y))
        bp.set_attribute("noise_gyro_stddev_z", str(self.noise_gyro_stddev_z))

        self.sensor = self.carla_world.spawn_actor(
            bp, carla.Transform(), attach_to=self.player
        )

        self.sensor.listen(self.on_sensor_update)

    def on_sensor_update(self, data):
        self.data = IMUMeasurement.from_simulator(data)


class CarlaIMUSrc(Source):
    def __init__(self, state, output):
        self.state = state
        self.output = output

    async def create_data(self):
        await asyncio.sleep(self.state.period)

        await self.output.send(self.state.data.serialize())
        return None

    def setup(
        self, configuration: Dict[str, Any], outputs: Dict[str, DataSender]
    ) -> Callable[[], Any]:
        state = CarlaIMUSrcState(configuration)
        output = outputs.get("IMU", None)
        i = CarlaIMUSrc(state, output)
        return i.create_data

    def finalize(self) -> None:
        return None


def register():
    return CarlaIMUSrc
