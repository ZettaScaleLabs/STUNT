import carla
import time

from stunt import (
    DEFAULT_SAMPLING_FREQUENCY,
    DEFAULT_CARLA_HOST,
    DEFAULT_CARLA_PORT,
)
from stunt.types import IMUMeasurement


DEFAULT_IMU_NAME = "stunt-imu"
DEFAULT_NOISE_ACC_X_STDDEV = 0.0
DEFAULT_NOISE_ACC_Y_STDDEV = 0.0
DEFAULT_NOISE_ACC_Z_STDDEV = 0.0
DEFAULT_NOISE_GYRO_X_STDDEV = 0.0
DEFAULT_NOISE_GYRO_Y_STDDEV = 0.0
DEFAULT_NOISE_GYRO_Z_STDDEV = 0.0


class IMUSensor:
    def __init__(self, configuration, on_data):
        configuration = {} if configuration is None else configuration

        self.carla_port = int(configuration.get("port", DEFAULT_CARLA_PORT))
        self.carla_host = configuration.get("host", DEFAULT_CARLA_HOST)
        self.period = 1 / int(
            configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)
        )
        self.name = configuration.get("name", DEFAULT_IMU_NAME)

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
        self.sensor = None

        while self.player is None:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter(
                "vehicle.*"
            )
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    self.player = vehicle
                    # check if there is a IMU we can use
                    possible_imus = self.carla_world.get_actors().filter(
                        "sensor.other.imu"
                    )
                    for imu in possible_imus:
                        if (
                            imu.parent.id == self.player.id
                            and imu.attributes["role_name"] == self.name
                        ):
                            self.sensor = imu
                            break
                    break

        if self.sensor is None:
            bp = self.carla_world.get_blueprint_library().find(
                "sensor.other.imu"
            )

            bp.set_attribute("role_name", self.name)
            bp.set_attribute("sensor_tick", str(self.period))
            bp.set_attribute(
                "noise_accel_stddev_x", str(self.noise_accel_stddev_x)
            )
            bp.set_attribute(
                "noise_accel_stddev_y", str(self.noise_accel_stddev_y)
            )
            bp.set_attribute(
                "noise_accel_stddev_z", str(self.noise_accel_stddev_z)
            )
            bp.set_attribute(
                "noise_gyro_stddev_x", str(self.noise_gyro_stddev_x)
            )
            bp.set_attribute(
                "noise_gyro_stddev_y", str(self.noise_gyro_stddev_y)
            )
            bp.set_attribute(
                "noise_gyro_stddev_z", str(self.noise_gyro_stddev_z)
            )

            self.sensor = self.carla_world.spawn_actor(
                bp, carla.Transform(), attach_to=self.player
            )

        self.sensor.listen(on_data)
