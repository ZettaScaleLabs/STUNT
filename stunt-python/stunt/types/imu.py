from stunt.types import Vector3D
from carla import IMUMeasurement as CarlaIMUMeasurement

from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import float64



@dataclass
class IMUMeasurement(IdlStruct):
    accelerometer: Vector3D
    compass: float64
    gyroscope: Vector3D
    timestamp: float64

    def __init__(
        self,
        accelerometer=Vector3D(),
        compass=0.0,
        gyroscope=Vector3D(),
        timestamp=0,
    ):
        self.accelerometer = accelerometer
        self.compass = compass
        self.gyroscope = gyroscope
        self.timestamp = timestamp

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"IMUMeasurement(accelerometer={self.accelerometer}, "
        +"compass={self.compass}, "
        +"gyroscope={self.gyroscope}, ts={self.timestamp})"

    @classmethod
    def from_simulator(cls, data):
        """Creates a STUNT imu from a simulator imu.

        Args:
            data: An instance of a simulator imu.

        Returns:
            :py:class:`.IMUMeasurement`: A STUNT IMU.
        """

        if not isinstance(data, CarlaIMUMeasurement):
            raise ValueError(
                "The data must be a Location or carla.IMUMeasurement"
            )

        accelerometer = Vector3D.from_simulator(data.accelerometer)
        gyroscope = Vector3D.from_simulator(data.gyroscope)

        return cls(
            accelerometer, data.compass, gyroscope, data.timestamp * 1000
        )

    def to_simulator(self):
        return CarlaIMUMeasurement(
            self.accelerometer.as_simulator_vector(),
            self.compass,
            self.gyroscope.as_simulator_vector(),
        )

    def to_dict(self):
        return {
            "accelerometer": self.accelerometer.to_dict(),
            "compass": self.compass,
            "gyroscope": self.gyroscope.to_dict(),
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, dictionary):

        accelerometer = Vector3D.from_dict(dictionary["accelerometer"])
        gyroscope = Vector3D.from_dict(dictionary["gyroscope"])

        return cls(
            accelerometer,
            dictionary["compass"],
            gyroscope,
            dictionary["timestamp"],
        )
