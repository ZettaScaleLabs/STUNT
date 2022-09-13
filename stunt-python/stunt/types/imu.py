import json
from stunt.types import Vector3D
from carla import IMUMeasurement as CarlaIMUMeasurement


class IMUMeasurement(object):
    def __init__(
        self,
        accelerometer=Vector3D(),
        compass=0.0,
        gyroscope=Vector3D(),
    ):
        self.accelerometer = accelerometer
        self.compass = compass
        self.gyroscope = gyroscope

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"IMUMeasurement(accelerometer={self.accelerometer}, compass={self.compass}, gyroscope={self.gyroscope})"

    @classmethod
    def from_simulator(cls, data):
        """Creates a STUNT imu from a simulator imu.

        Args:
            data: An instance of a simulator imu.

        Returns:
            :py:class:`.IMUMeasurement`: A STUNT IMU.
        """

        if not isinstance(data, CarlaIMUMeasurement):
            raise ValueError("The data must be a Location or carla.IMUMeasurement")

        accelerometer = Vector3D.from_simulator_vector(data.accelerometer)
        gyroscope = Vector3D.from_simulator_vector(data.gyroscope)

        return cls(
            accelerometer,
            data.compass,
            gyroscope,
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
        }

    @classmethod
    def from_dict(cls, dictionary):

        accelerometer = Vector3D.from_dict(dictionary["accelerometer"])
        gyroscope = Vector3Dfrom_dict(dictionary["gyroscope"])

        return cls(
            accelerometer,
            dictionary["compass"],
            gyroscope,
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)