import json
import numpy as np
from carla import LidarMeasurement as CarlaLidarMeasurement


class LidarMeasurement(object):
    def __init__(
        self,
        channels=0,
        horizontal_angle=0.0,
        point_cloud=np.ndarray([]),
    ):
        self.channels = channels
        self.horizontal_angle = horizontal_angle
        self.point_cloud = point_cloud

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"LidarMeasurement(channels={self.channels}, horizontal_angle={self.horizontal_angle}, point_cloud={self.point_cloud})"

    @classmethod
    def from_simulator(cls, data):
        """Creates a STUNT lidar from a simulator lidar.

        Args:
            data: An instance of a simulator lidar.

        Returns:
            :py:class:`.LidarMeasurement`: A STUNT lidar.
        """

        if not isinstance(data, CarlaLidarMeasurement):
            raise ValueError(
                "The data must be a Location or carla.CarlaLidarMeasurement"
            )

        return cls(
            data.channels,
            data.horizontal_angle,
            np.frombuffer(data.raw_data, dtype=np.dtype("f4")).tolist(),
        )

    def to_simulator(self):
        return CarlaLidarMeasurement(
            self.channels,
            self.horizontal_angle,
            self.point_cloud.tobytes(),
        )

    def to_dict(self):
        return {
            "channels": self.channels,
            "horizontal_angle": self.horizontal_angle,
            "point_cloud": self.point_cloud,
        }

    @classmethod
    def from_dict(cls, dictionary):

        return cls(
            dictionary["channels"],
            dictionary["horizontal_angle"],
            dictionary["point_cloud"],
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
