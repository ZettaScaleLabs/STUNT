import json
import numpy as np
from carla import Image as CarlaImage


class Image(object):
    def __init__(
        self,
        fov=0.0,
        height=0,
        width=0,
        raw_data=np.ndarray([]),
    ):
        self.fov = fov
        self.height = height
        self.width = width
        self.raw_data = raw_data

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"Image(fov={self.fov}, height={self.height}, width={self.width}, raw_data={self.raw_data})"

    @classmethod
    def from_simulator(cls, data):
        """Creates a STUNT Image from a simulator Image.

        Args:
            data: An instance of a simulator Image.

        Returns:
            :py:class:`.Image`: A STUNT Image.
        """

        if not isinstance(data, CarlaImage):
            raise ValueError("The data must be a Location or carla.Image")

        return cls(
            data.fov,
            data.height,
            data.width,
            np.frombuffer(data.raw_data, dtype=np.dtype("uint8")).tolist(),
        )

    def to_simulator(self):
        return CarlaImage(
            self.fov,
            self.height,
            self.width,
            self.point_cloud.tobytes(),
        )

    def to_dict(self):
        return {
            "fov": self.fov,
            "height": self.height,
            "width": self.width,
            "raw_data": self.raw_data,
        }

    @classmethod
    def from_dict(cls, dictionary):

        return cls(
            dictionary["fov"],
            dictionary["height"],
            dictionary["width"],
            dictionary["raw_data"],
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
