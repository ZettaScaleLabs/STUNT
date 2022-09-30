import json
import base64
import numpy as np
from carla import Image as CarlaImage


class Image(object):
    def __init__(
        self, fov=0.0, height=0, width=0, raw_data=np.ndarray([]), timestamp=0
    ):
        self.fov = fov
        self.height = height
        self.width = width
        self.raw_data = raw_data
        self.timestamp = timestamp

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"Image(fov={self.fov}, height={self.height}, "
        + "width={self.width}, raw_data={self.raw_data})"

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

        frame = (np.frombuffer(data.raw_data, dtype=np.dtype("uint8")),)
        frame = np.reshape(frame, (data.height, data.width, 4))
        return cls(
            data.fov,
            data.height,
            data.width,
            frame,
            data.timestamp * 1000,
        )

    def as_rgb_numpy_array(self):
        return self.raw_data[:, :, :3]

    def to_dict(self):

        raw_data_serialized = [
            str(self.raw_data.dtype),
            base64.b64encode(self.raw_data).decode("ascii"),
            self.raw_data.shape,
        ]

        return {
            "fov": self.fov,
            "height": self.height,
            "width": self.width,
            "raw_data": raw_data_serialized,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, dictionary):
        frame_info = dictionary["raw_data"]
        np_dtype = np.dtype(frame_info[0])
        raw_data = np.frombuffer(
            base64.b64decode(frame_info[1].encode("ascii")), dtype=np_dtype
        )
        raw_data = np.reshape(raw_data, frame_info[2])

        return cls(
            dictionary["fov"],
            dictionary["height"],
            dictionary["width"],
            raw_data,
            dictionary["timestamp"],
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
