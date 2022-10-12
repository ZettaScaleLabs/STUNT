import json
import base64
import cv2
import io
import numpy as np
from carla import Image as CarlaImage

JPEG_QUALITY = 25


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
        value = (
            f"Image(fov={self.fov}, height={self.height}, "
            + f"width={self.width}, shape={self.raw_data.shape},"
            + f"type={type(self.raw_data)})"
        )
        return value

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

    @classmethod
    def to_jpeg(cls, frame, quality=JPEG_QUALITY):
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        jpeg = cv2.imencode(".jpg", frame, encode_params)[1]
        buf = io.BytesIO()
        np.save(buf, jpeg, allow_pickle=False)
        return buf.getvalue()

    @classmethod
    def from_jpeg(cls, jpeg):
        img = np.load(io.BytesIO(jpeg), allow_pickle=False)
        return cv2.imdecode(img, cv2.IMREAD_COLOR)

    def to_dict(self, quality=JPEG_QUALITY):

        jpeg_data = base64.b64encode(
            Image.to_jpeg(self.raw_data, quality)
        ).decode("ascii")
        return {
            "fov": self.fov,
            "height": self.height,
            "width": self.width,
            "raw_data": jpeg_data,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, dictionary):
        height = dictionary["height"]
        width = dictionary["width"]
        jpeg_data = base64.b64decode(dictionary["raw_data"].encode("ascii"))
        frame = Image.from_jpeg(jpeg_data)

        return cls(
            dictionary["fov"],
            height,
            width,
            frame,
            dictionary["timestamp"],
        )

    def serialize(self, quality=JPEG_QUALITY):
        return json.dumps(self.to_dict(quality)).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
