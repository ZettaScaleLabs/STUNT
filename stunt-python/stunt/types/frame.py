import json
import base64
import cv2
import io
import numpy as np
from carla import Image as CarlaImage


from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import float64, uint64, sequence, uint8

JPEG_QUALITY = 25


@dataclass
class Image(IdlStruct):
    fov: float64
    height: uint64
    width: uint64
    raw_data: sequence[uint8]
    timestamp: float64

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
            + f"width={self.width}, len={len(self.raw_data)},"
            + f"type={type(self.raw_data)}),"
            + f"timestamp={self.timestamp})"
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
            raise ValueError("The data must be a carla.Image")

        frame = (np.frombuffer(data.raw_data, dtype=np.dtype("uint8")),)
        frame = np.reshape(frame, (data.height, data.width, 4))
        frame_jpeg = cls.to_jpeg(frame)
        # print(f'Frame JPEG from fucking carla type is : {type(frame_jpeg)} value: {frame_jpeg[:80]} ')

        # tried = Image.from_jpeg(frame_jpeg, data.height, data.width)

        # print(f'Frame JPEG after from jpeg from fucking carla is: {type(tried)} value: {tried[:80]} ')

        # exit(1)



        return cls(
            data.fov,
            data.height,
            data.width,
            frame_jpeg,
            data.timestamp * 1000,
        )

    def as_rgb_numpy_array(self):
        return self.raw_data[:, :, :3]

    @classmethod
    def to_jpeg(cls, frame, quality=JPEG_QUALITY):

        # cv2.imwrite('raw.bmp', frame)

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        jpeg = cv2.imencode(".jpg", frame, encode_params)[1]
        # print(f'Frame JPEG type: {type(jpeg)} value:{jpeg[:80]} ')

        # cv2.imwrite('img.jpeg', frame)

        buf = io.BytesIO()
        np.save(buf, jpeg, allow_pickle=False)

        return buf.getvalue()

        # return jpeg.tobytes()

    @classmethod
    def from_jpeg(cls, jpeg):
        img = np.load(io.BytesIO(jpeg), allow_pickle=False)
        # img = np.frombuffer(jpeg, dtype=np.dtype("uint8"))
        # img = np.reshape(img, (height, width, 4))
        return cv2.imdecode(img, cv2.IMREAD_COLOR)

    # def to_dict(self, quality=JPEG_QUALITY):

    #     jpeg_data = base64.b64encode(
    #         Image.to_jpeg(self.raw_data, quality)
    #     ).decode("ascii")
    #     return {
    #         "fov": self.fov,
    #         "height": self.height,
    #         "width": self.width,
    #         "raw_data": jpeg_data,
    #         "timestamp": self.timestamp,
    #     }

    # @classmethod
    # def from_dict(cls, dictionary):
    #     height = dictionary["height"]
    #     width = dictionary["width"]
    #     jpeg_data = base64.b64decode(dictionary["raw_data"].encode("ascii"))
    #     frame = Image.from_jpeg(jpeg_data)

    #     return cls(
    #         dictionary["fov"],
    #         height,
    #         width,
    #         frame,
    #         dictionary["timestamp"],
    #     )

    # def serialize(self, quality=JPEG_QUALITY):
    #     return json.dumps(self.to_dict(quality)).encode("utf-8")

    # @classmethod
    # def deserialize(cls, serialized):
    #     deserialized = json.loads(serialized.decode("utf-8"))
    #     return cls.from_dict(deserialized)
