import json
import time
from stunt.types import Vector3D, Transform


class Pose(object):
    """Class used to wrap ego-vehicle information.

    Args:
        transform (:py:class:`~STUNT.utils.Transform`): Transform of the ego
            vehicle.
        forward_speed (:obj:`int`): Forward speed in m/s.
        velocity_vector (:py:class:`~STUNT.utils.Vector3D`): Velocity vector
            in world frame

    Attributes:
        transform (:py:class:`~STUNT.utils.Transform`): Transform of the ego
            vehicle.
        forward_speed (:obj:`int`): Forward speed in m/s.
        velocity_vector (:py:class:`~STUNT.utils.Vector3D`): Velocity vector
            in world frame
    """

    def __init__(
        self,
        transform: Transform = Transform(),
        forward_speed: float = 0,
        velocity_vector: Vector3D = None,
        localization_time: float = None,
    ):
        if not isinstance(transform, Transform):
            raise ValueError(
                "transform should be of type STUNT.utils.Transform"
            )
        self.transform = transform
        # Forward speed in m/s.
        self.forward_speed = forward_speed
        self.velocity_vector = velocity_vector
        if localization_time is None:
            self.localization_time = time.time()
        else:
            self.localization_time = localization_time

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"Pose(transform: {self.transform}, "
        +"forward speed: {self.forward_speed}, "
        +"velocity vector: {self.velocity_vector},"
        +" ts: {self.localization_time})"

    def to_dict(self):
        return {
            "transform": self.transform.to_dict(),
            "forward_speed": self.forward_speed,
            "velocity_vector": self.velocity_vector.to_dict(),
            "localization_time": self.localization_time,
        }

    @classmethod
    def from_dict(cls, dictionary):
        transform = Transform.from_dict(dictionary["transform"])
        velocity_vector = Vector3D.from_dict(dictionary["velocity_vector"])

        return cls(
            transform,
            dictionary["forward_speed"],
            velocity_vector,
            dictionary["localization_time"],
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
