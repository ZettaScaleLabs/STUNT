import time
from stunt.types import Vector3D, Transform


from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import float64


@dataclass
class Pose(IdlStruct):
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
    transform: Transform
    forward_speed: float64
    velocity_vector: Vector3D
    localization_time: float64

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
        str_repr = f"Pose(transform: {self.transform}, " \
            f"forward speed: {self.forward_speed}, " \
            f"velocity vector: {self.velocity_vector}, " \
            f" ts: {self.localization_time})"

        return str_repr

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
