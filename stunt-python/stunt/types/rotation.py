import numpy as np
from carla import Rotation as CarlaRotation

from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import float64


@dataclass
class Rotation(IdlStruct):
    """Used to represent the rotation of an actor or obstacle.

    Rotations are applied in the order: Roll (X), Pitch (Y), Yaw (Z).
    A 90-degree "Roll" maps the positive Z-axis to the positive Y-axis.
    A 90-degree "Pitch" maps the positive X-axis to the positive Z-axis.
    A 90-degree "Yaw" maps the positive X-axis to the positive Y-axis.

    Args:
        pitch: Rotation about Y-axis.
        yaw:   Rotation about Z-axis.
        roll:  Rotation about X-axis.

    Attributes:
        pitch: Rotation about Y-axis.
        yaw:   Rotation about Z-axis.
        roll:  Rotation about X-axis.
    """
    pitch: float64
    yaw: float64
    roll: float64

    def __init__(self, pitch: float = 0, yaw: float = 0, roll: float = 0):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

    @classmethod
    def from_simulator(cls, rotation):
        """Creates a STUNT Rotation from a simulator rotation.

        Args:
            rotation: An instance of a simulator rotation.

        Returns:
            :py:class:`.Rotation`: A STUNT rotation.
        """
        if not isinstance(rotation, CarlaRotation):
            raise ValueError("rotation should be of type Rotation")
        return cls(rotation.pitch, rotation.yaw, rotation.roll)

    def as_simulator_rotation(self):
        """Retrieves the rotation as an instance of a simulator rotation.

        Returns:
            An instance of a simulator class representing the rotation.
        """

        return CarlaRotation(self.pitch, self.yaw, self.roll)

    def as_numpy_array(self):
        """Retrieves the Rotation as a numpy array."""
        return np.array([self.pitch, self.yaw, self.roll])

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Rotation(pitch={}, yaw={}, roll={})".format(
            self.pitch, self.yaw, self.roll
        )

    def to_dict(self):
        return {
            "pitch": self.pitch,
            "yaw": self.yaw,
            "roll": self.roll,
        }

    @classmethod
    def from_dict(cls, dictionary):
        return cls(dictionary["pitch"], dictionary["yaw"], dictionary["roll"])
