import json
import numpy as np
from carla import Vector3D as CarlaVector3D


class Vector3D(object):
    """Represents a 3D vector and provides useful helper functions.

    Args:
        x: The value of the first axis.
        y: The value of the second axis.
        z: The value of the third axis.

    Attributes:
        x: The value of the first axis.
        y: The value of the second axis.
        z: The value of the third axis.
    """

    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    @classmethod
    def from_simulator_vector(cls, vector):
        """Creates a STUNT Vector3D from a simulator 3D vector.

        Args:
            vector: An instance of a simulator 3D vector.

        Returns:
            :py:class:`.Vector3D`: A STUNT 3D vector.
        """
        if not isinstance(vector, CarlaVector3D):
            raise ValueError("The vector must be a Vector3D")
        return cls(vector.x, vector.y, vector.z)

    def as_numpy_array(self):
        """Retrieves the 3D vector as a numpy array."""
        return np.array([self.x, self.y, self.z])

    def as_numpy_array_2D(self):
        """Drops the 3rd dimension."""
        return np.array([self.x, self.y])

    def as_simulator_vector(self):
        """Retrieves the 3D vector as an instance of simulator 3D vector.

        Returns:
            An instance of the simulator class representing the 3D vector.
        """

        return CarlaVector3D(self.x, self.y, self.z)

    def l1_distance(self, other):
        """Calculates the L1 distance between the point and another point.

        Args:
            other (:py:class:`~.Vector3D`): The other vector used to
                calculate the L1 distance to.

        Returns:
            :obj:`float`: The L1 distance between the two points.
        """
        return abs(self.x - other.x) + abs(self.y - other.y) + abs(self.z - other.z)

    def l2_distance(self, other) -> float:
        """Calculates the L2 distance between the point and another point.

        Args:
            other (:py:class:`~.Vector3D`): The other vector used to
                calculate the L2 distance to.

        Returns:
            :obj:`float`: The L2 distance between the two points.
        """
        vec = np.array([self.x - other.x, self.y - other.y, self.z - other.z])
        return np.linalg.norm(vec)

    def magnitude(self):
        """Returns the magnitude of the 3D vector."""
        return np.linalg.norm(self.as_numpy_array())

    def to_camera_view(self, extrinsic_matrix, intrinsic_matrix):
        """Converts the given 3D vector to the view of the camera using
        the extrinsic and the intrinsic matrix.

        Args:
            extrinsic_matrix: The extrinsic matrix of the camera.
            intrinsic_matrix: The intrinsic matrix of the camera.

        Returns:
            :py:class:`.Vector3D`: An instance with the coordinates converted
            to the camera view.
        """
        position_vector = np.array([[self.x], [self.y], [self.z], [1.0]])

        # Transform the points to the camera in 3D.
        transformed_3D_pos = np.dot(np.linalg.inv(extrinsic_matrix), position_vector)

        # Transform the points to 2D.
        position_2D = np.dot(intrinsic_matrix, transformed_3D_pos[:3])

        # Normalize the 2D points.
        location_2D = type(self)(
            float(position_2D[0] / position_2D[2]),
            float(position_2D[1] / position_2D[2]),
            float(position_2D[2]),
        )
        return location_2D

    def rotate(self, angle: float):
        """Rotate the vector by a given angle.

        Args:
            angle (float): The angle to rotate the Vector by (in degrees).

        Returns:
            :py:class:`.Vector3D`: An instance with the coordinates of the
            rotated vector.
        """
        x_ = (
            math.cos(math.radians(angle)) * self.x
            - math.sin(math.radians(angle)) * self.y
        )
        y_ = (
            math.sin(math.radians(angle)) * self.x
            - math.cos(math.radians(angle)) * self.y
        )
        return type(self)(x_, y_, self.z)

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
        }

    @classmethod
    def from_dict(cls, dictionary):
        return cls(dictionary["x"], dictionary["y"], dictionary["z"])

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)

    def __add__(self, other):
        """Adds the two vectors together and returns the result."""
        return type(self)(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)

    def __sub__(self, other):
        """Subtracts the other vector from self and returns the result."""
        return type(self)(x=self.x - other.x, y=self.y - other.y, z=self.z - other.z)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Vector3D(x={}, y={}, z={})".format(self.x, self.y, self.z)
