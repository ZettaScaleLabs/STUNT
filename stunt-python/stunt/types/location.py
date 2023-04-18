from carla import Vector3D as CarlaVector3D
from carla import Location as CarlaLocation
import numpy as np
from stunt.types import Vector2D, Vector3D

import math

from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import float64


@dataclass
class Location(IdlStruct):
    """Stores a 3D location, and provides useful helper methods.

    Args:
        x: The value of the x-axis.
        y: The value of the y-axis.
        z: The value of the z-axis.

    Attributes:
        x: The value of the x-axis.
        y: The value of the y-axis.
        z: The value of the z-axis.
    """
    x: float64
    y: float64
    z: float64

    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    @classmethod
    def from_vector3d(cls, vector3: Vector3D):
        if not isinstance(vector3, Vector3D):
            raise ValueError("The vector3 must be a Vector3D")
        return cls(vector3.x, vector3.y, vector3.z)

    @classmethod
    def from_simulator(cls, location):
        """Creates a STUNT Location from a simulator location.

        Args:
            location: An instance of a simulator location.

        Returns:
            :py:class:`.Location`: A STUNT location.
        """

        if not (
            isinstance(location, CarlaLocation)
            or isinstance(location, CarlaVector3D)
        ):
            raise ValueError("The location must be a Location or Vector3D")
        return cls(location.x, location.y, location.z)

    @classmethod
    def from_gps(cls, latitude: float, longitude: float, altitude: float):
        """Creates Location from GPS (latitude, longitude, altitude).

        This is the inverse of the _location_to_gps method found in
        https://github.com/carla-simulator/scenario_runner/blob/master/srunner/tools/route_manipulation.py
        """
        EARTH_RADIUS_EQUA = 6378137.0
        # The following reference values are applicable for towns 1 through 7,
        # and are taken from the corresponding OpenDrive map files.
        # LAT_REF = 49.0
        # LON_REF = 8.0
        # TODO:: Do not hardcode. Get the references from the open drive file.
        LAT_REF = 0.0
        LON_REF = 0.0

        scale = math.cos(LAT_REF * math.pi / 180.0)
        basex = scale * math.pi * EARTH_RADIUS_EQUA / 180.0 * LON_REF
        basey = (
            scale
            * EARTH_RADIUS_EQUA
            * math.log(math.tan((90.0 + LAT_REF) * math.pi / 360.0))
        )
        x = scale * math.pi * EARTH_RADIUS_EQUA / 180.0 * longitude - basex
        y = (
            scale
            * EARTH_RADIUS_EQUA
            * math.log((math.tan((90.0 + latitude) * math.pi / 360.0)))
            - basey
        )
        # This wasn't in the original method, but seems to be necessary.
        y *= -1

        return cls(x, y, altitude)

    def distance(self, other) -> float:
        """Calculates the Euclidean distance between the given point and the
        other point.

        Args:
            other (:py:class:`~.Location`): The other location used to
                calculate the Euclidean distance to.

        Returns:
            :obj:`float`: The Euclidean distance between the two points.
        """
        return (self - other).magnitude()

    def as_vector_2D(self) -> Vector2D:
        """Transforms the Location into a Vector2D.

        Note:
            The method just drops the z-axis.

        Returns:
            :py:class:`.Vector2D`: A 2D vector.
        """
        return Vector2D(self.x, self.y)

    def as_simulator_location(self):
        """Retrieves the location as a simulator location instance.

        Returns:
            An instance of the simulator class representing the location.
        """

        return CarlaLocation(self.x, self.y, self.z)

    def as_vector3(self):
        return Vector3D(self.x, self.y, self.z)

    def l1_distance(self, other):
        """Calculates the L1 distance between the point and another point.

        Args:
            other (:py:class:`~.Vector3D`): The other vector used to
                calculate the L1 distance to.

        Returns:
            :obj:`float`: The L1 distance between the two points.
        """
        return (
            abs(self.x - other.x)
            + abs(self.y - other.y)
            + abs(self.z - other.z)
        )

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

    def as_numpy_array(self):
        """Retrieves the 3D vector as a numpy array."""
        return np.array([self.x, self.y, self.z])

    def as_numpy_array_2D(self):
        """Drops the 3rd dimension."""
        return np.array([self.x, self.y])

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
        transformed_3D_pos = np.dot(
            np.linalg.inv(extrinsic_matrix), position_vector
        )

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

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Location(x={}, y={}, z={})".format(self.x, self.y, self.z)

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
        }

    def __add__(self, other):
        """Adds the two vectors together and returns the result."""
        return type(self)(
            x=self.x + other.x, y=self.y + other.y, z=self.z + other.z
        )

    def __sub__(self, other):
        """Subtracts the other vector from self and returns the result."""
        return type(self)(
            x=self.x - other.x, y=self.y - other.y, z=self.z - other.z
        )

    @classmethod
    def from_dict(cls, dictionary):
        return cls(dictionary["x"], dictionary["y"], dictionary["z"])