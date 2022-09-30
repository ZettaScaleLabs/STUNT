import json
from carla import Vector3D as CarlaVector3D
from carla import Location as CarlaLocation

from stunt.types import Vector3D, Vector2D

import math


class Location(Vector3D):
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

    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        super(Location, self).__init__(x, y, z)

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

    @classmethod
    def from_dict(cls, dictionary):
        return cls(dictionary["x"], dictionary["y"], dictionary["z"])

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
