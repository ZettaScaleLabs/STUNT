import json
import math
import numpy as np

from stunt.types import Location, Rotation, Vector3D, Vector2D

from carla import (
    Location as CarlaLocation,
    Rotation as CarlaRotation,
    Transform as CarlaTransform,
)


class Transform(object):
    """A class that stores the location and rotation of an obstacle.

    It can be created from a simulator transform, defines helper functions
    needed in STUNT, and makes the simulator transform serializable.

    A transform object is instantiated with either a location and a rotation,
    or using a matrix.

    Args:
        location (:py:class:`.Location`, optional): The location of the object
            represented by the transform.
        rotation (:py:class:`.Rotation`, optional): The rotation  (in degrees)
            of the object represented by the transform.
        matrix: The transformation matrix used to convert points in the 3D
            coordinate space with respect to the location and rotation of the
            given object.

    Attributes:
        location (:py:class:`.Location`): The location of the object
            represented by the transform.
        rotation (:py:class:`.Rotation`): The rotation (in degrees) of the
            object represented by the transform.
        forward_vector (:py:class:`.Vector3D`): The forward vector of the
            object represented by the transform.
        matrix: The transformation matrix used to convert points in the 3D
            coordinate space with respect to the location and rotation of the
            given object.
    """

    def __init__(
        self,
        location: Location = Location(),
        rotation: Rotation = Rotation(),
        matrix=None,
    ):
        if matrix is not None:
            self.matrix = matrix
            self.location = Location(matrix[0, 3], matrix[1, 3], matrix[2, 3])

            # Forward vector is retrieved from the matrix.
            self.forward_vector = Vector3D(
                self.matrix[0, 0], self.matrix[1, 0], self.matrix[2, 0]
            )
            pitch_r = math.asin(np.clip(self.forward_vector.z, -1, 1))
            yaw_r = math.acos(
                np.clip(self.forward_vector.x / math.cos(pitch_r), -1, 1)
            )
            roll_r = math.asin(
                np.clip(matrix[2, 1] / (-1 * math.cos(pitch_r)), -1, 1)
            )
            self.rotation = Rotation(
                math.degrees(pitch_r),
                math.degrees(yaw_r),
                math.degrees(roll_r),
            )
        else:
            self.location, self.rotation = location, rotation
            self.matrix = Transform._create_matrix(
                self.location, self.rotation
            )

            # Forward vector is retrieved from the matrix.
            self.forward_vector = Vector3D(
                self.matrix[0, 0], self.matrix[1, 0], self.matrix[2, 0]
            )

    @classmethod
    def from_simulator(cls, transform):
        """Creates a STUNT transform from a simulator transform.

        Args:
            transform: A simulator transform.

        Returns:
            :py:class:`.Transform`: An instance of a STUNT transform.
        """

        if not isinstance(transform, CarlaTransform):
            raise ValueError("transform should be of type Transform")
        return cls(
            Location.from_simulator(transform.location),
            Rotation.from_simulator(transform.rotation),
        )

    @staticmethod
    def _create_matrix(location, rotation):
        """Creates a transformation matrix to convert points in the 3D world
        coordinate space with respect to the object.

        Use the transform_points function to transpose a given set of points
        with respect to the object.

        Args:
            location (:py:class:`.Location`): The location of the object
                represented by the transform.
            rotation (:py:class:`.Rotation`): The rotation of the object
                represented by the transform.

        Returns:
            A 4x4 numpy matrix which represents the transformation matrix.
        """
        matrix = np.identity(4)
        cy = math.cos(np.radians(rotation.yaw))
        sy = math.sin(np.radians(rotation.yaw))
        cr = math.cos(np.radians(rotation.roll))
        sr = math.sin(np.radians(rotation.roll))
        cp = math.cos(np.radians(rotation.pitch))
        sp = math.sin(np.radians(rotation.pitch))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = cp * cy
        matrix[0, 1] = cy * sp * sr - sy * cr
        matrix[0, 2] = -1 * (cy * sp * cr + sy * sr)
        matrix[1, 0] = sy * cp
        matrix[1, 1] = sy * sp * sr + cy * cr
        matrix[1, 2] = cy * sr - sy * sp * cr
        matrix[2, 0] = sp
        matrix[2, 1] = -1 * (cp * sr)
        matrix[2, 2] = cp * cr
        return matrix

    def __transform(self, points, matrix):
        """Internal function to transform the points according to the
        given matrix. This function either converts the points from
        coordinate space relative to the transform to the world coordinate
        space (using self.matrix), or from world coordinate space to the
        space relative to the transform (using inv(self.matrix))

        Args:
            points: An n by 3 numpy array, where each row is the
                (x, y, z) coordinates of a point.
            matrix: The matrix of the transformation to apply.

        Returns:
            An n by 3 numpy array of transformed points.
        """
        # Needed format: [[X0,..Xn],[Y0,..Yn],[Z0,..Zn]].
        # So let's transpose the point matrix.
        points = points.transpose()

        # Add 1s row: [[X0..,Xn],[Y0..,Yn],[Z0..,Zn],[1,..1]]
        points = np.append(points, np.ones((1, points.shape[1])), axis=0)

        # Point transformation (depends on the given matrix)
        points = np.dot(matrix, points)

        # Get all but the last row in array form.
        points = np.asarray(points[0:3].transpose()).astype(np.float16)

        return points

    def transform_points(self, points):
        """Transforms the given set of points (specified in the coordinate
        space of the current transform) to be in the world coordinate space.

        For example, if the transform is at location (3, 0, 0) and the
        location passed to the argument is (10, 0, 0), this function will
        return (13, 0, 0) i.e. the location of the argument in the world
        coordinate space.

        Args:
            points: A (number of points) by 3 numpy array, where each row is
                the (x, y, z) coordinates of a point.

        Returns:
            An n by 3 numpy array of transformed points.
        """
        return self.__transform(points, self.matrix)

    def inverse_transform_points(self, points):
        """Transforms the given set of points (specified in world coordinate
        space) to be relative to the given transform.

        For example, if the transform is at location (3, 0, 0) and the location
        passed to the argument is (10, 0, 0), this function will return
        (7, 0, 0) i.e. the location of the argument relative to the given
        transform.

        Args:
            points: A (number of points) by 3 numpy array, where each row is
                the (x, y, z) coordinates of a point.

        Returns:
            An n by 3 numpy array of transformed points.
        """
        return self.__transform(points, np.linalg.inv(self.matrix))

    def transform_locations(self, locations):
        """Transforms the given set of locations (specified in the coordinate
        space of the current transform) to be in the world coordinate space.

        This method has the same functionality as transform_points, and
        is provided for convenience; when dealing with a large number of
        points, it is advised to use transform_points to avoid the slow
        conversion between a numpy array and list of locations.

        Args:
            points (list(:py:class:`.Location`)): List of locations.

        Returns:
            list(:py:class:`.Location`): List of transformed points.
        """
        points = np.array([loc.as_numpy_array() for loc in locations])
        transformed_points = self.__transform(points, self.matrix)
        return [Location(x, y, z) for x, y, z in transformed_points]

    def inverse_transform_locations(self, locations):
        """Transforms the given set of locations (specified in world coordinate
        space) to be relative to the given transform.

        This method has the same functionality as inverse_transform_points,
        and is provided for convenience; when dealing with a large number of
        points, it is advised to use inverse_transform_points to avoid the slow
        conversion between a numpy array and list of locations.

        Args:
            points (list(:py:class:`.Location`)): List of locations.

        Returns:
            list(:py:class:`.Location`): List of transformed points.
        """

        points = np.array([loc.as_numpy_array() for loc in locations])
        transformed_points = self.__transform(
            points, np.linalg.inv(self.matrix)
        )
        return [Location(x, y, z) for x, y, z in transformed_points]

    def as_simulator_transform(self):
        """Converts the transform to a simulator transform.

        Returns:
            An instance of the simulator class representing the Transform.
        """
        return CarlaTransform(
            CarlaLocation(self.location.x, self.location.y, self.location.z),
            CarlaRotation(
                pitch=self.rotation.pitch,
                yaw=self.rotation.yaw,
                roll=self.rotation.roll,
            ),
        )

    def get_angle_and_magnitude(self, target_loc):
        """Computes relative angle between the transform and a target location.

        Args:
            target_loc (:py:class:`.Location`): Location of the target.

        Returns:
            Angle in radians and vector magnitude.
        """
        target_vec = target_loc.as_vector_2D() - self.location.as_vector_2D()
        magnitude = target_vec.magnitude()
        if magnitude > 0:
            forward_vector = Vector2D(
                math.cos(math.radians(self.rotation.yaw)),
                math.sin(math.radians(self.rotation.yaw)),
            )
            angle = target_vec.get_angle(forward_vector)
        else:
            angle = 0
        return angle, magnitude

    def is_within_distance_ahead(
        self, dst_loc: Location, max_distance: float
    ) -> bool:
        """Checks if a location is within a distance.

        Args:
            dst_loc (:py:class:`.Location`): Location to compute distance to.
            max_distance (:obj:`float`): Maximum allowed distance.

        Returns:
            bool: True if other location is within max_distance.
        """
        d_angle, norm_dst = self.get_angle_and_magnitude(dst_loc)
        # Return if the vector is too small.
        if norm_dst < 0.001:
            return True
        # Return if the vector is greater than the distance.
        if norm_dst > max_distance:
            return False
        return d_angle < 90.0

    def inverse_transform(self):
        """Returns the inverse transform of this transform."""
        new_matrix = np.linalg.inv(self.matrix)
        return Transform(matrix=new_matrix)

    def __mul__(self, other):
        new_matrix = np.dot(self.matrix, other.matrix)
        return Transform(matrix=new_matrix)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        if self.location:
            return "Transform(location: {}, rotation: {})".format(
                self.location, self.rotation
            )
        else:
            return "Transform({})".format(str(self.matrix))

    def to_dict(self):
        return {
            "location": self.location.to_dict(),
            "rotation": self.rotation.to_dict(),
        }

    @classmethod
    def from_dict(cls, dictionary):
        location = Location.from_dict(dictionary["location"])
        rotation = Rotation.from_dict(dictionary["rotation"])

        return cls(location, rotation)

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
