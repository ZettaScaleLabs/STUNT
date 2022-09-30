import copy
import json
import numpy as np
from numpy.linalg import inv

from stunt.types import Transform, Vector2D, Location

from carla import LidarMeasurement as CarlaLidarMeasurement


class LidarMeasurement(object):
    def __init__(
        self,
        channels=0,
        horizontal_angle=0.0,
        point_cloud=np.ndarray([]),
        timestamp=0,
    ):
        self.channels = channels
        self.horizontal_angle = horizontal_angle
        self.point_cloud = point_cloud
        self.timestamp = timestamp

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"LidarMeasurement(channels={self.channels}, "
        +" horizontal_angle={self.horizontal_angle}, "
        +" point_cloud={self.point_cloud})"

    @classmethod
    def from_simulator(cls, data):
        """Creates a STUNT lidar from a simulator lidar.

        Args:
            data: An instance of a simulator lidar.

        Returns:
            :py:class:`.LidarMeasurement`: A STUNT lidar.
        """

        if not isinstance(data, CarlaLidarMeasurement):
            raise ValueError(
                "The data must be a Location or carla.CarlaLidarMeasurement"
            )

        return cls(
            data.channels,
            data.horizontal_angle,
            np.frombuffer(data.raw_data, dtype=np.dtype("f4")).tolist(),
            data.timestamp * 1000,
        )

    def to_simulator(self):
        return CarlaLidarMeasurement(
            self.channels,
            self.horizontal_angle,
            self.point_cloud.tobytes(),
        )

    def to_dict(self):
        return {
            "channels": self.channels,
            "horizontal_angle": self.horizontal_angle,
            "point_cloud": self.point_cloud,
        }

    @classmethod
    def from_dict(cls, dictionary):

        return cls(
            dictionary["channels"],
            dictionary["horizontal_angle"],
            dictionary["point_cloud"],
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)


class PointCloud(object):
    def __init__(self, points, timestamp):
        self.timestamp = timestamp
        self.global_points = copy.deepcopy(points)
        self.points = self.to_camera_coordinates()

    def to_dict(self):
        return {
            "timestamp": self.timestamp,
            "global_points": self.global_points.to_bytes(),
        }

    @classmethod
    def from_dict(cls, dictionary):

        return cls(
            dictionary["timestamp"],
            np.frombuffer(
                dictionary["global_points"], dtype=np.dtype("f4")
            ).tolist(),
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)

    def to_camera_coordinates(
        self,
    ):
        # Converts points in lidar coordinates to points in camera coordinates.
        to_camera_tranform = Transform(
            matrix=np.array(
                [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
            )
        )
        return to_camera_tranform.transform_points(self.global_points)

    def merge(self, point_cloud):
        self.global_points = np.concatenate(
            (self.global_points, point_cloud.global_points), 0
        )
        self.points = np.concatenate((self.points, point_cloud.points), 0)

    def get_pixel_location(
        self, pixel, camera_intrinsic_matrix, camera_unreal_transform
    ):
        # Select only points that are in front.
        # Setting the threshold to 0.1 because super close points cause
        # floating point errors.
        fwd_points = self.points[np.where(self.points[:, 2] > 0.1)]
        if len(fwd_points) == 0:
            return None

        # Project our 2D pixel location into 3D space, onto the z=1 plane.
        p3d = np.dot(
            inv(camera_intrinsic_matrix),
            np.array([[pixel.x], [pixel.y], [1.0]]),
        )
        location = PointCloud.get_closest_point_in_point_cloud(
            fwd_points, Vector2D(p3d[0], p3d[1]), normalized=True
        )
        # Use the depth from the retrieved location.
        p3d *= np.array([location.z])
        p3d = p3d.transpose()
        # Convert from camera to unreal coordinates if the lidar type is
        # sensor.lidar.ray_cast
        camera_point_cloud = camera_unreal_transform.transform_points(p3d)[0]
        pixel_location = Location(
            camera_point_cloud[0], camera_point_cloud[1], camera_point_cloud[2]
        )
        return pixel_location

    @staticmethod
    def get_closest_point_in_point_cloud(
        fwd_points, pixel, normalized: bool = False
    ):
        """Finds the closest point in the point cloud to the given point.

        Args:
            pixel (:py:class:`~stunt.types.Vector2D`): Camera coordinates.

        Returns:
            :py:class:`~stunt.types.Location`: Closest point cloud point.
        """
        # Select x and y.
        pc_xy = fwd_points[:, 0:2]
        # Create an array from the x, y coordinates of the point.
        xy = np.array([pixel.x, pixel.y]).transpose()

        # Compute distance
        if normalized:
            # Select z
            pc_z = fwd_points[:, 2]
            # Divize x, y by z
            normalized_pc = pc_xy / pc_z[:, None]
            dist = np.sum((normalized_pc - xy) ** 2, axis=1)
        else:
            dist = np.sum((pc_xy - xy) ** 2, axis=1)

        # Select index of the closest point.
        closest_index = np.argmin(dist)

        # Return the closest point.
        return Location(
            fwd_points[closest_index][0],
            fwd_points[closest_index][1],
            fwd_points[closest_index][2],
        )

    @classmethod
    def from_lidar_measurement(cls, data):
        """Creates a STUNT lidar from a simulator lidar.

        Args:
            data: An instance of a simulator lidar.

        Returns:
            :py:class:`.LidarMeasurement`: A STUNT lidar.
        """

        if not isinstance(data, LidarMeasurement):
            raise ValueError("The data must be a stunt.types.LidarMeasurement")

        points = np.asarray(data.point_cloud)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        # Remove the intensity component of the point cloud.
        points = points[:, :3]
        return cls(points, data.timestamp)

    def __repr__(self):
        return "PointCloud(points: {})".format(self.points)

    def __str__(self):
        return "PointCloud(points: {}, number of points: {})".format(
            self.points, len(self.points)
        )
