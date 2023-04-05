import numpy as np

from stunt.types import Vector2D, Vector3D, Transform, Location, Rotation

from carla import BoundingBox as CarlaBoundingBox
from shapely.geometry import LineString

from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import float64
from typing import Dict

@dataclass
class BoundingBox2D(IdlStruct):
    """Class that stores a 2D bounding box."""
    x_min: float64
    x_max: float64
    y_min: float64
    y_max: float64

    def __init__(self, x_min, x_max, y_min, y_max):
        assert x_min < x_max and y_min < y_max
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

    def get_min_point(self) -> Vector2D:
        return Vector2D(self.x_min, self.y_min)

    def get_max_point(self) -> Vector2D:
        return Vector2D(self.x_max, self.y_max)

    def get_height(self):
        return self.y_max - self.y_min

    def get_width(self):
        return self.x_max - self.x_min

    def get_center_point(self) -> Vector2D:
        # revert to using // if a problem is raised in the future
        return Vector2D(
            (self.x_min + self.x_max) / 2, (self.y_min + self.y_max) / 2
        )

    def as_width_height_bbox(self):
        return [self.x_min, self.y_min, self.get_width(), self.get_height()]

    def is_within(self, point) -> bool:
        """Checks if a point is within the bounding box."""
        return (
            point.x >= self.x_min
            and point.x <= self.x_max
            and point.y >= self.y_min
            and point.y <= self.y_max
        )

    def calculate_iou(self, other_bbox) -> float:
        """Calculate the IoU of a single bounding box.

        Args:
            other_bbox (:py:class:`.BoundingBox2D`): The other bounding box.

        Returns:
            :obj:`float`: The IoU of the two bounding boxes.
        """
        if (
            other_bbox.x_min > other_bbox.x_max
            or other_bbox.y_min > other_bbox.y_max
        ):
            raise AssertionError(
                "Other bbox is malformed {}".format(other_bbox)
            )

        if self.x_min > self.x_max or self.y_min > self.y_max:
            raise AssertionError("Bounding box is malformed {}".format(self))

        if (
            self.x_max < other_bbox.x_min
            or other_bbox.x_max < self.x_min
            or self.y_max < other_bbox.y_min
            or other_bbox.y_max < self.y_min
        ):
            return 0.0

        inter_x1 = max([self.x_min, other_bbox.x_min])
        inter_x2 = min([self.x_max, other_bbox.x_max])

        inter_y1 = max([self.y_min, other_bbox.y_min])
        inter_y2 = min([self.y_max, other_bbox.y_max])

        inter_area = (inter_x2 - inter_x1 + 1) * (inter_y2 - inter_y1 + 1)
        gt_area = (self.x_max - self.x_min + 1) * (self.y_max - self.y_min + 1)
        pred_area = (other_bbox.x_max - other_bbox.x_min + 1) * (
            other_bbox.y_max - other_bbox.y_min + 1
        )
        return float(inter_area) / (gt_area + pred_area - inter_area)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "BoundingBox2D(xmin: {}, xmax: {}, ymin: {}, ymax: {})".format(
            self.x_min, self.x_max, self.y_min, self.y_max
        )

    def to_dict(self):
        return {
            "x_min": self.x_min,
            "x_max": self.x_max,
            "y_min": self.y_min,
            "y_max": self.y_max,
        }

    @classmethod
    def from_dict(cls, dictionary):
        return cls(
            dictionary["x_min"],
            dictionary["x_max"],
            dictionary["y_min"],
            dictionary["y_max"],
        )



@dataclass
class BoundingBox3D(IdlStruct):
    """Class used to store a 3D bounding box.

    Args:
        transform (:py:class:`~STUNT.utils.Transform`): Transform of the
            bounding box (rotation is (0, 0, 0)).
        extent (:py:class:`~STUNT.utils.Vector3D`): The extent of the bounding
            box.

    Attributes:
        transform (:py:class:`~STUNT.utils.Transform`): Transform of the
            bounding box (rotation is (0, 0, 0)).
        extent (:py:class:`~STUNT.utils.Vector3D`): The extent of the bounding
            box.
    """
    transform: Transform
    extent: Vector3D
    corners: Dict[float64, Dict[float64, float64]]

    def __init__(
        self,
        transform: Transform = None,
        extent: Vector3D = None,
        corners=None,
    ):
        self.transform = transform
        self.extent = extent
        self.corners = corners

    @classmethod
    def from_dimensions(cls, bbox_dimensions, location, rotation_y):
        """Creates a 3D bounding box.

        Args:
            bbox_dimensions: The height, width and length of the bbox.
            location: The location of the box in the camera frame.
            rotation: The rotation of the bbox.

        Returns:
            :py:class:`.BoundingBox3D`: A bounding box instance.
        """
        c, s = np.cos(rotation_y), np.sin(rotation_y)
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float32)
        l, w, h = bbox_dimensions[2], bbox_dimensions[1], bbox_dimensions[0]
        x_corners = [
            l / 2,
            l / 2,
            -l / 2,
            -l / 2,
            l / 2,
            l / 2,
            -l / 2,
            -l / 2,
        ]
        y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
        z_corners = [
            w / 2,
            -w / 2,
            -w / 2,
            w / 2,
            w / 2,
            -w / 2,
            -w / 2,
            w / 2,
        ]
        corners = np.array([x_corners, y_corners, z_corners], dtype=np.float32)
        corners_3d = np.dot(R, corners).transpose(1, 0)
        corners_3d = corners_3d + np.array(location, dtype=np.float32).reshape(
            1, 3
        )
        return cls(corners=corners_3d)

    @classmethod
    def from_simulator(cls, bbox):
        """Creates a STUNT bounding box from a simulator bounding box.

        Args:
            bbox: The bounding box to transform.

        Returns:
            :py:class:`.BoundingBox3D`: A bounding box instance.
        """
        transform = Transform(
            Location.from_simulator(bbox.location),
            Rotation(),
        )
        extent = Vector3D.from_simulator(bbox.extent)
        return cls(transform, extent)

    def as_simulator_bounding_box(self):
        """Retrieves the bounding box as instance of a simulator bounding box.

        Returns:
            A instance of a simulator class that represents the bounding box.
        """

        bb_loc = self.transform.location.as_simulator_location()
        bb_extent = self.extent.as_simulator_vector()
        return CarlaBoundingBox(bb_loc, bb_extent)

    def visualize(self, world, actor_transform, time_between_frames=100):
        """Visualizes the bounding box on the world.

        Args:
            world: The simulator world instance to visualize the bounding
                box on.
            actor_transform (:py:class:`~STUNT.utils.Transform`): The current
                transform of the actor that the bounding box is of.
            time_between_frames (:obj:`float`): Time in ms to show the bounding
                box for.
        """
        bb = self.as_simulator_bounding_box()
        bb.location += actor_transform.location()
        world.debug.draw_box(
            bb,
            actor_transform.rotation.as_simulator_rotation(),
            life_time=time_between_frames / 1000.0,
        )

    def to_camera_view(
        self,
        obstacle_transform: Transform,
        extrinsic_matrix,
        intrinsic_matrix,
    ):
        """Converts the coordinates of the bounding box for the given obstacle
        to the coordinates in the view of the camera.

        This method retrieves the extent of the bounding box, transforms them
        to coordinates relative to the bounding box origin, then converts those
        to coordinates relative to the obstacle.

        These coordinates are then considered to be in the world coordinate
        system, which is mapped into the camera view. A negative z-value
        signifies that the bounding box is behind the camera plane.

        Note that this function does not cap the coordinates to be within the
        size of the camera image.

        Args:
            obstacle_transform (:py:class:`~STUNT.utils.Transform`): The
                transform of the obstacle that the bounding box is associated
                with.
            extrinsic_matrix: The extrinsic matrix of the camera.
            intrinsic_matrix: The intrinsic matrix of the camera.

        Returns:
            A list of 8 Location instances specifying the 8 corners of the
            bounding box.
        """
        # Retrieve the eight coordinates of the bounding box with respect to
        # the origin of the bounding box.

        if self.corners is not None:
            pts_2d = np.dot(
                intrinsic_matrix, self.corners.transpose(1, 0)
            ).transpose(1, 0)
            pts_2d = pts_2d[:, :2] / pts_2d[:, 2:]
            camera_coordinates = [Vector2D(pt[0], pt[1]) for pt in pts_2d]
            return camera_coordinates

        extent = self.extent
        bbox = np.array(
            [
                Location(x=+extent.x, y=+extent.y, z=-extent.z),
                Location(x=-extent.x, y=+extent.y, z=-extent.z),
                Location(x=-extent.x, y=-extent.y, z=-extent.z),
                Location(x=+extent.x, y=-extent.y, z=-extent.z),
                Location(x=+extent.x, y=+extent.y, z=+extent.z),
                Location(x=-extent.x, y=+extent.y, z=+extent.z),
                Location(x=-extent.x, y=-extent.y, z=+extent.z),
                Location(x=+extent.x, y=-extent.y, z=+extent.z),
            ]
        )

        # Transform the vertices with respect to the bounding box transform.
        bbox = self.transform.transform_locations(bbox)

        # Convert the bounding box relative to the world.
        bbox = obstacle_transform.transform_locations(bbox)

        # Obstacle's transform is relative to the world. Thus, the bbox
        # contains the 3D bounding box vertices relative to the world.
        camera_coordinates = []
        for vertex in bbox:
            location_2D = vertex.to_camera_view(
                extrinsic_matrix, intrinsic_matrix
            )

            # Add the points to the image.
            camera_coordinates.append(location_2D)

        return camera_coordinates

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "BoundingBox3D(transform: {}, extent: {})".format(
            self.transform, self.extent
        )

    def to_dict(self):
        return {
            "transform": self.transform.to_dict()
            if self.transform is not None
            else None,
            "extent": self.extent.to_dict(),
            "corners": self.corners,
        }

    @classmethod
    def from_dict(cls, dictionary):

        transform = (
            Transform.from_dict(dictionary["transform"])
            if dictionary["transform"] is not None
            else None
        )
        extent = Vector3D.from_dict(dictionary["extent"])

        return cls(transform, extent, dictionary["corners"])


def get_bounding_box_in_camera_view(bb_coordinates, image_width, image_height):
    """Creates the bounding box in the view of the camera image using the
    coordinates generated with respect to the camera transform.

    Args:
        bb_coordinates: 8 :py:class:`~STUNT.utils.Location` coordinates of
            the bounding box relative to the camera transform.
        image_width (:obj:`int`): The width of the image being published by the
            camera.
        image_height (:obj:`int`): The height of the image being published by
            the camera.

    Returns:
        :py:class:`.BoundingBox2D`: a bounding box, or None if the bounding box
            does not fall into the view of the camera.
    """
    # Make sure that atleast 2 of the bounding box coordinates are in front.
    z_vals = [loc.z for loc in bb_coordinates if loc.z >= 0]
    if len(z_vals) < 2:
        return None

    # Create the thresholding line segments of the camera view.

    left = LineString(((0, 0), (0, image_height)))
    bottom = LineString(((0, image_height), (image_width, image_height)))
    right = LineString(((image_width, image_height), (image_width, 0)))
    top = LineString(((image_width, 0), (0, 0)))
    camera_thresholds = [left, bottom, right, top]

    def threshold(p1, p2):
        points = []
        # If the points are themselves within the image, add them to the
        # set of thresholded points.
        if (
            p1[0] >= 0
            and p1[0] < image_width
            and p1[1] >= 0
            and p1[1] < image_height
        ):
            points.append(p1)

        if (
            p2[0] >= 0
            and p2[0] < image_width
            and p2[1] >= 0
            and p2[1] < image_height
        ):
            points.append(p2)

        # Compute the intersection of the line segment formed by p1 -- p2
        # with all the thresholds of the camera image.
        p12 = LineString((p1, p2))
        for camera_threshold in camera_thresholds:
            p = p12.intersection(camera_threshold)
            if not p.is_empty:
                if p.geom_type == "Point":
                    points.append((p.x, p.y))
                elif p.geom_type == "LineString":
                    for coord in p.coords:
                        points.append((coord[0], coord[1]))
        return points

    # Go over each of the segments of the bounding box and threshold it to
    # be inside the image.
    thresholded_points = []
    points = [(int(loc.x), int(loc.y)) for loc in bb_coordinates]
    # Bottom plane thresholded.
    thresholded_points.extend(threshold(points[0], points[1]))
    thresholded_points.extend(threshold(points[1], points[2]))
    thresholded_points.extend(threshold(points[2], points[3]))
    thresholded_points.extend(threshold(points[3], points[0]))

    # Top plane thresholded.
    thresholded_points.extend(threshold(points[4], points[5]))
    thresholded_points.extend(threshold(points[5], points[6]))
    thresholded_points.extend(threshold(points[6], points[7]))
    thresholded_points.extend(threshold(points[7], points[4]))

    # Remaining segments thresholded.
    thresholded_points.extend(threshold(points[0], points[4]))
    thresholded_points.extend(threshold(points[1], points[5]))
    thresholded_points.extend(threshold(points[2], points[6]))
    thresholded_points.extend(threshold(points[3], points[7]))

    if len(thresholded_points) == 0:
        return None
    else:
        x = [int(x) for x, _ in thresholded_points]
        y = [int(y) for _, y in thresholded_points]
        if min(x) < max(x) and min(y) < max(y):
            return BoundingBox2D(min(x), max(x), min(y), max(y))
        else:
            return None
