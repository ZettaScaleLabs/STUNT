import csv
import numpy as np
import itertools
from collections import deque
import json

from stunt.types import Transform, Location, Rotation, RoadOption

DEFAULT_VIS_TIME = 0.1


class Waypoints(object):
    """Stores waypoints and provides methods to wrangle them."""

    def __init__(self, waypoints, target_speeds=None, road_options=None):
        self.waypoints = waypoints
        if target_speeds is None:
            # Set target speed to 0 if it is not specified.
            self.target_speeds = deque([0 for _ in range(len(waypoints))])
        else:
            self.target_speeds = target_speeds
        self.road_options = road_options

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Waypoints(waypoints: {}, target_speeds: {}, road_options: {})".format(
            self.waypoints, self.target_speeds, self.road_options
        )

    def to_dict(self):

        waypoints = []
        for t in list(self.waypoints):
            waypoints.append(t.to_dict())

        road_options = []
        for ro in list(self.road_options):
            road_options.append(ro.serialize())

        return {
            "waypoints": waypoints,
            "target_speeds": list(self.target_speeds),
            "road_options": road_options,
        }

    @classmethod
    def from_dict(cls, dictionary):

        waypoints = []
        for t in dictionary.get("waypoints", []):
            waypoints.append(Transform.from_dict(t))

        target_speeds = dictionary.get("target_speeds")
        if target_speeds is not None:
            target_speeds = deque(target_speeds)

        road_options = []
        for ro in dictionary.get("road_options", []):
            road_options.append(RoadOption.deserialize(ro))

        return cls(
            deque(waypoints),
            target_speeds,
            deque(road_options),
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)

    @classmethod
    def read_from_csv_file(cls, csv_file_name: str, target_speed):
        """Reads waypoints from a csv file."""
        csv_file = open(csv_file_name)
        csv_reader = csv.reader(csv_file)
        waypoints = []
        for row in csv_reader:
            x = float(row[0])
            y = float(row[1])
            z = float(row[2])
            waypoint = Transform(Location(x, y, z), Rotation(0, 0, 0))
            waypoints.append(waypoint)
        target_speeds = deque([target_speed for _ in range(len(waypoints))])
        return cls(deque(waypoints), target_speeds)

    def apply_speed_factor(self, speed_factor: float):
        if self.target_speeds:
            self.target_speeds = [
                speed_factor * ts for ts in self.target_speeds
            ]

    def as_numpy_array_2D(self):
        """Returns the waypoints as a numpy array of lists of x and y."""
        wx = []
        wy = []
        for wp in self.waypoints:
            wx.append(wp.location.x)
            wy.append(wp.location.y)
        return np.array([wx, wy])

    def closest_waypoint(self, location: Location):
        """Finds the closest waypoint to the location."""
        min_dist = np.infty
        min_index = 0
        for index, waypoint in enumerate(self.waypoints):
            dist = waypoint.location.distance(location)
            if dist < min_dist:
                min_dist = dist
                min_index = index
        return min_index

    def is_empty(self) -> bool:
        return len(self.waypoints) == 0

    def remove_waypoint_if_close(
        self, location: Location, distance: float = 5
    ) -> bool:
        """Removes the first waypoint if it is less than distance m away."""
        if self.waypoints is None or len(self.waypoints) == 0:
            return False
        if location.distance(self.waypoints[0].location) < distance:
            self.waypoints.popleft()
            if self.target_speeds:
                self.target_speeds.popleft()
            if self.road_options:
                self.road_options.popleft()
            return True
        return False

    def remove_completed(
        self,
        location: Location,
        ego_transform: Transform = None,
    ):
        """Removes waypoints that the ego vehicle has already completed.

        The method first finds the closest waypoint to the location,
        and then removes all waypoints that are before the closest waypoint.

        Args:
            location (:py:class:`STUNT.utils.Location`): The location
                of the ego-vehicle.
        """
        min_index = self.closest_waypoint(location)
        # Remove waypoints that are before the closest waypoint. The ego
        # vehicle already completed them.
        while min_index > 0:
            self.waypoints.popleft()
            if self.target_speeds:
                self.target_speeds.popleft()
            if self.road_options:
                self.road_options.popleft()
            min_index -= 1

        if self.waypoints is None or len(self.waypoints) == 0:
            if ego_transform is not None:
                self.waypoints = deque([ego_transform])
                self.target_speeds = deque([0])
                self.road_options = deque([RoadOption.LANE_FOLLOW])
            else:
                raise ValueError("No more waypoints to follow")

    def recompute_waypoints(self, hd_map, ego_location, goal_location):
        self.waypoints = hd_map.compute_waypoints(ego_location, goal_location)
        self.target_speeds = deque([0 for _ in range(len(self.waypoints))])

    def _get_index(self, transform: Transform, min_distance: float) -> int:
        min_index = -1
        for index, wp in enumerate(self.waypoints):
            distance = wp.location.distance(transform.location)
            if distance >= min_distance:
                min_index = index
                break
        if min_index == -1:
            min_index = len(self.waypoints) - 1
        if min_index < 0:
            raise ValueError("No more waypoints")
        return min_index

    def get_angle(self, transform: Transform, min_distance: float) -> float:
        """Returns the angle between the transform and the first waypoint that
        is at least min_distance away."""
        wp_index = self._get_index(transform, min_distance)
        angle, _ = transform.get_angle_and_magnitude(
            self.waypoints[wp_index].location
        )
        return angle

    def get_vector(self, transform: Transform, min_distance: float):
        """Returns the vector between the transform and the first waypoint that
        is at least min_distance away."""
        wp_index = self._get_index(transform, min_distance)
        return (
            self.waypoints[wp_index].location.as_vector_2D()
            - transform.location.as_vector_2D()
        )

    def get_target_speed(
        self, transform: Transform, min_distance: float
    ) -> float:
        """Gets the target speed at the first waypoint that is at least
        min_distance away."""
        wp_index = self._get_index(transform, min_distance)
        return self.target_speeds[wp_index]

    def slice_waypoints(
        self, start_index: int, end_index: int, target_speed: float = None
    ):
        head_wps = deque(
            itertools.islice(self.waypoints, start_index, end_index)
        )
        if target_speed is not None:
            # Use the specified target speed.
            head_target_speeds = deque(
                [target_speed for _ in range(len(head_wps))]
            )
        else:
            # Otherwise use the already existing target speeds.
            head_target_speeds = deque(
                itertools.islice(self.target_speeds, start_index, end_index)
            )
        if self.road_options is not None:
            head_road_options = deque(
                itertools.islice(self.road_options, start_index, end_index)
            )
        else:
            head_road_options = None
        return Waypoints(head_wps, head_target_speeds, head_road_options)

    def draw_on_frame(self, bgr_frame, inverse_transform=None):
        """Draw waypoints on a frame.

        Args:
            bgr_frame: Frame on which to draw the waypoints.
            inverse_transform (optional): To be used to transform the waypoints
                to relative to the ego vehicle.
        """
        extrinsic_matrix = bgr_frame.camera_setup.get_extrinsic_matrix()
        intrinsic_matrix = bgr_frame.camera_setup.get_intrinsic_matrix()
        for index, wp in enumerate(self.waypoints):
            if inverse_transform:
                wp = inverse_transform * wp
            pixel_location = wp.location.to_camera_view(
                extrinsic_matrix, intrinsic_matrix
            )
            bgr_frame.draw_point(pixel_location, [255, 255, 255])
            waypoint_txt = ""
            if self.target_speeds:
                waypoint_txt = "{:.1f}m/s".format(self.target_speeds[index])
            # if self.road_options:
            #     waypoint_txt = '{} {}'.format(waypoint_txt,
            #                                   self.road_options[index])
            if waypoint_txt != "":
                bgr_frame.draw_text(
                    pixel_location, waypoint_txt, [255, 255, 255]
                )

    def draw_on_world(self, world):
        """Draw waypoints on the simulator world."""
        for index, wp in enumerate(self.waypoints):
            # Adds 0.5 to z to ensure that the point is above the road surface.
            loc = (wp.location + Location(0, 0, 0.5)).as_simulator_location()
            world.debug.draw_point(loc, size=0.1, life_time=DEFAULT_VIS_TIME)
            # if self.road_options and index < len(self.road_options):
            #     world.debug.draw_string(loc,
            #                             str(self.road_options[index]),
            #                             life_time=DEFAULT_VIS_TIME)
