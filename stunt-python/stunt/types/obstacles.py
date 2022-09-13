import json
import numpy as np
import math


from enum import Enum

from carla import (
    Actor as CarlaActor,
    Vehicle as CarlaVehicle,
    Walker as CarlaWalker,
    TrafficLight as CarlaTrafficLight,
    TrafficLightState as CarlaTrafficLightState,
)


from stunt.types import (
    Transform,
    Vector3D,
    BoundingBox2D,
    BoundingBox3D,
    get_bounding_box_in_camera_view,
)

VEHICLE_LABELS = {"car", "bicycle", "motorcycle", "bus", "truck", "vehicle"}


class SimulatorObstacle(object):
    def __init__(
        self,
        attributes={},
        id=0,
        parent=0,
        is_alive=False,
        semantic_tags=[],
        type_id="",
    ):
        self.attributes = attributes
        self.id = id
        self.parent = parent
        self.is_alive = is_alive
        self.semantic_tags = semantic_tags
        self.type_id = type_id

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"Obstacle(attributes={self.attributes}, id={self.id}, is_alive={self.is_alive}, parent={self.parent}, semantic_tags={self.semantic_tags}, type_id={self.type_id})"

    @classmethod
    def from_simulator(cls, data):
        """Creates a STUNT Obstacle from a simulator Actor.

        Args:
            data: An instance of a simulator Actor.

        Returns:
            :py:class:`.Obstacle`: A STUNT Obstacle.
        """

        if not isinstance(data, CarlaActor):
            raise ValueError("The data must be a Location or carla.Actor")

        return cls(
            data.attributes,
            data.id,
            data.parent,
            data.is_alive,
            # TODO: bump CARLA version, this crashes in 0.9.10
            # data.semantic_tags,
            [],
            data.type_id,
        )

    # TODO: cannot convert back to simulator, yet.
    # def to_simulator(self):
    #     return CarlaActor(
    #         self.fov,
    #         self.height,
    #         self.width,
    #         self.point_cloud.tobytes(),
    #     )

    def to_dict(self):
        return {
            "attributes": self.attributes,
            "id": self.id,
            "parent": self.parent,
            "is_alive": self.is_alive,
            "semantic_tags": self.semantic_tags,
            "type_id": self.type_id,
        }

    @classmethod
    def from_dict(cls, dictionary):

        return cls(
            dictionary["attributes"],
            dictionary["id"],
            dictionary["parent"],
            dictionary["is_alive"],
            dictionary["semantic_tags"],
            dictionary["type_id"],
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)


class Obstacle(object):
    """Class used to store info about obstacles.

    This class provides helper functions to detect obstacles and provide
    bounding boxes for them.

    Args:
        bounding_box (:py:class:`.BoundingBox2D`): The bounding box of the
            obstacle (can be 2D or 3D).
        confidence (:obj:`float`): The confidence of the detection.
        label (:obj:`str`): The label of the obstacle.
        id (:obj:`int`): The identifier of the obstacle.
        transform (:py:class:`~STUNT.utils.Transform`, optional): Transform of
            the obstacle in the world.

    Attributes:
        bounding_box (:py:class:`~STUNT.utisl.BoundingBox2D`): Bounding box of
            the obstacle (can be 2D or 3D).
        confidence (:obj:`float`): The confidence of the detection.
        label (:obj:`str`): The label of the obstacle.
        id (:obj:`int`): The identifier of the obstacle.
        transform (:py:class:`~STUNT.utils.Transform`): Transform of the
            obstacle.
    """

    def __init__(
        self,
        bounding_box,
        confidence: float,
        label: str,
        id: int = -1,
        transform: Transform = None,
        detailed_label: str = "",
        bounding_box_2D: BoundingBox2D = None,
        timestamp=None,
    ):
        self.timestamp = timestamp
        self.bounding_box = bounding_box
        if isinstance(bounding_box, BoundingBox2D):
            self.bounding_box_2D = bounding_box
        else:
            self.bounding_box_2D = bounding_box_2D
        self.confidence = confidence
        self.label = label
        self.id = id
        self.transform = transform
        self.detailed_label = detailed_label
        if label == "vehicle":
            self.segmentation_class = 10
        elif label == "person":
            self.segmentation_class = 4
        else:
            self.segmentation_class = None
        # Thresholds to be used for detection of the obstacle.
        self.__segmentation_threshold = 0.20
        self.__depth_threshold = 5

    @classmethod
    def from_simulator_actor(cls, actor):
        """Creates an Obstacle from a simulator actor.

        Args:
            actor: The actor to initialize the obstacle with.

        Returns:
            :py:class:`.Obstacle`: An obstacle instance.
        """
        if not isinstance(actor, (CarlaVehicle, CarlaWalker)):
            raise ValueError(
                "The actor should be of type Vehicle or "
                "Walker to initialize the Obstacle class."
            )
        # We do not use everywhere from_simulator* methods in order to reduce
        # runtime.
        # Convert the transform provided by the simulation to the STUNT class.
        transform = Transform.from_simulator_transform(actor.get_transform())
        # Convert the bounding box from the simulation to the STUNT one.
        bounding_box = BoundingBox3D.from_simulator_bounding_box(actor.bounding_box)
        if isinstance(actor, CarlaVehicle):
            label = "vehicle"
        else:
            label = "person"
        # Get the simulator actor from type_id (e.g. vehicle.ford.mustang).
        detailed_label = actor.type_id
        # TODO: (Sukrit): Move from vehicles and people to separate classes
        # for bicycles, motorcycles, cars and persons.
        return cls(bounding_box, 1.0, label, actor.id, transform, detailed_label)

    def as_mot16_str(self, timestamp):
        if not self.bounding_box_2D:
            raise ValueError(
                "Obstacle {} does not have 2D bounding box".format(self.id)
            )
        log_line = "{},{},{},{},{},{},{},{},{},{}\n".format(
            timestamp,
            self.id,
            self.bounding_box_2D.x_min,
            self.bounding_box_2D.y_min,
            self.bounding_box_2D.get_width(),
            self.bounding_box_2D.get_height(),
            1.0,
            -1,
            -1,
            -1,
        )
        return log_line

    def _distance(self, other_transform: Transform):
        """Computes the distance from the obstacle to the other transform.

        The distance provides an estimate of the depth returned by the depth
        camera sensor in the simulator. As a result, the distance is defined
        as the displacement of the obstacle along either the X or the Y axis.

        Args:
            other_transform (:py:class:`~STUNT.utils.Transform`): The other
                transform.

        Returns:
            :obj:`float`: The distance (in metres) of the obstacle from the
            transform.
        """

        if self.transform is None:
            raise ValueError("Obstacle {} does not have a transform".format(self.id))
        # Get the location of the vehicle and the obstacle as numpy arrays.
        other_location = other_transform.location.as_numpy_array()
        obstacle_location = self.transform.location.as_numpy_array()

        # Calculate the vector from the vehicle to the obstacle.
        # Scale it by the forward vector, and calculate the norm.
        relative_vector = other_location - obstacle_location
        distance = np.linalg.norm(
            relative_vector * other_transform.forward_vector.as_numpy_array()
        )
        return distance

    def draw_on_frame(
        self,
        frame,
        bbox_color_map,
        ego_transform: Transform = None,
        text: str = None,
    ):
        """Annotate the image with the bounding box of the obstacle."""
        if text is None:
            text = "{}, {:.1f}".format(self.label, self.confidence)
            if self.id != -1:
                text += ", id:{}".format(self.id)
            if ego_transform is not None and self.transform is not None:
                text += ", {:.1f}m".format(
                    ego_transform.location.distance(self.transform.location)
                )
        if self.label in bbox_color_map:
            color = bbox_color_map[self.label]
        else:
            color = [255, 255, 255]
        # Show bounding box.
        if self.bounding_box_2D:
            # Draw the 2D bounding box if available.
            frame.draw_box(
                self.bounding_box_2D.get_min_point(),
                self.bounding_box_2D.get_max_point(),
                color,
            )
            frame.draw_text(self.bounding_box_2D.get_min_point(), text, color)
        elif isinstance(self.bounding_box, BoundingBox3D):
            if self.bounding_box.corners is None:
                raise ValueError(
                    "Obstacle {} does not have bbox corners".format(self.id)
                )
            corners = self.bounding_box.to_camera_view(
                None,
                frame.camera_setup.get_extrinsic_matrix(),
                frame.camera_setup.get_intrinsic_matrix(),
            )
            frame.draw_3d_box(corners, color)
        else:
            raise ValueError("Obstacle {} does not have bounding box".format(self.id))

    def draw_trajectory_on_frame(
        self, trajectory, frame, point_color, draw_label: bool = False
    ):
        # Intrinsic and extrinsic matrix of the top down camera.
        extrinsic_matrix = frame.camera_setup.get_extrinsic_matrix()
        intrinsic_matrix = frame.camera_setup.get_intrinsic_matrix()
        if isinstance(self.bounding_box, BoundingBox3D):
            # Draw bounding boxes.
            start_location = (
                self.bounding_box.transform.location - self.bounding_box.extent
            )
            end_location = (
                self.bounding_box.transform.location + self.bounding_box.extent
            )
            for transform in trajectory:
                [start_transform, end_transform] = transform.transform_locations(
                    [start_location, end_location]
                )
                start_point = start_transform.to_camera_view(
                    extrinsic_matrix, intrinsic_matrix
                )
                end_point = end_transform.to_camera_view(
                    extrinsic_matrix, intrinsic_matrix
                )
                if frame.in_frame(start_point) or frame.in_frame(end_point):
                    frame.draw_box(start_point, end_point, point_color)
        else:
            # Draw points.
            for transform in trajectory:
                screen_point = transform.location.to_camera_view(
                    extrinsic_matrix, intrinsic_matrix
                )
                if frame.in_frame(screen_point):
                    # Draw trajectory on frame.
                    frame.draw_point(screen_point, point_color)
        if draw_label and len(trajectory) > 0:
            text = "{}, {}".format(self.label, self.id)
            screen_point = trajectory[-1].location.to_camera_view(
                extrinsic_matrix, intrinsic_matrix
            )
            frame.draw_text(screen_point, text, point_color)

    def get_bounding_box_corners(self, obstacle_transform, obstacle_radius=None):
        """Gets the corners of the obstacle's bounding box.
        Note:
            The bounding box is applied on the given obstacle transfom, and not
            on the default obstacle transform.
        """
        # Use 3d bounding boxes if available, otherwise use default
        if isinstance(self.bounding_box, BoundingBox3D):
            start_location = (
                self.bounding_box.transform.location - self.bounding_box.extent
            )
            end_location = (
                self.bounding_box.transform.location + self.bounding_box.extent
            )
            [start_location, end_location] = obstacle_transform.transform_locations(
                [start_location, end_location]
            )
        else:
            obstacle_radius_loc = STUNTt.utils.Location(
                obstacle_radius, obstacle_radius
            )
            start_location = obstacle_transform.location - obstacle_radius_loc
            end_location = obstacle_transform.location + obstacle_radius_loc
        return [
            min(start_location.x, end_location.x),
            min(start_location.y, end_location.y),
            max(start_location.x, end_location.x),
            max(start_location.y, end_location.y),
        ]

    def get_in_log_format(self):
        if not self.bounding_box_2D:
            raise ValueError(
                "Obstacle {} does not have 2D bounding box".format(self.id)
            )
        min_point = self.bounding_box_2D.get_min_point()
        max_point = self.bounding_box_2D.get_max_point()
        return (
            self.label,
            self.detailed_label,
            self.id,
            ((min_point.x, min_point.y), (max_point.x, max_point.y)),
        )

    def is_animal(self):
        return self.label in [
            "cat",
            "dog",
            "horse",
            "sheep",
            "cow",
            "elephant",
            "bear",
            "zebra",
            "giraffe",
        ]

    def is_person(self):
        return self.label == "person"

    def is_speed_limit(self):
        return self.label in ["speed limit 30", "speed limit 60", "speed limit 90"]

    def is_stop_sign(self):
        return self.label == "stop sign" or self.label == "stop marking"

    def is_traffic_light(self):
        return self.label in [
            "red traffic light",
            "yellow traffic light",
            "green traffic light",
            "off traffic light",
        ]

    def is_vehicle(self):
        # Might want to include train.
        return self.label in VEHICLE_LABELS

    def populate_bounding_box_2D(self, depth_frame, segmented_frame):
        """Populates the 2D bounding box for the obstacle.

        Heuristically uses the depth frame and segmentation frame to figure out
        if the obstacle is in view of the camera or not.

        Args:
            depth_frame (:py:class:`~STUNT.perception.depth_frame.DepthFrame`):
                Depth frame used to compare the depth to the distance of the
                obstacle from the sensor.
            segmented_frame (:py:class:`~STUNT.perception.segmentation.segmented_frame.SegmentedFrame`):  # noqa: E501
                Segmented frame used to refine the conversions.

        Returns:
            :py:class:`~STUNT.utisl.BoundingBox2D`: An instance representing a
            rectangle over the obstacle if the obstacle is deemed to be
            visible, None otherwise.
        """
        if self.bounding_box_2D:
            return self.bounding_box_2D
        # Convert the bounding box of the obstacle to the camera coordinates.
        bb_coordinates = self.bounding_box.to_camera_view(
            self.transform,
            depth_frame.camera_setup.get_extrinsic_matrix(),
            depth_frame.camera_setup.get_intrinsic_matrix(),
        )

        # Threshold the bounding box to be within the camera view.
        bbox_2d = get_bounding_box_in_camera_view(
            bb_coordinates,
            depth_frame.camera_setup.width,
            depth_frame.camera_setup.height,
        )
        if not bbox_2d:
            return None
        # Crop the segmented and depth image to the given bounding box.
        cropped_image = segmented_frame.as_numpy_array()[
            bbox_2d.y_min : bbox_2d.y_max, bbox_2d.x_min : bbox_2d.x_max
        ]
        cropped_depth = depth_frame.as_numpy_array()[
            bbox_2d.y_min : bbox_2d.y_max, bbox_2d.x_min : bbox_2d.x_max
        ]

        # If the size of the bounding box is greater than 0, ensure that the
        # bounding box contains more than a threshold of pixels corresponding
        # to the required segmentation class.
        if cropped_image.size > 0:
            masked_image = np.zeros_like(cropped_image)
            masked_image[np.where(cropped_image == self.segmentation_class)] = 1
            seg_threshold = self.__segmentation_threshold * masked_image.size
            if np.sum(masked_image) >= seg_threshold:
                # The bounding box contains the required number of pixels that
                # belong to the required class. Ensure that the depth of the
                # obstacle is the depth in the image.
                masked_depth = cropped_depth[np.where(masked_image == 1)]
                mean_depth = np.mean(masked_depth) * 1000
                depth = self._distance(depth_frame.camera_setup.get_transform())
                if abs(depth - mean_depth) <= self.__depth_threshold:
                    self.bounding_box_2D = bbox_2d
                    return bbox_2d
        return None

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        obstacle = "Obstacle(id: {}, label: {}, confidence: {}, " "bbox: {})".format(
            self.id, self.label, self.confidence, self.bounding_box
        )
        if self.transform:
            return obstacle + " at " + str(self.transform)
        else:
            return obstacle

    def to_dict(self):
        return {
            "bounding_box": self.bounding_box.to_dict(),
            "confidence": self.confidence,
            "label": self.label,
            "id": self.id,
            "transform": self.transform.to_dict(),
            "detailed_label": self.detailed_label,
            "bounding_box_2D": self.bounding_box_2D.to_dict()
            if self.bounding_box_2D is not None
            else None,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, dictionary):

        bounding_box = BoundingBox3D.from_dict(dictionary["bounding_box"])
        transform = Transform.from_dict(dictionary["transform"])
        bounding_box_2D = (
            BoundingBox2D.from_dict(dictionary["bounding_box_2D"])
            if dictionary["bounding_box_2D"] is not None
            else None
        )

        return cls(
            bounding_box,
            dictionary["confidence"],
            dictionary["label"],
            dictionary["id"],
            transform,
            dictionary["detailed_label"],
            bounding_box_2D,
            dictionary["timestamp"],
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)


class ObstacleTrajectory(object):
    """Used to store the trajectory of an obstacle.

    Args:
        obstacle (:py:class:`~STUNT.perception.detection.obstacle.Obstacle`):
            The obstacle for which the trajectory is computed.
        trajectory (list(:py:class:`~STUNT.utils.Transform`)): List of past
            transforms.
    """

    def __init__(self, obstacle: Obstacle, trajectory):
        self.obstacle = obstacle
        self.trajectory = trajectory

    def draw_on_frame(self, frame, bbox_color_map, ego_transform: Transform = None):
        """Draws the tracked obstacle as a 2D bounding box."""
        self.obstacle.draw_on_frame(frame, bbox_color_map, ego_transform)

    def draw_trajectory_on_frame(self, frame, draw_label: bool = False):
        """Draws the trajectory on a bird's eye view frame."""
        if self.obstacle.is_person():
            color = [255, 0, 0]
        elif self.obstacle.is_vehicle():
            color = [128, 128, 0]
        else:
            color = [255, 255, 0]
        self.obstacle.draw_trajectory_on_frame(
            self.trajectory, frame, color, draw_label
        )

    def estimate_obstacle_orientation(self):
        """Uses the obstacle's past trajectory to estimate its angle from the
        positive x-axis (assumes trajectory points are in the ego-vehicle's
        coordinate frame)."""
        other_idx = len(self.trajectory) - 2
        # TODO:: Setting a default yaw is dangerous. Find some way to estimate
        # the orientation of a stationary object (e.g. 3D object detection).
        yaw = 0.0  # Default orientation for stationary objects.
        current_loc = self.trajectory[-1].location.as_vector_2D()
        while other_idx >= 0:
            past_ref_loc = self.trajectory[other_idx].location.as_vector_2D()
            vec = current_loc - past_ref_loc
            displacement = current_loc.l2_distance(past_ref_loc)
            if displacement > 0.001:
                # Angle between displacement vector and the x-axis, i.e.
                # the (1,0) vector.
                yaw = vec.get_angle(Vector2D(1, 0))
                break
            else:
                other_idx -= 1
        return math.degrees(yaw)

    def get_last_n_transforms(self, n: int):
        """Returns the last n steps of the trajectory. If we have not seen
        enough past locations of the obstacle, pad the trajectory with the
        appropriate number of copies of the earliest location."""
        num_past_locations = len(self.trajectory)
        if num_past_locations < n:
            initial_copies = [self.trajectory[0]] * (n - num_past_locations)
            last_n_steps = initial_copies + self.trajectory
        else:
            last_n_steps = self.trajectory[-n:]
        return last_n_steps

    def to_world_coordinates(self, ego_transform: Transform):
        """Transforms the trajectory into world coordinates."""
        cur_trajectory = []
        for past_transform in self.trajectory:
            cur_trajectory.append(ego_transform * past_transform)
        self.trajectory = cur_trajectory

    @property
    def id(self):
        return self.obstacle.id

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Obstacle {}, trajectory {}".format(self.obstacle, self.trajectory)

    def to_dict(self):
        trajectory = []
        for t in self.trajectory:
            trajectory.append(t.to_dict())
        return {
            "obstacle": self.obstacle.to_dict(),
            "trajectory": trajectory,
        }

    @classmethod
    def from_dict(cls, dictionary):
        obstacle = Obstacle.from_dict(dictionary["obstacle"])
        trajectory = []

        for t in dictionary["trajectory"]:
            trajectory.append(Transform.from_dict(t))

        return cls(obstacle, trajectory)

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)


class TrafficLightColor(Enum):
    """Enum to represent the states of a traffic light."""

    RED = 1
    YELLOW = 2
    GREEN = 3
    OFF = 4

    def get_label(self):
        """Gets the label of a traffic light color.

        Returns:
            :obj:`str`: The label string.
        """
        if self.value == 1:
            return "red traffic light"
        elif self.value == 2:
            return "yellow traffic light"
        elif self.value == 3:
            return "green traffic light"
        else:
            return "off traffic light"

    def get_color(self):
        if self.value == 1:
            return [255, 0, 0]
        elif self.value == 2:
            return [255, 165, 0]
        elif self.value == 3:
            return [0, 255, 0]
        else:
            return [0, 0, 0]

    def serialize(self):
        return self.value

    @classmethod
    def deserialize(cls, serialized):
        return cls(serialized)


class TrafficLight(Obstacle):
    """Class used to store info about traffic lights.

    Args:
        confidence (:obj:`float`): The confidence of the detection.
        state (:py:class:`.TrafficLightColor`): The state of the traffic light.
        id (:obj:`int`, optional): The identifier of the traffic light.
        transform (:py:class:`~Transform`, optional): Transform of
            the traffic light.
        trigger_volume_extent (:py:class:`Vector3D`, optional): The
            extent of the trigger volume of the light.
        bounding_box (:py:class:`.BoundingBox2D`, optional): The bounding box
            of the traffic light in camera view.

    Attributes:
        confidence (:obj:`float`): The confidence of the detection.
        state (:py:class:`.TrafficLightColor`): The state of the traffic light.
        id (:obj:`int`): The identifier of the traffic light.
        transform (:py:class:`~Transform`): Transform of the
            traffic light.
        trigger_volume_extent (:py:class:`Vector3D`): The extent
            of the trigger volume of the light.
        bounding_box (:py:class:`.BoundingBox2D`, optional): The bounding box
            of the traffic light in camera view.
    """

    def __init__(
        self,
        confidence: float,
        state: TrafficLightColor,
        id: int = -1,
        transform: Transform = None,
        trigger_volume_extent: Vector3D = None,
        bounding_box: BoundingBox2D = None,
        timestamp=None,
    ):
        super(TrafficLight, self).__init__(
            bounding_box, confidence, state.get_label(), id, transform
        )
        self.timestamp = timestamp
        self.state = state
        self.trigger_volume_extent = trigger_volume_extent

    @classmethod
    def from_simulator_actor(cls, traffic_light):
        """Creates a TrafficLight from a simulator traffic light actor.

        Args:
            traffic_light: A simulator traffic light actor.

        Returns:
            :py:class:`.TrafficLight`: A traffic light.
        """
        from carla import TrafficLight, TrafficLightState

        if not isinstance(traffic_light, TrafficLight):
            raise ValueError("The traffic light must be a TrafficLight")
        # Retrieve the Transform of the TrafficLight.
        transform = Transform.from_simulator_transform(traffic_light.get_transform())
        # Retrieve the Trigger Volume of the TrafficLight.
        trigger_volume_extent = Vector3D(
            traffic_light.trigger_volume.extent.x,
            traffic_light.trigger_volume.extent.y,
            traffic_light.trigger_volume.extent.z,
        )
        traffic_light_state = traffic_light.get_state()
        state = TrafficLightColor.OFF
        if traffic_light_state == TrafficLightState.Red:
            state = TrafficLightColor.RED
        elif traffic_light_state == TrafficLightState.Yellow:
            state = TrafficLightColor.YELLOW
        elif traffic_light_state == TrafficLightState.Green:
            state = TrafficLightColor.GREEN
        return cls(1.0, state, traffic_light.id, transform, trigger_volume_extent)

    def draw_on_bird_eye_frame(self, frame):
        # Intrinsic and extrinsic matrix of the top down camera.
        extrinsic_matrix = frame.camera_setup.get_extrinsic_matrix()
        intrinsic_matrix = frame.camera_setup.get_intrinsic_matrix()
        point = self.transform.location.to_camera_view(
            extrinsic_matrix, intrinsic_matrix
        )
        frame.draw_point(point, self.state.get_color(), r=10)
        frame.draw_text(point, self.state.get_label(), self.state.get_color())

    def is_traffic_light_visible(
        self,
        camera_transform: Transform,
        town_name: str = None,
        distance_threshold: int = 70,
    ):
        """Checks if the traffic light is visible from the camera transform.

        Args:
            transform (:py:class:`~Transform`): Transform of the
                camera in the world frame of reference.
            distance_threshold (:obj:`int`): Maximum distance to the camera
                (in m).

        Returns:
            bool: True if the traffic light is visible from the camera
            transform.
        """
        # We dot product the forward vectors (i.e., orientation).
        # Note: we have to rotate the traffic light forward vector
        # so that it's pointing out from the traffic light in the
        # opposite direction in which the ligth is beamed.
        prod = np.dot(
            [
                self.transform.forward_vector.y,
                -self.transform.forward_vector.x,
                self.transform.forward_vector.z,
            ],
            [
                camera_transform.forward_vector.x,
                camera_transform.forward_vector.y,
                camera_transform.forward_vector.z,
            ],
        )
        if (
            self.transform.location.distance(camera_transform.location)
            > distance_threshold
        ):
            return prod > 0.4

        if town_name is None:
            return prod > -0.80
        else:
            if town_name == "Town01" or town_name == "Town02":
                return prod > 0.3
        return prod > -0.80

    def get_all_detected_traffic_light_boxes(
        self, town_name: str, depth_frame, segmented_image
    ):
        """Returns traffic lights for all boxes of a simulator traffic light.

        Note:
            All the traffic lights returned will have the same id and
            transform.

        Args:
            town_name (:obj:`str`): Name of the town in which the traffic light
                is.
            depth_frame (:py:class:`~STUNTt.perception.depth_frame.DepthFrame`):
                 Depth frame.
            segmented_image: A segmented image np array used to refine the
                 bounding boxes.

        Returns:
            list(:py:class:`~STUNTt.perception.detection.traffic_light.TrafficLight`):
            Detected traffic lights, one for each traffic light box.
        """
        traffic_lights = []
        bboxes = self._get_bboxes(town_name)
        # Convert the returned bounding boxes to 2D and check if the
        # light is occluded. If not, add it to the traffic lights list.
        for bbox in bboxes:
            bounding_box = [
                loc.to_camera_view(
                    depth_frame.camera_setup.get_extrinsic_matrix(),
                    depth_frame.camera_setup.get_intrinsic_matrix(),
                )
                for loc in bbox
            ]
            bbox_2d = get_bounding_box_in_camera_view(
                bounding_box,
                depth_frame.camera_setup.width,
                depth_frame.camera_setup.height,
            )
            if not bbox_2d:
                continue

            # Crop the segmented and depth image to the given bounding box.
            cropped_image = segmented_image[
                bbox_2d.y_min : bbox_2d.y_max, bbox_2d.x_min : bbox_2d.x_max
            ]
            cropped_depth = depth_frame.frame[
                bbox_2d.y_min : bbox_2d.y_max, bbox_2d.x_min : bbox_2d.x_max
            ]

            if cropped_image.size > 0:
                masked_image = np.zeros_like(cropped_image)
                masked_image[
                    np.where(np.logical_or(cropped_image == 12, cropped_image == 18))
                ] = 1
                if np.sum(masked_image) >= 0.20 * masked_image.size:
                    masked_depth = cropped_depth[np.where(masked_image == 1)]
                    mean_depth = np.mean(masked_depth) * 1000
                    if abs(mean_depth - bounding_box[0].z) <= 2 and mean_depth < 150:
                        traffic_lights.append(
                            TrafficLight(
                                1.0,
                                self.state,
                                self.id,
                                self.transform,
                                self.trigger_volume_extent,
                                bbox_2d,
                            )
                        )
        return traffic_lights

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return (
            "TrafficLight(confidence: {}, state: {}, id: {}, "
            "transform: {}, trigger_volume_extent: {}, bbox: {})".format(
                self.confidence,
                self.state,
                self.id,
                self.transform,
                self.trigger_volume_extent,
                self.bounding_box,
            )
        )

    def _relative_to_traffic_light(self, points):
        """Transforms the bounding box specified in the points relative to the
        light.

        Args:
            points: An array of length 4 representing the 4 points of the
                rectangle.
        """

        def rotate(yaw, location):
            """Rotate a given 3D vector around the Z-axis."""
            rotation_matrix = np.identity(3)
            rotation_matrix[0, 0] = np.cos(yaw)
            rotation_matrix[0, 1] = -np.sin(yaw)
            rotation_matrix[1, 0] = np.sin(yaw)
            rotation_matrix[1, 1] = np.cos(yaw)
            location_vector = np.array([[location.x], [location.y], [location.z]])
            transformed = np.dot(rotation_matrix, location_vector)
            return STUNTt.utils.Location(
                x=transformed[0, 0], y=transformed[1, 0], z=transformed[2, 0]
            )

        transformed_points = [
            rotate(np.radians(self.transform.rotation.yaw), point) for point in points
        ]
        base_relative_points = [
            self.transform.location + point for point in transformed_points
        ]
        return base_relative_points
