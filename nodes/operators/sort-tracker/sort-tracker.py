from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from zenoh_flow.types import Context
from typing import Dict, Any
import asyncio

import json
import numpy as np


from stunt.types import (
    Obstacle,
    TimeToDecision,
    Image,
    BoundingBox2D,
)

from sort.sort import Sort

DEFAULT_MIN_MATCHING_IOU = 0.2
DEFAULT_EVAL_MIN_MATCHING_IOU = 0.5
DEFAULT_OBSTACLE_TRACK_MAX_AGE = 5
DEFAULT_IGNORE_OBSTACLES_SHORT_HISTORY = 1
DEFAULT_TRACK_NTH_DETECTION = 1
DEFAULT_MIN_HITS = 1


class SortTracker(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ):
        configuration = configuration if configuration is not None else {}

        self.pending = []

        self.image_input = inputs.get("Image", None)
        self.obstacles_input = inputs.get("Obstacles", None)
        self.ttd_input = inputs.get("TTD", None)
        self.output = outputs.get("ObstacleTrajectories", None)

        self.min_matching_iou = configuration.get(
            "min_matching_iou", DEFAULT_MIN_MATCHING_IOU
        )
        self.eval_min_matching_iou = configuration.get(
            "eval_min_matching_iou", DEFAULT_EVAL_MIN_MATCHING_IOU
        )
        self.obstacle_track_max_age = configuration.get(
            "obstacle_track_max_age", DEFAULT_OBSTACLE_TRACK_MAX_AGE
        )
        self.ignore_obstacles_with_short_history = configuration.get(
            "ignore_obstacles_with_short_history",
            DEFAULT_IGNORE_OBSTACLES_SHORT_HISTORY,
        )
        self.track_every_nth_detection = configuration.get(
            "track_every_nth_detection", DEFAULT_TRACK_NTH_DETECTION
        )

        self.min_hits = configuration.get("min_hits", DEFAULT_MIN_HITS)

        self.tracker = Sort(
            max_age=self.obstacle_track_max_age,
            min_hits=1,
            min_iou=self.min_matching_iou,
        )

        self.detection_update_count = -1
        self.frame = None
        self.ttd = None
        self.obstacles = []

    def reinitialize(self, frame, obstacles):
        """Reinitializes a multiple obstacle tracker.

        Args:
            frame (:py:class:`~STUNT.perception.camera_frame.CameraFrame`):
                Frame to reinitialize with.
            obstacles : List of perception.detection.obstacle.Obstacle.
        """
        detections, labels, ids = self.convert_detections_for_sort_alg(
            obstacles
        )
        self.tracker.update(detections, labels, ids)

    def track(self, frame):
        """Tracks obstacles in a frame.

        Args:
            frame (:py:class:`~STUNT.perception.camera_frame.CameraFrame`):
                Frame to track in.
        """
        # each track in tracks has format ([xmin, ymin, xmax, ymax], id)
        obstacles = []
        for track in self.tracker.trackers:
            coords = track.predict()[0].tolist()
            # changing to xmin, xmax, ymin, ymax format
            xmin = int(coords[0])
            xmax = int(coords[2])
            ymin = int(coords[1])
            ymax = int(coords[3])
            if xmin < xmax and ymin < ymax:
                bbox = BoundingBox2D(xmin, xmax, ymin, ymax)
                obstacles.append(Obstacle(bbox, 0, track.label, track.id))
            else:
                pass
        return True, obstacles

    def convert_detections_for_sort_alg(self, obstacles):
        converted_detections = []
        labels = []
        ids = []
        for obstacle in obstacles:
            bbox = [
                obstacle.bounding_box_2D.x_min,
                obstacle.bounding_box_2D.y_min,
                obstacle.bounding_box_2D.x_max,
                obstacle.bounding_box_2D.y_max,
                obstacle.confidence,
            ]
            converted_detections.append(bbox)
            labels.append(obstacle.label)
            ids.append(obstacle.id)
        return (np.array(converted_detections), labels, ids)

    async def wait_image(self):
        data_msg = await self.image_input.recv()
        return ("Image", data_msg)

    async def wait_obstacles(self):
        data_msg = await self.obstacles_input.recv()
        return ("Obstacles", data_msg)

    async def wait_ttd(self):
        data_msg = await self.ttd_input.recv()
        return ("TTD", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "Image" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_image(), name="Image")
            )

        if not any(t.get_name() == "TTD" for t in task_list):
            task_list.append(asyncio.create_task(self.wait_ttd(), name="TTD"))

        if not any(t.get_name() == "Obstacles" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_obstacles(), name="Obstacles")
            )
        return task_list

    async def iteration(self):

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )

        self.pending = list(pending)

        (who, data_msg) = done.pop().result()

        if who == "TTD":
            self.ttd = TimeToDecision.deserialize(data_msg.data)
        elif who == "Image":
            self.frame = Image.deserialize(data_msg.data)
        elif who == "Obstacles":
            obstacles = json.loads(data_msg.data.decode("utf-8"))
            self.obstacles = []
            for o in obstacles:
                self.obstacles.append(Obstacle.from_dict(o))

            self.detection_update_count += 1
            if (
                self.detection_update_count % self.track_every_nth_detection
                == 0
            ):
                detected_obstacles = []
                for obstacle in self.obstacles:
                    if obstacle.is_vehicle() or obstacle.is_person():
                        detected_obstacles.append(obstacle)

                self.reinitialize(self.frame, detected_obstacles)

                _, tracked_obstacles = self.track(self.frame)

                result = []
                for to in tracked_obstacles:
                    result.append(to.to_dict())

                await self.output.send(json.dumps(result).encode("utf-8"))

        return None

    def finalize(self) -> None:
        return None


def register():
    return SortTracker
