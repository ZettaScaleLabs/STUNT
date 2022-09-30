from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from zenoh_flow.types import Context
from typing import Dict, Any, Callable
import time
import asyncio

import json
import carla
import array
import numpy as np
import math
import tensorflow as tf

from stunt.types import (
    Obstacle,
    BoundingBox2D,
    TimeToDecision,
    Image,
)

DEFAULT_MODEL_PATH = "/tmp/model"
DEFAULT_MODEL_NAME = "model-name"
DEFAULT_MIN_SCORE_THRESHOLD = 0.7
DEFAULT_GPU_MEMORY_FRACTION = 0.3
DEFAULT_GPU = 0
DEFAULT_COCO_LABELS_PATH = "/tmp/cocolables"
DEFAULT_DYNAMIC_OBSTACLE_DISTANCE_THRESHOLD = 30.0
DEFAULT_STATIC_OBSTACLE_DISTANCE_THRESHOLD = 70.0


class ObstacleDetection(Operator):
    def __init__(self, context, configuration, inputs, outputs):

        configuration = {} if configuration is None else configuration

        self.image_input = inputs.get("Image", None)
        self.ttd_input = inputs.get("TTD", None)
        self.output = outputs.get("Obstacles", None)

        self.pending = []

        self.ttd_data = None
        self.frame = None

        self.model_path = configuration.get(
            "obstacle_detection_model_path", DEFAULT_MODEL_PATH
        )

        self.model_name = configuration.get(
            "obstacle_detection_model_name", DEFAULT_MODEL_NAME
        )

        self.min_threshold = configuration.get(
            "obstacle_detection_min_score_threshold",
            DEFAULT_MIN_SCORE_THRESHOLD,
        )
        self.gpu_memory_fraction = configuration.get(
            "obstacle_detection_gpu_memory_fraction",
            DEFAULT_GPU_MEMORY_FRACTION,
        )
        self.gpu_index = configuration.get(
            "obstacle_detection_gpu_index", DEFAULT_GPU
        )

        self.labels_path = configuration.get(
            "path_coco_labels", DEFAULT_COCO_LABELS_PATH
        )

        self.dynamic_obstacle_distance_threshold = configuration.get(
            "dynamic_obstacle_distance_threshold",
            DEFAULT_DYNAMIC_OBSTACLE_DISTANCE_THRESHOLD,
        )

        self.static_obstacle_distance_threshold = configuration.get(
            "static_obstacle_distance_threshold",
            DEFAULT_STATIC_OBSTACLE_DISTANCE_THRESHOLD,
        )

        self.coco_labels = self.load_coco_labels(self.labels_path)
        self.bbox_colors = self.load_coco_bbox_colors(self.coco_labels)

        # configure tf
        # Only sets memory growth for flagged GPU
        physical_devices = tf.config.experimental.list_physical_devices("GPU")
        tf.config.experimental.set_visible_devices(
            [physical_devices[self.gpu_index]], "GPU"
        )
        tf.config.experimental.set_memory_growth(
            physical_devices[self.gpu_index], True
        )

        # Unique bounding box id. Incremented for each bounding box.
        self.unique_id = 0

        # loads the model
        self.model = tf.saved_model.load(self.model_path)

        # Serve some junk image this forces tensorflow to load the model
        self.run_model(np.zeros((1080, 1920, 3), dtype="uint8"))

    def load_coco_labels(self, labels_path):
        """Returns a map from index to label.

        Args:
            labels_path (:obj:`str`): Path to a file storing a label on each line.
        """
        labels_map = {}
        with open(labels_path) as labels_file:
            labels = labels_file.read().splitlines()
            index = 1
            for label in labels:
                labels_map[index] = label
                index += 1
        return labels_map

    def load_coco_bbox_colors(self, coco_labels):
        """Returns a map from label to color."""
        # Transform to RGB values.
        bbox_color_list = coco_bbox_color_list.reshape((-1, 3)) * 255
        # Transform to ints
        bbox_colors = [
            (bbox_color_list[_]).astype(np.uint8)
            for _ in range(len(bbox_color_list))
        ]
        bbox_colors = np.array(bbox_colors, dtype=np.uint8).reshape(
            len(bbox_colors), 1, 1, 3
        )

        colors = {}
        for category, label in coco_labels.items():
            colors[label] = bbox_colors[category - 1][0][0].tolist()
        return colors

    def run_model(self, image_np):
        # Expand dimensions since the model expects images to have
        # shape: [1, None, None, 3]

        image_np_expanded = np.expand_dims(image_np, axis=0)
        infer = self.model.signatures["serving_default"]
        result = infer(tf.convert_to_tensor(value=image_np_expanded))

        boxes = result["boxes"].numpy()
        scores = result["scores"].numpy()
        classes = result["classes"].numpy()
        num_detections = result["detections"].numpy()

        num_detections = int(num_detections[0])
        res_classes = classes[0][:num_detections]
        res_boxes = boxes[0][:num_detections]
        res_scores = scores[0][:num_detections]

        return num_detections, res_boxes, res_scores, res_classes

    async def wait_image(self):
        data_msg = await self.image_input.recv()
        return ("Image", data_msg)

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
        return task_list

    async def iteration(self):

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )

        self.pending = list(pending)

        for d in done:
            (who, data_msg) = d.result()

            # We compute on IMU
            if who == "TTD":
                self.ttd_data = TimeToDecision.deserialize(data_msg.data)

            elif who == "Image":
                self.frame = Image.deserialize(data_msg.data)

                (
                    num_detections,
                    res_boxes,
                    res_scores,
                    res_classes,
                ) = self.run_model(self.frame.as_rgb_numpy_array())
                obstacles = []
                for i in range(0, num_detections):
                    if res_scores[i] >= self.min_threshold:
                        if res_classes[i] in self.coco_labels:
                            if (
                                self.coco_labels[res_classes[i]]
                                in OBSTACLE_LABELS
                            ):
                                obstacles.append(
                                    Obstacle(
                                        BoundingBox2D(
                                            int(
                                                res_boxes[i][1]
                                                * self.frame.width
                                            ),
                                            int(
                                                res_boxes[i][3]
                                                * self.frame.width
                                            ),
                                            int(
                                                res_boxes[i][0]
                                                * self.frame.height
                                            ),
                                            int(
                                                res_boxes[i][2]
                                                * self.frame.height
                                            ),
                                        ),
                                        float(res_scores[i]),
                                        self.coco_labels[res_classes[i]],
                                        id=self.unique_id,
                                        timestamp=self.frame.timestamp,
                                    ).to_dict()
                                )

                                self.unique_id += 1
                            else:
                                pass
                        else:
                            pass

                await self.output.send(json.dumps(obstacles).encode("utf-8"))

        return None

    def finalize(self) -> None:
        return None


def register():
    return ObstacleDetection


OBSTACLE_LABELS = {
    "car",
    "bicycle",
    "motorcycle",
    "bus",
    "truck",
    "vehicle",
    "person",
    "stop sign",
    "parking meter",
    "cat",
    "dog",
    "speed limit 30",
    "speed limit 60",
    "speed limit 90",
}

coco_bbox_color_list = np.array(
    [
        1.000,
        1.000,
        1.000,
        0.850,
        0.325,
        0.098,
        0.929,
        0.694,
        0.125,
        0.494,
        0.184,
        0.556,
        0.466,
        0.674,
        0.188,
        0.301,
        0.745,
        0.933,
        0.635,
        0.078,
        0.184,
        0.300,
        0.300,
        0.300,
        0.600,
        0.600,
        0.600,
        1.000,
        0.000,
        0.000,
        1.000,
        0.500,
        0.000,
        0.749,
        0.749,
        0.000,
        0.000,
        1.000,
        0.000,
        0.000,
        0.000,
        1.000,
        0.667,
        0.000,
        1.000,
        0.333,
        0.333,
        0.000,
        0.333,
        0.667,
        0.000,
        0.333,
        1.000,
        0.000,
        0.667,
        0.333,
        0.000,
        0.667,
        0.667,
        0.000,
        0.667,
        1.000,
        0.000,
        1.000,
        0.333,
        0.000,
        1.000,
        0.667,
        0.000,
        1.000,
        1.000,
        0.000,
        0.000,
        0.333,
        0.500,
        0.000,
        0.667,
        0.500,
        0.000,
        1.000,
        0.500,
        0.333,
        0.000,
        0.500,
        0.333,
        0.333,
        0.500,
        0.333,
        0.667,
        0.500,
        0.333,
        1.000,
        0.500,
        0.667,
        0.000,
        0.500,
        0.667,
        0.333,
        0.500,
        0.667,
        0.667,
        0.500,
        0.667,
        1.000,
        0.500,
        1.000,
        0.000,
        0.500,
        1.000,
        0.333,
        0.500,
        1.000,
        0.667,
        0.500,
        1.000,
        1.000,
        0.500,
        0.000,
        0.333,
        1.000,
        0.000,
        0.667,
        1.000,
        0.000,
        1.000,
        1.000,
        0.333,
        0.000,
        1.000,
        0.333,
        0.333,
        1.000,
        0.333,
        0.667,
        1.000,
        0.333,
        1.000,
        1.000,
        0.667,
        0.000,
        1.000,
        0.667,
        0.333,
        1.000,
        0.667,
        0.667,
        1.000,
        0.667,
        1.000,
        1.000,
        1.000,
        0.000,
        1.000,
        1.000,
        0.333,
        1.000,
        1.000,
        0.667,
        1.000,
        0.167,
        0.000,
        0.000,
        0.333,
        0.000,
        0.000,
        0.500,
        0.000,
        0.000,
        0.667,
        0.000,
        0.000,
        0.833,
        0.000,
        0.000,
        1.000,
        0.000,
        0.000,
        0.000,
        0.167,
        0.000,
        0.000,
        0.333,
        0.000,
        0.000,
        0.500,
        0.000,
        0.000,
        0.667,
        0.000,
        0.000,
        0.833,
        0.000,
        0.000,
        1.000,
        0.000,
        0.000,
        0.000,
        0.167,
        0.000,
        0.000,
        0.333,
        0.000,
        0.000,
        0.500,
        0.000,
        0.000,
        0.667,
        0.000,
        0.000,
        0.833,
        0.000,
        0.000,
        1.000,
        0.000,
        0.000,
        0.000,
        0.143,
        0.143,
        0.143,
        0.286,
        0.286,
        0.286,
        0.429,
        0.429,
        0.429,
        0.571,
        0.571,
        0.571,
        0.714,
        0.714,
        0.714,
        0.857,
        0.857,
        0.857,
        0.000,
        0.447,
        0.741,
        0.50,
        0.5,
        0,
    ]
).astype(np.float32)
