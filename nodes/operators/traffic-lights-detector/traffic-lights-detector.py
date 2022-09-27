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
    TrafficLightColor,
    TrafficLight,
    BoundingBox2D,
    TimeToDecision,
    Image,
)

DEFAULT_MODEL_PATH = "/tmp/model"
DEFAULT_MIN_SCORE_THRESHOLD = 0.7
DEFAULT_GPU_MEMORY_FRACTION = 0.3
DEFAULT_GPU = 0


class TrafficLightDetection(Operator):
    def __init__(self, context, configuration, inputs, outputs):

        configuration = {} if configuration is None else configuration

        self.image_input = inputs.get("Image", None)
        self.ttd_input = inputs.get("TTD", None)
        self.output = outputs.get("TrafficLights", None)

        self.pending = []

        self.ttd_data = None
        self.frame = None

        self.traffic_model_path = configuration.get(
            "traffic_light_det_model_path", DEFAULT_MODEL_PATH
        )
        self.min_threshold = configuration.get(
            "traffic_light_det_min_score_threshold", DEFAULT_MIN_SCORE_THRESHOLD
        )
        self.gpu_memory_fraction = configuration.get(
            "traffic_light_det_gpu_memory_fraction", DEFAULT_GPU_MEMORY_FRACTION
        )
        self.gpu_index = configuration.get("traffic_light_det_gpu_index", DEFAULT_GPU)

        self.labels = {
            1: TrafficLightColor.GREEN,
            2: TrafficLightColor.YELLOW,
            3: TrafficLightColor.RED,
            4: TrafficLightColor.OFF,
        }

        # configure tf
        # Only sets memory growth for flagged GPU
        physical_devices = tf.config.experimental.list_physical_devices("GPU")
        tf.config.experimental.set_visible_devices(
            [physical_devices[self.gpu_index]], "GPU"
        )
        tf.config.experimental.set_memory_growth(physical_devices[self.gpu_index], True)

        # Unique bounding box id. Incremented for each bounding box.
        self.unique_id = 0

        # loads the model
        self.model = tf.saved_model.load(self.traffic_model_path)

        # Serve some junk image this forces tensorflow to load the model
        self.run_model(np.zeros((1080, 1920, 3), dtype="uint8"))

    def run_model(self, image_np):
        # Expand dimensions since the model expects images to have
        # shape: [1, None, None, 3]

        image_np_expanded = np.expand_dims(image_np, axis=0)
        # print(f"image_np_expanded: Shape of frame {image_np_expanded.shape}")

        infer = self.model.signatures["serving_default"]
        result = infer(tf.convert_to_tensor(value=image_np_expanded))

        boxes = result["boxes"].numpy()
        scores = result["scores"].numpy()
        classes = result["classes"].numpy()
        num_detections = result["detections"].numpy()
        num_detections = int(num_detections[0])

        # np.ndarray is unhashable thus the use of tuple
        res_boxes = boxes[0][:num_detections]
        res_scores = scores[0][:num_detections]

        return res_boxes, res_scores, classes

    def convert_to_detected_tl(
        self, boxes, scores, labels, height: float, width: float
    ):
        traffic_lights = []

        for index in range(len(scores)):
            if scores[index] > self.min_threshold:
                label = self.labels[int(labels[0][index])]

                bbox = BoundingBox2D(
                    (boxes[index][1] * width),  # x_min
                    (boxes[index][3] * width),  # x_max
                    (boxes[index][0] * height),  # y_min
                    (boxes[index][2] * height),  # y_max
                )
                traffic_lights.append(
                    TrafficLight(
                        float(scores[index]),
                        label,
                        id=self.unique_id,
                        bounding_box=bbox,
                        timestamp=self.frame.timestamp,
                    )
                )
                self.unique_id += 1

        return traffic_lights

    async def wait_image(self):
        data_msg = await self.image_input.recv()
        return ("Image", data_msg)

    async def wait_ttd(self):
        data_msg = await self.ttd_input.recv()
        return ("TTD", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "Image" for t in task_list):
            task_list.append(asyncio.create_task(self.wait_image(), name="Image"))

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

                boxes, scores, labels = self.run_model(self.frame.as_rgb_numpy_array())

                traffic_lights = []

                traffic_lights_dec = self.convert_to_detected_tl(
                    boxes,
                    scores,
                    labels,
                    height=self.frame.height,
                    width=self.frame.width,
                )

                for tl in traffic_lights_dec:
                    traffic_lights.append(tl.to_dict())

                await self.output.send(json.dumps(traffic_lights).encode("utf-8"))

        return None

    def finalize(self) -> None:
        return None


def register():
    return TrafficLightDetection
