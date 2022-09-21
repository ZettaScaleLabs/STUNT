import array
import numpy as np
import math
import argparse
import json

import tensorflow as tf
from stunt.types import (
    TrafficLightColor,
    TrafficLight,
    BoundingBox2D,
    Image,
)

DEFAULT_MODEL_PATH = "/home/ato/Workspace/models/traffic_light_detection/faster-rcnn"
DEFAULT_MIN_SCORE_THRESHOLD = 0.7
DEFAULT_GPU_MEMORY_FRACTION = 0.3
DEFAULT_GPU = 0


class TrafficLightsTest:
    def __init__(
        self, traffic_model_path, min_threshold, gpu_memory_fraction, gpu_index
    ):

        self.traffic_model_path = traffic_model_path

        self.min_threshold = min_threshold

        self.gpu_memory_fraction = gpu_memory_fraction
        self.gpu_index = gpu_index

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

        self.frame = None

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
                        scores[index],
                        label,
                        id=self.unique_id,
                        bounding_box=bbox,
                        timestamp=0,  # self.frame.timestamp,
                    )
                )
                self.unique_id += 1

        return traffic_lights

    def run(self, frame):
        self.frame = Image.deserialize(frame)

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
            traffic_lights.append(tl)

        return tl


if __name__ == "__main__":

    # --- Command line argument parsing --- --- --- --- --- ---
    parser = argparse.ArgumentParser(
        prog="traffic lights", description="util script to test traffic lights"
    )
    parser.add_argument(
        "--frames",
        "-f",
        dest="frames",
        metavar="FILE",
        type=str,
        required=True,
        help="Frame readings file",
    )

    args = parser.parse_args()

    frames = []

    with open(args.frames, "r") as f:
        for line in f:
            frames.append(line.encode("utf-8"))

    traffic_lights = TrafficLightsTest(
        DEFAULT_MODEL_PATH,
        DEFAULT_MIN_SCORE_THRESHOLD,
        DEFAULT_GPU_MEMORY_FRACTION,
        DEFAULT_GPU,
    )

    print("#############################################")
    print(f"Traffic Lights press enter to process frame")
    print("#############################################")
    input("Press Enter to continue...")
    for i in range(0, len(frames)):
        frame = frames[i]

        result = traffic_lights.run(frame)
        print("#############################################")
        print(f"[{i}] Traffic Lights detected last frame: {result}")
        print("#############################################")
        input("Press Enter to continue...")
