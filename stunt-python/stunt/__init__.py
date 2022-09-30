from stunt import types
from stunt import simulator

import logging
import os
import time
import cv2
import numpy as np

DEFAULT_SAMPLING_FREQUENCY = 30
DEFAULT_CARLA_HOST = "localhost"
DEFAULT_CARLA_PORT = 2000


def add_timestamp(image_np, timestamp):
    """Adds a timestamp text to an image np array.

    Args:
        image_np: A numpy array of the image.
        timestamp (:obj:`int`): The timestamp of the image.
    """

    txt_font = cv2.FONT_HERSHEY_SIMPLEX
    timestamp_txt = "{}".format(timestamp)
    # Put timestamp text.
    cv2.putText(
        image_np,
        timestamp_txt,
        (5, 15),
        txt_font,
        0.5,
        (0, 0, 0),
        thickness=1,
        lineType=cv2.LINE_AA,
    )


def get_top_down_transform(transform, top_down_camera_altitude):
    # Calculation relies on the fact that the camera's FOV is 90.
    top_down_location = transform.location + types.Location(
        0, 0, top_down_camera_altitude
    )
    return types.Transform(top_down_location, types.Rotation(-90, 0, 0))


def time_epoch_ms() -> int:
    """Get current time in milliseconds."""
    return int(time.time() * 1000)


def set_tf_loglevel(level):
    """To be used to suppress TensorFlow logging."""

    if level >= logging.FATAL:
        os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
    if level >= logging.ERROR:
        os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
    if level >= logging.WARNING:
        os.environ["TF_CPP_MIN_LOG_LEVEL"] = "1"
    else:
        os.environ["TF_CPP_MIN_LOG_LEVEL"] = "0"
    logging.getLogger("tensorflow").setLevel(level)


def verify_keys_in_dict(required_keys, arg_dict):
    assert set(required_keys).issubset(
        set(arg_dict.keys())
    ), "one or more of {} not found in {}".format(required_keys, arg_dict)


def create_camera_intrinsic_matrix(width: int, height: int, fov: float):
    """Creates the intrinsic matrix for a camera with the given
    parameters.

    Args:
        width (int): The width of the image returned by the camera.
        height (int): The height of the image returned by the camera.
        fov (float): The field-of-view of the camera.

    Returns:
        :py:class:`numpy.ndarray`: A 3x3 intrinsic matrix of the camera.
    """
    k = np.identity(3)
    # We use width - 1 and height - 1 to find the center column and row
    # of the image, because the images are indexed from 0.

    # Center column of the image.
    k[0, 2] = (width - 1) / 2.0
    # Center row of the image.
    k[1, 2] = (height - 1) / 2.0
    # Focal length.
    k[0, 0] = k[1, 1] = (width - 1) / (2.0 * np.tan(fov * np.pi / 360.0))
    return k


def create_camera_unreal_transform(transform):
    """Converts a Transform from the camera coordinate space to the
    Unreal coordinate space.

    The camera space is defined as:
        +x to right, +y to down, +z into the screen.

    The unreal coordinate space is defined as:
        +x into the screen, +y to right, +z to up.

    Args:
        transform (:py:class:`~stunt.types.Transform`): The transform to
            convert to Unreal coordinate space.

    Returns:
        :py:class:`~stunt.types.Transform`: The given transform after
            transforming to the Unreal coordinate space.
    """

    to_unreal_transform = types.Transform(
        matrix=np.array(
            [[0, 0, 1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]]
        )
    )
    return transform * to_unreal_transform
