import carla
import time
import logging
import os

import cv2


DEFAULT_SAMPLING_FREQUENCY = 30
DEFAULT_CARLA_HOST = "localhost"
DEFAULT_CARLA_PORT = 2000


from stunt import types
from stunt import simulator


class CarlaSrcState(object):
    def __init__(self, configuration):

        self.carla_port = 2000
        self.carla_host = "localhost"
        self.period = 1 / 30

        if configuration is not None and configuration.get("port") is not None:
            self.carla_port = int(configuration["port"])

        if configuration is not None and configuration.get("host") is not None:
            self.carla_host = configuration["host"]

        if configuration is not None and configuration.get("frequency") is not None:
            self.period = 1 / configuration["frequency"]

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        self.player = None
        while self.player is None:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter("vehicle.*")
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    print("Ego vehicle found")
                    self.player = vehicle
                    break

    def on_world_tick(self, timestamp):
        None


# def run_visualizer_control_loop():
#     """Runs a pygame loop that waits for user commands.

#     The user commands are send on the control_display_stream
#     to control the pygame visualization window.
#     """
#     import pygame

#     clock = pygame.time.Clock()
#     from pygame.locals import K_n

#     while True:
#         clock.tick_busy_loop(60)
#         events = pygame.event.get()
#         for event in events:
#             if event.type == pygame.KEYUP:
#                 if event.key == K_n:
#                     return event.key
#             elif event.type == pygame.QUIT:
#                 raise KeyboardInterrupt
#             elif event.type == pygame.KEYDOWN:
#                 if event.key == pygame.K_c and pygame.key.get_mods() & pygame.KMOD_CTRL:
#                     raise KeyboardInterrupt


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
    top_down_location = transform.location + Location(0, 0, top_down_camera_altitude)
    return Transform(top_down_location, Rotation(-90, 0, 0))


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
