import argparse
import json
import zenoh
import os
import pygame
import time
from pygame.locals import K_ESCAPE
from pygame.locals import K_F1
from pygame.locals import KMOD_CTRL
from pygame.locals import KMOD_SHIFT
from pygame.locals import K_BACKSPACE
from pygame.locals import K_TAB
from pygame.locals import K_SPACE
from pygame.locals import K_UP
from pygame.locals import K_DOWN
from pygame.locals import K_LEFT
from pygame.locals import K_RIGHT
from pygame.locals import K_w
from pygame.locals import K_a
from pygame.locals import K_s
from pygame.locals import K_d
from pygame.locals import K_q
from pygame.locals import K_m
from pygame.locals import K_COMMA
from pygame.locals import K_PERIOD
from pygame.locals import K_p
from pygame.locals import K_i
from pygame.locals import K_l
from pygame.locals import K_z
from pygame.locals import K_x
from pygame.locals import K_r
from pygame.locals import K_MINUS
from pygame.locals import K_EQUALS
import numpy as np
import cv2

from stunt.types import Image, GnssMeasurement, IMUMeasurement, VehicleControl


HUD_TICK_MS = 166
# WINDOW_WIDTH = 1920
# WINDOW_HEIGHT = 1080
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600


class HUD(object):
    def __init__(self, configuration):

        # initialization of zenoh session

        self.zconf = zenoh.Config()
        self.zconf.insert_json5(
            zenoh.config.MODE_KEY, json.dumps(configuration["mode"])
        )
        self.zconf.insert_json5(
            zenoh.config.CONNECT_KEY, json.dumps(configuration["locators"])
        )
        self.zsession = zenoh.open(self.zconf)

        # initializing variables
        self.center_image = None
        self.tele_image = None
        self.gnss = None
        self.imu = None
        self.center_lidar = None
        self.tele_lidar = None
        self.control = None

        self.tick_ms = HUD_TICK_MS
        self.height = WINDOW_HEIGHT
        self.width = WINDOW_WIDTH
        self.dim = (self.width, self.height)
        self.display = None
        self.font = None
        self._font_mono = None
        # initializing hud
        pygame.init()
        pygame.font.init()

        self.display = pygame.display.set_mode(
            (self.width, self.height), pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        self.display.fill((0, 0, 0))
        pygame.display.flip()

        self.font = pygame.font.Font(pygame.font.get_default_font(), 20)

        font_name = "courier" if os.name == "nt" else "mono"
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = "ubuntumono"
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == "nt" else 14)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        self.surface = None

        # start zenoh subscribers

        self.center_camera_sub = self.zsession.declare_subscriber(
            configuration["center_camera"]["ke"], self.on_center_camera
        )

        self.gnss_sub = self.zsession.declare_subscriber(
            configuration["gnss"]["ke"], self.on_gnss
        )

        self.imu_sub = self.zsession.declare_subscriber(
            configuration["imu"]["ke"], self.on_imu
        )

        self.control_sub = self.zsession.declare_subscriber(
            configuration["control"]["ke"], self.on_control
        )

    def on_center_camera(self, sample):
        self.center_image = Image.deserialize(sample.payload)

    def on_tele_camera(self, sample):
        self.tele_image = Image.deserialize(sample.payload)

    def on_gnss(self, sample):
        self.gnss = GnssMeasurement.deserialize(sample.payload)

    def on_imu(self, sample):
        self.imu = IMUMeasurement.deserialize(sample.payload)

    def on_control(self, sample):
        self.control = VehicleControl.deserialize(sample.payload)

    def render(self):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            self.display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [
                            (x + 8, v_offset + 8 + (1.0 - y) * 30)
                            for x, y in enumerate(item)
                        ]
                        pygame.draw.lines(
                            self.display, (255, 136, 0), False, points, 2
                        )
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect(
                            (bar_h_offset, v_offset + 8), (6, 6)
                        )
                        pygame.draw.rect(
                            self.display,
                            (255, 255, 255),
                            rect,
                            0 if item[1] else 1,
                        )
                    else:
                        rect_border = pygame.Rect(
                            (bar_h_offset, v_offset + 8), (bar_width, 6)
                        )
                        pygame.draw.rect(
                            self.display, (255, 255, 255), rect_border, 1
                        )
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect(
                                (
                                    bar_h_offset + f * (bar_width - 6),
                                    v_offset + 8,
                                ),
                                (6, 6),
                            )
                        else:
                            rect = pygame.Rect(
                                (bar_h_offset, v_offset + 8),
                                (f * bar_width, 6),
                            )
                        pygame.draw.rect(self.display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(
                        item, True, (255, 255, 255)
                    )
                    self.display.blit(surface, (8, v_offset))
                v_offset += 18

    def generate_info_text(self):
        if (
            self.imu is not None
            and self.gnss is not None
            and self.control is not None
        ):
            self._info_text = [
                "Server:  % 16.0f FPS" % self.server_fps,
                "Client:  % 16.0f FPS" % self._server_clock.get_fps(),
                "",
                "Acceler: (%5.1f,%5.1f,%5.1f)"
                % (
                    self.imu.accelerometer.x,
                    self.imu.accelerometer.y,
                    self.imu.accelerometer.z,
                ),
                "Gyroscope: (%5.1f,%5.1f,%5.1f)"
                % (
                    self.imu.gyroscope.x,
                    self.imu.gyroscope.y,
                    self.imu.gyroscope.z,
                ),
                "GNSS:% 24s"
                % (
                    "(% 2.6f, % 3.6f)"
                    % (self.gnss.latitude, self.gnss.longitude)
                ),
                "",
            ]
            self._info_text += [
                ("Throttle:", self.control.throttle, 0.0, 1.0),
                ("Steer:", self.control.steer, -1.0, 1.0),
                ("Brake:", self.control.brake, 0.0, 1.0),
                ("Reverse:", self.control.reverse),
                ("Hand brake:", self.control.hand_brake),
                ("Manual:", self.control.manual_gear_shift),
                "Gear:        %s"
                % {-1: "R", 0: "N"}.get(self.control.gear, self.control.gear),
            ]

    def render_image(self):
        if self.center_image is not None:
            array = self.center_image.raw_data
            array = cv2.resize(
                array, dsize=self.dim, interpolation=cv2.INTER_CUBIC
            )
            array = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            if self.surface is not None:
                self.display.blit(self.surface, (0, 0))

    def game_loop(self):
        while True:
            self._server_clock.tick_busy_loop(60)
            self.generate_info_text()
            self.render_image()
            self.render()
            pygame.display.flip()


if __name__ == "__main__":
    # --- Command line argument parsing --- --- --- --- --- ---
    parser = argparse.ArgumentParser(
        prog="visualizer", description="STUNT world preparation"
    )
    parser.add_argument(
        "--config",
        "-c",
        dest="config",
        metavar="FILE",
        type=str,
        required=True,
        help="The configuration file.",
    )

    args = parser.parse_args()
    with open(args.config, "r") as file:
        data = file.read()
    conf = json.loads(data)

    hud = HUD(conf)
    hud.game_loop()
