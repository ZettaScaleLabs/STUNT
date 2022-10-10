#!/usr/bin/env python3


HELP = """
STUNT Zenoh-based visualizer

Use ARROWS or WASD keys for control.

    [1-9]        : change to sensor [1-9]
    H/?          : toggle help
    ESC          : quit
"""

import argparse
import json
import zenoh
import os
import pygame
import time
from pygame.locals import K_1, K_2, K_3, K_4, K_ESCAPE, K_h, K_QUESTION, K_q
import numpy as np
import cv2

from stunt.types import (
    Image,
    GnssMeasurement,
    IMUMeasurement,
    VehicleControl,
    LidarMeasurement,
    PointCloud,
)


HUD_TICK_MS = 166
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720


CENTER_CAMERA = 0
TELE_CAMERA = 1
CENTER_LIDAR = 2
TELE_LIDAR = 3

# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""

    def __init__(self, font, width, height):
        lines = HELP
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (
            0.5 * width - 0.5 * self.dim[0],
            0.5 * height - 0.5 * self.dim[1],
        )
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


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
        self.gnss = GnssMeasurement()
        self.imu = IMUMeasurement()
        self.center_lidar = None
        self.tele_lidar = None
        self.control = VehicleControl()

        self.visualizing = CENTER_CAMERA
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

        self.show_help = False

        self.notifications = FadingText(
            self.font, (self.width, 40), (0, self.height - 40)
        )
        # self.help = HelpText(
        #     pygame.font.Font(mono, 16), self.width, self.height
        # )

        # start zenoh subscribers

        self.center_camera_sub = self.zsession.declare_subscriber(
            configuration["center_camera"]["ke"], self.on_center_camera
        )

        self.tele_camera_sub = self.zsession.declare_subscriber(
            configuration["tele_camera"]["ke"], self.on_tele_camera
        )

        self.center_lidar_sub = self.zsession.declare_subscriber(
            configuration["center_lidar"]["ke"], self.on_center_lidar
        )

        self.tele_lidar_sub = self.zsession.declare_subscriber(
            configuration["tele_lidar"]["ke"], self.on_tele_lidar
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

    def on_tele_camera(self, sample):
        self.tele_image = Image.deserialize(sample.payload)

    def on_center_camera(self, sample):
        self.center_image = Image.deserialize(sample.payload)

    def on_gnss(self, sample):
        self.gnss = GnssMeasurement.deserialize(sample.payload)

    def on_imu(self, sample):
        self.imu = IMUMeasurement.deserialize(sample.payload)

    def on_control(self, sample):
        self.control = VehicleControl.deserialize(sample.payload)

    def on_center_lidar(self, sample):
        lidar_data = LidarMeasurement.deserialize(sample.payload)
        self.center_lidar = PointCloud.from_lidar_measurement(lidar_data)

    def on_tele_lidar(self, sample):
        lidar_data = LidarMeasurement.deserialize(sample.payload)
        self.tele_lidar = PointCloud.from_lidar_measurement(lidar_data)

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

    def parse_keyboard_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if event.key == K_ESCAPE or event.key == K_q:
                    return True
                elif event.key == K_1:
                    self.visualizing = CENTER_CAMERA
                    return False
                elif event.key == K_2:
                    self.visualizing = TELE_CAMERA
                    return False
                elif event.key == K_3:
                    self.visualizing = CENTER_LIDAR
                    return False
                elif event.key == K_4:
                    self.visualizing = TELE_LIDAR
                    return False
                elif event.type == K_h or event.type == K_QUESTION:
                    self.show_help = not self.show_help
                    return False

    def notification(self, text, seconds=2.0):
        self.notifications.set_text(text, seconds=seconds)

    def generate_info_text(self):
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
            % ("(% 2.6f, % 3.6f)" % (self.gnss.latitude, self.gnss.longitude)),
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

    def render_center_image(self):
        if self.center_image is not None:
            array = self.center_image.raw_data
            array = cv2.resize(
                array, dsize=self.dim, interpolation=cv2.INTER_CUBIC
            )
            array = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            if self.surface is not None:
                self.display.blit(self.surface, (0, 0))

    def render_tele_image(self):
        if self.tele_image is not None:
            array = self.tele_image.raw_data
            array = cv2.resize(
                array, dsize=self.dim, interpolation=cv2.INTER_CUBIC
            )
            array = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            if self.surface is not None:
                self.display.blit(self.surface, (0, 0))

    def render_center_lidar(self):
        lidar_range = 100
        if self.center_lidar is not None:
            lidar_data = np.array(self.center_lidar.global_points[:, :2])
            lidar_data *= min(self.width, self.height) / (2.0 * lidar_range)
            lidar_data += (0.5 * self.width, 0.5 * self.height)
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.width, self.height, 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)

    def render_tele_lidar(self):
        lidar_range = 85
        if self.tele_lidar is not None:
            lidar_data = np.array(self.tele_lidar.global_points[:, :2])
            lidar_data *= min(self.width, self.height) / (2.0 * lidar_range)
            lidar_data += (0.5 * self.width, 0.5 * self.height)
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.width, self.height, 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)

    def game_loop(self):
        self.notification("Press 'H' or '?' for help.", seconds=4.0)
        done = False
        while not done:
            self._server_clock.tick_busy_loop(60)
            done = self.parse_keyboard_events()
            if self.visualizing == CENTER_CAMERA:
                self.render_center_image()
            elif self.visualizing == TELE_CAMERA:
                self.render_tele_image()
            elif self.visualizing == CENTER_LIDAR:
                self.render_center_lidar()
            elif self.visualizing == TELE_LIDAR:
                self.render_tele_lidar()

            self.generate_info_text()
            self.render()
            # if self.show_help:
            #     self.help.render(self.display)
            pygame.display.flip()

        exit(0)


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
