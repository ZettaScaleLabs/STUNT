import argparse
import time
import cv2
import zenoh
import json
import numpy as np
from stunt.types import (
    Image,
    GnssMeasurement,
    IMUMeasurement,
    VehicleControl,
    LidarMeasurement,
    PointCloud,
    Vector3D,
)


HUD_TICK_MS = 166
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720


CENTER_CAMERA = 0
TELE_CAMERA = 1
CENTER_LIDAR = 2
TELE_LIDAR = 3


class UI(object):
    def __init__(self, configuration):

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
        self.speed = 0
        self.center_lidar = None
        self.tele_lidar = None
        self.control = VehicleControl()

        self.processing_time = 1.0

        self.visualizing = CENTER_CAMERA
        self.tick_ms = HUD_TICK_MS
        self.height = WINDOW_HEIGHT
        self.width = WINDOW_WIDTH
        self.dim = (self.width, self.height)
        self.display = np.zeros((self.height, self.width, 3), np.uint8)
        self.font = None
        self._font_mono = None

        self.window_name = "ZenohCar"

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

        self.can_sub = self.zsession.declare_subscriber(
            configuration["can"]["ke"], self.on_can
        )

    def start(self):
        while True:
            self.render()
            cv2.imshow(self.window_name, self.display)
            cv2.waitKey(1) & 0xFF
            time.sleep(1/self.tick_ms)

    def on_tele_camera(self, sample):
        tele_image = Image.deserialize(sample.payload)
        self.tele_image = Image.from_jpeg(bytes(tele_image.raw_data))

    def on_center_camera(self, sample):
        center_image = Image.deserialize(sample.payload)
        self.center_image = Image.from_jpeg(bytes(center_image.raw_data))

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

    def on_can(self, sample):
        speed_vec = Vector3D.deserialize(sample.payload)
        speed_ms = speed_vec.magnitude()
        self.speed = speed_ms * 3.6

    def generate_info_text(self):
        self._info_text = [
            "FPS:  % 16.0f FPS" % (1/self.processing_time),
            # "Client:  % 16.0f FPS" % self._server_clock.get_fps(),
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
            "Speed: %4f" % self.speed,
            "",

        ]
        self._info_text += [
            "Throttle: %1.2f" % self.control.throttle,
            "Steer: %1.2f" % self.control.steer,
            "Brake: %1.2f" % self.control.brake,
            "Reverse: %r" % self.control.reverse,
            "Hand brake: %r" % self.control.hand_brake,
            "Manual: %r" % self.control.manual_gear_shift,
            "Gear:        %s"
            % {-1: "R", 0: "N"}.get(self.control.gear, self.control.gear),
        ]

    def render(self):
        begin = time.time()

        x_cord = 10
        y_cord = 10
        text_color = (0, 255, 0)
        text_color_bg = (0, 0, 0)
        text_scale = 0.5
        text_tickness = 1

        if self.center_image is not None:
            self.display = self.center_image.copy()
            self.display = cv2.resize(self.display, (self.width, self.height))

        self.generate_info_text()

        for line in self._info_text:
            text_size, _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, text_scale, text_tickness)
            text_w, text_h = text_size
            cv2.rectangle(self.display, (x_cord, y_cord), (x_cord + text_w, y_cord + text_h), text_color_bg, -1)
            cv2.putText(self.display, line, (x_cord, y_cord+10), cv2.FONT_HERSHEY_SIMPLEX, text_scale, text_color, text_tickness)
            y_cord = y_cord + text_h

        self.processing_time = time.time() - begin


# img = None

# def on_center_camera(sample):
#     # now = datetime.now()
#     # print(f'[{now}] Received data!!')
#     deserialized = Image.deserialize(sample.value.payload)
#     global img
#     img = Image.from_jpeg(bytes(deserialized.raw_data))


def main(configuration):


    ui = UI(configuration)

    ui.start()

    # zconf = zenoh.Config()
    # zconf.insert_json5(
    #     zenoh.config.MODE_KEY, json.dumps(configuration["mode"])
    # )
    # zconf.insert_json5(
    #     zenoh.config.CONNECT_KEY, json.dumps(configuration["locators"])
    # )

    # zsession = zenoh.open(zconf)
    # print('Zenoh session is open')
    # center_camera_sub = zsession.declare_subscriber(
    #     configuration["center_camera"]["ke"], on_center_camera
    # )
    # print(f'Subscriber declared to: {configuration["center_camera"]["ke"]}')

    # while True:
    #     if img is not None:
    #         cv2.imshow("center", img)
    #     cv2.waitKey(1) & 0xFF
    #     # if key is True:
    #     #     break
    #     time.sleep(0.05)

    # center_camera_sub.undeclare()


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
    main(conf)



