import carla
import time

from stunt.types import Location, Rotation, Image
from stunt import (
    DEFAULT_SAMPLING_FREQUENCY,
    DEFAULT_CARLA_HOST,
    DEFAULT_CARLA_PORT,
)


DEFAULT_JPEG_QUALITY = 25
DEFAULT_CAMERA_LOCATION = (0.0, 0.0, 0.0)
DEFAULT_CAMERA_ROTATION = (0.0, 0.0, 0.0)
DEFAULT_CAMERA_TYPE = "sensor.camera.rgb"
DEFAULT_IMAGE_WIDTH = 1920
DEFAULT_IMAGE_HEIGHT = 1080
DEFAULT_CAMERA_FOV = 90
DEFAULT_CAMERA_NAME = "stunt-camera"


class CameraSensor:
    def __init__(self, configuration, on_data):

        configuration = {} if configuration is None else configuration
        self.period = 1 / configuration.get(
            "frequency", DEFAULT_SAMPLING_FREQUENCY
        )

        self.carla_port = int(configuration.get("port", DEFAULT_CARLA_PORT))
        self.carla_host = configuration.get("host", DEFAULT_CARLA_HOST)
        self.period = 1 / configuration.get(
            "frequency", DEFAULT_SAMPLING_FREQUENCY
        )

        self.jpeg_quality = configuration.get(
            "jpeg_quality", DEFAULT_JPEG_QUALITY
        )

        self.camera_location = Location(
            *configuration.get("location", DEFAULT_CAMERA_LOCATION)
        )
        self.camera_rotation = Rotation(
            *configuration.get("rotation", DEFAULT_CAMERA_ROTATION)
        )
        self.camera_type = configuration.get("type", DEFAULT_CAMERA_TYPE)
        self.image_width = configuration.get("width", DEFAULT_IMAGE_WIDTH)
        self.image_height = configuration.get("height", DEFAULT_IMAGE_HEIGHT)
        self.camera_fov = configuration.get("fov", DEFAULT_CAMERA_FOV)
        self.camera_name = configuration.get("name", DEFAULT_CAMERA_NAME)

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        self.player = None
        self.sensor = None

        self.cb = on_data

        # Waiting EGO vehicle
        while self.player is None:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter(
                "vehicle.*"
            )
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    self.player = vehicle

                    # check if there is a camera
                    # already attached to the vehicle

                    possible_cameras = self.carla_world.get_actors().filter(
                        self.camera_type
                    )
                    for camera in possible_cameras:
                        if (
                            camera.parent.id == self.player.id
                            and camera.attributes["role_name"]
                            == self.camera_name
                        ):
                            self.sensor = camera
                            break

                    break

        # if we did not find the camera we create a new one
        if self.sensor is None:

            #  Configuring the sensor in CARLA
            bp = self.carla_world.get_blueprint_library().find(
                self.camera_type
            )

            bp.set_attribute("role_name", self.camera_name)
            bp.set_attribute("image_size_x", str(self.image_width))
            bp.set_attribute("image_size_y", str(self.image_height))
            bp.set_attribute("fov", str(self.camera_fov))
            bp.set_attribute("sensor_tick", str(self.period))

            cam_location = self.camera_location.as_simulator_location()
            cam_rotation = self.camera_rotation.as_simulator_rotation()

            # Attaching the sensor to the vehicle
            self.sensor = self.carla_world.spawn_actor(
                bp,
                carla.Transform(cam_location, cam_rotation),
                attach_to=self.player,
            )

        self.sensor.listen(self.on_simulator_data)

    def on_simulator_data(self, data):
        frame = Image.from_simulator(data, self.jpeg_quality)
        self.cb(frame)
