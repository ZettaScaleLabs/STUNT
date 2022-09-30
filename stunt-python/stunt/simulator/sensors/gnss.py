import carla
import time

from stunt import (
    DEFAULT_SAMPLING_FREQUENCY,
    DEFAULT_CARLA_HOST,
    DEFAULT_CARLA_PORT,
)


DEFAULT_NOISE_ALT_STDDEV = 0.0
DEFAULT_NOISE_LAT_STDDEV = 0.0
DEFAULT_NOISE_LON_STDDEV = 0.0
DEFAULT_NOISE_ALT_BIAS = 0.0
DEFAULT_NOISE_LAT_BIAS = 0.0
DEFAULT_NOISE_LON_BIAS = 0.0
DEFAULT_GNSS_NAME = "stunt-gnss"


class GNSSSensor:
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
        self.name = configuration.get("name", DEFAULT_GNSS_NAME)

        self.noise_alt_stddev = configuration.get(
            "noise_alt_stddev", DEFAULT_NOISE_ALT_STDDEV
        )
        self.noise_lat_stddev = configuration.get(
            "noise_lat_stddev", DEFAULT_NOISE_LAT_STDDEV
        )
        self.noise_lon_stddev = configuration.get(
            "noise_lon_stddev", DEFAULT_NOISE_LON_STDDEV
        )
        self.noise_alt_bias = configuration.get(
            "noise_alt_bias", DEFAULT_NOISE_ALT_BIAS
        )
        self.noise_lat_bias = configuration.get(
            "noise_lat_bias", DEFAULT_NOISE_LAT_BIAS
        )
        self.noise_lon_bias = configuration.get(
            "noise_lon_bias", DEFAULT_NOISE_LON_BIAS
        )

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        self.player = None
        self.sensor = None

        while self.player is None:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter(
                "vehicle.*"
            )
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    self.player = vehicle

                    # check if there is a gnss we can use
                    possible_gnss = self.carla_world.get_actors().filter(
                        "sensor.other.gnss"
                    )
                    for gnss in possible_gnss:
                        if (
                            gnss.parent.id == self.player.id
                            and gnss.attributes["role_name"] == self.name
                        ):
                            self.sensor = gnss
                            break
                    break

        if self.sensor is None:

            bp = self.carla_world.get_blueprint_library().find(
                "sensor.other.gnss"
            )

            bp.set_attribute("role_name", self.name)
            bp.set_attribute("noise_alt_stddev", str(self.noise_alt_stddev))
            bp.set_attribute("noise_lat_stddev", str(self.noise_lat_stddev))
            bp.set_attribute("noise_lon_stddev", str(self.noise_lon_stddev))
            bp.set_attribute("noise_alt_bias", str(self.noise_alt_bias))
            bp.set_attribute("noise_lat_bias", str(self.noise_lat_bias))
            bp.set_attribute("noise_lon_bias", str(self.noise_lon_bias))

            bp.set_attribute("sensor_tick", str(self.period))

            self.sensor = self.carla_world.spawn_actor(
                bp, carla.Transform(), attach_to=self.player
            )

        self.sensor.listen(on_data)
