import carla
import time

from stunt import DEFAULT_SAMPLING_FREQUENCY, DEFAULT_CARLA_HOST, DEFAULT_CARLA_PORT


class ControlSensor:
    def __init__(self, configuration):

        configuration = {} if configuration is None else configuration
        self.period = 1 / configuration.get("frequency", DEFAULT_SAMPLING_FREQUENCY)

        self.carla_port = int(configuration.get("port", DEFAULT_CARLA_PORT))
        self.carla_host = configuration.get("host", DEFAULT_CARLA_HOST)

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        self.player = None
        self.sensor = None

        self.player = None
        while self.player is None:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter("vehicle.*")
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    self.player = vehicle
                    break

    def read_data(self):
        return self.player.get_control()
