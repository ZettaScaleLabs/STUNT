import carla
import time


from stunt import types


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
