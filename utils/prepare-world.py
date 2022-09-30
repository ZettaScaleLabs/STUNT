import json
import carla
import argparse
import random
import time

from carla import VehicleControl as CarlaVehicleControl

from stunt.types import (
    IMUMeasurement,
    GnssMeasurement,
    Image,
    LidarMeasurement,
    VehicleControl,
    SimulatorObstacle,
    TrafficLight,
    Pose,
)
from stunt.simulator.sensors import (
    IMUSensor,
    GNSSSensor,
    CameraSensor,
    LidarSensor,
    ZenohSensor,
    ZenohControl,
)

from stunt.simulator.ground_truth import Localization, Obstacles, TrafficLights

import zenoh
from zenoh import Reliability, SubMode


def spawn_people(client, world, num_people: int):
    """Spawns people at random locations inside the world.

    Args:
        num_people: The number of people to spawn.
    """
    from carla import command, Transform

    p_blueprints = world.get_blueprint_library().filter("walker.pedestrian.*")
    unique_locs = set([])
    spawn_points = []
    # Get unique spawn points.
    for i in range(num_people):
        attempt = 0
        while attempt < 100:
            spawn_point = Transform()
            loc = world.get_random_location_from_navigation()
            if loc is not None:
                # Transform to tuple so that location is comparable.
                p_loc = (loc.x, loc.y, loc.z)
                if p_loc not in unique_locs:
                    spawn_point.location = loc
                    spawn_points.append(spawn_point)
                    unique_locs.add(p_loc)
                    break
            attempt += 1
        if attempt == 10:
            print("Could not find unique person spawn point")
    # Spawn the people.
    batch = []
    for spawn_point in spawn_points:
        p_blueprint = random.choice(p_blueprints)
        if p_blueprint.has_attribute("is_invincible"):
            p_blueprint.set_attribute("is_invincible", "false")
        batch.append(command.SpawnActor(p_blueprint, spawn_point))
    # Apply the batch and retrieve the identifiers.
    ped_ids = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            print(
                "Received an error while spawning a person: {}".format(
                    response.error
                )
            )
        else:
            ped_ids.append(response.actor_id)
    # Spawn the person controllers
    ped_controller_bp = world.get_blueprint_library().find(
        "controller.ai.walker"
    )
    batch = []
    for ped_id in ped_ids:
        batch.append(
            command.SpawnActor(ped_controller_bp, Transform(), ped_id)
        )
    ped_control_ids = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            print(
                "Error while spawning a person controller: {}".format(
                    response.error
                )
            )
        else:
            ped_control_ids.append(response.actor_id)

    return (ped_ids, ped_control_ids)


def spawn_ego_vehicle(
    world,
    traffic_manager_port: int,
    spawn_point_index: int,
    auto_pilot: bool,
    blueprint: str = "vehicle.audi.etron",
):
    # vehicle.lincoln.mkz2017 for versions prior to 9.12
    v_blueprint = world.get_blueprint_library().filter(blueprint)[0]
    v_blueprint.set_attribute("role_name", "hero")
    ego_vehicle = None
    while not ego_vehicle:
        if spawn_point_index == -1:
            # Pick a random spawn point.
            start_pose = random.choice(world.get_map().get_spawn_points())
        else:
            spawn_points = world.get_map().get_spawn_points()
            assert spawn_point_index < len(spawn_points), (
                "Spawn point index is too big. "
                "Town does not have sufficient spawn points."
            )
            start_pose = spawn_points[spawn_point_index]

        ego_vehicle = world.try_spawn_actor(v_blueprint, start_pose)
    if auto_pilot:
        ego_vehicle.set_autopilot(True, traffic_manager_port)
    return ego_vehicle


def spawn_vehicles(
    client, world, traffic_manager_port: int, num_vehicles: int
):
    """Spawns vehicles at random locations inside the world.

    Args:
        num_vehicles: The number of vehicles to spawn.
    """
    from carla import command

    print("Trying to spawn {} vehicles.".format(num_vehicles))
    # Get the spawn points and ensure that the number of vehicles
    # requested are less than the number of spawn points.
    spawn_points = world.get_map().get_spawn_points()

    print((f"Available {len(spawn_points)} spawn points:"))
    for sp in spawn_points:
        print(f"# {sp}")

    if num_vehicles >= len(spawn_points):
        print(
            "Requested {} vehicles but only found {} spawn points".format(
                num_vehicles, len(spawn_points)
            )
        )
        num_vehicles = len(spawn_points)
    else:
        random.shuffle(spawn_points)

    # Get all the possible vehicle blueprints inside the world.
    v_blueprints = world.get_blueprint_library().filter("vehicle.*")

    # Construct a batch message that spawns the vehicles.
    batch = []
    for transform in spawn_points[:num_vehicles]:
        blueprint = random.choice(v_blueprints)

        # Change the color of the vehicle.
        if blueprint.has_attribute("color"):
            color = random.choice(
                blueprint.get_attribute("color").recommended_values
            )
            blueprint.set_attribute("color", color)

        # Let the vehicle drive itself.
        blueprint.set_attribute("role_name", "autopilot")

        batch.append(
            command.SpawnActor(blueprint, transform).then(
                command.SetAutopilot(
                    command.FutureActor, True, traffic_manager_port
                )
            )
        )

    # Apply the batch and retrieve the identifiers.
    vehicle_ids = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            print(
                "Received an error while spawning a vehicle: {}".format(
                    response.error
                )
            )
        else:
            vehicle_ids.append(response.actor_id)
    return vehicle_ids


def spawn_actors(
    client,
    world,
    traffic_manager_port: int,
    ego_spawn_point_index: int,
    num_people: int,
    num_vehicles: int,
):
    vehicle_ids = spawn_vehicles(
        client, world, traffic_manager_port, num_vehicles
    )
    ego_vehicle = spawn_ego_vehicle(
        world, traffic_manager_port, ego_spawn_point_index, False
    )
    people = []

    # People do not move in versions older than 0.9.6.
    (people, people_control_ids) = spawn_people(client, world, num_people)
    people_actors = world.get_actors(people_control_ids)
    for i, ped_control_id in enumerate(people_control_ids):
        # Start person.
        people_actors[i].start()
        people_actors[i].go_to_location(
            world.get_random_location_from_navigation()
        )
    return ego_vehicle, vehicle_ids, people


def main(config):
    carla_host = config["host"]
    carla_port = config["port"]
    num_vehicles = config["num_vehicles"]
    num_people = config["num_people"]
    ego_spawn_point_index = config["ego_spawn_point_index"]
    town_map = config["map"]
    weather = config["weather"]
    traffic_manager_port = config["traffic_manager_port"]

    # setting the world

    carla_client = carla.Client(carla_host, carla_port)
    carla_world = carla_client.load_world(town_map)
    carla_world.set_weather(getattr(carla.WeatherParameters, weather))
    tm = carla_client.get_trafficmanager(traffic_manager_port)

    # spawing things
    ego_vehicle, vehicle_ids, people_ids = spawn_actors(
        carla_client,
        carla_world,
        traffic_manager_port,
        ego_spawn_point_index,
        num_people,
        num_vehicles,
    )

    # # setting autopilot for non-ego vehicles
    # vehicles = carla_world.get_actors(vehicle_ids)
    # for v in vehicles:
    #     v.set_autopilot(True, tm.get_port())

    print(f"Ego vehicle {ego_vehicle}")

    if config["challenge_config"]["enabled"]:

        new_config = config["challenge_config"]

        print(f"Challenge Mode, adding sensors and Zenoh Publishers")

        # setting world as synchronous (simulator slows down a lot with all sensors)

        # configuring zenoh
        zconf = zenoh.Config()

        zconf.insert_json5(
            zenoh.config.MODE_KEY, json.dumps(new_config["mode"])
        )
        zconf.insert_json5(
            zenoh.config.CONNECT_KEY, json.dumps(new_config["locators"])
        )

        zsession = zenoh.open(zconf)

        # imu_sensor = imu_setup(carla_world, ego_vehicle, new_config["imu"])

        imu_pub = ZenohSensor(
            zsession,
            new_config["imu"]["ke"],
            IMUSensor,
            IMUMeasurement,
            new_config["imu"],
        )

        gnss_pub = ZenohSensor(
            zsession,
            new_config["gnss"]["ke"],
            GNSSSensor,
            GnssMeasurement,
            new_config["gnss"],
        )

        gnss_pub = ZenohSensor(
            zsession,
            new_config["gnss"]["ke"],
            GNSSSensor,
            GnssMeasurement,
            new_config["gnss"],
        )

        center_camera_pub = ZenohSensor(
            zsession,
            new_config["center_camera"]["ke"],
            CameraSensor,
            Image,
            new_config["center_camera"],
        )

        tele_camera_pub = ZenohSensor(
            zsession,
            new_config["tele_camera"]["ke"],
            CameraSensor,
            Image,
            new_config["tele_camera"],
        )

        center_lidar_pub = ZenohSensor(
            zsession,
            new_config["center_lidar"]["ke"],
            LidarSensor,
            LidarMeasurement,
            new_config["center_lidar"],
        )

        tele_lidar_pub = ZenohSensor(
            zsession,
            new_config["tele_lidar"]["ke"],
            LidarSensor,
            LidarMeasurement,
            new_config["tele_lidar"],
        )

        obstacles_pub = ZenohSensor(
            zsession,
            new_config["obstacles"]["ke"],
            Obstacles,
            SimulatorObstacle,
            new_config["obstacles"],
        )

        traffic_lights_pub = ZenohSensor(
            zsession,
            new_config["traffic-lights"]["ke"],
            TrafficLights,
            TrafficLight,
            new_config["traffic-lights"],
        )

        location_pub = ZenohSensor(
            zsession,
            new_config["location"]["ke"],
            Localization,
            Pose,
            new_config["location"],
        )

        print(f"Challenge Mode, setting simulator as synchronous")
        world_settings = carla_world.get_settings()
        world_settings.synchronous_mode = True
        world_settings.fixed_delta_seconds = 1 / int(config["fps"])
        carla_world.apply_settings(world_settings)

        counter = 0

        def on_ctrl_data(sample):
            ctrl = VehicleControl.deserialize(sample.payload)
            carla_ctrl = CarlaVehicleControl()

            carla_ctrl.throttle = ctrl.throttle
            carla_ctrl.steer = ctrl.steer
            carla_ctrl.brake = ctrl.brake
            ego_vehicle.apply_control(carla_ctrl)

            # frame_id = carla_world.tick()
            # print(f"Ticking the world frame id {frame_id} - Control Received {ctrl}")
            print(f"Control Received {ctrl}")
            # counter += 1

        control_sub = ZenohControl(
            zsession, new_config["control"]["ke"], on_ctrl_data
        )

        # counter = 0
        while True:
            time.sleep(float(new_config["sleep_time"]))
            frame_id = carla_world.tick()
            print(f"Ticking the world frame id {frame_id}")
            # counter += 1

    else:
        while True:
            time.sleep(1)


if __name__ == "__main__":
    # --- Command line argument parsing --- --- --- --- --- ---
    parser = argparse.ArgumentParser(
        prog="prepare-world", description="STUNT world preparation"
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
