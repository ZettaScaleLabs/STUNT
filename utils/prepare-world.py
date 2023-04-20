import json
import carla
import argparse
import random
import time
import logging
import zenoh
# from carla.command import SpawnActor, SetAutopilot, FutureActor
from carla import VehicleControl as CarlaVehicleControl
from stunt.types import VehicleControl
from stunt.simulator.sensors import ZenohControl

PEDESTRIAN_GENERATIONS = "All"
VEHICLES_GENERATIONS = "All"


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            logging.warning("Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except Exception as e:
        logging.warning(f"Warning! Actor Generation is not valid. No actor will be spawned: {e}")
        return []


def spawn_people(client, world, num_people: int):
    """Spawns people at random locations inside the world.

    Args:
        num_people: The number of people to spawn.
    """

    percentagePedestriansRunning = 0.0      # how many pedestrians will run
    percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
    walkers_list = []
    ped_ids = []
    all_actors = []

    p_blueprints = get_actor_blueprints(world, "walker.pedestrian.*", PEDESTRIAN_GENERATIONS)

    # 1. take all the random locations to spawn
    spawn_points = []
    for i in range(num_people):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if (loc is not None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    # 2. Spawn the walker objects.
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        p_blueprint = random.choice(p_blueprints)
        if p_blueprint.has_attribute("is_invincible"):
            p_blueprint.set_attribute("is_invincible", "false")

        if p_blueprint.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(p_blueprint.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(p_blueprint.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)

        batch.append(SpawnActor(p_blueprint, spawn_point))

    results = client.apply_batch_sync(batch, True)

    world.wait_for_tick()

    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])

    walker_speed = walker_speed2

    # 3. spawn the walker controller

    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id

    # 4. we put together the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        ped_ids.append(walkers_list[i]["con"])
        ped_ids.append(walkers_list[i]["id"])
        all_actors = world.get_actors(ped_ids)

    world.wait_for_tick()

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(ped_ids), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

    return all_actors


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

    logging.info("Trying to spawn {} vehicles.".format(num_vehicles))

    # Get the spawn points and ensure that the number of vehicles
    # requested are less than the number of spawn points.
    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)
    blueprints = get_actor_blueprints(world, "vehicle.*", VEHICLES_GENERATIONS)
    blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']
    blueprints = sorted(blueprints, key=lambda bp: bp.id)

    batch = []
    vehicles_list = []
    # print((f"Available {len(spawn_points)} spawn points:"))
    # for sp in spawn_points:
    #     print(f"# {sp}")

    if num_vehicles >= number_of_spawn_points:
        logging.warning(f"Requested {num_vehicles} vehicles but only found {number_of_spawn_points} spawn points")
        num_vehicles = number_of_spawn_points
    else:
        random.shuffle(spawn_points)

    for n, transform in enumerate(spawn_points):
        if n >= num_vehicles:
            break
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')

        batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True, traffic_manager_port)))

        responses = client.apply_batch_sync(batch, True)
        world.wait_for_tick()

        for response in responses:
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

    return vehicles_list


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
    people = spawn_people(client, world, num_people)

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
    carla_client.set_timeout(10)
    carla_world = carla_client.load_world(town_map)
    carla_world.set_weather(getattr(carla.WeatherParameters, weather))
    tm = carla_client.get_trafficmanager(traffic_manager_port)
    tm.set_global_distance_to_leading_vehicle(2.5)
    tm.global_percentage_speed_difference(30.0)
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

    logging.info(f"Ego vehicle {ego_vehicle}")

    # configuring zenoh
    zconf = zenoh.Config()

    zconf.insert_json5(
        zenoh.config.MODE_KEY, json.dumps(config["challenge_config"]["mode"])
        )
    zconf.insert_json5(
        zenoh.config.CONNECT_KEY, json.dumps(config["challenge_config"]["locators"])
    )

    zsession = zenoh.open(zconf)

    if config["challenge_config"]["enabled"]:

        new_config = config["challenge_config"]

        print("Challenge Mode, adding sensors and Zenoh Publishers")

        # setting world as synchronous
        # (simulator slows down a lot with all sensors)

        print("Challenge Mode, setting simulator as synchronous")
        world_settings = carla_world.get_settings()
        world_settings.synchronous_mode = True
        world_settings.fixed_delta_seconds = 1 / int(config["fps"])
        carla_world.apply_settings(world_settings)
        # counter = 0
        while True:
            time.sleep(float(new_config["sleep_time"]))
            frame_id = carla_world.tick()
            logging.info(f"Ticking the world frame id {frame_id}")

    def on_ctrl_data(sample):
        ctrl = VehicleControl.deserialize(sample.payload)
        carla_ctrl = CarlaVehicleControl()

        carla_ctrl.throttle = ctrl.throttle
        carla_ctrl.steer = ctrl.steer
        carla_ctrl.brake = ctrl.brake
        ego_vehicle.apply_control(carla_ctrl)

        # frame_id = carla_world.tick()
        # print(
        # f"Ticking the world frame id {frame_id} - Control Received {ctrl}"
        # )
        logging.info(f"Control Received {ctrl}")
        # counter += 1

    control_sub = ZenohControl(
        zsession, config["challenge_config"]["control"]["ke"], on_ctrl_data
    )

    while True:
        time.sleep(float(config["challenge_config"]["sleep_time"]))
        if config["challenge_config"]["enabled"]:
            frame_id = carla_world.tick()
            logging.info(f"Ticking the world frame id {frame_id}")

    control_sub.undeclare()


if __name__ == "__main__":
    # logging config
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

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
