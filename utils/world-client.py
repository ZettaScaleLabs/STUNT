import json
import carla
import argparse
import random
import time

from carla import VehicleControl as CarlaVehicleControl

from stunt.types import VehicleControl
from stunt.simulator.sensors import ZenohControl


import zenoh

last_tick = time.time()


def main(config, get_map=None):
    carla_host = config["host"]
    carla_port = config["port"]
    ego_name = config["ego_name"]
    is_sync = config["sync"]
    sleep_time = config["sleep_time"]
    timeout = config["timeout"]

    carla_client = carla.Client(carla_host, carla_port)
    carla_client.set_timeout(timeout)
    carla_world = carla_client.get_world()

    if get_map is not None:
        world_map = carla_world.get_map()
        world_map.save_to_disk(get_map)
        print(f"Map saved to {get_map}")
        exit(0)

    ego_vehicle = None

    # Waiting EGO vehicle
    while ego_vehicle is None:
        time.sleep(1)
        possible_vehicles = carla_world.get_actors().filter("vehicle.*")
        for vehicle in possible_vehicles:
            if vehicle.attributes["role_name"] == ego_name:
                ego_vehicle = vehicle
                break

    if ego_vehicle is None:
        print(f"Unable to find ego vehicle with name {ego_name}")
        exit(-1)

    zconf = zenoh.Config()

    zconf.insert_json5(zenoh.config.MODE_KEY, json.dumps(config["mode"]))
    zconf.insert_json5(
        zenoh.config.CONNECT_KEY, json.dumps(config["locators"])
    )

    zsession = zenoh.open(zconf)

    print(f"Ego vehicle {ego_vehicle}")

    def on_ctrl_data(sample):
        ctrl = VehicleControl.deserialize(sample.payload)
        carla_ctrl = CarlaVehicleControl()

        carla_ctrl.throttle = ctrl.throttle
        carla_ctrl.steer = ctrl.steer
        carla_ctrl.brake = ctrl.brake
        ego_vehicle.apply_control(carla_ctrl)

        frame_id = carla_world.tick()
        print(
            f"Ticking the world frame id {frame_id} - Control Received {ctrl}"
        )
        global last_tick
        last_tick = time.time()
        # print(f"Control Received {ctrl}")
        # counter += 1

    control_sub = ZenohControl(zsession, config["control_ke"], on_ctrl_data)

    if is_sync:

        print("Sync Mode, setting simulator as synchronous")
        world_settings = carla_world.get_settings()
        world_settings.synchronous_mode = True
        world_settings.fixed_delta_seconds = 1 / int(config["fps"])
        carla_world.apply_settings(world_settings)

        # counter = 0
        while True:
            time.sleep(float(sleep_time))
            now = time.time()
            global last_tick
            if now - last_tick > 1:
                frame_id = carla_world.tick()
                print(f"Timeout! Ticking the world frame id {frame_id}")
                last_tick = time.time()
            # frame_id = carla_world.tick()
            # print(f"Ticking the world frame id {frame_id}")

        control_sub.undeclare()

    else:
        while True:
            time.sleep(1)


if __name__ == "__main__":
    # --- Command line argument parsing --- --- --- --- --- ---
    parser = argparse.ArgumentParser(
        prog="world-client", description="STUNT world client"
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
    parser.add_argument(
        "--map",
        "-map",
        dest="map",
        type=str,
        required=False,
        help="Download map",
    )

    args = parser.parse_args()
    with open(args.config, "r") as file:
        data = file.read()
    conf = json.loads(data)

    main(conf, args.map)
