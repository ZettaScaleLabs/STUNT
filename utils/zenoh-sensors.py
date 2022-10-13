import json
import carla
import argparse
import signal
import time


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

done = False


def handler(_signum, _frame):
    global done
    done = True


def main(config):
    carla_host = config["host"]
    carla_port = config["port"]
    ego_vehicle_name = config["ego_name"]
    ego_vehicle = None

    # connecting to simulator

    carla_client = carla.Client(carla_host, carla_port)
    carla_client.set_timeout(10)
    carla_world = carla_client.get_world()

    # Waiting EGO vehicle
    while ego_vehicle is None:
        time.sleep(1)
        possible_vehicles = carla_world.get_actors().filter("vehicle.*")
        for vehicle in possible_vehicles:
            if vehicle.attributes["role_name"] == ego_vehicle_name:
                ego_vehicle = vehicle

    print(f"Ego vehicle {ego_vehicle}")

    print("Adding sensors and Zenoh Publishers")

    # configuring zenoh
    zconf = zenoh.Config()

    zconf.insert_json5(zenoh.config.MODE_KEY, json.dumps(config["mode"]))
    zconf.insert_json5(
        zenoh.config.CONNECT_KEY, json.dumps(config["locators"])
    )

    zsession = zenoh.open(zconf)

    # configuring and starting sensors
    imu_config = config["imu"]
    imu_config["host"] = config["host"]
    imu_config["port"] = config["port"]
    imu_pub = ZenohSensor(
        zsession,
        imu_config["ke"],
        IMUSensor,
        IMUMeasurement,
        imu_config,
    )

    gnss_config = config["gnss"]
    gnss_config["host"] = config["host"]
    gnss_config["port"] = config["port"]

    gnss_pub = ZenohSensor(
        zsession,
        gnss_config["ke"],
        GNSSSensor,
        GnssMeasurement,
        gnss_config,
    )

    center_camera_config = config["center_camera"]
    center_camera_config["host"] = config["host"]
    center_camera_config["port"] = config["port"]

    center_camera_pub = ZenohSensor(
        zsession,
        center_camera_config["ke"],
        CameraSensor,
        Image,
        center_camera_config,
    )

    tele_camera_config = config["tele_camera"]
    tele_camera_config["host"] = config["host"]
    tele_camera_config["port"] = config["port"]

    tele_camera_pub = ZenohSensor(
        zsession,
        tele_camera_config["ke"],
        CameraSensor,
        Image,
        tele_camera_config,
    )

    center_lidar_config = config["center_lidar"]
    center_lidar_config["host"] = config["host"]
    center_lidar_config["port"] = config["port"]

    center_lidar_pub = ZenohSensor(
        zsession,
        center_lidar_config["ke"],
        LidarSensor,
        LidarMeasurement,
        center_lidar_config,
    )

    tele_lidar_config = config["tele_lidar"]
    tele_lidar_config["host"] = config["host"]
    tele_lidar_config["port"] = config["port"]

    tele_lidar_pub = ZenohSensor(
        zsession,
        tele_lidar_config["ke"],
        LidarSensor,
        LidarMeasurement,
        tele_lidar_config,
    )

    obstacles_config = config["obstacles"]
    obstacles_config["host"] = config["host"]
    obstacles_config["port"] = config["port"]

    obstacles_pub = ZenohSensor(
        zsession,
        obstacles_config["ke"],
        Obstacles,
        SimulatorObstacle,
        obstacles_config,
    )

    traffic_lights_config = config["traffic_lights"]
    traffic_lights_config["host"] = config["host"]
    traffic_lights_config["port"] = config["port"]

    traffic_lights_pub = ZenohSensor(
        zsession,
        traffic_lights_config["ke"],
        TrafficLights,
        TrafficLight,
        traffic_lights_config,
    )

    location_config = config["location"]
    location_config["host"] = config["host"]
    location_config["port"] = config["port"]

    location_pub = ZenohSensor(
        zsession,
        location_config["ke"],
        Localization,
        Pose,
        location_config,
    )

    signal.signal(signal.SIGINT, handler)

    print("Sensor are ready!")
    while not done:
        time.sleep(1)
        # carla_world.tick()

    location_pub.undeclare()
    traffic_lights_pub.undeclare()
    obstacles_pub.undeclare()
    tele_lidar_pub.undeclare()
    center_lidar_pub.undeclare()
    tele_camera_pub.undeclare()
    center_camera_pub.undeclare()
    gnss_pub.undeclare()
    imu_pub.undeclare()


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
