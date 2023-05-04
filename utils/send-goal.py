import json
import carla
import argparse
import signal
import time

from threading import Thread

from stunt.types import Location


import zenoh

done = False
DEFAULT_LOCATION_GOAL = (387.73, 330.07, 0.0)


def handler(_signum, _frame):
    global done
    done = True


def main(config):
    # configuring zenoh
    zconf = zenoh.Config()

    zconf.insert_json5(zenoh.config.MODE_KEY, json.dumps(config["mode"]))
    zconf.insert_json5(
        zenoh.config.CONNECT_KEY, json.dumps(config["locators"])
    )

    zsession = zenoh.open(zconf)

    print("Sensor are ready!")
    while not done:
        # time.sleep(1)
        input("Press Enter to send goal")
        location = Location(*DEFAULT_LOCATION_GOAL)
        zsession.put("stunt/ego-control/destination", location.serialize())
        # carla_world.tick()

    exit(0)


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
