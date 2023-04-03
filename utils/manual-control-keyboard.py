from stunt.types import VehicleControl
import zenoh
import json
import signal
import argparse
import time
import curses

def handler(_signum, _frame):
    global done
    done = True


done = False

def main(config,stdscr):

    print('Configuring Zenoh')

    # configuring zenoh
    zconf = zenoh.Config()

    zconf.insert_json5(zenoh.config.MODE_KEY, json.dumps(config["mode"]))
    zconf.insert_json5(
        zenoh.config.CONNECT_KEY, json.dumps(config["locators"])
    )

    zsession = zenoh.open(zconf)

    pub = zsession.declare_publisher(config["control_ke"])

    signal.signal(signal.SIGINT, handler)

    # car is stopped when starting
    ctrl_data = VehicleControl(0, 0, 1.0)
    pub.put(ctrl_data.serialize())

    print("Control is ready!")
    while not done:
        time.sleep(config["sleep_time"])

        c = stdscr.getch()
        if c == curses.KEY_UP:
            ctrl_data.throttle = 1.0
            ctrl_data.brake = 0
        elif c == curses.KEY_DOWN:
            ctrl_data.brake = 1.0
            ctrl_data.throttle = 0
        elif c == curses.KEY_LEFT:
            ctrl_data.steer = -1.0
        elif c == curses.KEY_RIGHT:
            ctrl_data.steer = 1.0
        elif c == 32:
            # space
            ctrl_data.hand_brake = not ctrl_data.hand_brake
        elif c == ord('r'):
            if ctrl_data.reverse:
                ctrl_data.reverse = not ctrl_data.reverse
                ctrl_data.gear = 1
            else:
                ctrl_data.reverse = not ctrl_data.reverse
                ctrl_data.gear = -1
        elif c == 27 or c == ord('q'):
            exit(0)
            # global done
            # done = True

            # RT axis 5
            # LT axis 2
            # Y button 3
            # LB button 4
            # RB button 5
            # left stick axis 0
            # buger menu button 7

        # sending the command to the car

        print(f'Sending to car {ctrl_data}')
        pub.put(ctrl_data.serialize())


def wrapper_main(stdscr):

    stdscr.refresh()

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
    main(conf,stdscr)


if __name__ == "__main__":


    curses.wrapper(wrapper_main)


