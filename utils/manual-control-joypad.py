from stunt.types import VehicleControl
import pygame
import zenoh
import json
import signal
import argparse
import time

def handler(_signum, _frame):
    global done
    done = True


def convert_range_with_threshold(x, threshold=0.05):
    """
    Converts a range from -1 to 1 into a range from 0 to 1, while applying a threshold
    to map values that are too small to 0 and values that are too large to 1.

    Args:
    x (float): The input value to be converted.
    threshold (float): The threshold value. Values less than or equal to this value will be
                       mapped to 0, and values greater than or equal to 1 - threshold will be
                       mapped to 1. Defaults to 0.001.

    Returns:
    float: The converted value.
    """
    if x <= -1:
        return 0
    elif x >= 1:
        return 1
    elif abs(x) <= threshold:
        return 0
    elif abs(x - 1) <= threshold:
        return 1
    else:
        return (x + 1) / 2


done = False

def main(config):
    pygame.init()
    clock = pygame.time.Clock()

    print("Getting Joypad")

    if pygame.joystick.get_count() == 0:
        print('No joypad found, exiting!')
        return None

    # getting the first joypad
    joypad = pygame.joystick.Joystick(0)
    joypad.init()
    print(f'Using joypad: {joypad.get_name()}')

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

    print("Sensor are ready!")
    while not done:
        # time.sleep(config["sleep_time"])


        clock.tick(60)
        # reading the joypad events
        for event in pygame.event.get():
            # print(f'Event {event}')
            if event.type == pygame.QUIT:
                exit(0)
            if event.type == pygame.JOYAXISMOTION:
                if event.__dict__.get('axis') == 5: #RT
                    # this is the throttle
                    ctrl_data.throttle = convert_range_with_threshold(float(event.__dict__.get('value')))
                    pass
                elif event.__dict__.get('axis') == 2: #LT
                    # this is the brake
                    ctrl_data.brake = convert_range_with_threshold(event.__dict__.get('value'))
                    pass
                elif event.__dict__.get('axis') == 0: #left stick
                    # this is the steering wheel
                    ctrl_data.steer = float(event.__dict__.get('value'))
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.__dict__.get('button') == 3: #Y
                    # this toggles the reverse
                    if ctrl_data.reverse:
                        ctrl_data.reverse = not ctrl_data.reverse
                        ctrl_data.gear = 1
                    else:
                        ctrl_data.reverse = not ctrl_data.reverse
                        ctrl_data.gear = -1

                elif event.__dict__.get('button') == 2: #X
                    # toggles shifting
                    ctrl_data.manual_gear_shift = not ctrl_data.manual_gear_shift
                elif event.__dict__.get('button') == 1: #B
                    # toggles handbrake
                    ctrl_data.hand_brake = not ctrl_data.hand_brake
                elif event.__dict__.get('button') == 4: # LB
                    if ctrl_data.gear > -1:
                        ctrl_data.gear = ctrl_data.gear - 1
                elif event.__dict__.get('button') == 5: #RB
                    if ctrl_data.gear < 5:
                        ctrl_data.gear = ctrl_data.gear + 1
                elif event.__dict__.get('button') == 7: #menu
                    exit(0)


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
