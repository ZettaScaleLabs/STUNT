from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from zenoh_flow.types import Context
from typing import Dict, Any, Callable


import time
import asyncio

import math
import json
import numpy as np
from collections import deque

from stunt.types import Waypoints, Pose, VehicleControl


DEFAULT_P_PARAMETER = 1
DEFAULT_I_PARAMETER = 0
DEFAULT_D_PARAMETER = 0
DEFAULT_THROTTLE_MAX = 1.0
DEFAULT_STEER_GAIN = 1.0
DEFAULT_BRAKE_MAX = 10.0
DEFAULT_MIN_PID_WAYPOINT_SPEED = 5
DEFAULT_MIN_PID_WAYPOINT_STEER = 5
PID_DT_PARAMETER = 0.3


class PIDController(Operator):
    def __init__(
        self,
        context,
        configuration,
        inputs,
        outputs,
    ):
        configuration = configuration if configuration is not None else {}

        self.pending = []

        self.p = configuration.get("P", DEFAULT_P_PARAMETER)
        self.i = configuration.get("I", DEFAULT_I_PARAMETER)
        self.d = configuration.get("D", DEFAULT_D_PARAMETER)
        self.dt = configuration.get("DT", PID_DT_PARAMETER)

        self.throttle_max = configuration.get("throttle_max", DEFAULT_THROTTLE_MAX)
        self.steer_gain = configuration.get("steer_gain", DEFAULT_STEER_GAIN)
        self.brake_max = configuration.get("brake_max", DEFAULT_BRAKE_MAX)

        self.min_pid_steer_waypoint_distance = configuration.get(
            "min_pid_steer_waypoint_distance", DEFAULT_MIN_PID_WAYPOINT_STEER
        )
        self.min_pid_speed_waypoint_distance = configuration.get(
            "min_pid_speed_waypoint_distance", DEFAULT_MIN_PID_WAYPOINT_SPEED
        )

        self.last_time = time.time()

        self.error_buffer = deque(maxlen=10)

        self.waypoints_input = inputs.get("Waypoints", None)
        self.pose_input = inputs.get("Pose", None)
        self.output = outputs.get("Control", None)

        self.waypoints = Waypoints([])

        # Stop the car by default
        self.control = VehicleControl(brake=1.0)

    def run_step(self, target_speed: float, current_speed: float):
        """Computes the throttle/brake based on the PID equations.
        Args:
            target_speed (:obj:`float`): Target speed in m/s.
            current_speed (:obj:`float`): Current speed in m/s.
        Returns:
            Throttle and brake values.
        """
        # Transform to km/h
        error = (target_speed - current_speed) * 3.6
        self.error_buffer.append(error)

        time_now = time.time()
        dt = time_now - self.last_time
        self.last_time = time_now

        if len(self.error_buffer) >= 2:
            de = (self.error_buffer[-1] - self.error_buffer[-2]) / dt
            ie = sum(self.error_buffer) * dt
        else:
            de = 0.0
            ie = 0.0

        return np.clip((self.p * error) + (self.d * de) + (self.i * ie), -1.0, 1.0)

    def radians_to_steer(self, rad: float):
        """Converts radians to steer input.

        Returns:
            :obj:`float`: Between [-1.0, 1.0].
        """
        steer = self.steer_gain * rad
        if steer > 0:
            steer = min(steer, 1)
        else:
            steer = max(steer, -1)
        return steer

    def compute_throttle_and_brake(self, current_speed: float, target_speed: float):
        """Computes the throttle/brake required to reach the target speed.

        It uses the longitudinal controller to derive the required information.

        Args:
            self: The pid controller.
            current_speed (:obj:`float`): The current speed of the ego vehicle
                (in m/s).
            target_speed (:obj:`float`): The target speed to reach (in m/s).
            flags (absl.flags): The flags object.

        Returns:
            Throttle and brake values.
        """
        if current_speed < 0:
            non_negative_speed = 0
        else:
            non_negative_speed = current_speed
        acceleration = self.run_step(target_speed, non_negative_speed)
        if acceleration >= 0.0:
            throttle = min(acceleration, self.throttle_max)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(abs(acceleration), self.brake_max)
        # Keep the brake pressed when stopped or when sliding back on a hill.
        if (current_speed < 1 and target_speed == 0) or current_speed < -0.3:
            brake = 1.0
        return throttle, brake

    async def wait_pose(self):
        data_msg = await self.pose_input.recv()
        return ("Pose", data_msg)

    async def wait_waypoints(self):
        data_msg = await self.waypoints_input.recv()
        return ("Waypoints", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "Pose" for t in task_list):
            task_list.append(asyncio.create_task(self.wait_pose(), name="Pose"))

        if not any(t.get_name() == "Waypoints" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_waypoints(), name="Waypoints")
            )
        return task_list

    async def iteration(self):

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )

        self.pending = list(pending)
        for d in done:

            (who, data_msg) = d.result()

            if who == "Waypoints":
                self.waypoints = Waypoints.deserialize(data_msg.data)

            elif who == "Pose":
                pose = Pose.deserialize(data_msg.data)
                ego_transform = pose.transform
                current_speed = pose.forward_speed

                try:
                    angle_steer = self.waypoints.get_angle(
                        ego_transform, self.min_pid_steer_waypoint_distance
                    )
                    target_speed = self.waypoints.get_target_speed(
                        ego_transform, self.min_pid_speed_waypoint_distance
                    )

                    throttle, brake = self.compute_throttle_and_brake(
                        current_speed, target_speed
                    )

                    steer = self.radians_to_steer(angle_steer)

                    ctrl_data = VehicleControl(throttle, steer, brake)

                    await self.output.send(ctrl_data.serialize())

                except Exception:
                    await self.output.send(self.control.serialize())

            return None

    def finalize(self) -> None:
        return None


def register():
    return PIDController
