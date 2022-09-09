from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from typing import Dict, Any, Callable
import time
import asyncio

import math
import json
import carla
import array
import numpy as np


DEFAULT_GRAVITY_VECTOR = (0.0, 0.0, -9.81)
DEFAULT_IMU_F = 0.5
DEFAULT_IMU_W = 0.5
DEFAULT_GNSS = 0.1


class LocalizationState:
    def __init__(self, configuration):

        self.gravity = np.array(configuration.get("gravity", DEFAULT_GRAVITY_VECTOR))
        self.imu_f = configuration.get("kalman", DEFAULT_IMU_F).get(
            "imu_f", DEFAULT_IMU_F
        )
        self.imu_w = configuration.get("kalman", DEFAULT_IMU_F).get(
            "imu_w", DEFAULT_IMU_W
        )
        self.gnss = configuration.get("kalman", DEFAULT_GNSS).get("gnss", DEFAULT_GNSS)

        self.pose = None

        self.gnss_delta_t = 0.0
        self.imu_delta_t = 0.0
        self.gnss_last_ts = 0.0
        self.imu_last_ts = 0.0

        self.last_pose_estimate = None
        self.last_timestamp = None

        # Kalmap filter parameters and configuration

        self.__Q = np.identity(6)
        self.__Q[0:3, 0:3] = self.__Q[0:3, 0:3] * self.imu_f
        self.__Q[3:6, 3:6] = self.__Q[3:6, 3:6] * self.imu_w

        self.__F = np.identity(9)

        self.__L = np.zeros([9, 6])
        self.__L[3:9, :] = np.identity(6)

        self.__R_GNSS = np.identity(3) * self.gnss

        self._last_covariance = np.zeros((9, 9))

    def skew_symmetric(self, v):
        """Skew symmetric form of a 3x1 vector."""
        return np.array(
            [[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], dtype=np.float64
        )

    def update_from_gnss(
        self,
        location_estimate,
        velocity_estimate,
        rotation_estimate,
        gnss_reading,
        delta_t,
    ):
        # Construct H_k = [I, 0, 0] (shape=(3, 9))
        H_k = np.zeros((3, 9))
        H_k[:, :3] = np.identity(3)

        # Propogate uncertainty.
        Q = self.__Q * delta_t * delta_t
        self._last_covariance = (
            self.__F.dot(self._last_covariance).dot(self.__F.T)
        ) + (self.__L.dot(Q).dot(self.__L.T))

        # Compute Kalman gain. (shape=(9, 3))
        K_k = self._last_covariance.dot(
            H_k.T.dot(
                np.linalg.inv(H_k.dot(self._last_covariance.dot(H_k.T)) + self.__R_GNSS)
            )
        )

        # Compute error state. (9x3) x ((3x1) - (3x1)) = shape(9, 1)
        delta_x_k = K_k.dot(gnss_reading - location_estimate)

        # Correct predicted state.
        corrected_location_estimate = location_estimate + delta_x_k[0:3]
        corrected_velocity_estimate = velocity_estimate + delta_x_k[3:6]
        roll, pitch, yaw = delta_x_k[6:]
        corrected_rotation_estimate = (
            Quaternion.from_rotation(Rotation(roll=roll, pitch=pitch, yaw=yaw))
            * rotation_estimate
        )

        # Fix the covariance.
        self._last_covariance = (np.identity(9) - K_k.dot(H_k)).dot(
            self._last_covariance
        )

        return (
            corrected_location_estimate,
            corrected_velocity_estimate,
            corrected_rotation_estimate,
        )

    def on_sensor_update(self, data):
        self.pose = {
            "transform": {
                "location": (
                    vec_transform.location.x,
                    vec_transform.location.y,
                    vec_transform.location.z,
                ),
                "rotation": (
                    vec_transform.rotation.pitch,
                    vec_transform.rotation.yaw,
                    vec_transform.rotation.roll,
                ),
            },
            "forward_speed": forward_speed,
            "velocity_vector": (
                velocity_vector.x,
                velocity_vector.y,
                velocity_vector.z,
            ),
            "localization_time": snapshot.timestamp.elapsed_seconds * S_TO_MS,
        }


class Localization(Operator):
    def setup(
        self,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ) -> Callable[[], Any]:
        state = CarlaCameraSrcState(configuration)

        gnss_input = intpus.get("GNSS", None)
        imu_input = inputs.get("IMU", None)
        pose_input = inputs.get("Pose", None)

        output = outputs.get("Pose", None)

        return lambda: run(gnss_input, imu_input, pose_input, output, state)

    def finalize(self) -> None:
        return None


async def run(gnss_input, imu_input, pose_input, output, state):

    # wait for the inputs

    # await asyncio.sleep(state.period)

    if state.pose is not None:
        await self.output.send(json.dumps(state.pose).encode("utf-8"))

    return None


def register():
    return Localization
