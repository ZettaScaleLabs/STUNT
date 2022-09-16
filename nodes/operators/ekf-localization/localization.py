from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from typing import Dict, Any, Callable
import time
import asyncio

import json
import carla
import array
import numpy as np
import math

from stunt.types import (
    GnssMeasurement,
    IMUMeasurement,
    Transform,
    Vector3D,
    Pose,
    Quaternion,
    Location,
    Rotation,
)

from stunt import DEFAULT_CARLA_HOST, DEFAULT_CARLA_PORT

DEFAULT_GRAVITY_VECTOR = (0.0, 0.0, -9.81)
DEFAULT_IMU_F = 0.5
DEFAULT_IMU_W = 0.5
DEFAULT_GNSS = 0.1
S_TO_MS = 1000


class Localization(Operator):
    def __init__(self, configuration, gnss_input, imu_input, output):

        self.pending = []

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
        self.current_pose = None
        self.gnss_data = None
        self.gnss_timestamp = None

        # Kalmap filter parameters and configuration

        self.__Q = np.identity(6)
        self.__Q[0:3, 0:3] = self.__Q[0:3, 0:3] * self.imu_f
        self.__Q[3:6, 3:6] = self.__Q[3:6, 3:6] * self.imu_w

        self.__F = np.identity(9)

        self.__L = np.zeros([9, 6])
        self.__L[3:9, :] = np.identity(6)

        self.__R_GNSS = np.identity(3) * self.gnss

        self._last_covariance = np.zeros((9, 9))

        self.gnss_input = gnss_input
        self.imu_input = imu_input
        self.output = output

        # Should get the pose from the simulator at setup
        if configuration is not None and configuration.get("port") is not None:
            self.carla_port = int(configuration["port"])

        if configuration is not None and configuration.get("host") is not None:
            self.carla_host = configuration["host"]

        # Connecting to CARLA
        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        found = False
        # Waiting EGO vehicle
        while found is False:
            time.sleep(1)
            possible_vehicles = self.carla_world.get_actors().filter("vehicle.*")
            for vehicle in possible_vehicles:
                if vehicle.attributes["role_name"] == "hero":
                    found = True
                    vec_transform = Transform.from_simulator_transform(
                        vehicle.get_transform()
                    )
                    velocity_vector = Vector3D.from_simulator_vector(
                        vehicle.get_velocity()
                    )

                    forward_speed = np.linalg.norm(
                        np.array(
                            [velocity_vector.x, velocity_vector.y, velocity_vector.z]
                        )
                    )

                    self.last_pose_estimate = Pose(
                        vec_transform,
                        forward_speed,
                        velocity_vector,
                        time.time() * S_TO_MS,
                    )
                    self.last_timestamp = self.last_pose_estimate.localization_time

        # drop any connection to the simulator
        self.carla_world = None
        self.carla_client = None

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

    async def wait_gnss(self):
        data_msg = await self.gnss_input.recv()
        return ("GNSS", data_msg)

    async def wait_imu(self):
        data_msg = await self.imu_input.recv()
        return ("IMU", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "GNSS" for t in task_list):
            task_list.append(asyncio.create_task(self.wait_gnss(), name="GNSS"))

        if not any(t.get_name() == "IMU" for t in task_list):
            task_list.append(asyncio.create_task(self.wait_imu(), name="IMU"))
        return task_list

    async def run(self):

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )

        self.pending = list(pending)

        for d in done:
            (who, data_msg) = d.result()

            # We compute on IMU
            if who == "GNSS":
                self.gnss_data = GnssMeasurement.deserialize(data_msg.data)
                self.gnss_timestamp = self.gnss_data.timestamp

            elif who == "IMU":

                # skip if we are still waiting for first GPS fix
                if self.gnss_data is None:
                    return None

                imu = IMUMeasurement.deserialize(data_msg.data)

                # initializing the delta_t
                current_ts = max(self.gnss_timestamp, imu.timestamp)
                delta_t = (current_ts - self.last_timestamp) / 1000

                # retreiving last estimations
                last_rotation_estimate = Quaternion.from_rotation(
                    self.last_pose_estimate.transform.rotation
                )
                last_location_estimate = (
                    self.last_pose_estimate.transform.location.as_numpy_array()
                )
                last_velocity_estimate = (
                    self.last_pose_estimate.velocity_vector.as_numpy_array()
                )

                # Transform the IMU accelerometer data from the body frame to the
                # world frame, and retrieve location and velocity estimates.
                accelerometer_data = (
                    last_rotation_estimate.matrix.dot(
                        imu.accelerometer.as_numpy_array()
                    )
                    + self.gravity
                )

                # Estimate the location.
                location_estimate = (
                    last_location_estimate
                    + (delta_t * last_velocity_estimate)
                    + (((delta_t**2) / 2.0) * accelerometer_data)
                )
                # Estimate the velocity.
                velocity_estimate = last_velocity_estimate + (
                    delta_t * accelerometer_data
                )

                # Estimate rotation
                rotation_estimate = (
                    last_rotation_estimate
                    * Quaternion.from_angular_velocity(imu.gyroscope, delta_t)
                )

                # Fuse the GNSS values using an EKF to fix drifts and noise in
                # the estimates.

                # Linearize the motion model and compute Jacobians.

                self.__F[0:3, 3:6] = np.identity(3) * delta_t

                self.__F[3:6, 6:9] = (
                    last_rotation_estimate.matrix.dot(
                        -self.skew_symmetric(accelerometer_data.reshape((3, 1)))
                    )
                    * delta_t
                )

                # Fix estimates using GNSS
                gnss_reading = Location.from_gps(
                    self.gnss_data.latitude,
                    self.gnss_data.longitude,
                    self.gnss_data.altitude,
                ).as_numpy_array()

                (
                    location_estimate,
                    velocity_estimate,
                    rotation_estimate,
                ) = self.update_from_gnss(
                    location_estimate,
                    velocity_estimate,
                    rotation_estimate,
                    gnss_reading,
                    delta_t,
                )

                # Store the pose and send it downstream
                self.last_pose_estimate = Pose(
                    transform=Transform(
                        location=Location(*location_estimate),
                        rotation=rotation_estimate.as_Rotation(),
                    ),
                    forward_speed=Vector3D(*velocity_estimate).magnitude(),
                    velocity_vector=Vector3D(*velocity_estimate),
                    localization_time=current_ts,
                )
                self.last_timestamp = current_ts
                await self.output.send(self.last_pose_estimate.serialize())

        return None

    def setup(
        self,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ) -> Callable[[], Any]:

        gnss_input = inputs.get("GNSS", None)
        imu_input = inputs.get("IMU", None)
        output = outputs.get("Pose", None)

        l = Localization(configuration, gnss_input, imu_input, output)

        return l.run

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
