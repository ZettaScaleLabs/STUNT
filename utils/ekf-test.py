import array
import numpy as np
import math
import argparse
import json

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

DEFAULT_GRAVITY_VECTOR = (0.0, 0.0, -9.81)
DEFAULT_IMU_F = 0.5
DEFAULT_IMU_W = 0.5
DEFAULT_GNSS = 0.1


class EKF:
    def __init__(self, initial_pose, imu_f, imu_w, gnss, gravity):

        self.gravity = gravity

        self.imu_f = imu_f
        self.imu_w = imu_w
        self.gnss = gnss

        self.__Q = np.identity(6)
        self.__Q[0:3, 0:3] = self.__Q[0:3, 0:3] * DEFAULT_IMU_F
        self.__Q[3:6, 3:6] = self.__Q[3:6, 3:6] * DEFAULT_IMU_W

        self.__F = np.identity(9)

        self.__L = np.zeros([9, 6])
        self.__L[3:9, :] = np.identity(6)

        self.__R_GNSS = np.identity(3) * DEFAULT_GNSS

        self._last_covariance = np.zeros((9, 9))

        self.last_pose_estimate = Pose.from_dict(initial_pose)
        self.last_timestamp = self.last_pose_estimate.localization_time

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

    def compute_pose(self, imu, gnss):
        imu = IMUMeasurement.from_dict(imu)
        gnss_data = GnssMeasurement.from_dict(gnss)

        # initializing the delta_t
        current_ts = max(gnss_data.timestamp, imu.timestamp)
        delta_t = (current_ts - self.last_timestamp) / 1000

        print(f"DeltaT {delta_t}")


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
            last_rotation_estimate.matrix.dot(imu.accelerometer.as_numpy_array())
            + self.gravity
        )

        # Estimate the location.
        location_estimate = (
            last_location_estimate
            + (delta_t * last_velocity_estimate)
            + (((delta_t**2) / 2.0) * accelerometer_data)
        )
        # Estimate the velocity.
        velocity_estimate = last_velocity_estimate + (delta_t * accelerometer_data)

        # Estimate rotation
        rotation_estimate = last_rotation_estimate * Quaternion.from_angular_velocity(
            imu.gyroscope, delta_t
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
            gnss_data.latitude,
            gnss_data.longitude,
            gnss_data.altitude,
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


if __name__ == "__main__":

    # --- Command line argument parsing --- --- --- --- --- ---
    parser = argparse.ArgumentParser(
        prog="prepare-world", description="STUNT world preparation"
    )
    parser.add_argument(
        "--imu",
        "-i",
        dest="imu",
        metavar="FILE",
        type=str,
        required=True,
        help="IMU readings file",
    )
    parser.add_argument(
        "--localization",
        "-l",
        dest="localization",
        metavar="FILE",
        type=str,
        required=True,
        help="Localization readings file",
    )
    parser.add_argument(
        "--gnss",
        "-g",
        dest="gnss",
        metavar="FILE",
        type=str,
        required=True,
        help="GNSS readings file",
    )

    args = parser.parse_args()

    localizations = []
    gnss_readings = []
    imu_readings = []

    with open(args.imu, "r") as f:
        for line in f:
            imu_readings.append(json.loads(line))

    with open(args.gnss, "r") as f:
        for line in f:
            gnss_readings.append(json.loads(line))

    with open(args.localization, "r") as f:
        for line in f:
            localizations.append(json.loads(line))

    ekf = EKF(localizations[0], DEFAULT_IMU_F, DEFAULT_IMU_W, DEFAULT_GNSS, DEFAULT_GRAVITY_VECTOR)

    print("#############################################")
    print(f"EKF initial pose: {ekf.last_pose_estimate}")
    print("#############################################")
    for i in range(0, len(gnss_readings)):
        gnss = gnss_readings[i]
        imu = imu_readings[i]

        ekf.compute_pose(imu, gnss)
        print("#############################################")
        print(f"[{i}] EKF last pose: {ekf.last_pose_estimate}")
        print("#############################################")
        input("Press Enter to continue...")
