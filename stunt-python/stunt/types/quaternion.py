import json
import numpy as np
from stunt.types import Rotation


class Quaternion(object):
    """Represents the Rotation of an obstacle or vehicle in quaternion
    notation.

    Args:
        w: The real-part of the quaternion.
        x: The x-part (i) of the quaternion.
        y: The y-part (j) of the quaternion.
        z: The z-part (k) of the quaternion.

    Attributes:
        w: The real-part of the quaternion.
        x: The x-part (i) of the quaternion.
        y: The y-part (j) of the quaternion.
        z: The z-part (k) of the quaternion.
        matrix: A 3x3 numpy array that can be used to rotate 3D vectors from
            body frame to world frame.
    """

    def __init__(self, w: float = 0, x: float = 0, y: float = 0, z: float = 0):
        norm = np.linalg.norm([w, x, y, z])
        if norm < 1e-50:
            self.w, self.x, self.y, self.z = 0, 0, 0, 0
        else:
            self.w = w / norm
            self.x = x / norm
            self.y = y / norm
            self.z = z / norm
        self.matrix = Quaternion._create_matrix(self.w, self.x, self.y, self.z)

    @staticmethod
    def _create_matrix(w, x, y, z):
        """Creates a Rotation matrix that can be used to transform 3D vectors
        from body frame to world frame.

        Note that this yields the same matrix as a Transform object with the
        quaternion converted to the Euler rotation except this matrix only does
        rotation and no translation.

        Specifically, this matrix is equivalent to:
            Transform(location=Location(0, 0, 0),
                      rotation=self.as_rotation()).matrix[:3, :3]

        Returns:
            A 3x3 numpy array that can be used to rotate 3D vectors from body
            frame to world frame.
        """
        x2, y2, z2 = x * 2, y * 2, z * 2
        xx, xy, xz = x * x2, x * y2, x * z2
        yy, yz, zz = y * y2, y * z2, z * z2
        wx, wy, wz = w * x2, w * y2, w * z2
        m = np.array(
            [
                [1.0 - (yy + zz), xy - wz, xz + wy],
                [xy + wz, 1.0 - (xx + zz), yz - wx],
                [xz - wy, yz + wx, 1.0 - (xx + yy)],
            ]
        )
        return m

    @classmethod
    def from_rotation(cls, rotation: Rotation):
        """Creates a Quaternion from a rotation including pitch, roll, yaw.

        Args:
            rotation (:py:class:`.Rotation`): A STUNT rotation representing
                the rotation of the object in degrees.

        Returns:
            :py:class:`.Quaternion`: The quaternion representation of the
            rotation.
        """
        roll_by_2 = np.radians(rotation.roll) / 2.0
        pitch_by_2 = np.radians(rotation.pitch) / 2.0
        yaw_by_2 = np.radians(rotation.yaw) / 2.0

        cr, sr = np.cos(roll_by_2), np.sin(roll_by_2)
        cp, sp = np.cos(pitch_by_2), np.sin(pitch_by_2)
        cy, sy = np.cos(yaw_by_2), np.sin(yaw_by_2)

        w = cr * cp * cy + sr * sp * sy
        x = cr * sp * sy - sr * cp * cy
        y = -cr * sp * cy - sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return cls(w, x, y, z)

    @classmethod
    def from_rotation_array(cls, rotation: Rotation):
        """Creates a Quaternion from a rotation including pitch, roll, yaw.

        Args:
            rotation (:py:class:`.Rotation`): A STUNT rotation representing
                the rotation of the object in degrees.

        Returns:
            :py:class:`.Quaternion`: The quaternion representation of the
            rotation.
        """

        w = rotation[0]
        x = rotation[1]
        y = rotation[2]
        z = rotation[3]

        return cls(w, x, y, z)

    @classmethod
    def from_angular_velocity(cls, angular_velocity, dt: float):
        """Creates a Quaternion from an angular velocity vector and the time
        delta to apply it for.

        Args:
            angular_velocity (:py:class:`.Vector3D`): The vector representing
                the angular velocity of the object in the body-frame.
            dt (float): The time delta to apply the angular velocity for.

        Returns:
            :py:class:`.Quaternion`: The quaternion representing the rotation
                undergone by the object with the given angular velocity in the
                given delta time.
        """
        angular_velocity_np = angular_velocity.as_numpy_array() * dt
        magnitude = np.linalg.norm(angular_velocity_np)

        w = np.cos(magnitude / 2.0)
        if magnitude < 1e-50:
            # To avoid instabilities and nan.
            x, y, z = 0, 0, 0
        else:
            imaginary = (
                angular_velocity_np / magnitude * np.sin(magnitude / 2.0)
            )
            x, y, z = imaginary
        return cls(w, x, y, z)

    def as_rotation(self):
        """Retrieve the Quaternion as a Rotation in degrees.

        Returns:
            :py:class:`.Rotation`: The euler-angle equivalent of the
                Quaternion in degrees.
        """
        SINGULARITY_THRESHOLD = 0.4999995
        RAD_TO_DEG = (180.0) / np.pi

        singularity_test = self.z * self.x - self.w * self.y
        yaw_y = 2.0 * (self.w * self.z + self.x * self.y)
        yaw_x = 1.0 - 2.0 * (self.y**2 + self.z**2)

        pitch, yaw, roll = None, None, None
        if singularity_test < -SINGULARITY_THRESHOLD:
            pitch = -90.0
            yaw = np.arctan2(yaw_y, yaw_x) * RAD_TO_DEG
            roll = -yaw - (2.0 * np.arctan2(self.x, self.w) * RAD_TO_DEG)
        elif singularity_test > SINGULARITY_THRESHOLD:
            pitch = 90.0
            yaw = np.arctan2(yaw_y, yaw_x) * RAD_TO_DEG
            roll = yaw - (2.0 * np.arctan2(self.x, self.w) * RAD_TO_DEG)
        else:
            pitch = np.arcsin(2.0 * singularity_test) * RAD_TO_DEG
            yaw = np.arctan2(yaw_y, yaw_x) * RAD_TO_DEG
            roll = (
                np.arctan2(
                    -2.0 * (self.w * self.x + self.y * self.z),
                    (1.0 - 2.0 * (self.x**2 + self.y**2)),
                )
                * RAD_TO_DEG
            )
        return np.array([pitch, yaw, roll])

    def as_Rotation(self) -> Rotation:
        """Retrieve the Quaternion as a Rotation in degrees.

        Returns:
            :py:class:`.Rotation`: The euler-angle equivalent of the
                Quaternion in degrees.
        """
        SINGULARITY_THRESHOLD = 0.4999995
        RAD_TO_DEG = (180.0) / np.pi

        singularity_test = self.z * self.x - self.w * self.y
        yaw_y = 2.0 * (self.w * self.z + self.x * self.y)
        yaw_x = 1.0 - 2.0 * (self.y**2 + self.z**2)

        pitch, yaw, roll = None, None, None
        if singularity_test < -SINGULARITY_THRESHOLD:
            pitch = -90.0
            yaw = np.arctan2(yaw_y, yaw_x) * RAD_TO_DEG
            roll = -yaw - (2.0 * np.arctan2(self.x, self.w) * RAD_TO_DEG)
        elif singularity_test > SINGULARITY_THRESHOLD:
            pitch = 90.0
            yaw = np.arctan2(yaw_y, yaw_x) * RAD_TO_DEG
            roll = yaw - (2.0 * np.arctan2(self.x, self.w) * RAD_TO_DEG)
        else:
            pitch = np.arcsin(2.0 * singularity_test) * RAD_TO_DEG
            yaw = np.arctan2(yaw_y, yaw_x) * RAD_TO_DEG
            roll = (
                np.arctan2(
                    -2.0 * (self.w * self.x + self.y * self.z),
                    (1.0 - 2.0 * (self.x**2 + self.y**2)),
                )
                * RAD_TO_DEG
            )
        return Rotation(pitch, yaw, roll)

    def __mul__(self, other):
        """Returns the product self * other.  The product is NOT commutative.

        The product is defined in Unreal as:
         [ (Q2.w * Q1.x) + (Q2.x * Q1.w) + (Q2.y * Q1.z) - (Q2.z * Q1.y),
           (Q2.w * Q1.y) - (Q2.x * Q1.z) + (Q2.y * Q1.w) + (Q2.z * Q1.x),
           (Q2.w * Q1.z) + (Q2.x * Q1.y) - (Q2.y * Q1.x) + (Q2.z * Q1.w),
           (Q2.w * Q1.w) - (Q2.x * Q1.x) - (Q2.y * Q1.y) - (Q2.z * Q1.z) ]
        Copied from DirectX's XMQuaternionMultiply function.
        """
        q1, q2 = other, self
        x = (q2.w * q1.x) + (q2.x * q1.w) + (q2.y * q1.z) - (q2.z * q1.y)
        y = (q2.w * q1.y) - (q2.x * q1.z) + (q2.y * q1.w) + (q2.z * q1.x)
        z = (q2.w * q1.z) + (q2.x * q1.y) - (q2.y * q1.x) + (q2.z * q1.w)
        w = (q2.w * q1.w) - (q2.x * q1.x) - (q2.y * q1.y) - (q2.z * q1.z)
        return Quaternion(w, x, y, z)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Quaternion (w={}, x={}, y={}, z={})".format(
            self.w, self.x, self.y, self.z
        )

    def to_dict(self):
        return {"x": self.x, "y": self.y, "z": self.z, "w": self.w}

    @classmethod
    def from_dict(cls, dictionary):
        return cls(
            dictionary["w"], dictionary["x"], dictionary["y"], dictionary["z"]
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
