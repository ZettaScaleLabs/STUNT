import json
from carla import VehicleControl as CarlaVehicleControl


class VehicleControl(object):
    def __init__(
        self,
        throttle=0.0,
        steer=0.0,
        brake=0.0,
        hand_brake=False,
        reverse=False,
        manual_gear_shift=False,
        gear=0,
    ):
        self.throttle = throttle
        self.steer = steer
        self.brake = brake
        self.hand_brake = hand_brake
        self.reverse = reverse
        self.manual_gear_shift = manual_gear_shift
        self.gear = gear

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"VehicleControl(throttle={self.throttle}, steer={self.steer}, brake={self.brake}, hand_brake={self.hand_brake}, reverse={self.reverse}, manual_gear_shift={self.manual_gear_shift}, gear={self.gear})"

    @classmethod
    def from_simulator(cls, control):
        """Creates a STUNT control from a simulator control.

        Args:
            control: An instance of a simulator control.

        Returns:
            :py:class:`.control`: A STUNT control.
        """

        if not isinstance(control, CarlaVehicleControl):
            raise ValueError("The control must be a Location or CarlaVehicleControl")
        return cls(
            control.throttle,
            control.steer,
            control.brake,
            control.hand_brake,
            control.reverse,
            control.manual_gear_shift,
            control.gear,
        )

    def to_simulator(self):
        return CarlaVehicleControl(
            self.throttle,
            self.steer,
            self.brake,
            self.hand_brake,
            self.reverse,
            self.manual_gear_shift,
            self.gear,
        )

    def to_dict(self):
        return {
            "throttle": self.throttle,
            "steer": self.steer,
            "brake": self.brake,
            "hand_brake": self.hand_brake,
            "reverse": self.reverse,
            "manual_gear_shift": self.manual_gear_shift,
            "gear": self.gear,
        }

    @classmethod
    def from_dict(cls, dictionary):
        return cls(
            dictionary["throttle"],
            dictionary["steer"],
            dictionary["brake"],
            dictionary["hand_brake"],
            dictionary["reverse"],
            dictionary["manual_gear_shift"],
            dictionary["gear"],
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
