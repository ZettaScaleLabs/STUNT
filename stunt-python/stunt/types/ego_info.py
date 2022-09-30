from stunt.types import Pose
import json

DEFAULT_MIN_MOVING_SPEED = 0.7


class EgoInfo(object):
    def __init__(self, last_time_moving=0, last_time_stopped=0, current_time=0):
        self.last_time_moving = 0
        self.last_time_stopped = 0
        self.current_time = 0

    def update(self, pose: Pose):

        self.current_time = pose.localization_time
        if pose.forward_speed >= DEFAULT_MIN_MOVING_SPEED:
            self.last_time_moving = self.current_time
        else:
            self.last_time_stopped = self.current_time

    def to_dict(self):
        return {
            "last_time_moving": self.last_time_moving,
            "last_time_stopped": self.last_time_stopped,
            "current_time": self.current_time,
        }

    @classmethod
    def from_dict(cls, dictionary):
        return cls(
            dictionary["last_time_moving"],
            dictionary["last_time_stopped"],
            dictionary["current_time"],
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
