import json
from stunt.types import Waypoints, BehaviorPlannerState


class Trajectory:
    def __init__(self, waypoints, state):
        self.waypoints = waypoints
        self.state = state

    def to_dict(self):
        return {
            "waypoints": None if self.waypoints is None else self.waypoints.to_dict(),
            "state": self.state.serialize(),
        }

    @classmethod
    def from_dict(cls, dictionary):

        waypoints = (
            Waypoints.from_dict(dictionary["waypoints"])
            if dictionary["waypoints"] is not None
            else Waypoints([])
        )
        state = BehaviorPlannerState.deserialize(dictionary["state"])

        return cls(waypoints, state)

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
