import json

class TimeToDecision(object):
    def __init__(self, deadline):
        self.deadline = deadline

    def to_dict(self):
        return {
            "deadline": self.deadline,
        }

    @classmethod
    def from_dict(cls, dictionary):
        return cls(dictionary["deadline"])

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)