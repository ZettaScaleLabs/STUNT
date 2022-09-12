import json
import numpy as np
from carla import Actor as CarlaActor


class Obstacle(object):
    def __init__(
        self,
        attributes={},
        id=0,
        parent=0,
        is_alive=False,
        semantic_tags=[],
        type_id="",
    ):
        self.attributes = attributes
        self.id = id
        self.parent = parent
        self.is_alive = is_alive
        self.semantic_tags = semantic_tags
        self.type_id = type_id

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"Obstacle(attributes={self.attributes}, id={self.id}, is_alive={self.is_alive}, parent={self.parent}, semantic_tags={self.semantic_tags}, type_id={self.type_id})"

    @classmethod
    def from_simulator(cls, data):
        """Creates a STUNT Obstacle from a simulator Actor.

        Args:
            data: An instance of a simulator Actor.

        Returns:
            :py:class:`.Obstacle`: A STUNT Obstacle.
        """

        if not isinstance(data, CarlaActor):
            raise ValueError("The data must be a Location or carla.Actor")

        return cls(
            data.attributes,
            data.id,
            data.parent,
            data.is_alive,
            # TODO: bump CARLA version, this crashes in 0.9.10
            # data.semantic_tags,
            [],
            data.type_id,
        )

    # TODO: cannot convert back to simulator, yet.
    # def to_simulator(self):
    #     return CarlaActor(
    #         self.fov,
    #         self.height,
    #         self.width,
    #         self.point_cloud.tobytes(),
    #     )

    def to_dict(self):
        return {
            "attributes": self.attributes,
            "id": self.id,
            "parent": self.parent,
            "is_alive": self.is_alive,
            "semantic_tags": self.semantic_tags,
            "type_id": self.type_id,
        }

    @classmethod
    def from_dict(cls, dictionary):

        return cls(
            dictionary["attributes"],
            dictionary["id"],
            dictionary["parent"],
            dictionary["is_alive"],
            dictionary["semantic_tags"],
            dictionary["type_id"]
        )

    def serialize(self):
        return json.dumps(self.to_dict()).encode("utf-8")

    @classmethod
    def deserialize(cls, serialized):
        deserialized = json.loads(serialized.decode("utf-8"))
        return cls.from_dict(deserialized)
