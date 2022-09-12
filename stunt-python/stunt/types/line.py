import json
import cv2
from enum import Enum
import numpy as np


class LaneMarkingColor(Enum):
    """Enum that defines the lane marking colors according to OpenDrive 1.4.

    The goal of this enum is to make sure that lane colors are correctly
    propogated from the simulator to Pylot.
    """

    WHITE = 0
    BLUE = 1
    GREEN = 2
    RED = 3
    YELLOW = 4
    OTHER = 5

    def serialize(self):
        if self == WHITE:
            return "WHITE"
        elif self == BLUE:
            return "BLUE"
        elif self == GREEN:
            return "GREEN"
        elif self == RED:
            return "RED"
        elif self == YELLOW:
            return "YELLOW"
        else:
            return "OTHER"

    @classmethod
    def deserialize(cls, serialized):
        if serialized == "WHITE":
            return cls.WHITE
        elif self == "BLUE":
            return cls.BLUE
        elif self == "GREEN":
            return cls.GREEN
        elif self == "RED":
            return cls.RED
        elif self == "YELLOW":
            return cls.YELLOW
        else:
            return cls.OTHER


class LaneMarkingType(Enum):
    """Enum that defines the lane marking types according to OpenDrive 1.4.

    The goal of this enum is to make sure that lane markings are correctly
    propogated from the simulator to Pylot.
    """

    OTHER = 0
    BROKEN = 1
    SOLID = 2
    SOLIDSOLID = 3
    SOLIDBROKEN = 4
    BROKENSOLID = 5
    BROKENBROKEN = 6
    BOTTSDOTS = 7
    GRASS = 8
    CURB = 9
    NONE = 10

    def serialize(self):
        if self == BROKEN:
            return "BROKEN"
        elif self == SOLID:
            return "BROKEN"
        elif self == SOLIDSOLID:
            return "SOLIDSOLID"
        elif self == SOLIDBROKEN:
            return "SOLIDBROKEN"
        elif self == BROKENSOLID:
            return "BROKENSOLID"
        elif self == BROKENBROKEN:
            return "BROKENBROKEN"
        elif self == BOTTSDOTS:
            return "BOTTSDOTS"
        elif self == GRASS:
            return "GRASS"
        elif self == CURB:
            return "CURB"
        elif self == NONE:
            return "NONE"
        else:
            return "OTHER"

    @classmethod
    def deserialize(cls, serialized):
        if serialized == "BROKEN":
            return cls.BROKEN
        elif serialized == "SOLID":
            return cls.SOLID
        elif serialized == "SOLIDSOLID":
            return cls.SOLIDSOLID
        elif serialized == "SOLIDBROKEN":
            return cls.SOLIDBROKEN
        elif serialized == "BROKENSOLID":
            return cls.BROKENSOLID
        elif serialized == "BROKENBROKEN":
            return cls.BROKENBROKEN
        elif serialized == "BOTTSDOTS":
            return cls.BOTTSDOTS
        elif serialized == "GRASS":
            return cls.GRASS
        elif serialized == "CURB":
            return cls.CURB
        elif serialized == "NONE":
            return cls.NONE
        else:
            return "OTHER"


class LaneChange(Enum):
    """Enum that defines the permission to turn either left, right, both or
    none for a given lane.

    The goal of this enum is to make sure that the lane change types are
    correctly propogated from the simulator to Pylot.
    """

    NONE = 0
    RIGHT = 1
    LEFT = 2
    BOTH = 3


class LaneType(Enum):
    """Enum that defines the type of the lane according to OpenDrive 1.4.

    The goal of this enum is to make sure that the lane change types are
    correctly propogated from the simulator to Pylot.
    """

    NONE = 1
    DRIVING = 2
    STOP = 4
    SHOULDER = 8
    BIKING = 16
    SIDEWALK = 32
    BORDER = 64
    RESTRICTED = 128
    PARKING = 256
    BIDIRECTIONAL = 512
    MEDIAN = 1024
    SPECIAL1 = 2048
    SPECIAL2 = 4096
    SPECIAL3 = 8192
    ROADWORKS = 16384
    TRAM = 32768
    RAIL = 65536
    ENTRY = 131072
    EXIT = 262144
    OFFRAMP = 524288
    ONRAMP = 1048576
    ANY = 4294967294


class RoadOption(Enum):
    """Enum that defines the possible high-level route plans.

    RoadOptions are usually attached to waypoints we receive from
    the challenge environment.
    """

    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANE_FOLLOW = 4
    CHANGE_LANE_LEFT = 5
    CHANGE_LANE_RIGHT = 6

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return self.name


class LaneMarking(object):
    """Used to represent a lane marking.

    Attributes:
        marking_color (:py:class:`.LaneMarkingColor`): The color of the lane
            marking
        marking_type (:py:class:`.LaneMarkingType`): The type of the lane
            marking.
        lane_change (:py:class:`.LaneChange`): The type that defines the
            permission to either turn left, right, both or none.
    """

    def __init__(self, marking_color, marking_type, lane_change):
        self.marking_color = LaneMarkingColor(marking_color)
        self.marking_type = LaneMarkingType(marking_type)
        self.lane_change = LaneChange(lane_change)

    @classmethod
    def from_simulator_lane_marking(cls, lane_marking):
        """Creates a pylot LaneMarking from a simulator lane marking.

        Args:
            lane_marking: An instance of a simulator lane marking.

        Returns:
            :py:class:`.LaneMarking`: A pylot lane-marking.
        """
        return cls(lane_marking.color, lane_marking.type, lane_marking.lane_change)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "LaneMarking(color: {}, type: {}, change: {})".format(
            self.marking_color, self.marking_type, self.lane_change
        )
