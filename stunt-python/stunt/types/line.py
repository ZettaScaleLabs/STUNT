from enum import Enum


class LaneMarkingColor(Enum):
    """Enum that defines the lane marking colors according to OpenDrive 1.4.

    The goal of this enum is to make sure that lane colors are correctly
    propogated from the simulator to STUNT.
    """

    WHITE = 0
    BLUE = 1
    GREEN = 2
    RED = 3
    YELLOW = 4
    OTHER = 5

    def serialize(self):
        if self == LaneMarkingColor.WHITE:
            return "WHITE"
        elif self == LaneMarkingColor.BLUE:
            return "BLUE"
        elif self == LaneMarkingColor.GREEN:
            return "GREEN"
        elif self == LaneMarkingColor.RED:
            return "RED"
        elif self == LaneMarkingColor.YELLOW:
            return "YELLOW"
        else:
            return "OTHER"

    @classmethod
    def deserialize(cls, serialized):
        if serialized == "WHITE":
            return cls.WHITE
        elif serialized == "BLUE":
            return cls.BLUE
        elif serialized == "GREEN":
            return cls.GREEN
        elif serialized == "RED":
            return cls.RED
        elif serialized == "YELLOW":
            return cls.YELLOW
        else:
            return cls.OTHER


class LaneMarkingType(Enum):
    """Enum that defines the lane marking types according to OpenDrive 1.4.

    The goal of this enum is to make sure that lane markings are correctly
    propogated from the simulator to STUNT.
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
        if self == LaneMarkingType.BROKEN:
            return "BROKEN"
        elif self == LaneMarkingType.SOLID:
            return "BROKEN"
        elif self == LaneMarkingType.SOLIDSOLID:
            return "SOLIDSOLID"
        elif self == LaneMarkingType.SOLIDBROKEN:
            return "SOLIDBROKEN"
        elif self == LaneMarkingType.BROKENSOLID:
            return "BROKENSOLID"
        elif self == LaneMarkingType.BROKENBROKEN:
            return "BROKENBROKEN"
        elif self == LaneMarkingType.BOTTSDOTS:
            return "BOTTSDOTS"
        elif self == LaneMarkingType.GRASS:
            return "GRASS"
        elif self == LaneMarkingType.CURB:
            return "CURB"
        elif self == LaneMarkingType.NONE:
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
            return cls.OTHER


class LaneChange(Enum):
    """Enum that defines the permission to turn either left, right, both or
    none for a given lane.

    The goal of this enum is to make sure that the lane change types are
    correctly propogated from the simulator to STUNT.
    """

    NONE = 0
    RIGHT = 1
    LEFT = 2
    BOTH = 3

    def serialize(self):
        if self == LaneChange.RIGHT:
            return "RIGHT"
        elif self == LaneChange.LEFT:
            return "LEFT"
        elif self == LaneChange.BOTH:
            return "BOTH"
        else:
            return "NONE"

    @classmethod
    def deserialize(cls, serialized):
        if serialized == "RIGHT":
            return cls.RIGHT
        elif serialized == "LEFT":
            return cls.LEFT
        elif serialized == "BOTH":
            return cls.BOTH
        else:
            return cls.NONE


class LaneType(Enum):
    """Enum that defines the type of the lane according to OpenDrive 1.4.

    The goal of this enum is to make sure that the lane change types are
    correctly propogated from the simulator to STUNT.
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

    def serialize(self):
        if self == LaneType.DRIVING:
            return "DRIVING"
        elif self == LaneType.STOP:
            return "STOP"
        elif self == LaneType.SHOULDER:
            return "SHOULDER"
        elif self == LaneType.BIKING:
            return "BIKING"
        elif self == LaneType.SIDEWALK:
            return "SIDEWALK"
        elif self == LaneType.BORDER:
            return "BORDER"
        elif self == LaneType.RESTRICTED:
            return "RESTRICTED"
        elif self == LaneType.PARKING:
            return "PARKING"
        elif self == LaneType.BIDIRECTIONAL:
            return "BIDIRECTIONAL"
        elif self == LaneType.MEDIAN:
            return "MEDIAN"
        elif self == LaneType.SPECIAL1:
            return "SPECIAL1"
        elif self == LaneType.SPECIAL2:
            return "SPECIAL2"
        elif self == LaneType.SPECIAL3:
            return "SPECIAL3"
        elif self == LaneType.ROADWORKS:
            return "ROADWORKS"
        elif self == LaneType.TRAM:
            return "TRAM"
        elif self == LaneType.RAIL:
            return "RAIL"
        elif self == LaneType.ENTRY:
            return "ENTRY"
        elif self == LaneType.EXIT:
            return "EXIT"
        elif self == LaneType.OFFRAMP:
            return "OFFRAMP"
        elif self == LaneType.ONRAMP:
            return "ONRAMP"
        elif self == LaneType.ANY:
            return "ANY"
        else:
            return "NONE"

    @classmethod
    def deserialize(cls, serialized):
        if serialized == "DRIVING":
            return cls.DRIVING
        elif serialized == "STOP":
            return cls.STOP
        elif serialized == "SHOULDER":
            return cls.SHOULDER
        elif serialized == "BIKING":
            return cls.BIKING
        elif serialized == "SIDEWALK":
            return cls.SIDEWALK
        elif serialized == "BORDER":
            return cls.BORDER
        elif serialized == "RESTRICTED":
            return cls.RESTRICTED
        elif serialized == "PARKING":
            return cls.PARKING
        elif serialized == "BIDIRECTIONAL":
            return cls.BIDIRECTIONAL
        elif serialized == "MEDIAN":
            return cls.MEDIAN
        elif serialized == "SPECIAL1":
            return cls.SPECIAL1
        elif serialized == "SPECIAL2":
            return cls.SPECIAL2
        elif serialized == "SPECIAL3":
            return cls.SPECIAL3
        elif serialized == "ROADWORKS":
            return cls.ROADWORKS
        elif serialized == "TRAM":
            return cls.TRAM
        elif serialized == "RAIL":
            return cls.RAIL
        elif serialized == "ENTRY":
            return cls.ENTRY
        elif serialized == "EXIT":
            return cls.EXIT
        elif serialized == "OFFRAMP":
            return cls.OFFRAMP
        elif serialized == "ONRAMP":
            return cls.ONRAMP
        elif serialized == "ANY":
            return cls.ANY
        else:
            return cls.NONE


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

    def serialize(self):
        if self == RoadOption.LEFT:
            return "LEFT"
        elif self == RoadOption.RIGHT:
            return "RIGHT"
        elif self == RoadOption.STRAIGHT:
            return "STRAIGHT"
        elif self == RoadOption.LANE_FOLLOW:
            return "LANE_FOLLOW"
        elif self == RoadOption.CHANGE_LANE_LEFT:
            return "CHANGE_LANE_LEFT"
        elif self == RoadOption.CHANGE_LANE_RIGHT:
            return "CHANGE_LANE_RIGHT"
        else:
            return "VOID"

    @classmethod
    def deserialize(cls, serialized):
        if serialized == "LEFT":
            return cls.LEFT
        elif serialized == "RIGHT":
            return cls.RIGHT
        elif serialized == "STRAIGHT":
            return cls.STRAIGHT
        elif serialized == "LANE_FOLLOW":
            return cls.LANE_FOLLOW
        elif serialized == "CHANGE_LANE_LEFT":
            return cls.CHANGE_LANE_LEFT
        elif serialized == "CHANGE_LANE_RIGHT":
            return cls.CHANGE_LANE_RIGHT
        else:
            return cls.VOID

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
    def from_simulator(cls, lane_marking):
        """Creates a STUNT LaneMarking from a simulator lane marking.

        Args:
            lane_marking: An instance of a simulator lane marking.

        Returns:
            :py:class:`.LaneMarking`: A STUNT lane-marking.
        """
        return cls(
            lane_marking.color, lane_marking.type, lane_marking.lane_change
        )

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "LaneMarking(color: {}, type: {}, change: {})".format(
            self.marking_color, self.marking_type, self.lane_change
        )
