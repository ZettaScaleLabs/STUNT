import json
import enum


class BehaviorPlannerState(enum.Enum):
    """States in which the FSM behavior planner can be in."""

    FOLLOW_WAYPOINTS = 0
    READY = 1
    KEEP_LANE = 2
    PREPARE_LANE_CHANGE_LEFT = 3
    LANE_CHANGE_LEFT = 4
    PREPARE_LANE_CHANGE_RIGHT = 5
    LANE_CHANGE_RIGHT = 6
    OVERTAKE = 7

    def serialize(self):
        if self == BehaviorPlannerState.READY:
            return "READY"
        elif self == BehaviorPlannerState.KEEP_LANE:
            return "KEEP_LANE"
        elif self == BehaviorPlannerState.PREPARE_LANE_CHANGE_LEFT:
            return "PREPARE_LANE_CHANGE_LEFT"
        elif self == BehaviorPlannerState.LANE_CHANGE_LEFT:
            return "LANE_CHANGE_LEFT"
        elif self == BehaviorPlannerState.PREPARE_LANE_CHANGE_RIGHT:
            return "PREPARE_LANE_CHANGE_RIGHT"
        elif self == BehaviorPlannerState.LANE_CHANGE_RIGHT:
            return "LANE_CHANGE_RIGHT"
        elif self == BehaviorPlannerState.OVERTAKE:
            return "OVERTAKE"
        else:
            return "FOLLOW_WAYPOINTS"

    @classmethod
    def deserialize(cls, serialized):
        if serialized == "READY":
            return cls.READY
        elif serialized == "KEEP_LANE":
            return cls.KEEP_LANE
        elif serialized == "PREPARE_LANE_CHANGE_LEFT":
            return cls.PREPARE_LANE_CHANGE_LEFT
        elif serialized == "LANE_CHANGE_LEFT":
            return cls.LANE_CHANGE_LEFT
        elif serialized == "PREPARE_LANE_CHANGE_RIGHT":
            return cls.PREPARE_LANE_CHANGE_RIGHT
        elif serialized == "LANE_CHANGE_RIGHT":
            return cls.LANE_CHANGE_RIGHT
        elif serialized == "OVERTAKE":
            return cls.OVERTAKE
        else:
            return cls.FOLLOW_WAYPOINTS


def cost_overtake(current_state, future_state, ego_info):
    if ego_info.current_time - ego_info.last_time_moving > 50000:
        # Switch to OVERTAKE if ego hasn't moved for a while.
        if future_state == BehaviorPlannerState.OVERTAKE:
            return 0
        return 1
    else:
        if current_state == BehaviorPlannerState.OVERTAKE:
            # Do not speed too long in OVERTAKE state.
            if ego_info.current_time - ego_info.last_time_stopped > 3000:
                if future_state == BehaviorPlannerState.OVERTAKE:
                    return 1
                else:
                    return 0
            else:
                if future_state == BehaviorPlannerState.OVERTAKE:
                    return 0
                else:
                    return 1
        else:
            # Do not switch to overtake because the ego is not blocked.
            if future_state == BehaviorPlannerState.OVERTAKE:
                return 1
            return 0
