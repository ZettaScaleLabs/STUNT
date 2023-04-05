from zenoh_flow.interfaces import Source
from zenoh_flow import Output
from zenoh_flow.types import Context
from typing import Any, Dict
import asyncio

from stunt.map import HDMap
from stunt.types import (
    Waypoints,
    Location,
    RoadOption,
)
from collections import deque
from stunt.simulator.ground_truth import Localization

DEFAULT_LOCATION_GOAL = (0.0, 0.0, 0.0)
DEFAULT_SLEEP_TIME_S = 5


class Destination(Source):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration

        self.map_file = configuration.get("map", None)
        if self.map_file is None:
            raise ValueError("BehaviourPlanning cannot proceed without a map!")

        self.goal_location = Location(
            *configuration.get("goal", DEFAULT_LOCATION_GOAL)
        )

        # Getting carla map
        with open(self.map_file) as f:
            opendrive = f.read()
        self.map = HDMap.from_opendrive(opendrive)

        self.sensor = Localization(configuration, self.on_location_update)

        self.output = outputs.get("Trajectory", None)
        self.first = True
        self.ego_position = None

        self.output = outputs.get("Route")

    def on_location_update(self, data):
        self.ego_position = data.transform.location

    async def iteration(self):
        if self.first and self.ego_position is not None:
            self.first = False
            waypoints = self.map.compute_waypoints(
                self.ego_position, self.goal_location
            )
            road_options = deque(
                [RoadOption.LANE_FOLLOW for _ in range(len(waypoints))]
            )
            route = Waypoints(waypoints, road_options=road_options)
            self.ego_position == None
            await self.output.send(route.serialize())
        else:
            await asyncio.sleep(DEFAULT_SLEEP_TIME_S)

        return None

    def finalize(self) -> None:
        return None


def register():
    return Destination
