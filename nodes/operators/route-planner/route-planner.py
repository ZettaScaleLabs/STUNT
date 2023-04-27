from zenoh_flow.interfaces import Operator
from zenoh_flow import Output, Input
from zenoh_flow.types import Context
from typing import Any, Dict
import asyncio
import logging
from stunt.map import HDMap
from stunt.types import (
    Waypoints,
    RoadOption,
)
from collections import deque
from stunt.simulator.ground_truth import Localization

DEFAULT_SLEEP_TIME_S = 5
INPUT_GOAL = "Goal"
OUTPUT_ROUTE = "Route"


class RoutePlanner(Operator):
    def __init__(
            self,
            context: Context,
            configuration: Dict[str, Any],
            inputs: Dict[str, Input],
            outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration
        logging.basicConfig(
            format='%(levelname)s: %(message)s',
            level=logging.DEBUG
        )

        # Getting carla map
        self.map_file = configuration.get("map", None)
        if self.map_file is None:
            raise ValueError("[RoutePlanner] cannot proceed without a map!")

        with open(self.map_file) as f:
            opendrive = f.read()

        self.map = HDMap.from_opendrive(opendrive)

        self.sensor = Localization(configuration, self.on_location_update)

        self.ego_position = None

        self.input_goal = inputs.get(INPUT_GOAL)
        self.output = outputs.get(OUTPUT_ROUTE)

    def on_location_update(self, data):
        self.ego_position = data.transform.location

    async def iteration(self):
        """Compute and send the waypoints the ego vehicle should follow to
        reach its destination.

        The destination goal is received on the input `Goal`. If a new
        destination is received while another one is "active" (i.e. the ego
        vehicle is on its way there) the old destination is discarded in favor
        of the new one.
        """
        if self.ego_position is not None:
            goal_location = await self.input_goal.recv()

            waypoints = self.map.compute_waypoints(
                self.ego_position, goal_location
            )
            road_options = deque(
                [RoadOption.LANE_FOLLOW for _ in range(len(waypoints))]
            )
            route = Waypoints(waypoints, road_options=road_options)

            logging.debug(
                "[RoutePlanner] Sending route" +
                f" from {self.ego_position} to {goal_location}"
            )
            await self.output.send(route.serialize())
        else:
            await asyncio.sleep(DEFAULT_SLEEP_TIME_S)

        return None

    def finalize(self) -> None:
        return None


def register():
    return RoutePlanner
