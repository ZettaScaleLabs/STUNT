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
    Pose,
    Location,
)
from collections import deque
import json

DEFAULT_SLEEP_TIME_S = 5
INPUT_GOAL = "Goal"
INPUT_POSE = "Pose"
OUTPUT_ROUTE = "Route"
DEFAULT_LOCATION_GOAL = (0.0, 0.0, 0.0)


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
            format="%(levelname)s: %(message)s", level=logging.DEBUG
        )

        # Getting carla map
        self.map_file = configuration.get("map", None)
        if self.map_file is None:
            raise ValueError("[RoutePlanner] cannot proceed without a map!")

        with open(self.map_file) as f:
            opendrive = f.read()

        self.map = HDMap.from_opendrive(opendrive)

        self.ego_position = None
        self.pending = []

        self.goal_location = Location(
            *configuration.get("goal", DEFAULT_LOCATION_GOAL)
        )

        self.input_goal = inputs.get(INPUT_GOAL)
        self.input_pose = inputs.get(INPUT_POSE)
        self.output = outputs.get(OUTPUT_ROUTE)

    async def wait_goal(self):
        data_msg = await self.input_goal.recv()
        return (INPUT_GOAL, data_msg)

    async def wait_pose(self):
        data_msg = await self.input_pose.recv()
        return (INPUT_POSE, data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == INPUT_GOAL for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_goal(), name=INPUT_GOAL)
            )

        if not any(t.get_name() == INPUT_POSE for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_pose(), name=INPUT_POSE)
            )
        return task_list

    async def iteration(self):
        """Compute and send the waypoints the ego vehicle should follow to
        reach its destination.

        The destination goal is received on the input `Goal`. If a new
        destination is received while another one is "active" (i.e. the ego
        vehicle is on its way there) the old destination is discarded in favor
        of the new one.
        """

        try:
            (done, pending) = await asyncio.wait(
                self.create_task_list(),
                return_when=asyncio.FIRST_COMPLETED,
            )

            self.pending = list(pending)

            for d in done:
                (who, data_msg) = d.result()

            if who == INPUT_POSE:
                self.ego_position = Pose.deserialize(
                    data_msg.data
                ).transform.location
            elif who == INPUT_GOAL:
                if self.ego_position is not None:
                    self.goal_location = Location.deserialize(data_msg.data)

                    waypoints = self.map.compute_waypoints(
                        self.ego_position, self.goal_location
                    )
                    road_options = deque(
                        [RoadOption.LANE_FOLLOW for _ in range(len(waypoints))]
                    )
                    route = Waypoints(waypoints, road_options=road_options)

                    logging.debug(
                        "[RoutePlanner] Sending route"
                        + f" from {self.ego_position} to {self.goal_location}"
                    )
                    await self.output.send(route.serialize())
                    self.goal_location is None
            else:
                return None

        except Exception as e:
            logging.warning(f"[RoutePlanner] got error {e}")
        return None

    def finalize(self) -> None:
        return None


def register():
    return RoutePlanner
