from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any
import logging
import asyncio
import json
from stunt.types import (
    Pose,
    Obstacle,
    ObstacleTrajectory,
    ObstaclePrediction,
    BehaviorPlannerState,
    Trajectory,
    TrafficLight,
    World,
)
from stunt.map import HDMap

DEFAULT_STOP_FOR_TRAFFIC_LIGHTS = True
DEFAULT_STOP_FOR_PEOPLE = True
DEFAULT_STOP_FOR_VEHICLES = True
DEFAULT_STOP_FOR_UNCONTROLLED_JUNCTIONS = True

DEFAULT_TARGET_SPEED = 6.0


class WaypointPlanner(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        logging.basicConfig(level=logging.DEBUG)
        configuration = configuration if configuration is not None else {}

        self.map_file = configuration.get("map", None)
        if self.map_file is None:
            raise ValueError("BehaviourPlanning cannot proceed without a map!")

        self.pending = []

        self.traffic_lights_input = inputs.get("TrafficLights", None)
        self.trajectory_input = inputs.get("Trajectory", None)
        self.pose_input = inputs.get("Pose", None)
        self.obstacles_input = inputs.get("ObstaclePredictions", None)
        self.output = outputs.get("Waypoints", None)

        if self.traffic_lights_input is None:
            raise ValueError("Cannot find input: 'TrafficLights'")
        if self.trajectory_input is None:
            raise ValueError("Cannot find input: 'Trajectory'")
        if self.pose_input is None:
            raise ValueError("Cannot find input: 'Pose'")
        if self.obstacles_input is None:
            raise ValueError("Cannot find input: 'ObstaclePredictions'")
        if self.output is None:
            raise ValueError("Cannot find output: 'Waypoints'")

        self.start_time = 0
        self.world = World(configuration)

        self.obstacle_trajectories = []
        self.trajectory = None
        self.traffic_lights = []

        # TODO this should come from perception
        self.target_speed = configuration.get(
            "target_speed", DEFAULT_TARGET_SPEED
        )
        self.state = BehaviorPlannerState.FOLLOW_WAYPOINTS

        # Getting carla map
        with open(self.map_file) as f:
            opendrive = f.read()
        self.map = HDMap.from_opendrive(opendrive)

    def get_predictions(self, prediction_msg, ego_transform):
        """Extracts obstacle predictions out of the message.

        This method is useful to build obstacle predictions when
        the operator directly receives detections instead of predictions.
        The method assumes that the obstacles are static.

        (PS: [0] is added because the prediction msg is a list)
        """
        predictions = []
        if prediction_msg:
            # if it's not an empty list
            # TODO: check why the prediction msg is empty
            if isinstance(prediction_msg[0], Obstacle):
                # Transform the obstacle into a prediction.
                predictions = []
                for obstacle in prediction_msg.obstacles:
                    obstacle_trajectory = ObstacleTrajectory(obstacle, [])
                    prediction = ObstaclePrediction(
                        obstacle_trajectory,
                        obstacle.transform,
                        1.0,
                        [
                            ego_transform.inverse_transform()
                            * obstacle.transform
                        ],
                    )
                    predictions.append(prediction)
            elif isinstance(prediction_msg[0], ObstaclePrediction):
                # TODO: check if we should use this instead
                predictions = prediction_msg
                # predictions = prediction_msg[0]
            else:
                raise ValueError(
                    "Unexpected obstacles msg type {}".format(
                        type(prediction_msg[0])
                    )
                )
        return predictions

    async def wait_obstacles(self):
        data_msg = await self.obstacles_input.recv()
        return ("ObstaclePredictions", data_msg)

    async def wait_pose(self):
        data_msg = await self.pose_input.recv()
        return ("Pose", data_msg)

    async def wait_trajectory(self):
        data_msg = await self.trajectory_input.recv()
        return ("Trajectory", data_msg)

    async def wait_traffic_lights(self):
        data_msg = await self.traffic_lights_input.recv()
        return ("TrafficLights", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "ObstaclePredictions" for t in task_list):
            task_list.append(
                asyncio.create_task(
                    self.wait_obstacles(), name="ObstaclePredictions"
                )
            )

        if not any(t.get_name() == "Pose" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_pose(), name="Pose")
            )

        if not any(t.get_name() == "Trajectory" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_trajectory(), name="Trajectory")
            )

        if not any(t.get_name() == "TrafficLights" for t in task_list):
            task_list.append(
                asyncio.create_task(
                    self.wait_traffic_lights(), name="TrafficLights"
                )
            )
        return task_list

    async def iteration(self):
        try:
            (done, pending) = await asyncio.wait(
                self.create_task_list(),
                return_when=asyncio.FIRST_COMPLETED,
            )

            self.pending = list(pending)

            for d in done:
                (who, data_msg) = d.result()

                logging.debug(f"[WaypointPlanner] Received from input {who}")

                if who == "Trajectory":
                    # getting the trajectory from the behaviour planner
                    # it generates the kind of GPS waypoints towards the
                    # destination
                    self.trajectory = Trajectory.deserialize(data_msg.data)

                    self.state = self.trajectory.state

                    if (
                        self.trajectory.waypoints is not None
                        and len(self.trajectory.waypoints.waypoints) > 0
                    ):

                        self.world.update_waypoints(
                            self.trajectory.waypoints.waypoints[-1].location,
                            self.trajectory.waypoints,
                        )

                elif who == "ObstaclePredictions":
                    predictions_list = json.loads(data_msg.data.decode("utf-8"))
                    self.obstacle_trajectories = []
                    for p in predictions_list:
                        self.obstacle_trajectories.append(
                            ObstaclePrediction.from_dict(p)
                        )

                elif who == "TrafficLights":
                    traffic_lights = json.loads(data_msg.data.decode("utf-8"))
                    self.traffic_lights = []
                    for tl in traffic_lights:
                        self.traffic_lights.append(TrafficLight.from_dict(tl))

                elif who == "Pose":
                    pose = Pose.deserialize(data_msg.data)

                    # print(
                    #     f"Waypoint planner received pose, can compute checks obstacles:{self.obstacle_trajectories is not None}, traffic_lights:{self.traffic_lights is not None}, trajectory:{self.trajectory is not None}"
                    # )

                    if (
                        self.obstacle_trajectories is None
                        or self.traffic_lights is None
                        or self.trajectory is None
                    ):
                        return None

                    predictions = self.get_predictions(
                        self.obstacle_trajectories, pose.transform
                    )
                    self.world.update(
                        pose.localization_time,
                        pose,
                        predictions,
                        self.traffic_lights,
                        self.map,
                        None,
                    )

                    (
                        speed_factor,
                        _,
                        _,
                        speed_factor_tl,
                        speed_factor_stop,
                    ) = self.world.stop_for_agents(pose.localization_time)

                    target_speed = speed_factor * self.target_speed

                    output_wps = self.world.follow_waypoints(target_speed)

                    # remove waypoints that are too close (we already reach them)
                    output_wps.remove_waypoint_if_close(
                        pose.transform.location, distance=5
                    )

                    await self.output.send(output_wps.serialize())
                    self.obstacle_trajectories = None
                    self.traffic_lights = None
                    # self.trajectory = None
        except Exception as e:
            logging.warning(f"[WaypointPlanner] got error {e}")
        return None

    def finalize(self) -> None:
        return None


def register():
    return WaypointPlanner
