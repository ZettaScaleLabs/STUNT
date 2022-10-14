from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from zenoh_flow.types import Context
from typing import Dict, Any
import time
import asyncio
import json
import carla

from collections import deque
from stunt.types import (
    Pose,
    Obstacle,
    ObstacleTrajectory,
    ObstaclePrediction,
    BehaviorPlannerState,
    TimeToDecision,
    Trajectory,
    TrafficLight,
    World,
    Transform,
    Location,
    Rotation,
    Waypoints,
    RoadOption,
)
from stunt.map import HDMap

from stunt import DEFAULT_CARLA_HOST, DEFAULT_CARLA_PORT

from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory.fot_wrapper import (
    run_fot,
)

DEFAULT_STOP_FOR_TRAFFIC_LIGHTS = True
DEFAULT_STOP_FOR_PEOPLE = True
DEFAULT_STOP_FOR_VEHICLES = True
DEFAULT_STOP_FOR_UNCONTROLLED_JUNCTIONS = True

DEFAULT_PREDICTION_TTD = 500
DEFAULT_TARGET_SPEED = 6.0

DEFAULT_HYPERPARAMETERS = {
    "num_threads": 1,
    "max_speed": 6.0,
    "max_accel": 1.0,
    "max_curvature": 1.0,
    "max_road_width_l": 5.0,
    "max_road_width_r": 0.5,
    "d_road_w": 0.2,
    "dt": 0.1,
    "maxt": 5.0,
    "mint": 2.0,
    "d_t_s": 0.25,
    "n_s_sample": 2.0,
    "obstacle_clerance_fot": 0.7,
    "kd": 1.0,
    "kv": 0.1,
    "ka": 0.1,
    "kj": 0.01,
    "kt": 0.01,
    "ko": 100.0,
    "klat": 1.0,
    "klon": 1.0,
    "planning_deadline": 10.0,
}


class FreenetOptimalTrajectoryPlanner(object):
    def __init__(self, configuration, world, world_map):
        self.configuration = configuration
        self.world = world
        self.map = world_map

        self.s0 = 0.0
        self.hyperparameters = configuration.get(
            "hyperparameters", DEFAULT_HYPERPARAMETERS
        )

    def build_output_waypoints(self, path_x, path_y, speeds):
        """Builds a Waypoints object from 2D locations and speeds."""
        wps = deque()
        target_speeds = deque()
        for point in zip(path_x, path_y, speeds):
            if self.map is not None:
                # Use the HD Map to transform a 2D location into a
                # 3D location.
                p_loc = self.map.get_closest_lane_waypoint(
                    Location(x=point[0], y=point[1], z=0)
                ).location
            else:
                p_loc = Location(x=point[0], y=point[1], z=0)
            # Use the computed x and y (not the ones returned by the HDMap)
            # to ensure that the vehicles follows the computed plan.
            wps.append(
                Transform(
                    location=Location(x=point[0], y=point[1], z=p_loc.z),
                    rotation=Rotation(),
                )
            )
            target_speeds.append(point[2])
        return Waypoints(wps, target_speeds)

    def fot_parameters_using_99_percentile(self, ttd):
        maxt = self.hyperparameters["maxt"]
        runtimes = [309, 208, 148, 67, 40]
        dts = [0.09, 0.11, 0.13, 0.19, 0.31]
        d_road_ws = [0.3, 0.3, 0.3, 0.5, 0.7]

        for index, runtime in enumerate(runtimes):
            if ttd >= runtime:
                return maxt, dts[index], d_road_ws[index]

        # Not enough time to run the planner.
        return maxt, dts[-1], d_road_ws[-1]

    def update_hyper_parameters(self, timestamp, ttd=None):
        """Changes planning hyper parameters depending on time to decision."""
        # Change hyper paramters if static or dynamic deadlines are enabled.
        if ttd is not None:
            maxt, dt, d_road_w = self.fot_parameters_using_99_percentile(
                ttd.deadline
            )
        else:
            maxt, dt, d_road_w = self.fot_parameters_using_99_percentile(
                self.hyperparameters["planning_deadline"]
            )

        self.hyperparameters["maxt"] = maxt
        self.hyperparameters["dt"] = dt
        self.hyperparameters["d_road_w"] = d_road_w

    def run(self, timestamp, ttd=None):
        """Runs the planner.

        Note:
            The planner assumes that the world is up-to-date.

        Returns:
            :py:class:`~stunt.types.Waypoints`: Waypoints of the
            planned trajectory.
        """
        self.update_hyper_parameters(timestamp, ttd)
        initial_conditions = self._compute_initial_conditions()
        start = time.time()
        (
            path_x,
            path_y,
            speeds,
            ix,
            iy,
            iyaw,
            d,
            s,
            speeds_x,
            speeds_y,
            misc,
            costs,
            success,
        ) = run_fot(initial_conditions, self.hyperparameters)
        _ = (time.time() - start) * 1000

        if success:
            output_wps = self.build_output_waypoints(path_x, path_y, speeds)
        else:
            # Frenet failed. Sending emergency stop
            output_wps = self.world.follow_waypoints(0)

        # update current pose
        self.s0 = misc["s"]
        return output_wps

    def _compute_initial_conditions(self):
        ego_transform = self.world.ego_transform
        obstacle_list = self.world.get_obstacle_list()
        current_index = self.world.waypoints.closest_waypoint(
            ego_transform.location
        )
        # compute waypoints offset by current location
        wps = self.world.waypoints.slice_waypoints(
            max(current_index - self.configuration["num_waypoints_behind"], 0),
            min(
                current_index + self.configuration["num_waypoints_ahead"],
                len(self.world.waypoints.waypoints),
            ),
        )
        initial_conditions = {
            "ps": self.s0,
            "target_speed": self.configuration["target_speed"],
            "pos": ego_transform.location.as_numpy_array_2D(),
            "vel": self.world.ego_velocity_vector.as_numpy_array_2D(),
            "wp": wps.as_numpy_array_2D().T,
            "obs": obstacle_list,
        }
        return initial_conditions


class FOTPlanner(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ):
        configuration = configuration if configuration is not None else {}

        self.carla_port = int(configuration.get("port", DEFAULT_CARLA_PORT))
        self.carla_host = configuration.get("host", DEFAULT_CARLA_HOST)

        self.pending = []

        self.traffic_lights_input = inputs.get("TrafficLights", None)
        self.trajectory_input = inputs.get("Trajectory", None)
        self.pose_input = inputs.get("Pose", None)
        self.obstacles_input = inputs.get("ObstaclePredictions", None)
        self.ttd_input = inputs.get("TTD", None)
        self.output = outputs.get("Waypoints", None)

        self.start_time = 0
        self.world = World(configuration)

        self.obstacle_trajectories = []
        self.trajectory = None
        self.ttd = TimeToDecision(500)
        self.traffic_lights = []

        self.target_speed = configuration.get(
            "target_speed", DEFAULT_TARGET_SPEED
        )

        self.state = BehaviorPlannerState.FOLLOW_WAYPOINTS

        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()
        self.map = HDMap(self.carla_world.get_map())

        self.planner = FreenetOptimalTrajectoryPlanner(
            configuration, self.world, self.map
        )

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

    async def wait_ttd(self):
        data_msg = await self.ttd_input.recv()
        return ("TTD", data_msg)

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

        if not any(t.get_name() == "TTD" for t in task_list):
            task_list.append(asyncio.create_task(self.wait_ttd(), name="TTD"))

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

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )

        self.pending = list(pending)

        for d in done:
            (who, data_msg) = d.result()

            if who == "TTD":
                self.ttd = TimeToDecision.deserialize(data_msg.data)
            elif who == "Trajectory":
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
                #     f"FOTPlanner received pose, can compute checks obstcles:{self.obstacle_trajectories is not None}, traffic_lights:{self.traffic_lights is not None}, trajectory:{self.trajectory is not None}"
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
                    speed_factor_p,
                    speed_factor_v,
                    speed_factor_tl,
                    speed_factor_stop,
                ) = self.world.stop_for_agents(pose.localization_time)
                speed_factor = min(speed_factor_stop, speed_factor_tl)
                output_wps = self.planner.run(pose.localization_time)
                output_wps.apply_speed_factor(speed_factor)
                road_options = deque(
                    [
                        RoadOption.LANE_FOLLOW
                        for _ in range(len(output_wps.waypoints))
                    ]
                )
                output_wps.road_options = road_options
                # remove waypoints that are too close (we already reach them)
                output_wps.remove_waypoint_if_close(
                    pose.transform.location, distance=1
                )

                await self.output.send(output_wps.serialize())

                # consume data
                self.obstacle_trajectories = None
                self.traffic_lights = None
                self.trajectory = None

        return None

    def finalize(self) -> None:
        return None


def register():
    return FOTPlanner
