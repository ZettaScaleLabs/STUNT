from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from typing import Dict, Any, Callable
import time
import asyncio

import json
import numpy as np

from stunt.types import (
    Transform,
    Location,
    ObstacleTrajectory,
    TimeToDecision,
    ObstaclePrediction,
    get_nearby_obstacles_info,
)


DEFAULT_PREDICTION_RADIUS = 50
DEFAULT_PREDICTION_PAST_NUM_STEPS = 5
DEFAULT_PREDICTION_FUTURE_NUM_STEPS = 10

DEFAULT_PREDICTION_TTD = 500


class LinearPredictor(Operator):
    def __init__(
        self,
        configuration,
        obstacles_input,
        ttd_input,
        output,
    ):
        configuration = configuration if configuration is not None else {}

        self.prediction_radius = configuration.get(
            "prediction_radius", DEFAULT_PREDICTION_RADIUS
        )
        self.prediction_num_past_steps = configuration.get(
            "prediction_num_past_steps", DEFAULT_PREDICTION_PAST_NUM_STEPS
        )

        self.prediction_num_future_steps = configuration.get(
            "prediction_num_future_steps", DEFAULT_PREDICTION_FUTURE_NUM_STEPS
        )

        self.obstacles_input = obstacles_input
        self.ttd_input = ttd_input
        self.output = output

        self.ttd = 500

    async def wait_obstacles(self):
        data_msg = await self.obstacles_input.recv()
        return ("ObstacleTrajectories", data_msg)

    async def wait_ttd(self):
        data_msg = await self.ttd_input.recv()
        return ("Pose", data_msg)

    async def run(self):

        task_wait_obstacles = asyncio.create_task(self.wait_obstacles())
        task_wait_ttd = asyncio.create_task(self.wait_ttd())

        (done, pending) = await asyncio.wait(
            [
                task_wait_obstacles,
                task_wait_ttd,
            ],
            return_when=asyncio.FIRST_COMPLETED,
        )

        for t in pending:
            t.cancel()

        (who, data_msg) = done.pop().result()

        if who == "TTD":
            self.ttd = TimeToDecision.deserialize(data_msg.data)

        elif who == "ObstacleTrajectories":

            trajectory_list = json.loads(data_msg.data.decode("utf-8"))
            obstacle_trajectories = []
            for t in trajectory_list:
                obstacle_trajectories.append(ObstacleTrajectory.from_dict(t))

            obstacle_predictions_list = []

            (
                nearby_obstacle_trajectories,
                nearby_obstacles_ego_transforms,
            ) = get_nearby_obstacles_info(obstacle_trajectories, self.prediction_radius)
            num_predictions = len(nearby_obstacle_trajectories)

            for idx in range(len(nearby_obstacle_trajectories)):
                obstacle_trajectory = nearby_obstacle_trajectories[idx]
                # Time step matrices used in regression.
                num_steps = min(
                    self.prediction_num_past_steps,
                    len(obstacle_trajectory.trajectory),
                )
                ts = np.zeros((num_steps, 2))
                future_ts = np.zeros((self.prediction_num_future_steps, 2))
                for t in range(num_steps):
                    ts[t][0] = -t
                    ts[t][1] = 1
                for i in range(self.prediction_num_future_steps):
                    future_ts[i][0] = i + 1
                    future_ts[i][1] = 1

                xy = np.zeros((num_steps, 2))
                for t in range(num_steps):
                    # t-th most recent step
                    transform = obstacle_trajectory.trajectory[-(t + 1)]
                    xy[t][0] = transform.location.x
                    xy[t][1] = transform.location.y
                linear_model_params = np.linalg.lstsq(ts, xy, rcond=None)[0]
                # Predict future steps and convert to list of locations.
                predict_array = np.matmul(future_ts, linear_model_params)
                predictions = []
                for t in range(self.prediction_num_future_steps):
                    # Linear prediction does not predict vehicle orientation, so we
                    # use our estimated orientation of the vehicle at its latest
                    # location.
                    predictions.append(
                        Transform(
                            location=Location(
                                x=predict_array[t][0], y=predict_array[t][1]
                            ),
                            rotation=nearby_obstacles_ego_transforms[idx].rotation,
                        )
                    )
                obstacle_predictions_list.append(
                    ObstaclePrediction(
                        obstacle_trajectory,
                        obstacle_trajectory.obstacle.transform,
                        1.0,
                        predictions,
                    ).to_dict()
                )

            await self.output.send(
                json.dumps(obstacle_predictions_list).encode("utf-8")
            )

        return None

    def setup(
        self,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ) -> Callable[[], Any]:

        ttd_input = inputs.get("TTD", None)
        obstacles_input = inputs.get("ObstacleTrajectories", None)
        output = outputs.get("ObstaclePredictions", None)

        l = LinearPredictor(
            configuration,
            obstacles_input,
            ttd_input,
            output,
        )
        return l.run

    def finalize(self) -> None:
        return None


def register():
    return LinearPredictor