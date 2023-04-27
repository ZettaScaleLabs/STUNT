from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any
import logging
import json
import numpy as np

from stunt.types import (
    Transform,
    Location,
    ObstacleTrajectory,
    ObstaclePrediction,
    get_nearby_obstacles_info,
)


DEFAULT_PREDICTION_RADIUS = 50
DEFAULT_PREDICTION_PAST_NUM_STEPS = 5
DEFAULT_PREDICTION_FUTURE_NUM_STEPS = 10


class LinearPredictor(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
        configuration = configuration if configuration is not None else {}

        self.pending = []

        self.prediction_radius = configuration.get(
            "prediction_radius", DEFAULT_PREDICTION_RADIUS
        )
        self.prediction_num_past_steps = configuration.get(
            "prediction_num_past_steps", DEFAULT_PREDICTION_PAST_NUM_STEPS
        )

        self.prediction_num_future_steps = configuration.get(
            "prediction_num_future_steps", DEFAULT_PREDICTION_FUTURE_NUM_STEPS
        )

        self.obstacles_input = inputs.get("ObstacleTrajectories", None)
        self.output = outputs.get("ObstaclePredictions", None)

        if self.obstacles_input is None:
            raise ValueError("Cannot find input: 'ObstacleTrajectories'")
        if self.output is None:
            raise ValueError("Cannot find output: 'ObstaclePredictions'")

    async def iteration(self):
        try:
            data_msg = await self.obstacles_input.recv()
            trajectory_list = json.loads(data_msg.data.decode("utf-8"))
            # logging.debug(f"[LinearPredictor] received {len(trajectory_list)} trajectories")
            obstacle_trajectories = []
            for t in trajectory_list:
                obstacle_trajectories.append(
                    ObstacleTrajectory.from_dict(t)
                )

            obstacle_predictions_list = []

            (
                nearby_obstacle_trajectories,
                nearby_obstacles_ego_transforms,
            ) = get_nearby_obstacles_info(
                obstacle_trajectories, self.prediction_radius
            )

            # num_predictions = len(nearby_obstacle_trajectories)

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
                linear_model_params = np.linalg.lstsq(ts, xy, rcond=None)[
                    0
                ]
                # Predict future steps and convert to list of locations.
                predict_array = np.matmul(future_ts, linear_model_params)
                predictions = []
                for t in range(self.prediction_num_future_steps):
                    # Linear prediction does not predict
                    # vehicle orientation, so we use our estimated
                    # orientation of the vehicle at its latest location.
                    predictions.append(
                        Transform(
                            location=Location(
                                x=predict_array[t][0],
                                y=predict_array[t][1],
                            ),
                            rotation=nearby_obstacles_ego_transforms[
                                idx
                            ].rotation,
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

            # logging.debug(f"[LinearPredictor] computed {len(obstacle_predictions_list)}  predictions")
            await self.output.send(
                json.dumps(obstacle_predictions_list).encode("utf-8")
            )
        except Exception as e:
            logging.warning(f"[LinearPredictor] got error {e}")

        return None

    def finalize(self) -> None:
        return None


def register():
    return LinearPredictor
