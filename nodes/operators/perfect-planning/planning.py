from zenoh_flow.interfaces import Operator
from zenoh_flow import DataReceiver, DataSender
from typing import Dict, Any, Callable
import time
import asyncio

import carla

DEFAULT_CARLA_HOST = "localhost"
DEFAULT_CARLA_PORT = 2000


class State(object):
    def __init__(self, configuration):
        configuration = configuration if configuration is not None else {}

        self.carla_port = configuration.get("port", DEFAULT_CARLA_PORT)
        self.carla_host = configuration.get("host", DEFAULT_CARLA_HOST)

        self.carla_world = None
        self.carla_client = None

        # Connecting to CARLA to get the map
        self.carla_client = carla.Client(self.carla_host, self.carla_port)
        self.carla_world = self.carla_client.get_world()

        # Getting carla map
        self.map = self.carla_world.get_map().to_opendrive()


class PerfectPlanning(Operator):
    def __init__(self, state, gnss_input, imu_input, output):
        self.state = state
        self.gnss_input = gnss_input
        self.imu_input = imu_input
        self.output = output

    async def wait_gnss(self):
        data = await self.gnss_input.recv()
        return ("GNSS", data)

    async def wait_imu(self):
        data = await self.imu_input.recv()
        return ("IMU", data)

    async def wait_pose(self):
        data = await self.pose_input.recv()
        return ("Pose", data)

    async def run(self):

        # wait for one of the input to be available
        (iid, data) = await asyncio.wait(
            [self.wait_gnss(), self.wait_imu(), self.wait_pose()],
            return_when=asyncio.FIRST_COMPLETED,
        )

        # This is perfect localization, so does nothing with IMU and GNSS data
        if iid == "GNSS":
            return None
        elif iid == "IMU":
            return None
        elif iid == "Pose":
            # perfect localization, forwards the localization received from the
            # ground truth
            await self.output.send(data.data)

        return None

    def setup(
        self,
        configuration: Dict[str, Any],
        inputs: Dict[str, DataReceiver],
        outputs: Dict[str, DataSender],
    ) -> Callable[[], Any]:

        obs_predictions_input = inputs.get("ObstaclePredictions", None)
        static_obs_input = inputs.get("StaticObstacles", None)
        trajectory.input = inputs.get("Trajectory", None)
        pose_input = inputs.get("Pose", None)

        output = outputs.get("Pose", None)
        l = PerfectPlanning(gnss_input, imu_input, output)
        return l.run

    def finalize(self) -> None:
        return None


def register():
    return PerfectPlanning
