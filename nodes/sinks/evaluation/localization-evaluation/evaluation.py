from zenoh_flow.interfaces import Sink
from zenoh_flow import DataReceiver
from zenoh_flow.types import Context
from typing import Dict, Any, Callable
import json

import asyncio
from stunt.types import Pose


DEFAULT_OUTPUT_FILE = "/tmp/location-evaluation.csv"


class EvaluationLocalization(Sink):
    def finalize(self):
        return None

    def __init__(self, context, configuration, inputs):

        self.pending = []

        configuration = {} if configuration is None else configuration

        self.output_file = open(
            configuration.get("out_file", DEFAULT_OUTPUT_FILE), "w+"
        )

        self.output_file.write(
            (
                "module,"
                "perfect.transform.location.x,"
                "perfect.transform.location.y,"
                "perfect.transform.location.z,"
                "perfect.transform.rotation.pitch,"
                "perfect.transform.rotation.yaw,"
                "perfect.transform.rotation.roll,"
                "perfect.forward_speed,"
                "perfect.velocity_vector.x,"
                "perfect.velocity_vector.y,"
                "perfect.velocity_vector.z,"
                "perfect.localization_time,"
                "pose.transform.location.x,"
                "pose.transform.location.y,"
                "pose.transform.location.z,"
                "pose.transform.rotation.pitch,"
                "pose.transform.rotation.yaw,"
                "pose.transform.rotation.roll,"
                "pose.forward_speed,"
                "pose.velocity_vector.x,"
                "pose.velocity_vector.y,"
                "pose.velocity_vector.z,"
                "pose.localization_time"
                "\n"
            )
        )
        self.output_file.flush()

        self.perfect_input = inputs.get("Perfect", None)
        self.location_input = inputs.get("Localization", None)

        self.perfect = None
        self.location = None

    async def wait_perfect(self):
        data_msg = await self.perfect_input.recv()
        return ("Perfect", data_msg)

    async def wait_location(self):
        data_msg = await self.location_input.recv()
        return ("Localization", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "Localization" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_location(), name="Localization")
            )

        if not any(t.get_name() == "Perfect" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_perfect(), name="Perfect")
            )
        return task_list

    async def iteration(self):
        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.ALL_COMPLETED,
        )

        self.pending = list(pending)

        for d in done:

            (who, data_msg) = d.result()

            if who == "Perfect":
                self.perfect = Pose.deserialize(data_msg.data)

            elif who == "Localization":
                pose = Pose.deserialize(data_msg.data)

                if self.perfect is None:
                    return None

                line = (
                    "localization,"
                    f"{self.perfect.transform.location.x},"
                    f"{self.perfect.transform.location.y},"
                    f"{self.perfect.transform.location.z},"
                    f"{self.perfect.transform.rotation.pitch},"
                    f"{self.perfect.transform.rotation.yaw},"
                    f"{self.perfect.transform.rotation.roll},"
                    f"{self.perfect.forward_speed},"
                    f"{self.perfect.velocity_vector.x},"
                    f"{self.perfect.velocity_vector.y},"
                    f"{self.perfect.velocity_vector.z},"
                    f"{self.perfect.localization_time},"
                    f"{pose.transform.location.x},"
                    f"{pose.transform.location.y},"
                    f"{pose.transform.location.z},"
                    f"{pose.transform.rotation.pitch},"
                    f"{pose.transform.rotation.yaw},"
                    f"{pose.transform.rotation.roll},"
                    f"{pose.forward_speed},"
                    f"{pose.velocity_vector.x},"
                    f"{pose.velocity_vector.y},"
                    f"{pose.velocity_vector.z},"
                    f"{pose.localization_time}"
                    "\n"
                )
                self.output_file.write(line)
                self.output_file.flush()


def register():
    return EvaluationLocalization
