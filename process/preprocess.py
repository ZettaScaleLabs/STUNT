#!/usr/bin/env python3


import os
import pandas as pd
import json
import argparse


def read_log(log_dir, scenario_id, framework):
    log = None
    for l in os.scandir(log_dir):
        if l.is_file():
            dict_df = {
                "value": [],
                "scenario": [],
                "type": [],
                "framework": [],
            }

            with open(l) as log_file:
                raw_data = log_file.read()
            data = json.loads(raw_data)

            durations = []
            sys_duration = []

            for e in data["criteria"]:
                if e["name"] == "Duration":
                    durations.append(e)
                elif e["name"] == "SystemDuration":
                    sys_duration.append(e)

            for i in range(0,len(durations)):
                duration = durations[i]["actual"]
                sys_time = sys_duration[i]["actual"]

                dict_df["value"].append(sys_time)
                dict_df["scenario"].append(scenario_id)
                dict_df["type"].append("sys-duration")
                dict_df["framework"].append(framework)

                dict_df["value"].append(duration)
                dict_df["type"].append("duration")
                dict_df["scenario"].append(scenario_id)
                dict_df["framework"].append(framework)

                dict_df["value"].append(duration / sys_time)
                dict_df["type"].append("ratio")
                dict_df["scenario"].append(scenario_id)
                dict_df["framework"].append(framework)

                df = pd.DataFrame.from_dict(dict_df)

            if log is None:
                log = df
            else:
                log = log.append(df)
    log = log.reset_index()
    return log


def main():

    parser = argparse.ArgumentParser(
        description="Pre processing CARLA results"
    )
    parser.add_argument(
        "-s", "--scenario", help="ID of the scenariop", required=True, type=int
    )
    parser.add_argument(
        "-d", "--data", help="Logs directory", required=True, type=str
    )

    parser.add_argument(
        "-f", "--framework", help="Framework", required=True, type=str
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Output file name",
        required=False,
        type=str,
        default="scenario.csv",
    )

    args = vars(parser.parse_args())
    log_dir = args["data"]
    scenario_id = args["scenario"]
    output_file = args["output"]
    framework = args["framework"]

    log = read_log(log_dir, scenario_id, framework)
    print(f"Read a total of {len(log)} samples")
    log.to_csv(output_file, index=False, header=True)
    print(f"Processed data saved into {output_file}")


if __name__ == "__main__":
    main()
