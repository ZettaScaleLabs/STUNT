#!/usr/bin/env python3
import os
from io import StringIO
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import math

# os.environ["MODIN_ENGINE"] = "ray"  # Modin will use Dask

import pandas as pd

# import modin.pandas as pd
from pathlib import Path
import seaborn as sns
import sys
import argparse

# import ray

# ray.init()

palette = {
    "zenoh-flow": "tab:blue",
    "erdos": "tab:green",
}

styles = {
    "single": (0, 0),
    "multi": (1, 1),
}


# palette = 'bright' #sns.color_palette("bright", 6) #'plasma'
IMG_DIR = Path("img")


def pairwise(data):
    l = iter(data)
    return zip(l, l)


def bytes_label(n):
    kmod = pow(2, 10)
    kdiv = n / kmod
    if kdiv < 1:
        return "{}".format(n)

    mmod = pow(2, 20)
    mdiv = n / mmod
    if mdiv < 1:
        return "{0:.{c}f} KiB".format(kdiv, c=0 if n % kmod == 0 else 2)

    gmod = pow(2, 30)
    gdiv = n / gmod
    if gdiv < 1:
        return "{0:.{c}f} MiB".format(mdiv, c=0 if n % mmod == 0 else 2)

    tmod = pow(2, 40)
    tdiv = n / tmod
    if tdiv < 1:
        return "{0:.{c}f} GiB".format(gdiv, c=0 if n % gmod == 0 else 2)

    pmod = pow(2, 50)
    pdiv = n / pmod
    if pdiv < 1:
        return "{0:.{c}f} TiB".format(tdiv, c=0 if n % tmod == 0 else 2)

    emod = pow(2, 60)
    ediv = n / emod
    if ediv < 1:
        return "{0:.{c}f} PiB".format(pdiv, c=0 if n % pmod == 0 else 2)

    zmod = pow(2, 70)
    zdiv = n / zmod
    if zdiv < 1:
        return "{0:.{c}f} EiB".format(ediv, c=0 if n % emod == 0 else 2)

    ymod = pow(2, 80)
    ydiv = n / ymod
    if ydiv < 1:
        return "{0:.{c}f} ZiB".format(ediv, c=0 if n % zmod == 0 else 2)

    return "{0:.{c}f} YiB".format(ydiv, c=0 if n % ymod == 0 else 2)


def interval_label(n):
    if n == 0:
        return "inf"
    if n == 1:
        return "1"
    if n == 10:
        return "10"
    if n == 100:
        return "100"
    if n == 1000:
        return "1 K"
    if n == 10000:
        return "10 K"
    if n == 100000:
        return "100 K"
    if n == 1000000:
        return "1 M"

    return "1 M"


def convert_value(line):
    if line.unit == "s":
        return line.value
    if line.unit == "ms":
        return line.value / 1000
    if line.unit == "us":
        return line.value / 1000000
    if line.unit == "ns":
        return line.value / 1000000000


def read_log(log_dir):
    log = None
    for l in os.scandir(log_dir):
        if l.is_file():
            if log is None:
                log = pd.read_csv(l)
            else:
                log = pd.concat([log, pd.read_csv(l)])
    return log


def mask_first_and_last(x):
    mask = [True] * len(x)
    mask[0] = False
    mask[1] = False
    mask[-2] = False
    mask[-1] = False
    return mask


def prepare(log_dir, kind):
    log = read_log(log_dir)

    # filtering by kind of test
    log = log[log["test"] == kind]

    log["value"] = pd.to_numeric(log["value"], errors="coerce")
    if kind == "latency":
        # Remove first and last two samples of every test
        mask = log.groupby(
            ["framework", "scenario", "pipeline", "test", "payload", "rate"]
        ).transform(mask_first_and_last)["value"]
    elif kind == "throughput":
        # Remove first and last two samples of every test
        mask = log.groupby(
            ["framework", "scenario", "pipeline", "test", "payload", "rate"]
        ).transform(mask_first_and_last)["value"]
    log = log.loc[mask]

    if kind == "latency":
        # this converts everything to seconds, data is expected as micro seconds
        log["value"] = log.apply(convert_value, axis=1)
        log["label"] = [interval_label(v) for k, v in log["rate"].iteritems()]
        log.sort_values(by="rate", inplace=True, ascending=False)

    elif kind == "throughput":
        log["label"] = [bytes_label(v) for k, v in log["payload"].iteritems()]
        log.sort_values(by="payload", inplace=True)

    log["framework"] = log["framework"].astype(str)

    log = log.reset_index()

    return log


def filter(log, scenario=None, rate=None):
    layers = log["framework"].unique()

    if scenario is not None:
        # filtering if tcp or udl
        log = log[log["scenario"].isin([scenario])]

    # filtering is msg/s is set
    if rate is not None:
        log = log[log["rate"] == rate]

    return log


def rtt_ecfd_plot(log, scale, outfile):

    fig, axes = plt.subplots()

    g = sns.ecdfplot(
        data=log,
        x="value",
        palette=palette,
        hue="framework",
        label="framework",
    )

    plt.grid(which="major", color="grey", linestyle="-", linewidth=0.1)
    plt.grid(
        which="minor", color="grey", linestyle=":", linewidth=0.1, axis="y"
    )

    if scale == "log":
        g.set_xscale("log")

    plt.xticks(rotation=72.5)
    plt.xlabel("RTT (seconds)")

    # plt.legend(title='Legend', loc='center left', bbox_to_anchor=(1.0, 0.5))

    # ticker = mpl.ticker.EngFormatter(unit='')
    # axes.yaxis.set_major_formatter(ticker)

    plt.tight_layout()
    fig.savefig(IMG_DIR.joinpath(outfile))


def rtt_pdf_plot(log, scale, outfile):

    fig, axes = plt.subplots()

    g = sns.displot(
        data=log,
        x="value",
        palette=palette,
        hue="framework",
        label="framework",
    )

    plt.grid(which="major", color="grey", linestyle="-", linewidth=0.1)
    plt.grid(
        which="minor", color="grey", linestyle=":", linewidth=0.1, axis="y"
    )

    # if scale == 'log':
    #     g.set_xscale('log')

    plt.xticks(rotation=72.5)
    plt.xlabel("RTT (seconds)")

    # plt.legend(title='Legend', loc='center left', bbox_to_anchor=(1.0, 0.5))

    # ticker = mpl.ticker.EngFormatter(unit='')
    # axes.yaxis.set_major_formatter(ticker)

    plt.tight_layout()
    fig.savefig(IMG_DIR.joinpath(outfile))


def rtt_stat_plot(log, scale, outfile):

    fig, axes = plt.subplots()

    ticks = [1, 10, 100, 1000, 10000, 100000, 1000000, 10000000]

    g = sns.lineplot(
        data=log,
        x="label",
        y="value",
        palette=palette,
        ci=95,
        err_style="band",
        hue="framework",
        estimator=np.median,
        style="scenario",
        dashes=styles,
    )

    if scale == "log":
        g.set_yscale("log")

    plt.grid(which="major", color="grey", linestyle="-", linewidth=0.1)
    plt.grid(
        which="minor", color="grey", linestyle=":", linewidth=0.1, axis="y"
    )

    plt.xticks(rotation=72.5)
    plt.xlabel("Messages per seconds (msg/s)")

    # plt.yticks(ticks)
    plt.ylabel("Latency (seconds)")
    plt.legend(title="Legend", loc="center left", bbox_to_anchor=(1.0, 0.5))

    ticker = mpl.ticker.EngFormatter(unit="")
    axes.yaxis.set_major_formatter(ticker)

    plt.tight_layout()
    fig.savefig(IMG_DIR.joinpath(outfile))


def rtt_box_plot(log, scale, outfile):
    fig, axes = plt.subplots()

    ys = [math.pow(10, e) for e in range(-6, -0)]

    sns.set_style({"font.family": "serif", "font.serif": "Times New Roman"})
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = "Times New Roman"

    font = {"fontname": "Times New Roman"}

    g = sns.boxplot(
        data=log,
        x="rate",
        y="value",
        palette=palette,
        hue="framework",
        width=0.5,
        medianprops=dict(color="red", alpha=0.7),
        capprops=dict(linewidth=0.8),
        whiskerprops=dict(linewidth=0.8),
        flierprops=dict(marker="x", markersize=1),
        boxprops=dict(linewidth=0.8),
    )

    if scale == "log":
        g.set_yscale("log")

    plt.grid(which="major", color="grey", linestyle="-", linewidth=0.1)
    plt.grid(
        which="minor", color="grey", linestyle=":", linewidth=0.1, axis="y"
    )

    plt.xticks(rotation=72.5, **font)
    plt.xlabel("Messages per seconds (msg/s)", **font)

    plt.yticks(ys, **font)
    plt.ylabel("Latency (seconds)", **font)
    plt.legend(title="Legend", loc="upper left")

    ticker = mpl.ticker.EngFormatter(unit="")
    axes.yaxis.set_major_formatter(ticker)

    plt.tight_layout()
    fig.savefig(IMG_DIR.joinpath(outfile))


def throughput_stat_plot(log, scale, outfile):

    fig, axes = plt.subplots()

    g = sns.lineplot(
        data=log,
        x="label",
        y="value",
        palette=palette,
        ci=95,
        err_style="band",
        hue="framework",
        estimator=np.median,
        style="scenario",
        dashes=styles,
    )

    if scale == "log":
        g.set_yscale("log")

    plt.grid(which="major", color="grey", linestyle="-", linewidth=0.1)
    plt.grid(
        which="minor", color="grey", linestyle=":", linewidth=0.1, axis="y"
    )

    plt.xticks(rotation=72.5)
    plt.xlabel("Payload size (bytes)")

    plt.ylabel("Messages per second (msg/s)")
    plt.legend(title="Legend", loc="center left", bbox_to_anchor=(1.0, 0.5))

    ticker = mpl.ticker.EngFormatter(unit="")
    axes.yaxis.set_major_formatter(ticker)

    plt.tight_layout()
    fig.savefig(IMG_DIR.joinpath(outfile))


def main():
    parser = argparse.ArgumentParser(
        description="Parse zenoh flow performance results"
    )
    parser.add_argument(
        "-k",
        "--kind",
        help="Kind of the tests",
        required=False,
        choices=["latency", "throughput"],
        default="latency",
    )
    parser.add_argument(
        "-d", "--data", help="Logs directory", required=True, type=str
    )
    parser.add_argument(
        "-p",
        "--scenario",
        help="single or multi",
        choices=["single", "multi"],
        required=False,
    )
    parser.add_argument(
        "-t",
        "--type",
        help="Plot type",
        choices=["stat", "time", "ecdf", "pdf", "box"],
        default="stat",
        required=False,
    )
    parser.add_argument(
        "-s",
        "--scale",
        help="Plot scale",
        choices=["log", "lin"],
        default="log",
        required=False,
    )
    parser.add_argument(
        "-r", "--rate", help="Filter for this rate", required=False, type=float
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Output file name",
        required=False,
        type=str,
        default="plot.pdf",
    )

    args = vars(parser.parse_args())
    data = args["data"]
    print(f"[ START ] Processing data in { data }")

    if not os.path.exists(IMG_DIR):
        os.makedirs(IMG_DIR)

    log = prepare(args["data"], args["kind"])
    print(f"[ STEP1 ] Read a total of {log.size} samples")
    log = filter(log, args.get("scenario", None), args.get("rate", None))
    print(f"[ STEP2 ] After filtering we have {log.size} samples")
    if log.size == 0:
        print(f"[ ERR ] Cannot continue without samples!")
        exit(-1)

    if args["kind"] == "latency":
        if args["type"] == "stat":
            rtt_stat_plot(log, args["scale"], args["output"])
        if args["type"] == "box":
            rtt_box_plot(log, args["scale"], args["output"])
        elif args["type"] == "time":
            rtt_time_plot(log, args["scale"], args["output"])
        elif args["type"] == "ecdf":
            rtt_ecfd_plot(log, args["scale"], args["output"])
        elif args["type"] == "pdf":
            rtt_pdf_plot(log, args["scale"], args["output"])
    elif args["kind"] == "throughput":
        if args["type"] == "stat":
            throughput_stat_plot(log, args["scale"], args["output"])
        elif args["type"] == "time":
            print("Not implemented.")
            # rtt_time_plot(log, args['scale'], args['output'])
        elif args["type"] == "ecdf":
            print("Not implemented.")
            # rtt_ecfd_plot(log, args['scale'], args['output'])
        elif args["type"] == "pdf":
            print("Not implemented.")
            # rtt_pdf_plot(log, args['scale'], args['output'])

    out = IMG_DIR.joinpath(args["output"])
    print(f"[  DONE ] File saved to { out }")


if __name__ == "__main__":
    main()
