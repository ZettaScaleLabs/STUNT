#!/usr/bin/env python3

from io import StringIO
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import os
import pandas as pd
from pathlib import Path
import seaborn as sns
import sys
import json

palette = {
    "sys-duration": "tab:blue",
    "duration": "tab:green",
    "ratio": "tab:red",
}

styles = {
    "single": (0, 0),
    "multi": (1, 1),
}
img_dir = Path("img")

# if not os.path.exists(img_dir):
#     os.makedirs(img_dir)


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


def read_log(log_dir):
    log = None
    for l in os.scandir(log_dir):
        if l.is_file():
            if log is None:
                log = pd.read_csv(l)
            else:
                log = pd.concat([log, pd.read_csv(l)], ignore_index=True)
    return log


log_dir = Path(sys.argv[1])
# Read tests logs
log = read_log(log_dir)
log = log.reindex()
fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

# ys = [math.pow(10,e) for e in range(-6,-0)]

sns.set_style({"font.family": "serif", "font.serif": "Times New Roman"})
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = "Times New Roman"

font = {"fontname": "Times New Roman"}

non_ratio_log = log[log["type"] != "ratio"]
g = sns.boxplot(
    data=non_ratio_log,
    x="scenario",
    y="value",
    palette=palette,
    hue="type",
    width=0.5,
    medianprops=dict(color="red", alpha=0.7),
    capprops=dict(linewidth=0.8),
    whiskerprops=dict(linewidth=0.8),
    flierprops=dict(marker="x", markersize=1),
    boxprops=dict(linewidth=0.8),
    ax=ax1,
)

ratio_log = log[log["type"] == "ratio"]
print(ratio_log)
g2 = sns.lineplot(
    data=ratio_log,
    x="scenario",
    y="value",
    palette=palette,
    hue="type",
    ci=95,
    err_style="band",
    estimator=np.median,
    ax=ax2,
    # dodge=True,
)

# if scale == 'log':
#     g.set_yscale('log')

# plt.grid(which='major', color='grey', linestyle='-', linewidth=0.1)
# plt.grid(which='minor', color='grey', linestyle=':', linewidth=0.1, axis='y')

plt.xticks(rotation=72.5, **font)
plt.xlabel("Scenarios", **font)

plt.ylabel("Execution Time (s)", **font)
plt.legend(title="Legend", loc="upper left")

ticker = mpl.ticker.EngFormatter(unit="")
ax1.yaxis.set_major_formatter(ticker)
ax2.set_ylabel("Ratio", **font)

plt.tight_layout()
# fig.savefig(IMG_DIR.joinpath(outfile))
plt.show()
