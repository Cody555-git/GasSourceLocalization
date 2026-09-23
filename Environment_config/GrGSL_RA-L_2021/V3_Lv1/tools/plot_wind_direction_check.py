#!/usr/bin/env python3
"""Plots wind_direction_check.csv (spec-gazebo-measured-urdf sec.6 item 6).

Left: angle error vs robot yaw for the unrotated URDF anemometer_link
(roll 0) and the roll-180deg frame, with the 2*yaw prediction and the
+-15deg band. Right: per-point arrows (GADEN upwind vs what GrGSL sees)
at yaw 90deg for both frames.
Usage: plot_wind_direction_check.py <dir containing the CSV>
"""
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

d = sys.argv[1]
rows = list(csv.DictReader(open(os.path.join(d, "wind_direction_check.csv"))))
f = lambda r, k: float(r[k])

fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 5.2))
yaws = np.array([-90, 0, 45, 90, 180])
ax.axhspan(-15, 15, color="0.85", label="±15° (pass band)")
line = np.linspace(-90, 180, 50)
ax.plot(line, [math.degrees(math.atan2(math.sin(math.radians(2 * y)), math.cos(math.radians(2 * y)))) for y in line],
        ":", color="C3", lw=1, label="prediction: wrap(2·yaw)")
for roll, c, mk, lab in [(0, "C3", "o", "roll 0 (URDF anemometer_link as is)"),
                         (180, "C0", "s", "roll 180° (anemometer_gaden_link, fix)")]:
    sel = [r for r in rows if int(f(r, "roll_deg")) == roll]
    x = np.array([f(r, "yaw_deg") for r in sel]) + (-3 if roll == 0 else 3)
    y = np.array([f(r, "err_deg") for r in sel])
    y = np.where(np.isclose(np.abs(y), 180), 180, y)
    ax.scatter(x, y, s=18, color=c, marker=mk, alpha=0.5, label=f"{lab} (n={len(sel)})")
ax.set_xticks(yaws)
ax.set_xlabel("robot (sensor) yaw in map [deg]")
ax.set_ylabel("error = GrGSL wind dir in map − GADEN upwind [deg]")
ax.set_title("Wind direction end-to-end error (5 points × 2 rounds × 10 readings)")
ax.set_ylim(-20, 200)
ax.legend(fontsize=8, loc="upper left")
ax.grid(alpha=0.3)

pts = sorted({(f(r, "x"), f(r, "y")) for r in rows})
for (px, py) in pts:
    for roll, c, dy in [(0, "C3", -0.12), (180, "C0", 0.12)]:
        r = next(r for r in rows if f(r, "x") == px and f(r, "y") == py
                 and int(f(r, "roll_deg")) == roll and int(f(r, "yaw_deg")) == 90)
        up, gs = math.radians(f(r, "gaden_up_deg")), math.radians(f(r, "gsl_map_dir_deg"))
        ax2.annotate("", xy=(px + 0.35 * math.cos(up), py + dy + 0.35 * math.sin(up)), xytext=(px, py + dy),
                     arrowprops=dict(arrowstyle="->", color="k", lw=2))
        ax2.annotate("", xy=(px + 0.35 * math.cos(gs), py + dy + 0.35 * math.sin(gs)), xytext=(px, py + dy),
                     arrowprops=dict(arrowstyle="->", color=c, lw=1.5, ls="--"))
    ax2.text(px, py - 0.35, f"{f(r, 'speed'):.2f} m/s", ha="center", fontsize=7)
ax2.plot([], [], "k-", label="GADEN upwind (truth)")
ax2.plot([], [], "C3--", label="GrGSL sees, roll 0 (lower arrows)")
ax2.plot([], [], "C0--", label="GrGSL sees, roll 180° fix (upper arrows)")
ax2.set_xlim(-2.4, 2.7); ax2.set_ylim(-0.4, 1.5); ax2.set_aspect("equal")
ax2.set_xlabel("x [m]"); ax2.set_ylabel("y [m]")
ax2.set_title("Robot yaw = 90°: arrows at each probe point (lower = roll 0)")
ax2.legend(fontsize=8, loc="upper right"); ax2.grid(alpha=0.3)
fig.tight_layout()
out = os.path.join(d, "wind_direction_check.png")
fig.savefig(out, dpi=130)
print(out)
