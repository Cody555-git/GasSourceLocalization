#!/usr/bin/env python3
"""Plot gas_field_survey.py output. Usage: plot_gas_field_survey.py <survey_dir>"""
import csv
import os
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

os.chdir(sys.argv[1])
c = list(csv.DictReader(open("survey_cells.csv")))
t = list(csv.DictReader(open("survey_timeseries.csv")))
x = np.array([float(r["x"]) for r in c])
y = np.array([float(r["y"]) for r in c])
m = np.array([float(r["max_ppm"]) for r in c])
f = np.array([float(r["frac_time_gt_floor"]) for r in c])

fig, ax = plt.subplots(1, 3, figsize=(17, 4.8))
panels = [
    (ax[0], f, "Fraction of time > 4.57 ppm floor (TGS2620)", dict(vmin=0, vmax=1, cmap="viridis")),
    (ax[1], np.log10(np.maximum(m, 1e-3)), "log10 max concentration [ppm] over 300 s", dict(cmap="magma")),
]
for a, v, title, kw in panels:
    s = a.scatter(x, y, c=v, s=14, marker="s", **kw)
    fig.colorbar(s, ax=a)
    a.plot(2.87, 0.30, "r*", ms=14, label="source (2.87, 0.30)")
    a.plot(-1.67, 0.99, "c^", ms=10, label="lv1 start")
    a.set_aspect("equal")
    a.set_title(title, fontsize=10)
    a.set_xlabel("x [m]")
    a.set_ylabel("y [m]")
    a.legend(fontsize=7, loc="lower left")
tt = [float(r["t_wall_s"]) for r in t]
ax[2].plot(tt, [int(r["n_cells_gt_floor"]) for r in t], label="cells > 4.57 ppm floor")
ax[2].axhline(len(c), ls="--", c="gray", label=f"cells with any gas (={len(c)})")
ax[2].set_xlabel("playback time [s] (player 10 Hz)")
ax[2].set_ylabel("cells (0.1 m grid, z = 0.11 m)")
ax[2].set_ylim(0, len(c) * 1.05)
ax[2].legend(fontsize=8)
ax[2].set_title("Cells above the TGS2620 floor over time", fontsize=10)
fig.suptitle("V3 lv1 GADEN gas field vs TGS2620 detection floor - th_gas_present = 5.0 ppm; "
             "clean-air reading 4.5735 ppm, sigma 0 (n=3000)", fontsize=10)
fig.tight_layout()
fig.savefig("gas_field_vs_tgs2620_floor.png", dpi=130)
start = (abs(x + 1.7) < 0.06) & (abs(y - 1.0) < 0.06)
print("cells frac>=0.5:", int((f >= 0.5).sum()), "of", len(f), "| median frac:", float(np.median(f)),
      "| start-cell frac:", f[start].tolist())
