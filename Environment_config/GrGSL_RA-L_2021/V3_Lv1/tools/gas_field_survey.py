#!/usr/bin/env python3
"""V3 lv1 gas-field survey for spec-gazebo-measured-urdf sec.6 item 0 (b, c).

Needs gaden_player running (gaden_player_v3.launch.py use_rviz:=False) plus a
simulated_gas_sensor node named "probe" (sensor_model 0 = TGS2620) whose
frame sits somewhere gas never reaches.

c) Every --period s, asks /odor_value for a grid at height --z and counts
   cells above the TGS2620 floor (4.573 ppm: below it GADEN's MOX model
   clamps RS/R0 at the clean-air value, so the sensor cannot tell).
b) Converts every probe/Sensor_reading with the same formula as the fixed
   Algorithm::ppmFromGasMsg (R0 by mpn), i.e. what GrGSL would see.

Outputs (in --out): survey_timeseries.csv, survey_cells.csv, probe_ppm.csv
"""
import argparse
import csv
import math
import os
import time

import numpy as np
import rclpy
from gaden_msgs.srv import GasPosition
from olfaction_msgs.msg import GasSensor

R0_BY_MPN = {
    GasSensor.MPN_TGS2620: 3000.0,
    GasSensor.MPN_TGS2600: 50000.0,
    GasSensor.MPN_TGS2611: 3740.0,
    GasSensor.MPN_TGS2610: 3740.0,
    GasSensor.MPN_TGS2612: 4500.0,
}
# TGS2620 ethanol: RS/R0 = 62.32 * c^-0.7155, clean air RS/R0 = 21
FLOOR_PPM = (21.0 / 62.32) ** (1.0 / -0.7155)


def ppm_from_msg(msg):
    if msg.raw_units == msg.UNITS_OHM:
        rs_r0 = msg.raw / R0_BY_MPN.get(msg.mpn, 50000.0)
        return (rs_r0 / msg.calib_a) ** (1.0 / msg.calib_b)
    return msg.raw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--z", type=float, default=0.11)
    ap.add_argument("--step", type=float, default=0.1)
    ap.add_argument("--xmin", type=float, default=-3.0)
    ap.add_argument("--xmax", type=float, default=4.0)
    ap.add_argument("--ymin", type=float, default=-2.5)
    ap.add_argument("--ymax", type=float, default=2.5)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("gas_field_survey")
    cli = node.create_client(GasPosition, "/odor_value")
    probe = []
    t0 = time.time()
    node.create_subscription(
        GasSensor, "probe/Sensor_reading",
        lambda m: probe.append((time.time() - t0, m.raw, m.mpn, ppm_from_msg(m))), 10)
    if not cli.wait_for_service(timeout_sec=30.0):
        raise SystemExit("/odor_value not available")
    t0 = time.time()

    xs = np.arange(a.xmin, a.xmax + 1e-9, a.step)
    ys = np.arange(a.ymin, a.ymax + 1e-9, a.step)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    req = GasPosition.Request()
    req.x = gx.ravel().tolist()
    req.y = gy.ravel().tolist()
    req.z = [a.z] * gx.size

    cmax = np.zeros(gx.size)
    n_above = np.zeros(gx.size, dtype=int)
    n_samples = 0
    rows = []
    next_t = 0.0
    while time.time() - t0 < a.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - t0 < next_t:
            continue
        next_t += a.period
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            continue
        c = np.array([sum(p.concentration) if len(p.concentration) else 0.0 for p in res.positions])
        cmax = np.maximum(cmax, c)
        n_above += c > FLOOR_PPM
        n_samples += 1
        rows.append((round(time.time() - t0, 2), int((c > 0).sum()), int((c > FLOOR_PPM).sum()),
                     float(c.max()), float(np.percentile(c[c > 0], 99)) if (c > 0).any() else 0.0))
        print(f"t={rows[-1][0]:6.1f}s  >0:{rows[-1][1]:5d}  >{FLOOR_PPM:.2f}ppm:{rows[-1][2]:4d}  max={rows[-1][3]:.2f}", flush=True)

    with open(os.path.join(a.out, "survey_timeseries.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_wall_s", "n_cells_gt0", "n_cells_gt_floor", "max_ppm", "p99_ppm_of_gt0"])
        w.writerows(rows)
    with open(os.path.join(a.out, "survey_cells.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "max_ppm", "frac_time_gt_floor"])
        for i in range(gx.size):
            if cmax[i] > 0:
                w.writerow([f"{req.x[i]:.2f}", f"{req.y[i]:.2f}", f"{cmax[i]:.4f}", f"{n_above[i] / max(n_samples, 1):.4f}"])
    with open(os.path.join(a.out, "probe_ppm.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_wall_s", "raw_ohm", "mpn", "ppm_grgsl_formula"])
        w.writerows(probe)

    pp = np.array([p[3] for p in probe]) if probe else np.array([math.nan])
    print(f"SUMMARY floor_ppm={FLOOR_PPM:.4f} samples={n_samples} grid={gx.size} "
          f"cells_ever_gt0={(cmax > 0).sum()} cells_ever_gt_floor={(cmax > FLOOR_PPM).sum()} "
          f"cells_gt_floor_ge10pct_time={(n_above / max(n_samples, 1) >= 0.1).sum()} "
          f"probe_n={len(probe)} probe_mean={pp.mean():.4f} probe_std={pp.std():.6f} probe_max={pp.max():.4f}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
