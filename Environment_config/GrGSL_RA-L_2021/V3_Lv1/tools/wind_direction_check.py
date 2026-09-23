#!/usr/bin/env python3
"""V3 lv1 wind-direction check for spec-gazebo-measured-urdf sec.6 item 6.

Needs gaden_player running plus a simulated_anemometer (noise_std 0) whose
sensor_frame is "anemo_test" and topic /anemo_test/reading. This node owns the
map -> anemo_test TF and steps it through points x roll x yaw.

For every anemometer reading it asks /wind_value at the same point and logs
  gaden_up     upwind direction in map from GADEN's (u, v)   = atan2(-v, -u)
  msg_dir      what simulated_anemometer published (sensor frame)
  gsl_map_dir  msg_dir brought back to map exactly like
               Algorithm::windCallback does (yaw pose transformed by TF)
  err_deg      wrap(gsl_map_dir - gaden_up)
roll=0 is the URDF anemometer_link (no rotation); roll=pi is the old
static_transform_publisher "0 0 0.1 1 0 0 0" (qx=1) of Exp_A.

Output (in --out): wind_direction_check.csv
"""
import argparse
import csv
import math
import os
import time

import rclpy
from gaden_msgs.srv import WindPosition
from geometry_msgs.msg import TransformStamped
from olfaction_msgs.msg import Anemometer
from tf2_ros import TransformBroadcaster

POINTS = [(-1.67, 0.99), (-1.0, 0.3), (0.0, 0.3), (1.0, 0.3), (2.0, 0.3)]
ROLLS = [0.0, math.pi]
YAWS_DEG = [0, 90, 180, -90, 45]


def quat_rpy(roll, yaw):
    # R = Rz(yaw) * Rx(roll)
    qz = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
    qx = (math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0)
    return qmul(qz, qx)


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--z", type=float, default=0.2)
    ap.add_argument("--rounds", type=int, default=2)
    ap.add_argument("--per_config", type=int, default=10)
    ap.add_argument("--settle", type=float, default=1.5)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("wind_direction_check")
    br = TransformBroadcaster(node)
    cli = node.create_client(WindPosition, "/wind_value")
    latest = {"msg": None, "n": 0}

    def on_msg(m):
        latest["msg"] = m
        latest["n"] += 1

    node.create_subscription(Anemometer, "/anemo_test/reading", on_msg, 20)
    while not cli.wait_for_service(timeout_sec=2.0):
        print("waiting for /wind_value")

    cfg = {"x": 0.0, "y": 0.0, "roll": 0.0, "yaw": 0.0}

    def send_tf():
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "anemo_test"
        t.transform.translation.x = cfg["x"]
        t.transform.translation.y = cfg["y"]
        t.transform.translation.z = args.z
        q = quat_rpy(cfg["roll"], cfg["yaw"])
        t.transform.rotation.w, t.transform.rotation.x = q[0], q[1]
        t.transform.rotation.y, t.transform.rotation.z = q[2], q[3]
        br.sendTransform(t)

    node.create_timer(0.02, send_tf)

    path = os.path.join(args.out, "wind_direction_check.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["round", "x", "y", "roll_deg", "yaw_deg", "t", "u", "v",
                    "speed", "gaden_up_deg", "msg_dir_deg", "gsl_map_dir_deg",
                    "err_deg"])
        t0 = time.time()
        for rnd in range(args.rounds):
            for (x, y) in POINTS:
                for roll in ROLLS:
                    for yaw_deg in YAWS_DEG:
                        cfg.update(x=x, y=y, roll=roll, yaw=math.radians(yaw_deg))
                        end = time.time() + args.settle
                        while time.time() < end:
                            rclpy.spin_once(node, timeout_sec=0.02)
                        got = 0
                        seen = latest["n"]
                        while got < args.per_config:
                            rclpy.spin_once(node, timeout_sec=0.05)
                            if latest["n"] == seen:
                                continue
                            seen = latest["n"]
                            m = latest["msg"]
                            req = WindPosition.Request()
                            req.x, req.y, req.z = [x], [y], [args.z]
                            fut = cli.call_async(req)
                            rclpy.spin_until_future_complete(node, fut, timeout_sec=1.0)
                            if not fut.done():
                                continue
                            r = fut.result()
                            u, v = r.u[0], r.v[0]
                            up = math.atan2(-v, -u)
                            q_msg = (math.cos(m.wind_direction / 2), 0.0, 0.0,
                                     math.sin(m.wind_direction / 2))
                            gsl = yaw_of(qmul(quat_rpy(roll, math.radians(yaw_deg)), q_msg))
                            w.writerow([rnd, x, y, round(math.degrees(roll)), yaw_deg,
                                        round(time.time() - t0, 2), u, v,
                                        math.hypot(u, v), math.degrees(up),
                                        math.degrees(m.wind_direction),
                                        math.degrees(gsl),
                                        math.degrees(wrap(gsl - up))])
                            got += 1
                        f.flush()
                        print(f"r{rnd} ({x},{y}) roll={math.degrees(roll):.0f} yaw={yaw_deg} done", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
