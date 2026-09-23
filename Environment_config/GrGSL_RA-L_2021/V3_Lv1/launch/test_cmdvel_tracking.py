#!/usr/bin/env python3
"""
Persistent rclpy node for cmd_vel tracking accuracy test (turn10).
Replaces turn9's short-lived `ros2 topic pub` per-phase approach, which
showed timing-dependent results (rotation 2x target, then 0x target)
suspected to be caused by DDS discovery jitter on freshly-spawned processes.

stop_and_confirm now checks BOTH position AND yaw convergence (turn9 bug:
in-place rotation barely moves position, so a position-only check declared
"stopped" while the robot kept spinning).

Usage (after gazebo_v3_lv1.launch.py is up and nav2/gsl nodes have been
killed so the raw /cmd_vel isn't fought by a controller -- see turn10's
turn10_launch_and_freeze.sh recipe in research/loop/2026-09-23.md):
    python3 test_cmdvel_tracking.py <n_trials> <output_csv>

Trials CHAIN (trial N's end pose is trial N+1's baseline), so a bad
measurement in one trial's rotation phase contaminates the next trial's
forward-phase yaw. For an uncontaminated single measurement, re-launch
fresh and run with n_trials=1.

Turn10 finding (reproduced identically across 2 independent fresh-launch
n=1 runs, spawn pose (-1.67, 0.99)): forward 1 m tracks within +3.3mm
(well inside the +-5cm spec), but in-place 90 deg rotation reproducibly
undershoots by exactly -5.275 deg (84.725 deg actual) -- outside the
+-5deg spec by 0.275 deg. Real-time-factor was confirmed ~1.0 via
`ign topic -e -t /world/<world>/stats` during the command window, ruling
out sim slowdown. The deficit is a FIXED offset, not proportional noise
(byte-identical across runs), so it reads as a systematic delay/ramp in
VelocityControl's angular channel rather than DDS/measurement jitter.
Root cause not yet confirmed -- see spec-gazebo-measured-urdf Done-when
item 4 and research/loop/2026-09-23.md turn 10 for the open hypothesis
and next diagnostic step.
"""
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node

TOPIC_CMDVEL = "/cmd_vel"
TOPIC_GT = "/TurtleBot3Waffle/ground_truth"
N_TRIALS = int(sys.argv[1]) if len(sys.argv) > 1 else 3
RESULT_CSV = sys.argv[2] if len(sys.argv) > 2 else "/tmp/turn10_result.csv"

FWD_SPEED = 0.2  # m/s
FWD_DURATION = 5.0  # s -> target 1.0 m
ROT_SPEED = 0.5  # rad/s
ROT_DURATION = (math.pi / 2) / ROT_SPEED  # -> target 90 deg


def yaw_from_quat(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angdiff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class CmdVelTester(Node):
    def __init__(self):
        super().__init__("cmdvel_tracking_tester")
        self.pub = self.create_publisher(Twist, TOPIC_CMDVEL, 10)
        self.pose = None  # (x, y, yaw)
        self.create_subscription(PoseWithCovarianceStamped, TOPIC_GT, self._gt_cb, 10)
        self.log = []

    def _gt_cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.pose = (p.x, p.y, yaw)

    def spin_until_pose(self, timeout=5.0):
        t0 = time.time()
        while self.pose is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose

    def publish_for(self, vx, wz, duration):
        """Publish at 10 Hz for `duration` seconds from THIS persistent node
        (no per-call process spawn -> no DDS discovery jitter)."""
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        t0 = time.time()
        while time.time() - t0 < duration:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.1)

    def stop_and_confirm(self, max_iters=20, pos_eps=0.001, yaw_eps=0.001):
        """Send zero twist, then poll ground_truth until BOTH position and
        yaw are stable between two samples 0.3s apart. Fixes turn9 bug where
        only position was checked (missed in-place rotation not stopping)."""
        zero = Twist()
        for _ in range(5):
            self.pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)
        for i in range(max_iters):
            p1 = self.spin_until_pose()
            time.sleep(0.3)
            rclpy.spin_once(self, timeout_sec=0.1)
            p2 = self.pose
            dpos = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            dyaw = abs(angdiff(p2[2], p1[2]))
            self.log.append(f"stop_and_confirm iter={i} dpos={dpos:.5f} dyaw_deg={math.degrees(dyaw):.3f}")
            if dpos < pos_eps and dyaw < yaw_eps:
                return True
            for _ in range(5):
                self.pub.publish(zero)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.02)
        self.log.append(f"stop_and_confirm: DID NOT CONVERGE after {max_iters} iters")
        return False

    def run_trial(self, trial_idx):
        row = {"trial": trial_idx}

        self.stop_and_confirm()
        rclpy.spin_once(self, timeout_sec=0.2)
        baseline = self.pose
        row["baseline_x"] = baseline[0]
        row["baseline_y"] = baseline[1]
        row["baseline_yaw_deg"] = math.degrees(baseline[2])

        # forward
        self.publish_for(FWD_SPEED, 0.0, FWD_DURATION)
        self.stop_and_confirm()
        rclpy.spin_once(self, timeout_sec=0.2)
        after_fwd = self.pose
        fwd_dist = math.hypot(after_fwd[0] - baseline[0], after_fwd[1] - baseline[1])
        fwd_yaw_drift = math.degrees(abs(angdiff(after_fwd[2], baseline[2])))
        row["fwd_dist_m"] = fwd_dist
        row["fwd_target_m"] = FWD_SPEED * FWD_DURATION
        row["fwd_error_m"] = fwd_dist - FWD_SPEED * FWD_DURATION
        row["fwd_yaw_drift_deg"] = fwd_yaw_drift

        # rotation (in place, from post-forward pose)
        self.publish_for(0.0, ROT_SPEED, ROT_DURATION)
        self.stop_and_confirm()
        rclpy.spin_once(self, timeout_sec=0.2)
        after_rot = self.pose
        rot_deg = math.degrees(abs(angdiff(after_rot[2], after_fwd[2])))
        rot_pos_drift = math.hypot(after_rot[0] - after_fwd[0], after_rot[1] - after_fwd[1])
        row["rot_deg"] = rot_deg
        row["rot_target_deg"] = math.degrees(ROT_DURATION * ROT_SPEED)
        row["rot_error_deg"] = rot_deg - math.degrees(ROT_DURATION * ROT_SPEED)
        row["rot_pos_drift_m"] = rot_pos_drift

        return row


def main():
    rclpy.init()
    node = CmdVelTester()
    node.spin_until_pose(timeout=10.0)
    if node.pose is None:
        print("ERROR: never received ground_truth pose", file=sys.stderr)
        sys.exit(1)

    rows = []
    for t in range(N_TRIALS):
        print(f"=== trial {t} start, pose={node.pose} ===")
        row = node.run_trial(t)
        print(row)
        rows.append(row)

    with open(RESULT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    with open(RESULT_CSV + ".stopconfirm.log", "w") as f:
        f.write("\n".join(node.log))

    print(f"wrote {RESULT_CSV}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
