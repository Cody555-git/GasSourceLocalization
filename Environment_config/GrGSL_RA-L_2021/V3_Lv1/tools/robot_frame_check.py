#!/usr/bin/env python3
"""
sim-recheck-checklist step 1 (shape and frames), measured against the running
robot_only.launch.py (or the full launch):

  A. TF sensor frames vs the 2026-09-18 measured mount positions
  B. Gazebo's own link poses vs robot_state_publisher's TF (same robot twice?)
  C. Resting pose (on the floor, level)
  D. Axis directions by motion: +x cmd -> forward, +y cmd -> left, +wz -> CCW

    python3 robot_frame_check.py            # prints a table, exit code 1 on any NG

D judges direction only. The printed speed is the average over the command
window including start-up -- how well speed is tracked is step 2, not this.
D moves the robot about 0.4 m forward / left from where it stands; put it
somewhere clear first. The robot is always sent a zero command at the end.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

PREFIX = "TurtleBot3Waffle_"
MODEL_NAME = "osl_robot"

# research/robot-rebuild-plan.md sec.2.5 (measured 2026-09-18). x forward, y left
# from base_link; z is height above the floor. [m]
MEASURED = {
    "base_scan": (0.1575, 0.0, 0.230),
    "gas_front_top": (0.1275, 0.0, 0.160),
    "gas_front_mid": (0.1275, 0.0, 0.110),
    "gas_front_bottom": (0.1275, 0.0, 0.060),
    "gas_left": (0.0275, 0.2075, 0.070),
    "gas_right": (-0.0275, -0.2075, 0.070),
    "gas_rear": (-0.2225, 0.030, 0.070),
    "anemometer_link": (0.0, 0.0, 0.050),
    "front_left_steer_link": (0.1275, 0.1275, 0.0325),
    "front_right_steer_link": (0.1275, -0.1275, 0.0325),
    "rear_left_steer_link": (-0.1275, 0.1275, 0.0325),
    "rear_right_steer_link": (-0.1275, -0.1275, 0.0325),
}
GZ_LINKS = [f"{c}_{k}_link" for c in ("front_left", "front_right", "rear_left", "rear_right") for k in ("steer", "wheel")]

POS_TOL = 0.001      # 1 mm
ANG_TOL = 0.5        # deg
CMD_LIN = 0.2        # m/s
CMD_ANG = 0.5        # rad/s
CMD_SEC = 2.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rpy_deg(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw_of(q)))


class Check(Node):
    def __init__(self):
        super().__init__("robot_frame_check")
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.gt = None
        self.gz = {}
        self.create_subscription(PoseWithCovarianceStamped, f"/{PREFIX[:-1]}/ground_truth", self.on_gt, 10)
        self.create_subscription(TFMessage, "/gz_all_poses", self.on_gz, QoSPresetProfiles.SENSOR_DATA.value)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.rows = []
        self.ng = 0

    def on_gt(self, msg):
        self.gt = msg

    def on_gz(self, msg):
        for t in msg.transforms:
            self.gz[t.child_frame_id] = t.transform

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def row(self, section, name, got, want, ok):
        self.ng += 0 if ok else 1
        self.rows.append((section, name, got, want, "OK" if ok else "NG"))

    def tf(self, parent, child):
        return self.buf.lookup_transform(PREFIX + parent, PREFIX + child, Time()).transform

    def check_tf(self):
        for name, (x, y, z) in MEASURED.items():
            b = self.tf("base_link", name)
            f = self.tf("base_footprint", name)
            got = (b.translation.x, b.translation.y, f.translation.z)
            r = rpy_deg(b.rotation)
            ok = all(abs(g - w) <= POS_TOL for g, w in zip(got, (x, y, z))) and all(abs(a) <= ANG_TOL for a in r)
            self.row("A TF vs measured [mm]", name,
                     "%.1f, %.1f, %.1f" % tuple(1000 * g for g in got),
                     "%.1f, %.1f, %.1f" % (1000 * x, 1000 * y, 1000 * z), ok)
        g = self.tf("anemometer_link", "anemometer_gaden_link")
        r = rpy_deg(g.rotation)
        ok = abs(abs(r[0]) - 180) <= ANG_TOL and abs(r[1]) <= ANG_TOL and abs(r[2]) <= ANG_TOL and abs(g.translation.z - 0.001) <= 1e-4
        self.row("A TF vs measured [mm]", "anemometer_gaden_link (child)",
                 "z %.1f, roll %.0f deg" % (1000 * g.translation.z, abs(r[0])), "z 1.0, roll 180 deg", ok)

    def check_gz(self):
        for name in GZ_LINKS:
            g = self.gz.get(PREFIX + name)
            if g is None:
                self.row("B Gazebo link vs TF [mm]", name, "not in Gazebo", "present", False)
                continue
            t = self.tf("base_footprint", name)
            d = [1000 * (getattr(g.translation, a) - getattr(t.translation, a)) for a in "xyz"]
            self.row("B Gazebo link vs TF [mm]", name, "diff %.2f, %.2f, %.2f" % tuple(d), "0, 0, 0",
                     all(abs(v) <= 1000 * POS_TOL for v in d))

    def check_rest(self):
        p = self.gt.pose.pose
        r = rpy_deg(p.orientation)
        self.row("C resting pose", "height of base_footprint [mm]", "%.2f" % (1000 * p.position.z), "0", abs(p.position.z) <= POS_TOL)
        self.row("C resting pose", "roll, pitch [deg]", "%.2f, %.2f" % r[:2], "0, 0", abs(r[0]) <= ANG_TOL and abs(r[1]) <= ANG_TOL)

    def drive(self, vx=0.0, vy=0.0, wz=0.0):
        """Returns (forward, left, dyaw, dt_sim) in the robot's starting body frame."""
        self.spin_for(0.3)
        a = self.gt
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        end = time.time() + CMD_SEC
        while time.time() < end:
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        b = self.gt
        self.stop()
        yaw = yaw_of(a.pose.pose.orientation)
        dx = b.pose.pose.position.x - a.pose.pose.position.x
        dy = b.pose.pose.position.y - a.pose.pose.position.y
        fwd = math.cos(yaw) * dx + math.sin(yaw) * dy
        left = -math.sin(yaw) * dx + math.cos(yaw) * dy
        dyaw = (yaw_of(b.pose.pose.orientation) - yaw + math.pi) % (2 * math.pi) - math.pi
        dt = (Time.from_msg(b.header.stamp) - Time.from_msg(a.header.stamp)).nanoseconds * 1e-9
        return fwd, left, dyaw, dt

    def stop(self):
        for _ in range(10):
            self.cmd.publish(Twist())
            self.spin_for(0.05)
        self.spin_for(0.5)

    def check_motion(self):
        f, l, y, dt = self.drive(vx=CMD_LIN)
        self.row("D axes by motion", "cmd +x %.1f m/s" % CMD_LIN,
                 "forward %+.3f m, left %+.3f m, %.3f m/s" % (f, l, f / dt), "forward +, left 0",
                 f > 0.1 and abs(l) < 0.01)
        f, l, y, dt = self.drive(vy=CMD_LIN)
        self.row("D axes by motion", "cmd +y %.1f m/s" % CMD_LIN,
                 "forward %+.3f m, left %+.3f m, %.3f m/s" % (f, l, l / dt), "forward 0, left +",
                 l > 0.1 and abs(f) < 0.01)
        f, l, y, dt = self.drive(wz=CMD_ANG)
        self.row("D axes by motion", "cmd +wz %.1f rad/s" % CMD_ANG,
                 "yaw %+.1f deg (CCW +), %.3f rad/s, drift %.3f m" % (math.degrees(y), y / dt, math.hypot(f, l)),
                 "yaw +, drift 0",
                 y > 0.2 and math.hypot(f, l) < 0.01)

    def report(self):
        section = None
        for s, name, got, want, ok in self.rows:
            if s != section:
                print(f"\n== {s} ==")
                section = s
            print(f"  [{ok}] {name:32s} got: {got:48s} want: {want}")
        print(f"\n{len(self.rows) - self.ng} OK / {self.ng} NG")


def main():
    rclpy.init()
    n = Check()
    try:
        deadline = time.time() + 15
        while (n.gt is None or not n.gz) and time.time() < deadline:
            rclpy.spin_once(n, timeout_sec=0.1)
        if n.gt is None or not n.gz:
            print("no ground_truth / gz_all_poses -- is the launch running?")
            return 2
        n.spin_for(2.0)
        n.stop()
        n.check_tf()
        n.check_gz()
        n.check_rest()
        n.check_motion()
    finally:
        n.stop()
    n.report()
    return 1 if n.ng else 0


if __name__ == "__main__":
    sys.exit(main())
