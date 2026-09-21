#!/usr/bin/env python3
"""
Filters the world-frame model pose out of Gazebo's per-entity TF stream and
republishes it as (a) a dynamic TF map -> <child_frame> and (b) a
geometry_msgs/msg/PoseWithCovarianceStamped on <output_topic>, matching the
"ground_truth" topic gsl_actionserver_node/nav_assistant_node expect
(Algorithm.cpp:29, robot_location_topic).

Why this exists: ros_gz_bridge can turn Gazebo's
/world/<world>/dynamic_pose/info (ignition.msgs.Pose_V) into a
tf2_msgs/msg/TFMessage, but that message bundles one entry per *entity*
(the model root AND every link), each expressed relative to its own parent
(links relative to the model, not to the world) and with an empty
header.frame_id. Feeding that whole bundle into /tf would fight with
robot_state_publisher's own joint-based TF chain for the same link names.
This node picks out only the single entry named `model_name` (the model
root, which Gazebo reports in world/map coordinates) and treats that as the
external localization source the rest of the TF tree hangs off of.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped


class GroundTruthBridge(Node):
    def __init__(self):
        super().__init__("ground_truth_bridge")
        self.declare_parameter("world_pose_topic", "/gz_all_poses")
        self.declare_parameter("model_name", "turtlebot3_waffle")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("child_frame", "${namespace}base_footprint")
        self.declare_parameter("output_topic", "ground_truth")

        self.model_name = self.get_parameter("model_name").value
        self.map_frame = self.get_parameter("map_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        self.tf_broadcaster = TransformBroadcaster(self)
        # Reliable, not SENSOR_DATA (best-effort): gsl_actionserver_node's
        # localizationSub (Algorithm.cpp:29) is a plain create_subscription
        # with the rclcpp default (RELIABLE), so a best-effort publisher here
        # is QoS-incompatible and silently drops every message -- confirmed
        # 2026-09-21 turn 8 (gsl_node logged "New publisher discovered ...
        # incompatible QoS ... RELIABILITY_QOS_POLICY" and hung forever on
        # "Waiting to hear from localization topic"). The incoming TF stream
        # subscription can stay best-effort; only the output pose needs to change.
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_topic").value,
            10,
        )
        self.create_subscription(
            TFMessage,
            self.get_parameter("world_pose_topic").value,
            self.on_tf_message,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.got_first_pose = False

    def on_tf_message(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id != self.model_name:
                continue
            stamp = self.get_clock().now().to_msg()

            tf_out = TransformStamped()
            tf_out.header.stamp = stamp
            tf_out.header.frame_id = self.map_frame
            tf_out.child_frame_id = self.child_frame
            tf_out.transform = t.transform
            self.tf_broadcaster.sendTransform(tf_out)

            pose_out = PoseWithCovarianceStamped()
            pose_out.header.stamp = stamp
            pose_out.header.frame_id = self.map_frame
            pose_out.pose.pose.position.x = t.transform.translation.x
            pose_out.pose.pose.position.y = t.transform.translation.y
            pose_out.pose.pose.position.z = t.transform.translation.z
            pose_out.pose.pose.orientation = t.transform.rotation
            # Ground truth: no uncertainty. Downstream (Algorithm.cpp) only
            # reads pose.pose, but a zeroed covariance is still the honest value.
            pose_out.pose.covariance = [0.0] * 36
            self.pose_pub.publish(pose_out)

            if not self.got_first_pose:
                self.got_first_pose = True
                self.get_logger().info(
                    f"ground_truth_bridge: first pose for '{self.model_name}' "
                    f"-> TF {self.map_frame}->{self.child_frame} + "
                    f"{self.pose_pub.topic_name}"
                )
            return


def main():
    rclpy.init()
    node = GroundTruthBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
