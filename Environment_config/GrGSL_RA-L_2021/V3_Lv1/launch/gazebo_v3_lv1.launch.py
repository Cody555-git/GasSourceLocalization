"""
Gazebo Fortress (ign gazebo) smoke test: V3 lv1 room (../gazebo/worlds)
+ a public TurtleBot3 Waffle spawned at the real-robot start pose
(-1.67, 0.99, yaw=0). No GADEN action server / nav2 / GrGSL action server
here yet -- this is A-1 (backlog P1): room+robot in RViz (done), ground
truth pose+TF from Gazebo (done), and now a DiffDrive plugin so /cmd_vel
actually moves the robot in Gazebo (2026-09-21) -- a prerequisite for
nav2 to have anything to drive. nav2 and gsl_server are still separate
pieces to wire in.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GAZEBO_DIR = "/home/ros2_ws/src/GasSourceLocalization/Environment_config/GrGSL_RA-L_2021/V3_Lv1/gazebo"
LAUNCH_DIR = "/home/ros2_ws/src/GasSourceLocalization/Environment_config/GrGSL_RA-L_2021/V3_Lv1/launch"
WORLD_PATH = os.path.join(GAZEBO_DIR, "worlds", "s5406a_v3_lv1.world")
RVIZ_CONFIG = os.path.join(GAZEBO_DIR, "waffle_room_v3.rviz")
V3_PROJECT_PATH = "/home/osl_env_s5406a/gaden_scenarios/s5406a_lv1/environment_configurations/config1"

# SDF <world name="..."> in s5406a_v3_lv1.world -- Gazebo topics are namespaced
# under this (confirmed via `ign topic -l` after launch, 2026-09-21).
WORLD_NAME = "s5406a_v3_lv1"
MODEL_NAME = "turtlebot3_waffle"

# turtlebot3_description's raw URDF keeps a literal "${namespace}" prefix on
# every link/joint name (not resolved by xacro since we load the .urdf as-is).
BASE_FOOTPRINT_FRAME = "${namespace}base_footprint"

# Real-robot start pose, fixed for lv1/lv2 (backlog P1, current-status 2026-09-18).
START_X = "-1.67"
START_Y = "0.99"
START_YAW = "0.0"

# Wheel separation/radius read directly from turtlebot3_waffle.urdf's wheel
# joint origins (y=+-0.144 -> 0.288 m apart) and collision cylinder radius
# (0.033 m), 2026-09-21. The raw URDF ships with no <gazebo> plugin tags at
# all (checked: `grep -i plugin turtlebot3_waffle.urdf` is empty), so without
# this the robot spawns as a static prop -- cmd_vel has nothing to act on.
DIFFDRIVE_PLUGIN_XML = """
  <gazebo>
    <plugin filename="ignition-gazebo-diff-drive-system"
            name="ignition::gazebo::systems::DiffDrive">
      <left_joint>${namespace}wheel_left_joint</left_joint>
      <right_joint>${namespace}wheel_right_joint</right_joint>
      <wheel_separation>0.288</wheel_separation>
      <wheel_radius>0.033</wheel_radius>
      <topic>cmd_vel</topic>
    </plugin>
  </gazebo>
"""


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    turtlebot3_description_dir = get_package_share_directory("turtlebot3_description")
    urdf_path = os.path.join(turtlebot3_description_dir, "urdf", "turtlebot3_waffle.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read()
    robot_description = robot_description.replace("</robot>", DIFFDRIVE_PLUGIN_XML + "</robot>")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="True"),

        # turtlebot3_description's URDF uses package://turtlebot3_description/meshes/...
        # which sdformat rewrites to model://turtlebot3_description/meshes/... -- Ignition
        # resolves that by looking for a "turtlebot3_description" dir directly under one
        # of these resource paths, which /opt/ros/humble/share provides.
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            GAZEBO_DIR + os.pathsep + "/opt/ros/humble/share",
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            GAZEBO_DIR + os.pathsep + "/opt/ros/humble/share",
        ),

        # -s: headless server only. RViz is the only GUI process on this
        # container's single Xvfb display -- running "ign gazebo" GUI too
        # made both fight over the same screen and neither rendered reliably.
        ExecuteProcess(
            cmd=["ign", "gazebo", "-s", "-r", "-v", "3", WORLD_PATH],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),

        # Nothing bridges Gazebo's joint states to ROS yet, so without this
        # robot_state_publisher never gets /joint_states and RViz's RobotModel
        # display errors out on the wheel/camera links (TF chain incomplete).
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_turtlebot3_waffle",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-name", MODEL_NAME,
                "-x", START_X, "-y", START_Y, "-z", "0.05",
                "-Y", START_YAW,
            ],
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ign_clock_bridge",
            output="screen",
            arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        ),

        # DiffDrive's <topic>cmd_vel</topic> resolves to the plain, unscoped
        # GZ topic "/cmd_vel" for a single-model world -- NOT "/model/<name>/
        # cmd_vel" as ros_gz_sim's own topic-scoping convention would suggest.
        # Verified 2026-09-21 by direct `ign topic -p` tests: publishing to
        # the /model/.../cmd_vel name did nothing (0 real movement over 2s of
        # commands); publishing to plain /cmd_vel drove the robot ~3.7 m in
        # 2s. If a second model is ever spawned in this world, re-check this
        # (multi-robot would likely need <topic>/model/<name>/cmd_vel</topic>
        # set explicitly in DIFFDRIVE_PLUGIN_XML to disambiguate).
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="cmd_vel_bridge",
            output="screen",
            arguments=["/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"],
        ),

        # Gazebo's scene broadcaster publishes one Pose_V entry per entity
        # (the model root in world coords, plus every link relative to its
        # own parent). Bridging it wholesale to /tf would collide with
        # robot_state_publisher's joint-based TF for the same link names,
        # so it lands on a private topic and ground_truth_bridge.py below
        # picks out only the model-root entry.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ign_pose_bridge",
            output="screen",
            arguments=[
                f"/world/{WORLD_NAME}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            ],
            remappings=[
                (f"/world/{WORLD_NAME}/dynamic_pose/info", "/gz_all_poses"),
            ],
        ),

        # Room walls as RViz markers, reusing the same GADEN environment node
        # (and V3 lv1 projectPath) as the step-1 GADEN smoke test -- gives us
        # a "map"-frame view of the room without re-deriving it from the Gazebo SDF.
        Node(
            package="gaden_environment",
            executable="environment",
            name="gaden_environment",
            output="screen",
            parameters=[{"projectPath": V3_PROJECT_PATH, "fixed_frame": "map"}],
        ),

        # map -> base_footprint from Gazebo's real model pose (ground truth,
        # spec-gazebo-waffle-robot: "位置は当面シミュレーションの真値を使う").
        # Replaces the fixed-pose static_transform_publisher used in step 2.
        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(LAUNCH_DIR, "ground_truth_bridge.py"),
                "--ros-args",
                "-p", "world_pose_topic:=/gz_all_poses",
                "-p", f"model_name:={MODEL_NAME}",
                "-p", "map_frame:=map",
                "-p", f"child_frame:={BASE_FOOTPRINT_FRAME}",
                "-p", "output_topic:=/ground_truth",
                "-p", "use_sim_time:=True",
            ],
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", RVIZ_CONFIG],
            condition=IfCondition(use_rviz),
        ),
    ])
