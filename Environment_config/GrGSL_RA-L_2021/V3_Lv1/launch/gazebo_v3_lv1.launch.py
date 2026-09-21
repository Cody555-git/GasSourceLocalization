"""
Gazebo Fortress (ign gazebo) smoke test: V3 lv1 room (../gazebo/worlds)
+ a public TurtleBot3 Waffle spawned at the real-robot start pose
(-1.67, 0.99, yaw=0). No GADEN / nav2 / GrGSL here yet -- this is
A-1 step 2 (backlog P1): confirm the room and the robot show up
together before wiring ground_truth / TF / sensors (spec-gazebo-waffle-robot).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GAZEBO_DIR = "/home/ros2_ws/src/GasSourceLocalization/Environment_config/GrGSL_RA-L_2021/V3_Lv1/gazebo"
WORLD_PATH = os.path.join(GAZEBO_DIR, "worlds", "s5406a_v3_lv1.world")
RVIZ_CONFIG = os.path.join(GAZEBO_DIR, "waffle_room_v3.rviz")
V3_PROJECT_PATH = "/home/osl_env_s5406a/gaden_scenarios/s5406a_lv1/environment_configurations/config1"

# turtlebot3_description's raw URDF keeps a literal "${namespace}" prefix on
# every link/joint name (not resolved by xacro since we load the .urdf as-is).
BASE_FOOTPRINT_FRAME = "${namespace}base_footprint"

# Real-robot start pose, fixed for lv1/lv2 (backlog P1, current-status 2026-09-18).
START_X = "-1.67"
START_Y = "0.99"
START_YAW = "0.0"


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    turtlebot3_description_dir = get_package_share_directory("turtlebot3_description")
    urdf_path = os.path.join(turtlebot3_description_dir, "urdf", "turtlebot3_waffle.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read()

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
                "-name", "turtlebot3_waffle",
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

        # Placeholder map->base_footprint at the fixed start pose, only so RViz has
        # a common frame to overlay robot + room markers in. Step 3 (backlog P1,
        # today's plan item 3) replaces this with Gazebo's real ground_truth + TF.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="placeholder_map_to_base_footprint",
            output="screen",
            arguments=[
                "--x", START_X, "--y", START_Y, "--z", "0",
                "--yaw", START_YAW,
                "--frame-id", "map",
                "--child-frame-id", BASE_FOOTPRINT_FRAME,
            ],
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
