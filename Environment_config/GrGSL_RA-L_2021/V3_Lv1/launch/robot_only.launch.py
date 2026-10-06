"""
Robot-only bring-up for V3 lv1: room + the measured osl_robot in Gazebo,
TF, and the cmd_vel bridge -- no GADEN, no Nav2, no GSL. For checking the
robot's shape, frames and motion by hand (sim-recheck-checklist steps 1-2).

    ros2 launch /home/ros2_ws/src/GasSourceLocalization/Environment_config/GrGSL_RA-L_2021/V3_Lv1/launch/robot_only.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard     # another terminal

Everything robot-related (URDF path, frame prefix, VelocityControl plugin,
start pose, OSL_WHEEL_MU) is taken from gazebo_v3_lv1.launch.py so the robot
here is the same one the full launch spawns.

Unlike the full launch, the Gazebo GUI is shown by default (gui:=False for
the headless server only).
"""
import importlib.util
import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_spec = importlib.util.spec_from_file_location(
    "gazebo_v3_lv1", os.path.join(os.path.dirname(os.path.abspath(__file__)), "gazebo_v3_lv1.launch.py")
)
full = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(full)


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = subprocess.run(
        ["xacro", full.OSL_ROBOT_XACRO, f"prefix:={full.FRAME_PREFIX}"],
        check=True, capture_output=True, text=True,
    ).stdout
    robot_description = robot_description.replace("</robot>", full.VELOCITY_CONTROL_PLUGIN_XML + "</robot>")
    wheel_mu = os.environ.get("OSL_WHEEL_MU", "")
    if wheel_mu:
        mu_xml = "".join(
            f'\n  <gazebo reference="{full.FRAME_PREFIX}{w}_wheel_link"><mu1>{wheel_mu}</mu1><mu2>{wheel_mu}</mu2></gazebo>'
            for w in ("front_right", "front_left", "rear_left", "rear_right")
        )
        robot_description = robot_description.replace("</robot>", mu_xml + "\n</robot>")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="True"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument("start_x", default_value=full.START_X),
        DeclareLaunchArgument("start_y", default_value=full.START_Y),
        DeclareLaunchArgument("start_yaw", default_value=full.START_YAW),

        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            full.GAZEBO_DIR + os.pathsep + "/opt/ros/humble/share",
        ),
        # Fortress reads the IGN_ name; the room's STL meshes are resolved from here.
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            full.GAZEBO_DIR + os.pathsep + "/opt/ros/humble/share",
        ),

        ExecuteProcess(
            cmd=["ign", "gazebo", "-r", "-v", "3", full.WORLD_PATH],
            output="screen",
            condition=IfCondition(gui),
        ),
        ExecuteProcess(
            cmd=["ign", "gazebo", "-s", "-r", "-v", "3", full.WORLD_PATH],
            output="screen",
            condition=UnlessCondition(gui),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
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
            name="spawn_osl_robot",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-name", full.MODEL_NAME,
                "-x", LaunchConfiguration("start_x"),
                "-y", LaunchConfiguration("start_y"), "-z", "0.05",
                "-Y", LaunchConfiguration("start_yaw"),
            ],
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ign_clock_bridge",
            output="screen",
            arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="cmd_vel_bridge",
            output="screen",
            arguments=["/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ign_pose_bridge",
            output="screen",
            arguments=[
                f"/world/{full.WORLD_NAME}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            ],
            remappings=[
                (f"/world/{full.WORLD_NAME}/dynamic_pose/info", "/gz_all_poses"),
            ],
        ),

        # map -> base_footprint from Gazebo's model pose, same as the full launch.
        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(full.LAUNCH_DIR, "ground_truth_bridge.py"),
                "--ros-args",
                "-p", "world_pose_topic:=/gz_all_poses",
                "-p", f"model_name:={full.MODEL_NAME}",
                "-p", "map_frame:=map",
                "-p", f"child_frame:={full.BASE_FOOTPRINT_FRAME}",
                "-p", f"output_topic:={full.GROUND_TRUTH_TOPIC}",
                "-p", "use_sim_time:=True",
            ],
            output="screen",
        ),

        # Same child frame the full launch feeds to the simulated anemometer
        # (see the comment there), so it shows up in the TF tree here too.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="anemometer_gaden_tf_pub",
            arguments=[
                "--z", "0.001",
                "--roll", "3.141592653589793",
                "--frame-id", full.FRAME_PREFIX + "anemometer_link",
                "--child-frame-id", full.FRAME_PREFIX + "anemometer_gaden_link",
            ],
            parameters=[{"use_sim_time": True}],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", full.RVIZ_CONFIG],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(use_rviz),
        ),
    ])
