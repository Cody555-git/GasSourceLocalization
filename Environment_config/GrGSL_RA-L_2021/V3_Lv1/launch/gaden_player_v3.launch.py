"""
GADEN player smoke test against the V3 lv1 environment (S5-406A).

Points gaden_environment / gaden_player directly at the post-3.0
projectPath produced by osl_env_s5406a (source (2.87, 0.30, 0.15),
300 s playback window 6090..9089). Does not touch nav2 / GrGSL --
this is only to confirm GADEN itself loads V3 correctly before
wiring the rest of the A-1 stack (backlog P1, research/gsl-sim-stack.md).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

V3_PROJECT_PATH = "/home/osl_env_s5406a/gaden_scenarios/s5406a_lv1/environment_configurations/config1"
RVIZ_CONFIG = "/home/ros2_ws/src/GasSourceLocalization/Environment_config/GrGSL_RA-L_2021/gaden.rviz"


def generate_launch_description():
    player_freq = LaunchConfiguration("player_freq")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument("player_freq", default_value="10.0"),
        DeclareLaunchArgument("use_rviz", default_value="True"),

        Node(
            package="gaden_environment",
            executable="environment",
            name="gaden_environment",
            output="screen",
            parameters=[{"projectPath": V3_PROJECT_PATH, "fixed_frame": "map"}],
        ),
        Node(
            package="gaden_player",
            executable="player",
            name="gaden_player",
            output="screen",
            parameters=[{
                "projectPath": V3_PROJECT_PATH,
                "playbackID": "scene1",
                "player_freq": player_freq,
                "fixed_frame": "map",
            }],
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
