"""
nav2 bring-up for V3 lv1, adapted from
GrGSL_RA-L_2021/navigation_config/nav2_launch.py.

Why a separate file instead of reusing the upstream one directly: upstream
derives the map path as get_package_share_directory("grgsl_env")/<scenario>/
occupancy.yaml -- V3's map lives outside that package, at
/home/osl_env_s5406a/maps_nav/s5406a_lv1.yaml (backlog A-1 "まず直すところ").
This file takes map_yaml as a plain launch arg instead.

Also drops upstream's extra robot_state_publisher (a "giraff" xacro model
used only for nav2's own visualization) -- gazebo_v3_lv1.launch.py already
runs the real robot_state_publisher for the actual TurtleBot3 Waffle, and a
second one publishing different geometry under the same TF frame names would
just be redundant/confusing, not additive.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
	map_yaml = LaunchConfiguration("map_yaml").perform(context)
	use_sim_time = True

	configured_params = ParameterFile(
		LaunchConfiguration("nav_params_yaml").perform(context), allow_substs=True
	)

	navigation_nodes = [
		Node(
			package="nav2_map_server",
			executable="map_server",
			name="map_server",
			output="screen",
			parameters=[
				{"use_sim_time": use_sim_time},
				{"yaml_filename": map_yaml},
				{"frame_id": "map"},
			],
		),
		Node(
			package="nav2_bt_navigator",
			executable="bt_navigator",
			name="bt_navigator",
			output="screen",
			parameters=[configured_params],
		),
		Node(
			package="nav2_planner",
			executable="planner_server",
			name="planner_server",
			output="screen",
			parameters=[configured_params],
		),
		Node(
			package="nav2_controller",
			executable="controller_server",
			name="controller_server",
			output="screen",
			parameters=[configured_params],
			# controller_server publishes velocity on the relative topic
			# "cmd_vel" (nav2_controller/controller_server.hpp), so under
			# PushRosNamespace(TurtleBot3Waffle) it resolves to
			# /TurtleBot3Waffle/cmd_vel -- but the Gazebo-facing cmd_vel_bridge
			# in gazebo_v3_lv1.launch.py subscribes to the plain, unscoped
			# /cmd_vel (see that file's DiffDrive comment). Remap here to an
			# absolute name so the two actually connect (research/loop/
			# 2026-09-22.md turn 3: robot was computing paths but /cmd_vel had
			# 0 publishes, ending every run in a spin-recovery loop).
			remappings=[("cmd_vel", "/cmd_vel")],
		),
		Node(
			package="nav2_behaviors",
			executable="behavior_server",
			name="behavior_server",
			output="screen",
			parameters=[configured_params],
		),
		Node(
			package="nav2_lifecycle_manager",
			executable="lifecycle_manager",
			name="lifecycle_manager_navigation",
			output="screen",
			parameters=[
				{"use_sim_time": use_sim_time},
				{"autostart": True},
				{
					"node_names": [
						"map_server",
						"planner_server",
						"controller_server",
						"bt_navigator",
						"behavior_server",
					]
				},
			],
		),
	]

	actions = [PushRosNamespace(LaunchConfiguration("namespace"))]
	actions.extend(navigation_nodes)
	return [GroupAction(actions=actions)]


def generate_launch_description():
	grgsl_nav_dir = get_package_share_directory("grgsl_env")

	return LaunchDescription(
		[
			DeclareLaunchArgument("namespace", default_value="TurtleBot3Waffle"),
			DeclareLaunchArgument(
				"map_yaml",
				default_value="/home/osl_env_s5406a/maps_nav/s5406a_lv1.yaml",
			),
			DeclareLaunchArgument(
				"nav_params_yaml",
				default_value=os.path.join(
					grgsl_nav_dir, "navigation_config", "nav2_params.yaml"
				),
			),
			OpaqueFunction(function=launch_setup),
		]
	)
