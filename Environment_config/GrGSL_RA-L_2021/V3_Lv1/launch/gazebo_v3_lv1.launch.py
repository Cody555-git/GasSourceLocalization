"""
Gazebo Fortress (ign gazebo) launch for V3 lv1: room (../gazebo/worlds) +
a public TurtleBot3 Waffle spawned at the real-robot start pose
(-1.67, 0.99, yaw=0), driven by a DiffDrive plugin, with ground-truth
pose+TF from Gazebo, nav2 bring-up, GADEN gas playback, the anemometer/PID/
gmrf_wind sensor stack, and gsl_server (GrGSL) itself (backlog A-1, "GSL を
「中身のある完走」までもっていく").

Namespace policy (decided 2026-09-21, see research/loop/2026-09-21.md
turn 7): follow the upstream GrGSL convention of a single PascalCase
robot_name used two ways -- (a) as a literal TF frame prefix
"<robot_name>_" on every link (matches nav2_params.yaml's
"$(var namespace)_base_footprint"), and (b) as the ROS namespace
(PushRosNamespace) for nav2, gsl_server, anemometer and PID.
Gazebo-facing bridge nodes (cmd_vel, pose, clock) stay outside that ROS
namespace and use fully-qualified topic names instead, because the
DiffDrive plugin's <topic> was already found not to respect ros_gz_sim's
/model/<name>/... scoping (see the cmd_vel_bridge comment below) --
namespacing those bridges would just add a mismatch to debug.

gaden_player and gmrf_wind_mapping_node are deliberately left OUTSIDE the
ROS namespace too, matching upstream Lv1/launch/main_simbot.py exactly --
not a stylistic choice. simulated_gas_sensor/simulated_anemometer call
gaden_player's "/odor_value" and "/wind_value" services with a *leading
slash* (gaden_ws/src/gaden/simulated_gas_sensor/src/*.cpp,
simulated_anemometer/src/*.cpp), and gsl_server's GrGSL algorithm calls
gmrf's wind-estimation service the same way (MovingStateGrGSL.cpp:12,
create_client<WindEstimation>("/WindEstimation")) -- both are absolute
names that PushRosNamespace would silently break by rewriting them to
/TurtleBot3Waffle/odor_value etc., which nothing would be listening on.
gmrf's own sensor_topic/map_topic params are instead given the namespaced
topic strings explicitly (e.g. "TurtleBot3Waffle/Anemometer/..."), which
still resolves correctly because gmrf itself lives in the root namespace.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

GAZEBO_DIR = "/home/ros2_ws/src/GasSourceLocalization/Environment_config/GrGSL_RA-L_2021/V3_Lv1/gazebo"
LAUNCH_DIR = "/home/ros2_ws/src/GasSourceLocalization/Environment_config/GrGSL_RA-L_2021/V3_Lv1/launch"
WORLD_PATH = os.path.join(GAZEBO_DIR, "worlds", "s5406a_v3_lv1.world")
RVIZ_CONFIG = os.path.join(GAZEBO_DIR, "waffle_room_v3.rviz")
V3_PROJECT_PATH = "/home/osl_env_s5406a/gaden_scenarios/s5406a_lv1/environment_configurations/config1"
V3_NAV_MAP_YAML = "/home/osl_env_s5406a/maps_nav/s5406a_lv1.yaml"

# SDF <world name="..."> in s5406a_v3_lv1.world -- Gazebo topics are namespaced
# under this (confirmed via `ign topic -l` after launch, 2026-09-21).
WORLD_NAME = "s5406a_v3_lv1"
MODEL_NAME = "turtlebot3_waffle"

# Upstream robot_name convention (e.g. "PioneerP3DX" in Exp_*/Lv1's
# main_simbot.py) -- used as both the ROS namespace and the TF frame prefix.
ROBOT_NAME = "TurtleBot3Waffle"
FRAME_PREFIX = ROBOT_NAME + "_"
BASE_FOOTPRINT_FRAME = FRAME_PREFIX + "base_footprint"
GROUND_TRUTH_TOPIC = f"/{ROBOT_NAME}/ground_truth"

# Real-robot start pose, fixed for lv1/lv2 (backlog P1, current-status 2026-09-18).
START_X = "-1.67"
START_Y = "0.99"
START_YAW = "0.0"

# V3 gas source, research/environments.md:265 ("ガス源 ... (2.87, 0.30, 0.15)").
SOURCE_X = 2.87
SOURCE_Y = 0.30
SOURCE_Z = 0.15

# 0.05 m/cell nav map x scale=10 -> 0.5 m belief-map cell (backlog A-1,
# current-status "2026-09-21 の決定" D6). gmrf's cell_size is set to match
# so the wind grid and GrGSL's occupancy-derived belief grid share a scale.
GSL_SCALE = 10
GMRF_CELL_SIZE = 0.5

# 300 s trial cap, same value as [[Ojeda2021]]'s simulation setting (backlog A-1).
MAX_SEARCH_TIME = 300.0

RESULTS_FILE = "/home/ros2_ws/Results/GridGSL_V3_Lv1_A1.csv"
NAVIGATION_PATH_FILE = "/home/ros2_ws/Results/GridGSL_V3_Lv1_A1_path.csv"

# Wheel separation/radius read directly from turtlebot3_waffle.urdf's wheel
# joint origins (y=+-0.144 -> 0.288 m apart) and collision cylinder radius
# (0.033 m), 2026-09-21. The raw URDF ships with no <gazebo> plugin tags at
# all (checked: `grep -i plugin turtlebot3_waffle.urdf` is empty), so without
# this the robot spawns as a static prop -- cmd_vel has nothing to act on.
DIFFDRIVE_PLUGIN_XML = f"""
  <gazebo>
    <plugin filename="ignition-gazebo-diff-drive-system"
            name="ignition::gazebo::systems::DiffDrive">
      <left_joint>{FRAME_PREFIX}wheel_left_joint</left_joint>
      <right_joint>{FRAME_PREFIX}wheel_right_joint</right_joint>
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
    # Raw (non-xacro) URDF ships with a literal "${namespace}" placeholder on
    # every link/joint name -- resolve it to FRAME_PREFIX so frame IDs match
    # nav2_params.yaml's "$(var namespace)_base_footprint" convention instead
    # of staying as the unresolved literal string (2026-09-21 cleanup).
    robot_description = robot_description.replace("${namespace}", FRAME_PREFIX)
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

        # Actual gas playback (the step-1 smoke test only exercised this
        # through gaden_player_v3.launch.py in isolation -- it was missing
        # here, so simulated_gas_sensor/simulated_anemometer below had
        # nothing to call). 10 Hz matches scene1.yaml's 0.1 s frame spacing
        # (research/loop/2026-09-21.md turn 1). Root namespace: see module
        # docstring ("/odor_value"/"/wind_value" are absolute service names).
        Node(
            package="gaden_player",
            executable="player",
            name="gaden_player",
            output="screen",
            parameters=[{
                "projectPath": V3_PROJECT_PATH,
                "playbackID": "scene1",
                "player_freq": 10.0,
                "fixed_frame": "map",
            }],
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
                "-p", f"output_topic:={GROUND_TRUTH_TOPIC}",
                "-p", "use_sim_time:=True",
            ],
            output="screen",
        ),

        # nav2 bring-up (backlog A-1 "まず直すところ": V3's map lives outside
        # the grgsl_env package share dir, so this uses the map_yaml-arg
        # variant instead of upstream's scenario-derived path -- see
        # nav2_launch_v3.py docstring).
        # Identity base_link->odom TF (upstream main_simbot.py's odom_tf
        # GroupAction). nav2's local_costmap won't activate without an odom
        # frame to transform into; ground truth already gives an accurate
        # base_link pose, so odom==base_link (no separate odometry source).
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_tf_pub",
            arguments=[
                "0", "0", "0", "0", "0", "0",
                FRAME_PREFIX + "base_link", FRAME_PREFIX + "odom",
            ],
            parameters=[{"use_sim_time": True}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(LAUNCH_DIR, "nav2_launch_v3.py")
            ),
            launch_arguments={
                "namespace": ROBOT_NAME,
                "map_yaml": V3_NAV_MAP_YAML,
            }.items(),
        ),

        # Anemometer: wind reading + its mount TF (0.1 m, floor-mounted,
        # spec-gazebo-waffle-robot / 2026-09-15 decision -- provisional mount
        # height, real sensor placement unmeasured).
        GroupAction(actions=[
            PushRosNamespace(ROBOT_NAME),
            Node(
                package="simulated_anemometer",
                executable="simulated_anemometer",
                name="Anemometer",
                output="screen",
                parameters=[{
                    "sensor_frame": FRAME_PREFIX + "anemometer_frame",
                    "fixed_frame": "map",
                    "noise_std": 0.3,
                    "use_map_ref_system": False,
                    "use_sim_time": True,
                }],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="anemometer_tf_pub",
                arguments=[
                    "0", "0", "0.1", "1.0", "0.0", "0", "0",
                    FRAME_PREFIX + "base_link", FRAME_PREFIX + "anemometer_frame",
                ],
                parameters=[{"use_sim_time": True}],
            ),
        ]),

        # PID gas sensor: concentration reading + its mount TF (0.17 m,
        # front-center, provisional -- same spec/decision as the anemometer).
        GroupAction(actions=[
            PushRosNamespace(ROBOT_NAME),
            Node(
                package="simulated_gas_sensor",
                executable="simulated_gas_sensor",
                name="PID",
                output="screen",
                parameters=[{
                    "sensor_model": 30,
                    "sensor_frame": FRAME_PREFIX + "pid_frame",
                    "fixed_frame": "map",
                    "noise_std": 20.1,
                    "use_sim_time": True,
                }],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="pid_tf_pub",
                arguments=[
                    "0", "0", "0.17", "1.0", "0.0", "0", "0",
                    FRAME_PREFIX + "base_link", FRAME_PREFIX + "pid_frame",
                ],
                parameters=[{"use_sim_time": True}],
            ),
        ]),

        # Wind mapping (GMRF). Root namespace -- see module docstring.
        Node(
            package="gmrf_wind_mapping",
            executable="gmrf_wind_mapping_node",
            name="gmrf",
            output="screen",
            parameters=[{
                "sensor_topic": f"{ROBOT_NAME}/Anemometer/WindSensor_reading",
                "map_topic": f"{ROBOT_NAME}/map",
                "cell_size": GMRF_CELL_SIZE,
                "use_sim_time": True,
            }],
        ),

        # GrGSL itself. gsl_actionserver_call is the client that actually
        # sends the "start searching" goal to gsl_actionserver_node; without
        # it the action server just sits idle (upstream Lv1/main_simbot.py
        # pattern -- both always launched together).
        GroupAction(actions=[
            PushRosNamespace(ROBOT_NAME),
            Node(
                package="gsl_server",
                executable="gsl_actionserver_call",
                name="gsl_call",
                output="screen",
                parameters=[{"method": "GrGSL"}],
            ),
            Node(
                package="gsl_server",
                executable="gsl_actionserver_node",
                name="gsl_node",
                output="screen",
                parameters=[{
                    "robot_location_topic": "ground_truth",
                    "stop_and_measure_time": 1.0,
                    "th_gas_present": 0.5,
                    "th_wind_present": 0.1,
                    "ground_truth_x": SOURCE_X,
                    "ground_truth_y": SOURCE_Y,
                    "resultsFile": RESULTS_FILE,
                    "navigationPathFile": NAVIGATION_PATH_FILE,
                    "maxSearchTime": MAX_SEARCH_TIME,
                    "anemometer_frame": FRAME_PREFIX + "anemometer_frame",
                    "scale": GSL_SCALE,
                    "stdevHit": 1.0,
                    "stdevMiss": 1.5,
                    "convergence_thr": 0.5,
                    "infoTaxis": False,
                    "step": 0.8,
                    "use_sim_time": True,
                }],
            ),
        ]),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", RVIZ_CONFIG],
            condition=IfCondition(use_rviz),
        ),
    ])
