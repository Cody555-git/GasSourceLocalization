"""
Gazebo Fortress (ign gazebo) launch for V3 lv1: room (../gazebo/worlds) +
the measured osl_robot (Cody555-git/osl_robot, osl_description package)
spawned at the real-robot start pose (-1.67, 0.99, yaw=0), driven by a
VelocityControl plugin, with ground-truth pose+TF from Gazebo, nav2
bring-up, GADEN gas playback, the gas(x6)/anemometer/gmrf_wind sensor
stack, and gsl_server (GrGSL) itself (backlog A-2,
spec-gazebo-measured-urdf -- replaces the public TurtleBot3 Waffle used
for A-1 with the real robot's own URDF, same room/start pose/GrGSL
params).

Locomotion (2026-09-23, spec-gazebo-measured-urdf sec.4, "意図的に変更"):
osl_robot's URDF has no <gazebo> plugin of its own and its 4-wheel swerve
kinematics are out of scope here -- only straight/turn-in-place motion is
needed (real robot's actual usage). Ignition's DiffDrive plugin needs
exactly two wheel joints and osl_robot has eight (4 steer + 4 wheel), so
it was replaced with ignition::gazebo::systems::VelocityControl, which
applies cmd_vel directly to the model body instead of driving wheel
joints -- steer/wheel joints stay visually static (no dynamics). Assumed
(not yet verified for this plugin) to share the same unscoped /cmd_vel
topic behaviour DiffDrive showed in the Waffle build (see cmd_vel_bridge
comment below); re-check with `ign topic -l` after first run.

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
import subprocess

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
MODEL_NAME = "osl_robot"

# osl_robot's own URDF -- single source of truth shared with the real robot
# (Cody555-git/osl_robot, osl_description package). Not copied into this repo.
OSL_ROBOT_XACRO = "/home/ros2_ws/src/osl_robot/src/osl_description/urdf/osl_robot.urdf.xacro"

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

# osl_robot.urdf.xacro ships with no <gazebo> plugin tags either (same as
# the old public Waffle URDF), so without this the robot spawns as a static
# prop -- cmd_vel has nothing to act on. See module docstring "Locomotion"
# paragraph for why VelocityControl (whole-body) replaces DiffDrive
# (two-wheel) here.
VELOCITY_CONTROL_PLUGIN_XML = """
  <gazebo>
    <plugin filename="ignition-gazebo-velocity-control-system"
            name="ignition::gazebo::systems::VelocityControl">
      <topic>cmd_vel</topic>
    </plugin>
  </gazebo>
"""


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    # osl_robot.urdf.xacro's own "prefix" xacro:arg (default "", real robot
    # unaffected) already applies FRAME_PREFIX to every link/joint name --
    # same convention the old raw-Waffle-URDF string-replace achieved by
    # hand (spec-gazebo-measured-urdf sec.2, [[loop-state]] 2026-09-23).
    robot_description = subprocess.run(
        ["xacro", OSL_ROBOT_XACRO, f"prefix:={FRAME_PREFIX}"],
        check=True, capture_output=True, text=True,
    ).stdout
    robot_description = robot_description.replace("</robot>", VELOCITY_CONTROL_PLUGIN_XML + "</robot>")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="True"),

        # osl_robot's visuals are all primitive box/cylinder geometry (no mesh
        # files), so only the room world's own resources need resolving --
        # /opt/ros/humble/share is kept for GAZEBO_DIR-relative world assets.
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
            name="spawn_osl_robot",
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

        # DiffDrive's <topic>cmd_vel</topic> resolved to the plain, unscoped
        # GZ topic "/cmd_vel" for a single-model world -- NOT "/model/<name>/
        # cmd_vel" as ros_gz_sim's own topic-scoping convention would suggest.
        # Verified 2026-09-21 by direct `ign topic -p` tests with DiffDrive:
        # publishing to the /model/.../cmd_vel name did nothing (0 real
        # movement over 2s of commands); publishing to plain /cmd_vel drove
        # the robot ~3.7 m in 2s. VelocityControl (2026-09-23, see module
        # docstring) is assumed to behave the same way but this has not been
        # re-verified for that plugin -- check with `ign topic -p` again
        # after the first A-2 run. If a second model is ever spawned in this
        # world, re-check regardless (multi-robot would likely need
        # <topic>/model/<name>/cmd_vel</topic> set explicitly in
        # VELOCITY_CONTROL_PLUGIN_XML to disambiguate).
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

        # Anemometer: wind reading. TF comes straight from osl_robot's own
        # "anemometer_link" fixed joint via robot_state_publisher now (no
        # manual static_transform_publisher -- spec-gazebo-measured-urdf
        # sec.1/2, 2026-09-23). Unlike the old Waffle-build static TF (qx=1,
        # 180deg about x), the URDF joint has no rotation at all, so the
        # sec.3 "180deg flip" hypothesis doesn't apply here; still needs the
        # sec.6 wind-direction check against GADEN's field to confirm.
        GroupAction(actions=[
            PushRosNamespace(ROBOT_NAME),
            Node(
                package="simulated_anemometer",
                executable="simulated_anemometer",
                name="Anemometer",
                output="screen",
                parameters=[{
                    "sensor_frame": FRAME_PREFIX + "anemometer_link",
                    "fixed_frame": "map",
                    "noise_std": 0.3,
                    "use_map_ref_system": False,
                    "use_sim_time": True,
                }],
            ),
        ]),

        # Gas sensors: all 6 of osl_robot's measured MiCS-5524 mounts, each
        # its own simulated_gas_sensor node named after the URDF frame it
        # reads TF from (node name -> default topic "<name>/Sensor_reading",
        # fake_gas_sensor.cpp:33) -- TF again comes from osl_robot's own
        # fixed joints, no manual static_transform_publisher.
        # sensor_model 0 = TGS2620 (spec-gazebo-measured-urdf sec.3,
        # 2026-09-23 user decision: closest MOX model GADEN ships to the
        # real MiCS-5524; GADEN has no MiCS-5524 model). noise_std unchanged
        # (still the PID-era 20.1 -- sec.3 says record, don't retune).
        # Only gas_front_mid is wired into gsl_actionserver_node below
        # (enose_topic param); the other 5 publish for rosbag/inspection only.
        GroupAction(actions=[
            PushRosNamespace(ROBOT_NAME),
            *[
                Node(
                    package="simulated_gas_sensor",
                    executable="simulated_gas_sensor",
                    name=gas_frame,
                    output="screen",
                    parameters=[{
                        "sensor_model": 0,
                        "sensor_frame": FRAME_PREFIX + gas_frame,
                        "fixed_frame": "map",
                        "noise_std": 20.1,
                        "use_sim_time": True,
                    }],
                )
                for gas_frame in [
                    "gas_front_top", "gas_front_mid", "gas_front_bottom",
                    "gas_left", "gas_right", "gas_rear",
                ]
            ],
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
                    # th_gas_present was tuned for the PID sensor's ppm-like
                    # output. Left unchanged per spec-gazebo-measured-urdf
                    # sec.3 ("閾値を変える必要があれば変えずにoutboxで報告
                    # する") even though sensor_model is now TGS2620 (MOX,
                    # resistance-based "raw" output) -- whether 0.5 still
                    # means anything for that scale is unverified, flag in
                    # outbox if it looks broken.
                    "th_gas_present": 0.5,
                    "th_wind_present": 0.1,
                    "ground_truth_x": SOURCE_X,
                    "ground_truth_y": SOURCE_Y,
                    "resultsFile": RESULTS_FILE,
                    "navigationPathFile": NAVIGATION_PATH_FILE,
                    "maxSearchTime": MAX_SEARCH_TIME,
                    # Default is "PID/Sensor_reading" (Algorithm.cpp:39) --
                    # osl_robot has no PID node anymore, only the 6 TGS2620
                    # nodes named after their URDF frame (see gas sensor
                    # GroupAction above). Only gas_front_mid feeds GrGSL
                    # (spec-gazebo-measured-urdf sec.3).
                    "enose_topic": "gas_front_mid/Sensor_reading",
                    "anemometer_frame": FRAME_PREFIX + "anemometer_link",
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
