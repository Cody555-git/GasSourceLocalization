#!/bin/bash
# Runs gas_field_survey.py against V3 lv1's GADEN playback (item 0 b/c of
# spec-gazebo-measured-urdf). Usage: run_gas_field_survey.sh <out_dir> [duration_s]
# Probe (TGS2620) sits at the lv1 start pose (-1.67, 0.99), 0.11 m high.
OUT=$1; DUR=${2:-300}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source /home/gaden_ws/install/setup.bash
source /home/ros2_ws/install/setup.bash
PIDS=()
cleanup() { trap - EXIT INT TERM; for p in "${PIDS[@]}"; do pkill -INT -P "$p" 2>/dev/null; kill -INT "$p" 2>/dev/null; done; sleep 3; for p in "${PIDS[@]}"; do pkill -9 -P "$p" 2>/dev/null; kill -9 "$p" 2>/dev/null; done; }
trap cleanup EXIT INT TERM
ros2 launch "$HERE/../launch/gaden_player_v3.launch.py" use_rviz:=False > "$OUT/player.log" 2>&1 & PIDS+=($!)
ros2 run tf2_ros static_transform_publisher --x "${PROBE_X:--1.67}" --y "${PROBE_Y:-0.99}" --z 0.11 --frame-id map --child-frame-id probe > "$OUT/tf.log" 2>&1 & PIDS+=($!)
ros2 run simulated_gas_sensor simulated_gas_sensor --ros-args -r __node:=probe -p sensor_model:=0 -p sensor_frame:=probe -p fixed_frame:=map > "$OUT/probe.log" 2>&1 & PIDS+=($!)
python3 "$HERE/gas_field_survey.py" --out "$OUT" --duration "$DUR" 2>&1 | tee "$OUT/survey.log"
