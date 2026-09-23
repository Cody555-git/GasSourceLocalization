#!/bin/bash
# Runs wind_direction_check.py against V3 lv1's GADEN playback (item 6 of
# spec-gazebo-measured-urdf). Usage: run_wind_direction_check.sh <out_dir> [rounds]
OUT=$1; ROUNDS=${2:-2}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source /home/gaden_ws/install/setup.bash
source /home/ros2_ws/install/setup.bash
PIDS=()
cleanup() { trap - EXIT INT TERM; for p in "${PIDS[@]}"; do pkill -INT -P "$p" 2>/dev/null; kill -INT "$p" 2>/dev/null; done; sleep 3; for p in "${PIDS[@]}"; do pkill -9 -P "$p" 2>/dev/null; kill -9 "$p" 2>/dev/null; done; }
trap cleanup EXIT INT TERM
ros2 launch "$HERE/../launch/gaden_player_v3.launch.py" use_rviz:=False > "$OUT/player.log" 2>&1 & PIDS+=($!)
ros2 run simulated_anemometer simulated_anemometer --ros-args -r __node:=anemo_test -p sensor_frame:=anemo_test -p fixed_frame:=map -p noise_std:=0.0 -p use_map_ref_system:=false -p topic:=/anemo_test/reading > "$OUT/anemo.log" 2>&1 & PIDS+=($!)
python3 "$HERE/wind_direction_check.py" --out "$OUT" --rounds "$ROUNDS" 2>&1 | tee "$OUT/check.log"
