#!/usr/bin/env bash
# Gazebo + V3_Lv1 部屋で GrGSL (GSL) を一発で起動する。
# 想定実行場所: humble_container の中（gaden_ws・ros2_ws が /home/ 直下にある前提）。
#
#   docker exec -it humble_container bash -lc \
#     '/home/ros2_ws/src/GasSourceLocalization/run_lv1.sh'
#
# Ctrl-C / SIGTERM で Gazebo・nav2・GADEN など起動した全ノードを後片付けしてから終了する。
set -uo pipefail

source /opt/ros/humble/setup.bash
source /home/gaden_ws/install/setup.bash
source /home/ros2_ws/install/setup.bash
export DISPLAY="${DISPLAY:-:10.0}"

LAUNCH_PID=""

cleanup() {
    echo "[run_lv1] 後片付け中..."
    if [ -n "$LAUNCH_PID" ]; then
        kill -INT "$LAUNCH_PID" 2>/dev/null
        wait "$LAUNCH_PID" 2>/dev/null
    fi
    # 既知の罠: ros2 launch 本体を止めても子ノードが生き残ることがある
    # (research/loop-state.md の罠表参照)。取りこぼしを掃除する。
    pkill -INT -f "gazebo_v3_lv1.launch.py" 2>/dev/null
    pkill -INT -f "ign gazebo .*s5406a_v3_lv1" 2>/dev/null
    sleep 2
    pkill -KILL -f "gazebo_v3_lv1.launch.py" 2>/dev/null
    pkill -KILL -f "ign gazebo .*s5406a_v3_lv1" 2>/dev/null
    echo "[run_lv1] 終了"
}
trap cleanup EXIT INT TERM

echo "[run_lv1] Gazebo + V3_Lv1 + GrGSL を起動します（結果は resultsFile/navigationPathFile の絶対パス先を確認）"
ros2 launch grgsl_env gazebo_v3_lv1.launch.py "$@" &
LAUNCH_PID=$!
wait "$LAUNCH_PID"
