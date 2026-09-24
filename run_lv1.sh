#!/usr/bin/env bash
# Gazebo + V3_Lv1 部屋で GrGSL (GSL) を一発で起動する。
# 想定実行場所: humble_container の中（gaden_ws・ros2_ws が /home/ 直下にある前提）。
#
#   docker exec -it humble_container bash -lc \
#     '/home/ros2_ws/src/GasSourceLocalization/run_lv1.sh'
#
# Ctrl-C / SIGTERM で Gazebo・nav2・GADEN など起動した全ノードを後片付けしてから終了する。
set -o pipefail

source /opt/ros/humble/setup.bash
source /home/gaden_ws/install/setup.bash
source /home/ros2_ws/install/setup.bash
export DISPLAY="${DISPLAY:-:10.0}"

LAUNCH_PID=""
# Nav2 の lifecycle_manager が configure のサービス応答を待ったまま固まることがある
# （2026-09-24 に4試行中2回。map_server / planner_server で停止し、GSL は move_base を
# 300 秒待ち続ける）。NAV2_TIMEOUT 秒以内に "Managed nodes are active" が出なければ
# 全ノードを止めて最初からやり直す（最大 MAX_ATTEMPTS 回）。
NAV2_TIMEOUT="${NAV2_TIMEOUT:-60}"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-3}"
ATTEMPT_LOG="$(mktemp /tmp/run_lv1_attempt.XXXXXX)"

stop_all() {
    # 固まったノードがいると launch は SIGINT で終わらないので、待つのは最大 10 秒
    if [ -n "$LAUNCH_PID" ]; then
        kill -INT "$LAUNCH_PID" 2>/dev/null
        for _ in $(seq 1 10); do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 1
        done
        kill -KILL "$LAUNCH_PID" 2>/dev/null
    fi
    # 既知の罠: ros2 launch 本体を止めても子ノードが生き残ることがある
    # (research/loop-state.md の罠表参照)。取りこぼしを掃除する。
    # 固まった lifecycle_manager は SIGINT では止まらないので --ros-args 付きの全ノードも KILL する。
    pkill -INT -f "gazebo_v3_lv1.launch.py" 2>/dev/null
    pkill -INT -f "ign gazebo .*s5406a_v3_lv1" 2>/dev/null
    sleep 2
    pkill -KILL -f "gazebo_v3_lv1.launch.py" 2>/dev/null
    pkill -KILL -f "ign gazebo .*s5406a_v3_lv1" 2>/dev/null
    pkill -KILL -f "[-]-ros-args" 2>/dev/null
    LAUNCH_PID=""
}

cleanup() {
    trap - EXIT INT TERM
    echo "[run_lv1] 後片付け中..."
    stop_all
    rm -f "$ATTEMPT_LOG"
    echo "[run_lv1] 終了"
}
trap cleanup EXIT INT TERM

echo "[run_lv1] Gazebo + V3_Lv1 + GrGSL を起動します（結果は resultsFile/navigationPathFile の絶対パス先を確認）"
for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
    : > "$ATTEMPT_LOG"
    ros2 launch grgsl_env gazebo_v3_lv1.launch.py "$@" > >(tee -a "$ATTEMPT_LOG") 2>&1 &
    LAUNCH_PID=$!
    ready=0
    for _ in $(seq 1 "$NAV2_TIMEOUT"); do
        if grep -q "Managed nodes are active" "$ATTEMPT_LOG"; then ready=1; break; fi
        kill -0 "$LAUNCH_PID" 2>/dev/null || break
        sleep 1
    done
    if [ "$ready" = 1 ]; then
        echo "[run_lv1] Nav2 active（試行 $attempt 回目）"
        wait "$LAUNCH_PID"
        exit 0
    fi
    echo "[run_lv1] Nav2 が ${NAV2_TIMEOUT} 秒以内に active にならなかった（試行 $attempt 回目）。止めてやり直す"
    stop_all
    sleep 3
done
echo "[run_lv1] Nav2 が ${MAX_ATTEMPTS} 回とも起動しなかった。中止"
exit 1
