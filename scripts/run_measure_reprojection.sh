#!/bin/bash
# 反投影真值量测一键入口（sim-only，见 measure_reprojection.py 文件头）。
# 自起 headless gazebo(bench world) + interface + perception，跑量测，按 PID 拆除
# （只杀自己起的进程组，绝不 pkill 模式串，见 docs/lessons.md）。
#
# Usage: ./scripts/run_measure_reprojection.sh
# 退出码 = 量测 PASS(0)/FAIL(非0)，可当保真回归门。

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"
BENCH_WORLD="$(ros2 pkg prefix cuadc_sim)/share/cuadc_sim/worlds/cuadc_field_drop_zone_bench.sdf"
source ~/gazebo_ros2_ws/install/setup.bash
export GZ_VERSION=harmonic

PIDS=()
cleanup() {
    for pid in "${PIDS[@]}"; do
        kill -INT -- "-$pid" 2>/dev/null || true
    done
    sleep 6
    for pid in "${PIDS[@]}"; do
        kill -9 -- "-$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT

setsid ros2 launch cuadc_sim gazebo_bringup.launch.py "world:=$BENCH_WORLD" gui:=false \
    > /tmp/measure_reproj_gazebo.log 2>&1 &
PIDS+=($!)
echo "waiting for gazebo world..."
for _ in $(seq 1 40); do
    gz topic -l 2>/dev/null | grep -q "camera_link/sensor/camera/image" && break
    sleep 3
done

setsid ros2 launch cuadc_interface interface_bringup.launch.py connection:=udp:127.0.0.1:14550 \
    > /tmp/measure_reproj_iface.log 2>&1 &
PIDS+=($!)
setsid ros2 launch cuadc_perception cv_perception_bringup.launch.py profile:=sim frame_source:=ros \
    > /tmp/measure_reproj_percep.log 2>&1 &
PIDS+=($!)
sleep 8

python3 "$REPO_ROOT/scripts/measure_reprojection.py"
