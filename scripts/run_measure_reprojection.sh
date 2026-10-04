#!/bin/bash
# 反投影真值量测一键入口（sim-only，见 measure_reprojection.py 文件头）。
# 自起 headless gazebo(bench world) + interface + perception，跑量测，按 PID 组拆除
# （只杀自己起的进程组，绝不 pkill 模式串，见 docs/lessons.md）。
#
# Usage: ./scripts/run_measure_reprojection.sh [latency_ms] [pose_at] [gazebo 透传参数…]
#   latency_ms  相机->CV 图像延迟(保留拍照戳);省略或 default = 仿真世界默认(disturbances.yaml)
#   pose_at     capture(默认,生产行为) | latest(仅证据链 A/B 对照)
#   透传        如 imperfections:=false(干净 sim)、sim_realism_params:=''(只关风/噪声)
# 证据链(ADR-0010 B/C): 0 capture=PASS; 150 latest=moving 行出大偏差; 150 capture=恢复。
# 退出码 = 量测 PASS(0)/FAIL(非0)，可当保真回归门。

set -eo pipefail

LATENCY_MS="${1:-default}"
POSE_AT="${2:-capture}"
shift $(( $# < 2 ? $# : 2 ))
GZ_ARGS=("$@")
[ "$LATENCY_MS" != "default" ] && GZ_ARGS+=("image_latency_ms:=$LATENCY_MS")

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

setsid ros2 launch cuadc_sim gazebo_bringup.launch.py "world:=$BENCH_WORLD" gui:=false "${GZ_ARGS[@]}" \
    > /tmp/measure_reproj_gazebo.log 2>&1 &
PIDS+=($!)
echo "waiting for gazebo world..."
for _ in $(seq 1 40); do
    gz topic -l 2>/dev/null | grep -q "camera_link/sensor/camera/image" && break
    sleep 3
done

setsid ros2 launch cuadc_interface interface_bringup.launch.py connection:=udp:127.0.0.1:14550 use_sim_time:=true \
    > /tmp/measure_reproj_iface.log 2>&1 &
PIDS+=($!)

IMAGE_TOPIC=/camera/image_raw  # 仿真世界已按 image_latency_ms 延迟(保留拍照戳)

setsid ros2 launch cuadc_perception cv_perception_bringup.launch.py profile:=sim frame_source:=ros \
    use_sim_time:=true "image_topic:=$IMAGE_TOPIC" "pose_at:=$POSE_AT" \
    > /tmp/measure_reproj_percep.log 2>&1 &
PIDS+=($!)
sleep 8

python3 "$REPO_ROOT/scripts/measure_reprojection.py"
