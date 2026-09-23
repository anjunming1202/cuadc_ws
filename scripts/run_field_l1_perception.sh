#!/bin/bash
# 投放区单点测试 · L1 感知单点：接口层 + 真实 CV 感知，**不发任何飞行指令**。
# 飞机停在地面、飞手 RC 随时可接管。验证 shm 取帧→检测→反投影→尺寸判别这条链路。
#
# 前置（Pi 上）：底座在跑（mav_fanout 出 14550、camera_service 出 shm）；且 shm 只能
# 一个消费者——若 ground-monitor 的 cuadc-onboard-cv-preview 单元在占 shm，先停它
# （用 profile=none 或 systemctl stop），否则本节点取不到帧。详见
# docs/spec/field-test-runbook.md。
#
# 起来后看检测结果：
#   ros2 topic echo /perception/detections                # DetectionArray（世界坐标/半径）
#   地面 CV 预览：本节点标注推本地 5604，由 ground-monitor 的 cv-forward 转 5606 到地面
#
# Usage: ./scripts/run_field_l1_perception.sh [extra ros2 launch args]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"
source "$REPO_ROOT/scripts/real_camera_intrinsics.sh"   # -> REAL_CV_LAUNCH_ARGS

exec ros2 launch "$REPO_ROOT/launch/field_perception_bench.launch.py" \
    connection:=udp:127.0.0.1:14550 profile:=real "${REAL_CV_LAUNCH_ARGS[@]}" "$@"
