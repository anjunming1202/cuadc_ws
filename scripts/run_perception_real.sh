#!/bin/bash
# 投放区单点测试 · L1 感知单点：接口层 + 真实 CV 感知，**不发任何飞行指令**。
# 飞机停在地面、飞手 RC 随时可接管。验证 shm 取帧→检测→反投影→尺寸判别这条链路。
#
# 前置（Pi 上）：底座在跑（mav_fanout 出 14550、camera_service 出 shm）。详见
# docs/spec/field-test-runbook.md。
#
# 起来后看检测结果：
#   ros2 topic echo /perception/detections                # DetectionArray（世界坐标/半径）
#   地面 CV 预览：用地面站按钮开 onboard 的 cv-preview-ctl（自己从 5602 取图，不占 shm）
#
# Usage: ./scripts/run_perception_real.sh [extra ros2 launch args]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

# 内参按 profile=real 从 camera.real.yaml 读（见 ADR-0009）；frame_source 由 launch 默认 shm。
exec ros2 launch "$REPO_ROOT/launch/field_perception_bench.launch.py" \
    connection:=udp:127.0.0.1:14550 profile:=real "$@"
