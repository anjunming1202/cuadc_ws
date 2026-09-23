#!/bin/bash
# 投放区单点测试 · L2 投放机构单点：只起接口层，手动调 release_payload 验证舵机动作。
# 不起感知/决策、不发飞行/导航指令。
#
# 安全前提——release_payload 有意在「未解锁」时拒绝执行（地面不由软件驱动投放，交给
# 遥控器/人工，见 ADR-0007）。所以想用本服务驱动舵机，飞控必须处于 armed。地面 bench
# 验证请**拆桨**后再解锁，或让飞控/机械子队直接用遥控器打投放通道验证舵机——两条路都行，
# 但绝不要带桨在地面解锁。飞手 RC 全程可接管。详见 docs/spec/field-test-runbook.md。
#
# 起来后（另一个终端）：
#   # 1) 解锁（拆桨后！）：
#   ros2 service call /cuadc_interface/arm cuadc_interface_msgs/srv/Arm "{arm: true}"
#   # 2) 触发一次投放（跑 forward→drop_run_s→neutral→释放，物理动作完成才返回）：
#   ros2 service call /cuadc_interface/release_payload cuadc_interface_msgs/srv/ReleasePayload "{}"
#   # 投放通道/PWM/时长由接口层参数配置（drop_rc_channel/drop_forward_us/drop_run_s…），
#   # 要临时改：给 run_mission_hardware/本脚本追加 drop_run_s:=... 等 launch 参数。
#
# Usage: ./scripts/run_field_l2_drop_actuator.sh [extra ros2 launch args, e.g. drop_run_s:=1.5]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

exec ros2 launch cuadc_interface interface_bringup.launch.py \
    connection:=udp:127.0.0.1:14550 "$@"
