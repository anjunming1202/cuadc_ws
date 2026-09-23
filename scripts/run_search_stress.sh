#!/bin/bash
# 仿真压力测试台入口：source 好环境(ROS + 本工作空间 + Gazebo 额外项),再跑
# 无头多轮跑分器 scripts/search_stress.py。见 docs/spec/sim-stress-testbed.md。
# 编排器内部会各自起 run_gazebo_demo.sh / run_mission_sim.sh(它们再各自 source
# 一次;子进程独立 session,便于整棵树 killpg 清场),这里 source 只是让本进程
# 能 import rclpy、发/收话题。
#
# Usage: ./scripts/run_search_stress.sh [search_stress.py args]
#   ./scripts/run_search_stress.sh --trials 1 --gui                 # 看一轮(带窗口)
#   ./scripts/run_search_stress.sh --trials 10 --out /tmp/base.csv  # 基线 10 轮(无头)
#   ./scripts/run_search_stress.sh --help                           # 全部参数

# No -u: ROS setup.bash files reference unset vars (see run_mission_sim.sh).
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"
# Gazebo 额外环境,与 run_gazebo_demo.sh 一致(编排器要起的子栈也依赖它,
# 但编排器本进程 import rclpy / 发收话题也在同一环境下最省事)。
source ~/gazebo_ros2_ws/install/setup.bash
export GZ_VERSION=harmonic

exec python3 "$REPO_ROOT/scripts/search_stress.py" "$@"
