# CUADC 2026 —— 固定翼无人机侦察与打击（伴随计算机软件）

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Status](https://img.shields.io/badge/status-design--phase-lightgrey)

固定翼无人机自主侦察与打击任务的伴随计算机（Companion Computer）软件，
飞控为 Pixhawk + ArduPilot。本仓库仅覆盖软件组负责的部分：感知/视觉、
规划/决策、以及与飞控的通信接口；飞控本身的调参、机体、投放机构由
另一个子队负责。

## 现状

设计阶段，尚无可运行代码。设计规范正在 `docs/spec/`（建立中）里逐步
讨论完善。

## 目录结构

- `docs/spec/` —— 设计规范（架构、模块、接口、约定），持续维护
- `docs/reference/` —— 从竞赛规则等外部资料摘录的背景参考
- `src/` —— ROS2 功能包（待建立）
- `scripts/` —— 工具/仿真脚本

## 相关比赛项目

2026 中国大学生飞行器设计创新大赛 ——「固定翼无人机侦察与打击」。
