# CUADC 2026 —— 固定翼无人机侦察与打击（伴随计算机软件）

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Status](https://img.shields.io/badge/status-design--phase-lightgrey)

固定翼无人机自主侦察与打击任务的伴随计算机（Companion Computer）软件，
飞控为 Pixhawk + ArduPilot。本仓库仅覆盖软件组负责的部分：感知/视觉、
规划/决策、以及与飞控的通信接口；飞控本身的调参、机体、投放机构由
另一个子队负责。

## 现状

设计阶段，尚无可运行代码。设计规范正在 `docs/spec/`（建立中）里逐步
讨论完善；讨论到目前为止的决策和待定问题见 `docs/spec/working-notes.md`。

外部时间约束：飞控组 phase 1（人工飞行 + 按软件组模拟指令调试控制器）
需要在 2026-08-01 前完成，依赖本仓库尽早给出稳定的指令接口。

## 目录结构

- `docs/spec/` —— 设计规范（架构、模块、接口、约定），持续维护
  - `docs/spec/adr/` —— 架构决策记录（ADR）：记录关键决策"为什么"这么定的，
    一个决策一份文件，写完不回改；跟当前设计本身（上面那些 spec 文档）是
    两回事
  - `docs/spec/working-notes.md` —— 讨论中但还没整理进正式 spec 的决策/
    待定问题，过渡性质，内容迁入正式文档后应删除对应部分
- `docs/reference/` —— 从竞赛规则等外部资料摘录的背景参考，详见其自己的
  `docs/reference/README.md`
- `docs/lessons.md` —— 踩坑记录（环境/工具坑 + 实验发现），只增不改
- `src/` —— ROS2 功能包（待建立）
- `scripts/` —— 工具/仿真脚本

## 相关比赛项目

2026 中国大学生飞行器设计创新大赛 ——「固定翼无人机侦察与打击」。
