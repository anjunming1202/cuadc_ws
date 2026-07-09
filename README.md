# CUADC 2026 —— 多旋翼无人机侦察与救援（伴随计算机软件）

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Status](https://img.shields.io/badge/status-design--phase-lightgrey)

多旋翼无人机自主完成标准载荷投放与灾情侦察任务的伴随计算机（Companion
Computer）软件，飞控为 Pixhawk + ArduPilot（ArduCopter）。本仓库仅覆盖
软件组负责的部分：感知/视觉、规划/决策、以及与飞控的通信接口；飞控本身
的调参、机体、投放机构由另一个子队负责。

## 现状

设计阶段，尚无可运行代码。顶层架构见 `docs/spec/00-index.md`；已拍板的
架构决策见 `docs/spec/adr/`；讨论中、尚未拍板的开放问题见
`docs/spec/working-notes.md`。

外部时间约束：飞控组 phase 1（人工飞行 + 按软件组模拟指令调试控制器）
需要在 2026-08-01 前完成，依赖本仓库尽早给出稳定的指令接口。

## 目录结构

- `docs/spec/` —— 设计规范（架构、模块、接口、约定），持续维护，规则见
  其自己的 `docs/spec/README.md`
  - `docs/spec/adr/` —— 架构决策记录，规则见 `docs/spec/README.md`
  - `docs/spec/working-notes.md` —— 过渡性讨论记录，性质与生命周期见
    文件自己头部说明
- `docs/reference/` —— 从竞赛规则等外部资料摘录的背景参考，详见其自己的
  `docs/reference/README.md`
- `docs/lessons.md` —— 踩坑记录（环境/工具坑 + 实验发现），只增不改
- `src/` —— ROS2 功能包（待建立）
- `scripts/` —— 工具/仿真脚本

## 相关比赛项目

2026 中国大学生飞行器设计创新大赛 ——「多旋翼无人机侦察与救援」。
