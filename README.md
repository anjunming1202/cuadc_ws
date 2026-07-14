# CUADC 2026 —— 多旋翼无人机侦察与救援（伴随计算机软件）

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Status](https://img.shields.io/badge/status-design--phase-lightgrey)

多旋翼无人机自主完成标准载荷投放与灾情侦察任务的伴随计算机（Companion
Computer）软件，飞控为 Pixhawk + ArduPilot（ArduCopter）。本仓库仅覆盖
软件组负责的部分：感知/视觉、规划/决策、以及与飞控的通信接口；飞控本身
的调参、机体、投放机构由另一个子队负责。

## 现状

架构设计 + 仿真环境搭建阶段。顶层架构见 `docs/spec/00-index.md`；已拍板
的架构决策见 `docs/spec/adr/`；讨论中、尚未拍板的开放问题见
`docs/spec/working-notes.md`。

`src/cuadc_sim` 已跑通 Gazebo+SITL 端到端仿真闭环（物理、摄像头画面桥接
到 ROS2、QGroundControl 验证正常），感知/规划/接口层这几个正式模块还
未开始实现，详见 `src/cuadc_sim/README.md`。

外部时间约束：飞控组 phase 1（人工飞行 + 按软件组模拟指令调试控制器）
需要在 2026-08-01 前完成，依赖本仓库尽早给出稳定的指令接口。

## 目录结构

- `docs/spec/` —— 设计规范（架构、模块、接口、约定、场地几何建模），
  持续维护，规则见其自己的 `docs/spec/README.md`
  - `docs/spec/adr/` —— 架构决策记录，规则见 `docs/spec/README.md`
  - `docs/spec/working-notes.md` —— 过渡性讨论记录，性质与生命周期见
    文件自己头部说明
- `docs/reference/` —— 从竞赛规则等外部资料摘录的背景参考，详见其自己的
  `docs/reference/README.md`
- `docs/lessons.md` —— 踩坑记录（环境/工具坑 + 实验发现），只增不改
- `src/cuadc_sim` —— 仿真资产（Gazebo/SITL bringup launch 文件、验证
  节点），详见 `src/cuadc_sim/README.md`；感知/规划/接口层等正式模块的
  包还未建立
- `scripts/` —— 工具/仿真脚本

## 相关比赛项目

2026 中国大学生飞行器设计创新大赛 ——「多旋翼无人机侦察与救援」。
