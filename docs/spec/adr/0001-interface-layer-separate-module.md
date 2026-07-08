# 0001. 接口层（ArduPilot/MAVLink 通信）独立成模块

**Status**: Accepted

## Context

伴随计算机软件需要和 ArduPilot 飞控通信（订阅 telemetry、发送 GUIDED
指令、上传 Mission 等，见
[`ardupilot-interface-overview.md`](../../reference/ardupilot-interface-overview.md)）。
这部分通信细节可以独立成一个模块，也可以直接并入规划/决策模块内部实现。

早期 Windows 侧的架构草案（`CUADC项目框架（初步方案）.md`，写于知道
具体比赛规则之前）把"飞控通信模块"列成规划/决策内部的一个子模块，倾向
合并；但那份草案的范围比现在宽，也没有考虑到两人团队分工和测试隔离
的实际需求。

外部约束：飞控组需要在 2026-08-01 前完成 phase 1（人工飞行 + 按软件组
给出的模拟指令调试控制器），这依赖软件组尽早给出稳定的指令接口，也就
需要先确定规划/决策与飞控通信之间的模块边界。

## Decision

ArduPilot/MAVLink 通信独立成一个模块（预期是一个独立的 ROS2 包），
不并入规划/决策模块内部。暂定包名沿用 working notes 中提到的
`mavros_bridge`——**暂定，未最终确定**，与 `cuadc_sim` 包名的处理方式
一致，最终命名留到 `01-conventions.md` / 实际建包时再定。

## Consequences

- 规划/决策与接口层之间需要定义清晰的话题/服务/动作契约，具体化工作
  留给 `04-interfaces.md`。
- 规划/决策模块可以用桩（stub）替代真实/仿真飞控进行单独测试，不必
  每次都跑起完整的 SITL 才能测试决策逻辑。
- 两人团队可以按模块边界分工（一人负责决策逻辑，一人负责飞控接口
  适配），减少互相阻塞。
- 代价：多一层进程/组件边界，需要维护跨模块的接口契约，增加一点点
  工程复杂度。
