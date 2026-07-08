# Claude Agent Workflow

## Project overview

CUADC 2026 —— 固定翼无人机侦察与打击（Fixed-wing UAV Reconnaissance & Strike）。
本仓库是伴随计算机（Companion Computer）软件部分：感知/视觉、规划/决策、
以及与 Pixhawk/ArduPilot 的通信接口。飞控本身（Pixhawk 参数、PID、机体、
投放机构）由另一个子队（飞控组）负责，不在本仓库范围内。

ROS2 Humble（与本机其它项目共用的环境一致）。

## Current status

设计阶段。`docs/spec/` 尚未建立——架构和各模块的设计规范正在与人类架构师
讨论中，讨论确定后才会落成正式文档；spec 确定前不要在 `src/` 下开始实现
具体功能包。

## Working mode

- 人：系统设计、架构、技术决策。
- AI：代码实现、算法实现、文档完善——始终以 `docs/spec/` 里已确认的设计
  为依据，不做无依据的自由发挥。

## Language

用中文或英文回复，不用其它语言。

## Notes

本文件内容会随 `docs/spec/` 的建立不断扩充和调整，目前只是占位。
