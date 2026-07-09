# 项目技术栈参考

本项目用到的软件/工具/协议一览，每条只写一句"是什么"，不追求写全，
遇到新工具随时补充。外部背景资料，不适用"只有来源变了才改"的纪律——这份文件本来就是随认识的
工具增加持续累积的。

- **ROS2 (Humble)**：机器人操作系统，本项目所有代码的运行框架，`src/`
  下的每个包、launch 文件、节点都是 ROS2 的概念。

- **colcon**：ROS2 官方的编译/包管理工具，`colcon build` 是编译
  `src/` 下每个包的方式。

- **ArduPilot**：开源飞控软件项目，本项目用的飞控栈；同一份源码能编译
  出多旋翼（ArduCopter）、固定翼（ArduPlane）等不同飞行器的固件。

- **ArduCopter**：从 ArduPilot 源码编译出的多旋翼专用飞控程序，是本
  项目的编译目标。

- **SITL（Software In The Loop）**：ArduPilot 官方自带的软件在环模拟
  器，把飞控程序编译成能在电脑上直接运行的程序，配合内置的简化飞行
  物理模型模拟飞行，不需要真飞控硬件。

- **MAVLink**：无人机与地面站/伴随计算机之间通信用的协议标准，规定了
  位置、模式切换、指令等消息的格式。

- **pymavlink**：MAVLink 协议的 Python 实现库，负责编码/解码协议消息；
  `cuadc_sim` 的验证脚本用它连接 SITL。

- **MAVProxy**：基于 pymavlink 的命令行地面站+消息路由器，能把一路
  MAVLink 连接转发给多个客户端（比如同时给 QGroundControl 和我们自己
  的脚本用）。

- **QGroundControl**：图形化地面站软件，用地图和仪表盘显示飞控状态
  （位置、姿态、模式、电量），仿真和真机通用。
  
- **Gazebo（新一代，gz-sim / "Gazebo Sim"）**：开源 3D 机器人仿真器，
  能渲染场景和摄像头等传感器画面。本项目用 **Harmonic**（LTS，`gz sim`
  版本号 8.x），是 ArduPilot 官方当前支持的版本；跟已被淘汰的
  **Gazebo Classic 11**（2025-01 EOL，和 ArduPilot 现行工具链不兼容）
  是两套不同的东西。已决定接入（见 `docs/spec/working-notes.md`）。

- **ardupilot_gazebo**：ArduPilot 官方的 C++ Gazebo 插件（从源码编译，
  产物在 `~/ardupilot_gazebo`），提供 ArduPilotPlugin 等；SITL 用
  `--model JSON` 通过一个本地 socket（默认 9002）把物理/传感器计算
  交给 Gazebo，替换掉 SITL 自带的简化物理模型。

- **ardupilot_gz**：ArduPilot 官方的 ROS2 集成层（另一个仓库，规划中，
  `~/gazebo_ros2_ws`），负责把 Gazebo 和 ROS2 话题打通，跟上面那个
  纯 C++ 插件不是一回事。

- **gz sim**：新一代 Gazebo 的命令行入口。`-s` 只跑物理服务端（无 GUI，
  headless），带 GUI 则省略 `-s`；靠 `GZ_SIM_SYSTEM_PLUGIN_PATH`（找
  插件 .so）和 `GZ_SIM_RESOURCE_PATH`（找 world/model）两个环境变量
  定位 `ardupilot_gazebo` 的资产。
