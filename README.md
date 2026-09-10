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
到 ROS2、QGroundControl 验证正常），详见 `src/cuadc_sim/README.md`。
接口层（`src/cuadc_interface` + `src/cuadc_interface_msgs`）已实现
`docs/spec/04-interfaces.md` 里的全部话题/服务并过 SITL 验证，详见
`src/cuadc_interface/README.md`。感知/决策骨架（桩感知 + 完整任务状态机）
已在 SITL+Gazebo 里跑通全流程闭环，详见 `src/cuadc_perception/README.md`、
`src/cuadc_decision/README.md`；真实 CV 感知算法还未实现。

外部时间约束：飞控组 phase 1（人工飞行调姿态/悬停控制环）需要在
2026-08-01 前完成，与软件组互不阻塞，详见 `docs/spec/working-notes.md`
"已经决定的事"一节。

## 目录结构

- `docs/` —— 文档
  - `docs/spec/` —— 设计规范（架构、模块、接口、约定、场地几何建模），
    持续维护，规则见其自己的 `docs/spec/README.md`
    - `docs/spec/adr/` —— 架构决策记录，规则见 `docs/spec/README.md`
    - `docs/spec/working-notes.md` —— 过渡性讨论记录，性质与生命周期见
      文件自己头部说明
  - `docs/reference/` —— 从竞赛规则等外部资料摘录的背景参考，详见其自己
    的 `docs/reference/README.md`
  - `docs/lessons.md` —— 踩坑记录（环境/工具坑 + 实验发现），只增不改
- `src/` —— colcon workspace 下的 ROS2 包
  - `src/cuadc_sim` —— 仿真资产（Gazebo/SITL bringup launch 文件、验证
    节点），详见 `src/cuadc_sim/README.md`
  - `src/cuadc_interface` —— 接口层（与 ArduPilot 的 MAVLink 通信桥接
    节点），详见 `src/cuadc_interface/README.md`
  - `src/cuadc_interface_msgs` —— 接口层用到的自定义 ROS2 消息/服务
    定义，详见 `src/cuadc_interface_msgs/README.md`
  - `src/cuadc_perception` —— 感知模块（目前是桩：从场地已知坐标发布
    检测结果，契约与真实 CV 节点一致），详见 `src/cuadc_perception/README.md`
  - `src/cuadc_perception_msgs` —— 感知/决策之间的检测结果消息定义，
    详见 `src/cuadc_perception_msgs/README.md`
  - `src/cuadc_decision` —— 决策模块（任务状态机），详见
    `src/cuadc_decision/README.md`
- `scripts/` —— 工具/仿真脚本

## Quickstart

跑仿真需要先起仿真世界，再起任务节点（接口+感知+决策），最后单独触发
任务开始——三层分开是有意的，任务节点这一层在真机上原样复用。

```bash
# 1. 仿真世界（Gazebo + ArduPilot SITL），真机上跳过这一步
./scripts/run_gazebo_demo.sh

# 2. 任务节点：接口层 + 感知 + 决策，另开一个终端
./scripts/run_task_nodes.sh
#   真机上换成真实串口：
#   ./scripts/run_task_nodes.sh connection:=/dev/ttyACM0:57600

# 3. 等终端2日志停在 WAIT_TASK_START，再开一个终端触发任务开始
#    （模拟物理开关；真机上这里会换成 GPIO，尚未实现）
source scripts/activate_env.sh
ros2 topic pub -1 /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"
```

看状态机走到哪一步：`ros2 topic echo /cuadc_decision/mission_state`

各节点参数、话题/服务契约见对应包的 README 和 `docs/spec/`。

## 相关比赛项目

2026 中国大学生飞行器设计创新大赛 ——「多旋翼无人机侦察与救援」。
