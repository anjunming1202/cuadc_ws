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
`src/cuadc_interface/README.md`。感知（真实 CV 算法 + 地面真值桩两个
实现）/决策（完整任务状态机）已在 SITL+Gazebo 里跑通全流程闭环，详见
`src/cuadc_perception/README.md`、`src/cuadc_decision/README.md`。地面站
实时监测 dashboard 见 `src/cuadc_viz/README.md`。

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
  - `src/cuadc_perception` —— 感知模块（真实 CV 检测算法 + 地面真值桩，
    两者发布同一份契约），详见 `src/cuadc_perception/README.md`
  - `src/cuadc_perception_msgs` —— 感知/决策之间的检测结果消息定义，
    详见 `src/cuadc_perception_msgs/README.md`
  - `src/cuadc_decision` —— 决策模块（任务状态机），详见
    `src/cuadc_decision/README.md`
  - `src/cuadc_viz` —— 地面站实时监测 dashboard，详见
    `src/cuadc_viz/README.md`
- `launch/` —— 组合多个包节点的顶层 launch 文件（不属于任何单个包），详见
  Quickstart
- `scripts/` —— 工具/仿真脚本

## 依赖与环境搭建

从 clone 本仓库到跑通下面的 Quickstart，把这一节从上到下走一遍即可。
所有外部依赖**不 vendor 进本仓库**，按下表装到指定目录（目录路径被脚本
写死，改路径要同步改 `scripts/*.sh`）。选型/落盘位置的依据见
`docs/reference/tech-stack.md` 与 `docs/spec/working-notes.md`。

### 系统要求

- **Ubuntu 22.04**（ROS2 Humble 的官方目标平台；WSL2 亦可，SITL 相关
  命令务必带 `--no-wsl2-network`，见 `docs/lessons.md`）。
- **Python 3.10**（Ubuntu 22.04 系统自带，也是 ROS2 Humble 的目标版本，
  无需另装）。
- 独立显卡非必需，但 Gazebo 完整仿真需要能渲染的 OpenGL 环境。

### 外部依赖一览

| 依赖 | 版本 | 装法 | 位置 |
|---|---|---|---|
| ROS2 | Humble | apt（加 ROS2 官方源） | 系统 `/opt/ros/humble` |
| Python | 3.10 | Ubuntu 22.04 自带 | 系统 |
| ArduPilot + SITL | ArduCopter，clone 默认分支、未锁定 tag | `git clone --recurse-submodules` | `~/ardupilot` |
| Gazebo | Harmonic（`gz-sim` 8.x，LTS） | apt（加 OSRF 官方源） | 系统 |
| ardupilot_gazebo（C++ 插件） | `main` | 源码编译 | `~/ardupilot_gazebo` |
| ardupilot_gz（`ros_gz_bridge`） | 官方 `ros2_gz.repos` 锁定版本 | vcs import + colcon build | `~/gazebo_ros2_ws` |
| Micro-XRCE-DDS-Gen | ArduPilot 分支 | 源码编译 | `~/Micro-XRCE-DDS-Gen` |
| QGroundControl（可选，图形地面站） | 官方发布版 AppImage，未锁定 | AppImage | `~/QGroundControl.AppImage` |
| Python 包 | 见步骤 3（`empy` 锁 3.3.4） | `pip3 install --user`（不走 apt/rosdep） | 用户级 |
| ROS 二进制包 + 本仓库依赖 | 跟随 Humble | apt / rosdep | 系统 |

### 安装步骤

```bash
# 1) ROS2 Humble：按官方文档加源后
sudo apt install ros-dev-tools python3-rosdep python3-colcon-common-extensions
sudo rosdep init && rosdep update   # 首次

# 2) ArduPilot 源码 + SITL 编译工具链（脚本会装系统依赖，需要 sudo）
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y

# 3) Python 包（SITL / MAVLink / 接口层都要）
pip3 install --user pymavlink MAVProxy pexpect pyserial empy==3.3.4 geocoder ptyprocess dronecan

# 4) Micro-XRCE-DDS-Gen（编译 SITL 的 DDS 支持需要，缺了会报
#    microxrceddsgen not found）；Gazebo Harmonic + ardupilot_gazebo +
#    ardupilot_gz(~/gazebo_ros2_ws) 三者的具体命令按官方文档 / 
#    docs/reference/tech-stack.md 装到上表所列目录

# 5) 编译 SITL 的 ArduCopter（首次）
cd ~/ardupilot && ./waf configure --board sitl && ./waf copter

# 6) viz 用到的 ROS 二进制包（不在 rosdep 默认基础安装里）
sudo apt install ros-humble-rosbridge-suite ros-humble-web-video-server

# 7) 编译本工作空间
cd <本仓库根目录>
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # 补齐 package.xml 里的 ROS 依赖
colcon build
```

装完后即可开始 Quickstart（每个新终端先 `source scripts/activate_env.sh`）。

## Quickstart

跑仿真需要先起仿真世界，再起任务节点（接口+感知+决策），最后单独触发
任务开始——三层分开是有意的，任务节点这一层在真机上原样复用。

任务节点的启动脚本按"一个脚本对应一种场景"分开，不靠参数默认值切换：

| 脚本 | 感知 | connection 目标 | 用途 |
|---|---|---|---|
| `scripts/run_mission_sim.sh` | 真实 CV | SITL（写死） | **正式任务/完整仿真验证，最常用的一个** |
| `scripts/run_mission_real.sh` | 真实 CV | 真实串口（写死） | 真机部署 |
| `scripts/run_mission_stub_sim.sh` | 桩（地面真值） | SITL（写死） | 只想快速迭代决策层逻辑，不关心真实检测时用 |

```bash
# 1. 仿真世界（Gazebo + ArduPilot SITL），真机上跳过这一步
./scripts/run_gazebo_demo.sh

# 2. 任务节点：接口层 + 真实 CV 感知 + 决策，另开一个终端
./scripts/run_mission_sim.sh
#   真机上换成：
#   ./scripts/run_mission_real.sh

# 3. 等终端2日志停在 WAIT_TASK_START，再开一个终端触发任务开始
#    （模拟物理开关；真机上这里会换成 GPIO，尚未实现）
source scripts/activate_env.sh
ros2 topic pub -1 /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"
```

看状态机走到哪一步：`ros2 topic echo /cuadc_decision/mission_state`

地面站实时监测网页（可选，另开一个终端，跟上面任意组合并行跑都行）：
`ros2 launch cuadc_viz viz_bringup.launch.py`，详见 `src/cuadc_viz/README.md`。

各节点参数、话题/服务契约见对应包的 README 和 `docs/spec/`。

## 相关比赛项目

2026 中国大学生飞行器设计创新大赛 ——「多旋翼无人机侦察与救援」。
