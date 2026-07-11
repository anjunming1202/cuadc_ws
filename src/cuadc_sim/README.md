# cuadc_sim

仿真资产：比赛场地世界文件、投放区/灾情侦察区的圆筒与危险化学品标识
模型（还没做，见下方"现状"）、spawn 和话题桥接的 launch 文件（已完成）。
仓库范围/仿真资产归属的决策见
[`docs/spec/working-notes.md`](../../docs/spec/working-notes.md)。

## 外部依赖（不 vendor 进本仓库）

- **ArduPilot 源码 + SITL**：`git clone --recurse-submodules
  https://github.com/ArduPilot/ardupilot.git ~/ardupilot`
- Python 包（`pip3 install --user`，不走 apt/rosdep）：`pymavlink`、
  `MAVProxy`、`pexpect`、`pyserial`、`empy==3.3.4`、`geocoder`、
  `ptyprocess`、`dronecan`
- 完整系统依赖（编译工具链等）由 ArduPilot 官方脚本安装，需要 sudo：
  `~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y`
- **Gazebo Harmonic**（apt，需要单独加 OSRF 官方源，见
  `docs/reference/tech-stack.md`）
- **`ardupilot_gazebo`**（C++ 插件，从源码编译，`~/ardupilot_gazebo`，
  路径与官方文档一致）
- **`ardupilot_gz`**（`~/gazebo_ros2_ws`，vcs import 官方 `ros2_gz.repos`
  锁定版本 + colcon build，见 `docs/reference/tech-stack.md`）
- **Micro-XRCE-DDS-Gen**（`~/Micro-XRCE-DDS-Gen`，从源码构建，见
  `docs/reference/tech-stack.md`）
- **QGroundControl**（AppImage，`~/QGroundControl.AppImage`）

各外部依赖具体装在哪个目录，见
[`docs/spec/working-notes.md`](../../docs/spec/working-notes.md)"已经
决定的事"一节。选型依据（为什么后来决定接 Gazebo）也在同一份文件里。

## 现状

本包有两个 launch 文件，服务两个不同场景，不是新旧替代关系：

- **`sitl_bringup.launch.py`**（SITL 内置简化物理，不接 Gazebo）：轻量
  级飞控指令接口沙盒，不需要 GPU/渲染，启动快。适合单独验证"发指令、
  读遥测"这条 MAVLink 通信逻辑，不涉及摄像头/感知。
- **`gazebo_bringup.launch.py`**（接 Gazebo 物理）：完整仿真闭环，已
  验证端到端可用——Gazebo 物理 + 可视化、SITL 通过 JSON 协议连上 Gazebo
  飞行、QGroundControl 显示正常（`Ready to Fly`，正确的机架参数）、
  摄像头画面通过 `ros_gz_bridge` 桥接到 ROS2 话题 `/camera/image_raw`。
  目前用的是 `ardupilot_gazebo` 仓库自带的示例场景/机体（`iris_runway.sdf`
  + `iris_with_gimbal`），不是我们自己的比赛场地/机体。

**这个 launch 文件没有使用 `ardupilot_gz` 官方自带的 launch 文件**（如
`iris_runway.launch.py`）——那一套在本机环境下必现崩溃，是上游一个未
解决的已知 bug，具体机制和排查过程见 `docs/lessons.md`。两个 launch
文件都**只负责搭建仿真世界本身**，不集成任何任务/验证节点（见下方
"用法"）——"搭仿真世界"和"在这个世界里做什么"永远是两条独立的命令，
任务/规划模块以后也遵循这个边界。

**还没做**：我们自己的比赛场地 world 文件、圆筒/危险化学品标识模型。
另外 Gazebo 画面里跑道贴图目前渲染成灰色网格（贴图文件本身存在，怀疑
是 WSLg 的 GPU 转译层限制），不影响功能，做自己的场地时一并排查。

## 已知问题（未解决）

- **GUIDED 起飞指令被接受，但电机从不真正爬升，几十秒后自动上锁**：
  `sitl_smoke_test` 目前每次跑到 `MAV_CMD_NAV_TAKEOFF` 都收到
  `ACCEPTED`、脚本自己打印"complete"，但 Gazebo 里飞机的高度（ground
  truth）全程不变、`SERVO_OUTPUT_RAW` 卡在怠速 PWM，几秒后飞控自己发
  `STATUSTEXT: "Disarming motors"` 并上锁——**这条命令目前不能真的让
  飞机飞起来**。已排除：wrapper 脚本封装（手动敲命令复现一致）、
  ArduPilot 版本差异（`master`/`Copter-4.6.3` 复现一致）、编译缓存
  陈旧（`rm -rf build` 全新 clean rebuild 后依旧）、WSL2/整机重启后的
  残留状态。当前最可疑的根因：ArduCopter `ArduCopter/takeoff_check.cpp`
  里唯一负责解除电机 `spool-up block` 的逻辑，整个函数体被
  `#if HAL_WITH_ESC_TELEM` 包住，SITL 大概率没有这个宏（没有真实 ESC
  遥测硬件），导致 block 永远解不开、电机一直卡在地面怠速。**尚未
  确认**，只是排除法后最支持的假设，还没验证 SITL 编译时
  `HAL_WITH_ESC_TELEM` 的实际值，也没找到官方文档说明 SITL 该怎么绕开
  这个限制。

## 用法

| 想做什么 | 命令 |
|---|---|
| 轻量接口测试（不接 Gazebo） | `./scripts/run_sitl_demo.sh` |
| 完整仿真（接 Gazebo） | `./scripts/run_gazebo_demo.sh` |
| 看一次解锁/起飞演示 | `ros2 run cuadc_sim sitl_smoke_test`（前两者任一先跑起来之后，另开终端跑这条） |

首次在某个 `sitl_run_dir` 里跑，两个包装脚本后面都加
`extra_sitl_args:=-w` 清空 EEPROM。

两个包装脚本只是把该 source 的环境、该 export 的变量收起来（具体内容
见脚本文件自己的注释），换机器/路径不一样时对照脚本内容手动展开即可。
MAVProxy 默认的 `udp:127.0.0.1:14550` 留给 QGroundControl（装好直接
自动连），我们自己的脚本/节点走专属的 `udp:127.0.0.1:14551`。

`sitl_smoke_test` 不是本项目正式的接口层实现（见
[ADR-0001](../../docs/spec/adr/0001-interface-layer-separate-module.md)），
只是仿真环境的验证脚手架：连上 SITL 后读几条遥测，然后解锁、切
GUIDED、起飞到 10m、发一个位置目标点。
