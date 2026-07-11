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

## 已解决问题

- **GUIDED 起飞指令被接受，但电机从不真正爬升，几十秒后自动上锁**（已
  修复）：根因已确认——SITL 默认编译 `HAL_WITH_ESC_TELEM` 为假（没有
  CAN/串口 RCOut），导致 `ArduCopter/takeoff_check.cpp` 里唯一负责解除
  电机 `spool-up block` 的函数整个函数体被编译掉，block 永远解不开。
  两个 launch 文件现在都在 `sim_vehicle.py` 命令行里固定加了
  `--configure-define=HAL_WITH_ESC_TELEM=1`（具体原因见两个 launch
  文件里对应的行内注释），已验证起飞后飞机能在约 8 秒内爬升到目标
  高度并稳定悬停，不再自动上锁。首次生效需要 SITL 完整重新编译（waf
  define 变了会触发 clean rebuild），后续正常增量编译。

## 用法

| 想做什么 | 命令 |
|---|---|
| 轻量接口测试（不接 Gazebo） | `./scripts/run_sitl_demo.sh` |
| 完整仿真（接 Gazebo） | `./scripts/run_gazebo_demo.sh` |
| 激活测试节点终端的环境（每个终端一次） | `source scripts/activate_env.sh`（注意是 `source`，不是 `./`，否则变量出了这条命令就没了；前两者任一先跑起来之后，另开一个终端跑这条） |
| 验证 MAVLink 通信本身（协议层） | `ros2 run cuadc_sim sitl_smoke_test`（同一个已激活的终端里跑，不用重新 source） |
| 验证真的能起飞/悬停/降落（物理层） | `ros2 run cuadc_sim flight_acceptance_test`（同上） |

两个测试都假设起始状态是飞机在地面、已解锁、prearm 检查通过（也就是
QGroundControl 里显示 `Ready to Fly` 的那个状态）——如果之前手动在
QGC 里解锁/起飞过，飞机还在空中，测试会因为状态不符（比如再收到一次
`NAV_TAKEOFF` 会被飞控拒绝）而失败；先在 QGC 里降落、上锁，回到
`Ready to Fly` 再跑。

首次在某个 `sitl_run_dir` 里跑，两个包装脚本后面都加
`extra_sitl_args:=-w` 清空 EEPROM。

`scripts/activate_env.sh` 是三个入口（`sitl_bringup`、`gazebo_bringup`、
测试节点）共同需要的环境交集（ROS Humble + 本工作空间 `install` +
`AMENT_PREFIX_PATH` workaround）。两个 `run_*_demo.sh` 都靠 `source` 它
拿到这部分，`run_gazebo_demo.sh` 再自己加 Gazebo 专属的两行（source
`gazebo_ros2_ws` + `GZ_VERSION`）；两者本身都用 `exec` 收尾，所以运行
仿真的终端和运行测试节点的终端天然是两个独立进程，不会互相污染环境。
测试节点特意只依赖这份交集、不碰 Gazebo 专属变量，这也是它们协议层/
物理层测试都和物理后端无关（SITL、Gazebo 都能测）这条设计的自然结果。
环境变量是按 shell 进程独立的，换一个终端要重新 `source` 一次，但同一
个终端里 `source` 一次之后可以连续跑任意多条 `ros2 run`/`ros2 topic`
之类的命令，不用每条前面都重新 `source`。换机器/路径不一样时对照脚本
文件自己的注释手动展开即可。MAVProxy 默认的 `udp:127.0.0.1:14550` 留
给 QGroundControl（装好直接自动连），我们自己的脚本/节点走专属的
`udp:127.0.0.1:14551`。

`sitl_smoke_test` 和 `flight_acceptance_test` 都不是本项目正式的接口层
实现（见
[ADR-0001](../../docs/spec/adr/0001-interface-layer-separate-module.md)），
只是仿真环境的验证脚手架，且分工不同：

- **`sitl_smoke_test`**（协议层）：连上后读几条遥测、解锁、切 GUIDED、
  发起飞指令、发一个位置目标点——只确认这些指令有没有被飞控接受
  （`COMMAND_ACK` 是不是 `ACCEPTED`），指令发完就退出，不管后续物理
  世界有没有真的反应。
- **`flight_acceptance_test`**（物理层）：额外断言起飞后飞机真的爬升到
  目标高度（容差内）、能稳定悬停、发 `LAND` 后能正常下降并在地面上锁
  （而不是空中失控上锁）——任何一步没达成都会抛异常、非零退出。这个
  正是能抓出"指令被接受但电机没有真实推力"这类问题的测试（见上面
  "已解决问题"里 `HAL_WITH_ESC_TELEM` 那个 bug）。

两者都只跟 MAVLink 通信打交道，不关心底下是哪种物理后端——`connection`
参数默认指向 `udp:127.0.0.1:14551`，两个 bringup launch 文件都把这个
端口留给自己的脚本用，所以不用改代码，指向 `sitl_bringup`（快速回归）
还是 `gazebo_bringup`（换 world/model 后的最终验收）都能直接跑。
