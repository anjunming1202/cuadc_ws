# cuadc_sim

仿真资产：比赛场地世界文件、投放区/灾情侦察区的圆筒与危险化学品标识
模型、spawn 和话题桥接的 launch 文件（后两者目前还没写，见下方"现状"）。
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
- **`ardupilot_gz`**（ROS2 集成层，规划中，`~/gazebo_ros2_ws`）
- **QGroundControl**（AppImage，`~/QGroundControl.AppImage`）

各外部依赖具体装在哪个目录，见
[`docs/spec/working-notes.md`](../../docs/spec/working-notes.md)"已经
决定的事"一节。选型依据（为什么后来决定接 Gazebo）也在同一份文件里。

## 现状

- SITL 起飞验证闭环（`launch/sitl_bringup.launch.py` + `sitl_smoke_test`
  节点），验证"能起 SITL、遥测能读、GUIDED 指令能发出去"，同时接了
  QGroundControl（MAVProxy 双路输出，见下方"用法"）。
- Gazebo Harmonic + `ardupilot_gazebo` 已验证：SITL 能通过 JSON 物理
  接口连上 Gazebo、用 Gazebo 算出来的物理数据飞（而不是 SITL 自己内置
  的简化模型），目前用的是官方示例场景/机体（`iris_runway.sdf` +
  `iris_with_gimbal`），不是我们自己的场地/机体。
- 还没有：ROS2 集成层（`ardupilot_gz`）、我们自己的比赛场地世界文件、
  圆筒/危险化学品标识模型、摄像头画面桥接到 ROS2。

## 用法

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# 环境限定的 workaround，见 docs/lessons.md 2026-07-09 条目：
export AMENT_PREFIX_PATH="$(pwd)/install/cuadc_sim:$AMENT_PREFIX_PATH"

# 第一次在某个 sitl_run_dir 里跑，需要 -w 清空 EEPROM：
ros2 launch cuadc_sim sitl_bringup.launch.py extra_sitl_args:=-w

# 之后同一个 sitl_run_dir 不需要再传 -w：
ros2 launch cuadc_sim sitl_bringup.launch.py
```

`sitl_smoke_test` 节点连接 SITL 的 MAVLink 端口后：读几条遥测（GPS 位置、
电池电压），然后解锁、切 GUIDED、起飞到 10m、发一个 GUIDED 位置目标点，
证明"遥测能读到、GUIDED 指令能发出去"这条闭环是通的。这不是本项目正式
的接口层实现（见 [ADR-0001](../../docs/spec/adr/0001-interface-layer-separate-module.md)），
只是仿真环境的验证脚手架。

launch 文件用正常的 MAVProxy 模式启动 SITL（不是 `--no-mavproxy`），
这样能同时接多个客户端：MAVProxy 默认的 `udp:127.0.0.1:14550` 留给
QGroundControl（装好直接自动连），`sitl_smoke_test` 节点走专属的
`udp:127.0.0.1:14551`，两边不用抢同一个连接。
