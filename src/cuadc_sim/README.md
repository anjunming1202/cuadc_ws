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

选型依据（为什么现阶段只用 SITL、不接 Gazebo）见
[`docs/spec/working-notes.md`](../../docs/spec/working-notes.md) 开放
问题 2。

## 现状

只有 SITL 起飞验证闭环（`launch/sitl_bringup.launch.py` +
`sitl_smoke_test` 节点），验证"能起 SITL、遥测能读、GUIDED 指令能发出去"。
还没有：场地世界文件、圆筒/标识模型、Gazebo 桥接（这些要接 Gazebo 视觉
仿真时才需要）。

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

默认连接串是 `tcp:127.0.0.1:5760`——因为 launch 文件用
`--no-mavproxy` 启动 SITL，这种模式下没有 MAVProxy 转发到常见的
`udp:14550`，SITL 直接把 MAVLink 主端口开在 `5760 + 10*instance` 的
TCP 端口上。
