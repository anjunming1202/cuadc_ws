# cuadc_sim

仿真资产：比赛场地世界文件（`field/`，config 驱动生成）、投放区/灾情
侦察区的圆筒与危险化学品标识模型、spawn 和话题桥接的 launch 文件。
仓库范围/仿真资产归属的决策见
[`docs/spec/working-notes.md`](../../docs/spec/working-notes.md)。场地
的几何设计依据见
[`docs/spec/field-layout.md`](../../docs/spec/field-layout.md)。

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

## Launch 文件

`gazebo_bringup.launch.py`**只负责搭建仿真世界本身**（Gazebo 物理 + SITL 经 JSON 协议 + 相机桥到
`/camera/image_raw`），不集成任何任务/验证节点——"搭仿真世界"和"在这个世界里做什么"永远是两条
独立的命令。默认加载比赛场地 `worlds/cuadc_field.sdf`（`world` 参数可换，如投放区单练世界）。

世界默认带真机实测的扰动（风/阵风/偏航扭矩、传感器噪声、相机柔化/暗角、图像延迟），
`imperfections:=false` 全关；清单与出处见
[`sim-stress-testbed.md`](../../docs/spec/sim-stress-testbed.md) §3.2。

没有使用 `ardupilot_gz` 官方自带的 launch 文件（如 `iris_runway.launch.py`）——那一套在本机环境下
必现崩溃，是上游一个未解决的已知 bug，具体机制和排查过程见 `docs/lessons.md`。

## 场地资产（field/）

`config.py` 是场地每个尺寸的唯一权威来源（跟
[`docs/spec/field-layout.md`](../../docs/spec/field-layout.md) 里的
设计对齐），`gen_world.py`/`gen_textures.py`/`gen_tube_mesh.py` 都从它
生成，不手抄数字；投放/侦察圆筒是程序生成的空心筒网格（不是实心圆
盘；投放筒有底，侦察筒无底、标识卡片放在地面上从筒口可见）；危险化学品标识是从原始 PDF 里逐像素提取的贴图；地面标线是真实
薄方块几何体；`check_dimensions.py` 能直接校验生成出来的世界跟规则
文档是否一致，不用靠肉眼在 GUI 里比对。已支持双场地并行布置（规则第
13 节），场地 A/B 用独立命名的对象，共享一块加宽的地面。`camera_view.py`
提供预置机位（overview/field/landing/drop/recon/flyover）方便调试时
快速切视角。

## 用法

| 想做什么 | 命令 |
|---|---|
| 完整场地仿真世界 | `./scripts/run_gazebo_demo.sh` |
| 投放区单练世界（出生在投放区前） | `./scripts/run_gazebo_drop_zone_bench.sh` |
| 侦察区单练世界（出生在侦察区前） | `./scripts/run_gazebo_recon_zone_bench.sh` |
| 在世界里跑任务 / 飞行测试 | 另一个终端：`run_mission_sim*.sh`、`run_flight_check_sim.sh`、`run_flight_suite_sim.sh`、`run_arm_check_sim.sh`（各脚本头部有说明） |

首次在某个 `sitl_run_dir` 里跑，后面加 `extra_sitl_args:=-w` 清空 EEPROM。

`scripts/activate_env.sh` 是仿真世界和任务/测试节点共同需要的环境（ROS + 本工作空间 `install` +
`AMENT_PREFIX_PATH` workaround），要 `source`（不是 `./`）；`run_gazebo_*.sh` 再自己加 Gazebo
专属的两行（source `gazebo_ros2_ws` + `GZ_VERSION`）。任务/测试经 MAVProxy 的
`udp:127.0.0.1:14550` 连 SITL，`14551` 是第二路输出（地面站/临时工具）。

改了 `config/`、`worlds/`、`models/` 下的文件要重新 `colcon build --packages-select cuadc_sim`：
symlink 安装指向的是 `build/` 里的副本，不是 `src/`。
