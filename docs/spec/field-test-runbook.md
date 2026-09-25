# 真机现场测试 runbook

> 操作手册（非设计文档）：真机现场分级流程，覆盖三类测试——CV 检测（不飞）、
> 树莓派控制飞控的飞行指令测试（会飞）、投放区单点（接近对准/投放机构）。架构不在
> 这里重复——底座/端点见 [`onboard-runtime.md`](onboard-runtime.md)，感知取帧/多 sink
> 见 [ADR-0008](adr/0008-perception-raw-frame-multi-sink.md)，投放执行见
> [ADR-0007](adr/0007-drop-servo-vs-relay.md)，配置分层见
> [`mission-config-layer.md`](mission-config-layer.md)。
>
> 每个飞行测试节点跑完打一张 target/achieved/error 汇总表；这些节点已在 Gazebo+SITL
> 端到端验证（sim 验指令/时序逻辑，real 换连接目标量物理精度，同格式对比）。

## 0. 硬约束（每一级都成立）

- **CV 检测/投放机构单点不让飞机起飞**；飞行指令测试与接近对准会起飞，**飞手 RC
  全程待命、随时接管**。
- **绝不停/改 `dronelink-network`、`eth0`**——现场 SSH 骑在上面。
- Pi profile 用 **`none`**（或 `ground-monitor`），**不用 `mission` 自启**；测试节点
  全靠 SSH 手动 `ros2 launch`/`run`。
- **遥测/图传（`cuadc_groundstation`）+ recording（`onboard_record`）是解耦的独立组件**：
  飞行测试脚本只驱动飞行，不启停它们——飞前各自单独开好（地面站看实时位置/高度/速度、
  录制供回看），飞行脚本照跑。
- 推 master 到 origin 需**用户明确确认**（安全策略默认分支门）。

## 1. Pi 部署前置

底座在跑（`mav_fanout` 出 `udp:127.0.0.1:14550`、`camera_service` 出
`shm:/tmp/cuadc-cam-raw.shm`）。任务节点要能 import 重构后的 `cuadc_perception`：

- **首选**：把 Pi 的 `~/cuadc_ws` 理到含本工作的 master——`git fetch` 后
  `git reset --hard origin/<ref>`（Pi 现停在 cylinder-dataset 且脏；那处
  `dataset_core.py` 改动 origin 已有等价版），再 `colcon build`。
  ⚠ 需要 master 先推 origin（**用户确认后**才推）。
- **备选**（不想动 Pi 工作区）：干净部署 `cuadc_perception` 包到 `/opt`，把
  ground-monitor 的 cv-preview 单元 `PYTHONPATH` 指过去（沿用 cv_preview 免
  colcon 的老做法）。

**shm 单消费者**：`shmsrc` 一次只喂一个客户端。感知节点要吃 shm，就不能同时让
ground-monitor 的 `cuadc-onboard-cv-preview` 占着 shm——测前用 `profile=none`，
或 `systemctl stop cuadc-onboard-cv-preview` 让出 shm。感知节点自己带预览
（`preview_enabled=true` → 本地 5604），ground-monitor 的 `cv-forward` 照样把
5604 转 5606 到地面，不缺画面。

## 2. 分级流程（都在 Pi 上 SSH 手动跑；连接见记忆 pi_network_access）

### 感知单点（不飞）
```
./scripts/run_perception_real.sh
```
起接口层 + 真实 CV 感知，**不发飞行指令**。验证 shm 取帧→检测→反投影→尺寸判别：
- `ros2 topic echo /perception/detections` 看世界坐标/半径是否合理；
- 地面看 5606 的标注框（经 cv-forward）。
内参按 profile=real 从 `src/cuadc_perception/cuadc_perception/camera.real.yaml` 读
（960x540，来自 `camera_calibration.npz` 换算，见 ADR-0009）。

### 投放机构单点（不飞）
```
./scripts/run_drop_actuator_real.sh
```
只起接口层，手动调服务验证舵机。**`release_payload` 未解锁会拒绝**（ADR-0007：
地面投放归遥控器/人工）。地面 bench 想用服务驱动舵机 → **必须先拆桨再解锁**：
```
ros2 service call /cuadc_interface/arm cuadc_interface_msgs/srv/Arm "{arm: true}"
ros2 service call /cuadc_interface/release_payload cuadc_interface_msgs/srv/ReleasePayload "{}"
```
或让机械子队直接用遥控器打投放通道验证——**绝不带桨在地面解锁**。

> **arm+takeoff 放行闸门（两个飞行测试都有）**：脚本起来后节点停在 arm **之前**（桨不转），
> 打印 `READY — NOT armed` 并等放行。**确认现场安全后**，从另一个 SSH 发一次：
> ```
> ros2 topic pub -1 /cuadc_flight_test/start std_msgs/msg/Empty "{}"
> ```
> 收到才 arm+起飞。桨转前必须有人显式放行，不会一跑脚本就飞。

### 起降 smoke（**会飞**，最简，RC 待命）
```
./scripts/run_flight_check_real.sh                          # 起来后等放行(见上),默认起飞 5m
./scripts/run_flight_check_real.sh --ros-args -p takeoff_altitude_m:=3.0
```
最短链路：（放行后）解锁→起飞→悬停观察→降落→自动上锁，全程盯遥测。**飞行指令测试的
第一步**，先确认树莓派能指挥飞控起降、遥测如实报回，再上动作合集。脚本自起接口层连
fanout、跑完自动收尾。飞机只在起飞点正上方上下动。

### 飞行动作合集（**会飞**，RC 待命）
```
./scripts/run_flight_suite_real.sh                          # 起来后等放行(见上),全阶段
./scripts/run_flight_suite_real.sh --ros-args -p phases:=box # 只跑某阶段（电量/时间）
```
（放行后）一次连续飞行走 起飞→定 yaw 方框（全向平移、机头不转）→高度扫描（含 1m，贴近
地效谨慎）→速度扫描（0.25–2 m/s，每档停悬 settle）→RTL 降落，量实机指令/传感器精度。
⚠ **首飞前按场地核对水平余量**：方框边长 `box_leg_m`（默认 5m）、速度段单向
`vel_leg_distance_m`（默认 3.5m）都要在场地内，必要时 `-p` 调小；起飞点四周及上方净空。
sim 侧对应 `run_flight_check_sim.sh` / `run_flight_suite_sim.sh`（连 SITL，先在仿真验逻辑）。

### 接近+对准单点（**会飞**，RC 待命）
```
./scripts/run_drop_zone_real.sh          # = run_mission_real.sh scenario:=drop_zone_bench
# 栈停在 WAIT_TASK_START 后，另一个终端触发：
ros2 topic pub -1 /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"
```
⚠ **首飞前核对 `mission.real.yaml`**：投放区中心/扫描航点相对**解锁点** home（真机
home 跟解锁位置走，不像仿真固定原点）。当天按场地复测坐标/高度，确认航线安全。

## 3. 旁路验证（机上端到端，不起 ROS 栈）

无-ROS 预览直接读实时 shm → 出框，抓端口验证标注 H.264：
```
python3 -m cuadc_perception.cv_preview --port 5614 --profile real
```
注意 shm 单消费者（见 §1）：测时要么先停 `cuadc-onboard-cv-preview` 让出 shm，
要么直接用感知单点节点自带的预览，不要两个一起抢 shm。

## 4. sim 回归（不受本次真机接线影响）

`frame_source` 默认 `ros`，`run_mission_sim.sh` 只传 `profile:=sim`，Gazebo→
`/camera/image_raw` 老路不变。真机接线全走 real 专属脚本，碰不到仿真路径。
