# 真机投放区单点测试 runbook

> 操作手册（非设计文档）：真机第一次把「重构后的感知 + 投放机构 + 接近对准」拉到
> 场地上跑的分级流程。架构不在这里重复——底座/端点见
> [`onboard-runtime.md`](onboard-runtime.md)，感知取帧/多 sink 见
> [ADR-0008](adr/0008-perception-raw-frame-multi-sink.md)，投放执行见
> [ADR-0007](adr/0007-drop-servo-vs-relay.md)，配置分层见
> [`mission-config-layer.md`](mission-config-layer.md)。

## 0. 硬约束（每一级都成立）

- **L1/L2 不让飞机起飞**；L3 会自主起飞，**飞手 RC 全程待命、随时接管**。
- **绝不停/改 `dronelink-network`、`eth0`**——现场 SSH 骑在上面。
- Pi profile 用 **`none`**（或 `ground-monitor`），**不用 `mission` 自启**；测试节点
  全靠 SSH 手动 `ros2 launch`/`run`。
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

### L1 · 感知单点（不飞）
```
./scripts/run_field_l1_perception.sh
```
起接口层 + 真实 CV 感知，**不发飞行指令**。验证 shm 取帧→检测→反投影→尺寸判别：
- `ros2 topic echo /perception/detections` 看世界坐标/半径是否合理；
- 地面看 5606 的标注框（经 cv-forward）。
内参按 profile=real 从 `src/cuadc_perception/cuadc_perception/camera.real.yaml` 读
（960x540，来自 `camera_calibration.npz` 换算，见 ADR-0009）。

### L2 · 投放机构单点（不飞）
```
./scripts/run_field_l2_drop_actuator.sh
```
只起接口层，手动调服务验证舵机。**`release_payload` 未解锁会拒绝**（ADR-0007：
地面投放归遥控器/人工）。地面 bench 想用服务驱动舵机 → **必须先拆桨再解锁**：
```
ros2 service call /cuadc_interface/arm cuadc_interface_msgs/srv/Arm "{arm: true}"
ros2 service call /cuadc_interface/release_payload cuadc_interface_msgs/srv/ReleasePayload "{}"
```
或让机械子队直接用遥控器打投放通道验证——**绝不带桨在地面解锁**。

### L3 · 接近+对准单点（**会飞**，RC 待命）
```
./scripts/run_field_l3_approach.sh          # = run_mission_hardware.sh scenario:=drop_zone_bench
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
要么直接用 L1 感知节点自带的预览，不要两个一起抢 shm。

## 4. sim 回归（不受本次真机接线影响）

`frame_source` 默认 `ros`，`run_mission_sim.sh` 只传 `profile:=sim`，Gazebo→
`/camera/image_raw` 老路不变。真机接线全走 real 专属脚本，碰不到仿真路径。
