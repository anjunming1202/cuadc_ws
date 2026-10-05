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
  机载录制（飞前开、落地桨停后 Ctrl-C；视频 + 全部话题含 `fault_events`，落到 `~/flight_recordings/`）：
  `python3 src/cuadc_onboard/scripts/onboard_record.py --label <标签>`
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

> **所有会解锁/起飞的流程（arm 检查 / 起降 / 合集 / 投放单点 / 任务）同一套起飞条件，固定顺序**
> （[ADR-0011](adr/0011-pilot-arm-takeoff-point-frame.md)）：
> 1. 飞手：**模式开关拨到 STABILIZE 挡**，arm 开关在 disarm 挡；
> 2. 脚本起来后停在 `READY`（桨不转）。**确认现场安全后**，从另一个 SSH 发放行：
>    ```
>    ros2 topic pub -t 5 -r 2 /cuadc_flight_test/start std_msgs/msg/Empty "{}"      # 测试节点
>    ros2 topic pub -t 5 -r 2 /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"  # 任务/投放单点
>    ```
>    连发 5 次是必要的：`-1` 只发一条，订阅端还没完成连接就会丢（话题是 volatile，不补发），
>    节点就一直停在 READY（2026-10-04 手机热点下实测）。重复收到无副作用。
> 3. 日志出现 `WAITING FOR PILOT ARM` 后，**飞手用 RC 解锁**；Pi 看到解锁后 1s 内切 GUIDED 起飞。
>    **Pi 不发 arm**，arm 开关挡位始终和飞机状态一致，任何时刻拨一下就停桨。
> 4. 结束后（RTL/LAND 落地自动上锁）飞手把 arm 开关拨回 disarm 挡。RTL 落地后飞控停在 RTL 模式，
>    下次飞前拨一下模式开关回到 STABILIZE。
>
> 判定细节：放行之前就解锁的不算（要求放行后看到一次“未解锁→解锁”，否则提示先上锁）；解锁时
> 模式不是 STABILIZE 不起飞（开关在 ALT_HOLD 等挡会把 GUIDED 顶掉）。
>
> **飞手夺控**：Pi 接管后，飞手**拨模式开关**或 **RC 上锁**，Pi 立刻停发一切指令、不再切模式，打
> `ABORTED — mode changed to …` / `vehicle DISARMED in flight`（任务进 `PILOT_TAKEOVER`）。只有 Pi
> 自己的步骤出错、飞手没介入时，才兜底切 RTL（合集/任务）或 LAND（起降）；兜底也失败打 `PILOT TAKE OVER`。
>
> **坐标**：测试、任务、感知都用**起飞坐标系**——原点 = 解锁时飞控位置，x = 解锁时机头方向，y = 右，
> z = 下，与东南西北无关（定义见 [coordinate-frames.md](coordinate-frames.md)）。**飞机起飞前怎么摆，“前”就是哪
> 个方向**：任务时机头对准投放区。解锁时接口层日志打 `armed: takeoff frame frozen — origin at EKF NED
> (...), +x = nose heading ...deg`，那是被换算掉的飞控内部原点和航向，不为 0 正常。

### ARM + RC 接管检查（**每次飞行测试前必做**；比赛前给裁判演示 RC 夺控的必要环节）
```
./scripts/run_arm_check_real.sh          # 放行后飞手 RC 解锁 → 15s 观察窗口(每 1s 打倒计时)
```
流程和实际飞行一致。验证 ① Pi 能看到飞手解锁，并核对解锁时是否 STABILIZE 挡（不是会 WARN：
飞行会拒绝起飞）② **RC 能否 disarm/接管**（桨转前必须证明人能随时停下）。节点**只如实观察+汇报、
不判 PASS/FAIL**，按时间线打事件：
- 窗口内 `armed→false` → 「检测到 armed→false @T+Xs」：你拨了 RC 杀锁即 **RC disarm 有效**；
- `mode` 被切走（仍 armed）→ 「RC 成功接管 mode X→Y」；
- 窗口结束仍 armed → 等飞控自带 `DISARM_DELAY` 自动 disarm；宽限内没自动 disarm → Pi 兜底 disarm 并 WARN。

⚠ 解锁会让电机转（有桨则桨转）：**按队内 SOP 决定是否拆桨**——只验 RC 信号链路拆桨最安全，要给裁判
看电机真停再按其要求带桨并约束好。飞手 RC 全程在手。默认不等 EKF（室内无 GPS 也能做）。
**这一步没看到 RC 能 disarm/夺控，就不要往下做任何会飞的测试。**

### 起降 smoke（**会飞**，最简，RC 待命）
```
./scripts/run_flight_check_real.sh                          # 默认起飞 1m
./scripts/run_flight_check_real.sh --ros-args -p takeoff_altitude_m:=3.0
```
最短链路：（放行 + 飞手解锁后）起飞→悬停观察→降落→自动上锁，全程盯遥测。**飞行指令测试的
第一步**，先确认树莓派能指挥飞控起降、遥测如实报回，再上动作合集。脚本自起接口层连
fanout、跑完自动收尾。飞机只在起飞点正上方上下动。

### 飞行动作合集（**会飞**，RC 待命）
```
./scripts/run_flight_suite_real.sh                          # 全阶段
./scripts/run_flight_suite_real.sh --ros-args -p phases:=box # 只跑某阶段（电量/时间）
```
（放行 + 飞手解锁后）一次连续飞行走 起飞 3m→定 yaw 方框（3m，以起飞点为中心 ±2.5m，边沿起飞时的前/右方向，
机头不转）→
高度扫描（1→2→3→5m，回 3m）→速度扫描（3m，沿起飞时机头方向往返，0.25–2 m/s，每档停悬
settle）→RTL 降落。基准 3m 是因为 1m 地效下实机明显发飘。yaw 漂移、停发漂移超容差只记录
+告警、不中断；结尾有超差项打 `COMPLETED with N out-of-tolerance item(s)`，全达标才 `PASSED`。
⚠ **首飞前按场地核对水平余量**：方框 `box_leg_m`（默认 5m）、速度段单向
`vel_leg_distance_m`（默认 3.5m）都要在场地内，必要时 `-p` 调小；起飞点四周及上方净空。
⚠ **RTL 返航高度 `RTL_ALT_M`**：合集结尾、任务结尾和兜底都用 RTL，飞控会先爬到 `RTL_ALT_M` 再返航。
默认 15m；本项目定 5m（同 sim `mission_defaults.parm`），首飞前在 Mission Planner 核对（本固件单位是米）。
⚠ **速度参数**（同 sim `mission_defaults.parm`）：`WP_SPD` 3（搜索/粗对准，也是兜底）、`RTL_SPEED_MS` 5；
升降 `WP_SPD_UP`/`WP_SPD_DN` 保持飞控默认 2.5/1.5。转场 5 m/s、区内升降复写由任务按阶段下发，不靠参数。
⚠ **Pi 串口 `SERIAL5_BAUD` = 921**（TELEM3，与 `cuadc-onboard-fanout` 的 `--baud 921600` 一致；改后整机断电重启）。
sim 侧对应 `run_flight_check_sim.sh` / `run_flight_suite_sim.sh`（连 SITL，先在仿真验逻辑；sim
没有飞手，由节点代飞手切 STABILIZE+解锁）。

### 接近+对准单点（**会飞**，RC 待命）
```
./scripts/run_drop_zone_real.sh          # = run_mission_real.sh scenario:=drop_zone_bench
# 栈停在 WAIT_TASK_START，另一个终端放行（之后飞手 RC 解锁才起飞）：
ros2 topic pub -t 5 -r 2 /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"
```
投放区中心在起飞时机头前方 2m（`drop_zone_bench.real.yaml`），投 1 次、看到第一个可信筒就去投。
换末端算法（默认 `cone_descent_ibvs`）：追加 `approach:=position_lock_descent`（[ADR-0012](adr/0012-terminal-position-lock.md)）或 `approach_variant:=lowgain`；变体可逗号叠加（如 `approach_variant:=high,fastgate`，见各 `approaches/<approach>.<variant>.yaml`）；启动日志 `approach=... variant=...` 可核对。
现场看失效处理：日志里的 `[FAULT]` 行，或另开终端
`ros2 topic echo /cuadc_decision/fault_events`（代码表见 [failure-handling.md](failure-handling.md)）。
⚠ **首飞前核对 `mission.real.yaml`**：投放区中心/扫描航点是**场地坐标**（+x = 起飞时机头方向，+y = 右），
现按规则书填写，`takeoff_point` = 起降点圆心 `[0, 0]`。飞机放在起降点、**机头对准投放区**（摆偏 1° ≈ 32.5m 处
横偏 0.57m）；场地不同时按实测改区中心。

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
