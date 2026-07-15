# 04. 接口层——ROS2 契约

> **本文档状态：草案，待用户签字确认**。标"待确认"的地方是设计草案，
> 不是拍板的决策；确认后再删掉这个提示，且在正式建包（写代码）之前
> 必须先确认——这是 [ADR-0001](adr/0001-interface-layer-separate-module.md)
> 明确要求的顺序（先有契约，规划/决策才能用桩测试，接口层实现才有
> 明确目标）。

## 范围与定位

接口层是**纯粹的 ROS2 ↔ MAVLink 翻译层**，只做协议转换，不包含任何
任务语义：

- 只提供不带语义的通用原语（"给一个坐标飞过去""设一个布尔量输出"），
  不提供"飞去投放区""执行侦察"这类任务级封装。
- 不追踪"是否到达目标点"之类的完成判定——规划/决策自己比对接口层
  报出的遥测位置来判断，接口层对此无感。
- "投放区在哪""什么时候切换任务状态"完全是规划/决策模块的逻辑，
  接口层不需要、也不应该知道这些概念。

这是 [ADR-0001](adr/0001-interface-layer-separate-module.md)"接口层
独立于规划/决策模块"的具体化，边界讨论见
[`working-notes.md`](working-notes.md)。

## 技术路线

自研，基于 `pymavlink`（不用 MAVROS 2）。原因见
[`working-notes.md`](working-notes.md) 2026-07-15 决策记录：两人团队
+ 外部时限下，`pymavlink` 已是现有依赖且 `cuadc_sim` 的
`sitl_smoke_test`/`flight_acceptance_test` 已验证过核心协议动作，
增量工作量小于新增 MAVROS 依赖的配置/学习成本。

## MVP 原则

只覆盖任务状态机（见 [`00-index.md`](00-index.md)）实际用到的能力，
不追求一次覆盖 [`ardupilot-interface-overview.md`](../reference/ardupilot-interface-overview.md)
里的全部飞控能力。功能不够用时再扩展这份契约，不预先设计。

**MVP 明确不做**（对照 `ardupilot-interface-overview.md` 划掉的部分，
理由见 `working-notes.md`）：Mission 整套（Upload/Start/Pause/Resume/
Clear）、Read/Write Parameter、Request Calibration、Reboot Autopilot、
Set Home Position、Air Speed（多旋翼不适用）。

## 与规划/决策的边界：谁管什么

| 关注点 | 归属 |
|---|---|
| 具体飞到哪个坐标、什么时候飞、飞完下一步做什么 | 规划/决策 |
| "到没到目标点"的判定 | 规划/决策（自己比对接口层报出的位置遥测） |
| 任务开关（触发 `WAIT_TASK_START`） | 伴随计算机上独立于接口层的 GPIO 驱动，不经过接口层/MAVLink |
| 物理安全开关 | 纯硬件直接切断动力，不经过任何软件 |
| 飞控硬 Failsafe（低电、失联、GPS 异常等） | 飞控固件自己处理 |
| 任务逻辑安全响应（如电量低于阈值提前 RTL） | 规划/决策，消费接口层转发的 Battery/Failsafe 遥测 |
| MAVLink 协议细节（编解码、连接管理、心跳） | 接口层，规划/决策不可见 |

## 契约总览：指令（规划/决策 → 接口层 → ArduPilot）

> 话题/服务名、消息类型均为暂定草案，实际实现时可能微调（比如合并/
> 拆分某个接口），不算独立决策。

| 能力 | ROS2 通信方式 | 名称（暂定） | 类型（暂定） | 说明 |
|---|---|---|---|---|
| Arm / Disarm | service | `~/arm` | 自定义 `Arm.srv`（bool arm → bool success, string message） | 一次性动作，需要明确的成功/失败反馈 |
| Set Mode | service | `~/set_mode` | 自定义 `SetMode.srv`（string mode → bool success） | 模式名用字符串（`GUIDED`/`RTL`/`LAND`/`LOITER`…），跟 ArduCopter 模式表直接对应，不建 ROS 枚举，避免和飞控模式表脱节 |
| Go To Location | service | `~/goto_location` | 自定义 `GotoLocation.srv`（lat/lon/alt → bool success） | 目标点式指令；是否需要持续发送见下方"待验证事项" |
| Set Position Target | topic | `~/cmd/position_target` | 自定义 `PositionTarget.msg`（局部坐标 x/y/z） | 精确悬停微调用（对准圆筒/降落点），规划/决策按需持续发布 |
| Set Velocity | topic | `~/cmd/velocity` | `geometry_msgs/TwistStamped` | 速度向量控制；发送频率下限见"待验证事项" |
| Set Yaw | topic | `~/cmd/yaw` | `std_msgs/Float64`（目标偏航角，弧度） | 摄像头对位/环视用；topic vs service 的取舍见"待验证事项" |
| 投放机构输出 | service | `~/set_relay` | 自定义 `SetRelay.srv`（uint8 channel, bool state → bool success） | 底层用 `MAV_CMD_DO_SET_RELAY`，布尔开关量（2026-07-15 已定，不用 PWM Servo） |

## 契约总览：遥测（ArduPilot → 接口层 → 规划/决策）

| 能力 | 话题名（暂定） | 类型（暂定） | 备注 |
|---|---|---|---|
| 心跳/连接状态 | `~/status/connected` | `std_msgs/Bool` | |
| Armed State | `~/status/armed` | `std_msgs/Bool` | |
| Current Mode | `~/status/mode` | `std_msgs/String` | |
| Global Position | `~/telemetry/global_position` | `sensor_msgs/NavSatFix` | |
| Local Position + Relative Altitude | `~/telemetry/local_position` | `geometry_msgs/PoseStamped` | |
| Velocity | `~/telemetry/velocity` | `geometry_msgs/TwistStamped` | |
| Attitude/Heading | `~/telemetry/attitude` | 自定义或复用 `sensor_msgs/Imu` 的姿态部分 | 待实现时定 |
| Battery | `~/telemetry/battery` | `sensor_msgs/BatteryState` | 任务逻辑安全响应（提前 RTL）依赖这个 |
| GPS Status | `~/telemetry/gps_status` | 自定义（fix_type, satellites_visible, hdop） | |
| EKF Status | `~/telemetry/ekf_status` | 自定义 | |
| Failsafe State | `~/status/failsafe` | 自定义（区分具体哪种 failsafe，不只是 bool） | 任务逻辑安全响应依赖这个 |
| 投放机构状态反馈 | `~/status/relay_state` | 自定义（uint8 channel, bool state） | 确认投放动作真的执行了 |
| Command ACK | `~/status/command_ack` | 自定义（string command, bool result） | |

## 与测试桩（stub）的关系

规划/决策做单元测试时，用一个假节点替代真实接口层——桩节点要实现的
就是上面两张表（同样的话题/服务名和类型），发布写死或脚本控制的遥测、
对指令返回简单的确认。桩不需要理解 MAVLink 或 ArduPilot 的任何细节，
只要符合这份契约即可（这是 [ADR-0001](adr/0001-interface-layer-separate-module.md)
"规划/决策可用桩替代真实/仿真飞控测试"的落地方式）。

## 待验证事项（实现阶段用 SITL 验证，不在这里下结论）

- **Go To Location 是否需要持续发送才能维持效果**：目前按"一次性
  指令，ArduPilot 收到后自主飞过去并悬停"设计（对应 service，而非
  topic）；如果实测发现 GUIDED 模式需要持续 setpoint 流才不会超时，
  需要改成 topic。
- **Set Velocity / Set Position Target 的发送频率下限**：ArduPilot
  GUIDED 模式对 setpoint 超时判定的具体阈值需要实测确认，规划/决策
  按这个频率持续发布。
- **Set Yaw 用 topic 还是 service**：如果只是"转到某个角度就不用管
  了"，one-shot service 更合适；如果需要连续追踪（比如摄像头持续
  扫描目标），topic 更合适——取决于感知/视觉模块的实际用法，待该
  模块设计时再对齐。

## 包名

暂定 `cuadc_interface`（沿用 `cuadc_sim` 的命名风格）。同
[ADR-0001](adr/0001-interface-layer-separate-module.md) 对
`mavros_bridge`（现已不适用，因为不用 MAVROS 了）的处理方式——先用
暂定名建包，用起来后再最终确认，不算独立决策。
