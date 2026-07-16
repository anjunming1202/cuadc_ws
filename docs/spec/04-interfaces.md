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
| Takeoff | service | `~/takeoff` | 自定义 `Takeoff.srv`（float64 alt → bool success） | 底层 `MAV_CMD_NAV_TAKEOFF`。2026-07-15 SITL 实测确认：光发 `Go To Location`/`Set Velocity` 不会让停在地面的飞机爬升，ArduCopter GUIDED 模式必须先收到这个指令才会真正起飞——原契约漏了这一项，任务状态机的 `ARM_AND_TAKEOFF` 步骤离不开它。**`alt` 是相对起飞点的高度，不是海拔**（见下方"高度基准"） |
| Go To Location | service | `~/goto_location` | 自定义 `GotoLocation.srv`（lat/lon/alt → bool success） | 目标点式指令，假设已在空中（先 Takeoff）；2026-07-15 SITL 实测确认单次调用即可，不需要持续发送（见"待验证事项"）。**`alt` 是相对起飞点的高度，不是海拔**（见下方"高度基准"） |
| Set Position Target | topic | `~/cmd/position_target` | 自定义 `PositionTarget.msg`（局部坐标 x/y/z） | 精确悬停微调用（对准圆筒/降落点），规划/决策按需持续发布 |
| Set Velocity | topic | `~/cmd/velocity` | `geometry_msgs/TwistStamped` | 速度向量控制；2026-07-15 SITL 实测确认停止发送后飞机会安全悬停、不会持续漂移（见"待验证事项"） |
| Set Yaw | topic | `~/cmd/yaw` | `std_msgs/Float64`（目标偏航角，弧度） | 摄像头对位/环视用；topic vs service 的取舍见"待验证事项"。**只有飞机静止（没有并发的位置类指令）时才是独立可控的**，见下方"自动朝向行为"。接口层内部会把弧度转度并归一化到 `[0,360)` 再发送——ArduCopter 收到负数会直接拒绝该指令（2026-07-16 SITL 实测确认） |
| 投放机构输出 | service | `~/set_relay` | 自定义 `SetRelay.srv`（uint8 channel, bool state → bool success） | 底层用 `MAV_CMD_DO_SET_RELAY`，布尔开关量（2026-07-15 已定，不用 PWM Servo） |

## 契约总览：遥测（ArduPilot → 接口层 → 规划/决策）

| 能力 | 话题名（暂定） | 类型（暂定） | 备注 |
|---|---|---|---|
| 心跳/连接状态 | `~/status/connected` | `std_msgs/Bool` | |
| Armed State | `~/status/armed` | `std_msgs/Bool` | |
| Current Mode | `~/status/mode` | `std_msgs/String` | |
| Global Position | `~/telemetry/global_position` | `sensor_msgs/NavSatFix` | `altitude` 是**海拔**（MSL），不是相对起飞点高度，见下方"高度基准" |
| Local Position + Relative Altitude | `~/telemetry/local_position` | `geometry_msgs/PoseStamped` | `position.z` 是局部 NED 坐标，**相对起飞点**，且 z 轴向下为正——"离地高度"要用 `-position.z`，见下方"高度基准" |
| Velocity | `~/telemetry/velocity` | `geometry_msgs/TwistStamped` | |
| Attitude/Heading | `~/telemetry/attitude` | 自定义或复用 `sensor_msgs/Imu` 的姿态部分 | 待实现时定 |
| Battery | `~/telemetry/battery` | `sensor_msgs/BatteryState` | 任务逻辑安全响应（提前 RTL）依赖这个 |
| GPS Status | `~/telemetry/gps_status` | 自定义（fix_type, satellites_visible, hdop） | |
| EKF Status | `~/telemetry/ekf_status` | 自定义 | |
| Failsafe State | `~/status/failsafe` | 自定义（区分具体哪种 failsafe，不只是 bool） | 任务逻辑安全响应依赖这个 |
| 投放机构状态反馈 | `~/status/relay_state` | 自定义（uint8 channel, bool state） | 确认投放动作真的执行了 |
| Command ACK | `~/status/command_ack` | 自定义（string command, bool result） | |

## 高度基准：相对高度 vs 海拔，不能混用

**指令里的 `alt`（`Takeoff.alt`、`GotoLocation.alt`）都是相对起飞点的
高度**，底层用的是 `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`/
`MAV_CMD_NAV_TAKEOFF` 的相对高度语义。**但 `~/telemetry/global_position`
的 `altitude` 是海拔**（MSL，SITL 默认场地大约 584m）。这两个不是同一个
基准，直接相加/相减会算出错得离谱的数字。

2026-07-16 实测踩过这个坑：拿海拔（约 584）加 10 算出"目标高度"
（约 594），当成相对高度传给 `GotoLocation.alt`，飞控理解成"爬升到
离地 594m"，飞机全程持续爬升不停——不是 `interface_node.py` 的 bug，
是调用方把两种高度基准算混了。**结论：算相对高度目标时直接用小数字
（比如 10、20），不要拿 `global_position.altitude` 做加减；如果需要
"离地高度"这个量，用 `local_position` 的 `-position.z`，它本来就是
相对起飞点的，不用换算。**

## 自动朝向行为：位置类指令会自动改 yaw

ArduCopter `WP_YAW_BEHAVIOR` 参数默认是"面向飞行方向"，`Go To
Location`/`Set Position Target` 这些位置类指令都不带 yaw 信息，飞控
会自己接管朝向、转向飞行方向，跟发没发 `Set Yaw` 无关。2026-07-16
拍板：接受这个默认行为，不改飞控参数——`Set Yaw` 只在飞机静止（没有
并发的位置类指令）时才是真正独立可控的，需要飞行中独立朝向的场景不
在当前任务范围内。

## 与测试桩（stub）的关系

规划/决策做单元测试时，用一个假节点替代真实接口层——桩节点要实现的
就是上面两张表（同样的话题/服务名和类型），发布写死或脚本控制的遥测、
对指令返回简单的确认。桩不需要理解 MAVLink 或 ArduPilot 的任何细节，
只要符合这份契约即可（这是 [ADR-0001](adr/0001-interface-layer-separate-module.md)
"规划/决策可用桩替代真实/仿真飞控测试"的落地方式）。

## 待验证事项（实现阶段用 SITL 验证，不在这里下结论）

- ~~Go To Location 是否需要持续发送才能维持效果~~——**已用 SITL 实测
  确认（2026-07-15）**：起飞后单次调用 `goto_location`（目标点 50m
  外），飞机自主飞过去并稳定悬停在目标点，全程只发了一次，不需要
  持续重发。维持 service（一次性指令）这个设计，契约不用改。
- ~~Set Velocity 的发送频率下限~~——**已用 SITL 实测确认
  （2026-07-15）**：发 3 秒速度指令后停止发送，飞机没有按最后一次
  速度持续漂移，而是停下并稳定悬停——ArduPilot GUIDED 模式对速度
  setpoint 有内建的丢失保护，不会无限期沿用旧指令。规划/决策不需要
  为了"安全兜底"专门维护一个持续重发的看门狗逻辑，但仍建议按合理
  频率（比如几 Hz）持续发布，让控制更平滑，不是协议要求的安全刚需。
  精确的超时阈值这次没测（不是这次要回答的问题），需要更细的频率
  才知道。
- **Set Yaw 用 topic 还是 service**：如果只是"转到某个角度就不用管
  了"，one-shot service 更合适；如果需要连续追踪（比如摄像头持续
  扫描目标），topic 更合适——取决于感知/视觉模块的实际用法，待该
  模块设计时再对齐。仍未验证。

## 包名

暂定 `cuadc_interface`（沿用 `cuadc_sim` 的命名风格）。同
[ADR-0001](adr/0001-interface-layer-separate-module.md) 对
`mavros_bridge`（现已不适用，因为不用 MAVROS 了）的处理方式——先用
暂定名建包，用起来后再最终确认，不算独立决策。
