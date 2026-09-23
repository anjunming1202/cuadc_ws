# ADR-0007：投放机构改用 PWM 舵机（RC 直通 + 任务期 RC override），取代 relay 布尔量

> 状态：**提议中，待负责人 sign-off**。Supersedes 04-interfaces.md 中 2026-07-15
> 记录的「投放机构用 `MAV_CMD_DO_SET_RELAY` 布尔开关量、不用 PWM Servo」决定。

## 背景

投放机构由硬件子队负责，已**在真机上验证成功**：电脑经飞控数传/USB 发 MAVLink
指令驱动 SERVO9(=AUX1) 输出 PWM，带动投放舵机。交付物（脚本 + 部署文档）见
`docs/reference/drop-mechanism/`（原样归档，含读取时 SHA-256）。

验证过的链路（其文档「架构B」）：
```
电脑/树莓派 → MAVLink(数传/USB) → Pixhawk → SERVO9(AUX1) PWM → 投放舵机
```
- 指令：`MAV_CMD_DO_SET_SERVO`，servo=9，值是 **PWM 微秒**。
- 执行器是**连续旋转**舵机/齿轮电机：**1500µs=停**、**1100µs=一个方向**、1900µs=另一方向。
  一次投放 = 发 1100µs **持续 run_s(约 1.9–3.33s)**，再**必须回 1500µs 停住**。
- 飞控侧需：`SERVO9_FUNCTION=0`（未分配 PWM 输出，允许 `DO_SET_SERVO` 驱动）、
  `SERVO9_MIN/TRIM/MAX=1000/1500/2000`、`SERVO9_REVERSED=0`、`SERVO_GPIO_MASK=0`。

这与本仓库现状冲突：我们的投放原语是 **relay 布尔量**（决策 `_tick_drop_payload` →
`set_relay` 服务 → 接口层 `MAV_CMD_DO_SET_RELAY`），且 04-interfaces.md 明确记过
「不用 PWM Servo」。交付的真机硬件是 PWM 舵机——旧假设被实物推翻。

## 决策

投放通道**默认归遥控器**（飞控把该输出设为 RC 直通 `SERVO9_FUNCTION=RCIN9`）——除任务
的 `DROP_PAYLOAD` 态外，飞手全程掌控投放；**只有投放那一段**由树莓派经 RC override 短暂
接管。即负责人要求的「除 drop state 听树莓派，其它都听遥控器」。

1. **投放原语 = 任务期短暂 `RC_CHANNELS_OVERRIDE`**，压投放 RC 通道到投放 PWM 跑完再撤销，
   走硬件子队同一条飞控/舵机物理链路（仅飞控侧从 `DO_SET_SERVO` 改为 RC 直通 + override）。
2. **接口层拥有执行器机制**（唯一 MAVLink 属主）：新增语义化服务 `ReleasePayload`，内部完成
   整段 override 定时序列，**投放物理完成、且 override 已撤销/回中立后才返回 success**。
3. **决策层保持语义化**：`_tick_drop_payload` 改调 `ReleasePayload`（一次调用=一次投放），
   决策不认识 RC 通道/PWM/时长。
4. **移除 relay 投放路径**：删 `SetRelay.srv`/`~/set_relay`/`RelayState`/`relay_state`
   （已被实物推翻的死路径），04-interfaces.md 契约行同步更新。
5. **执行器参数属接口层**：投放 RC 通道号、forward/neutral µs、run_s、override 撤销策略、ack
   超时 = 接口节点 launch 参数（默认 = 交付验证值 9 / 1100 / 1500 / run_s）。sim 与 real 用
   同一套（SITL 支持 RC 直通 + override、`ch9out` 会动，无需物理舵机）；日后真机若需不同时长
   再拆档，现在不预造机关。
6. **只在投放态 override**：接口层可加防御性 armed/state 守卫，但根本保证来自"只有决策进
   `DROP_PAYLOAD` 才调此服务"——其余任何时刻我们零 override，遥控器独占。

## 执行模型（关键：飞机等投放完成 + 连续舵机必须停住）

舵机物理投放需要 run_s。`ReleasePayload` 服务处理如下（与现有 arm/takeoff 同样跑在 spin
线程、阻塞等 ACK 的模式一致——遥测在独立 `_io_thread` 上发布，不受影响）：
1. `RC override(drop_ch, forward_us)`，持续 run_s（投放物理动作期间）。
2. `RC override(drop_ch, neutral_us)` 短暂——**先把连续舵机停在中立**。
3. 撤销 override（该通道发 0/释放）→ 遥控器收回控制。
4. 上述完成才返回 `success=True`。

于是决策 `_dispatch_and_await` 在 `DROP_PAYLOAD` 态**一直等到 success**；该态**不发任何
运动指令 = 飞机在 GUIDED 原地悬停**，直到投放物理完成才转下一阶段（重搜/去侦察/RTL）。
既满足「舵机跑完飞机再走」，又不需要多线程（阻塞 spin 线程 run_s 期间遥测照发、投放态本就
不发运动指令，无副作用）。

**连续舵机的停**：撤销 override 后遥控器拨杆若不在中立，连续舵机会继续转——故第 2 步先主动
停在中立，并要求**飞手投放拨杆平时停在中立位**（1500µs）。这是本方案对飞手的约定。

## 契约变更（04-interfaces.md）

| 能力 | 通道 | 类型 | 底层 |
|---|---|---|---|
| ~~投放机构输出~~ | ~~`~/set_relay`~~ | ~~SetRelay~~ | ~~DO_SET_RELAY（2026-07-15，本 ADR 取代）~~ |
| 投放执行 | service `~/release_payload` | `ReleasePayload.srv`（空请求 → bool success, string message） | 投放 RC 通道 `RC_CHANNELS_OVERRIDE` 定时序列（override→forward run_s→neutral→撤销），投放完成才返回 |
| 投放状态反馈 | `~/status/drop` | `DropStatus`（bool actuated, 时间戳） | 确认投放序列已执行 |

飞控侧配置随之从硬件交付时的 `SERVO9_FUNCTION=0`（DO_SET_SERVO）改为 **`SERVO9_FUNCTION=RCIN9`
（RC 直通）**，其余（MIN/TRIM/MAX/REVERSED）不变。这是本 ADR 需硬件子队协调 + 复验的一项。

## 影响 / 已知限制（沿用硬件文档 §14/§15，不夸大能力）

- **ACK≠载荷已脱离**：MAVLink ACK 只表示飞控接了指令；无称重/角度闭环、MAVLink 路径无
  HOME 微动开关反馈。投放是否成功脱离由机组图传/赛后确认，本层只保证「按验证时序驱动了
  舵机并回中立」。
- **安全**：软件 1500µs 回中立不能替代物理臂展开关/急停（硬件文档 §15）。真机测试按其
  §12 分级（先台架空载 → 挂载 → 整机），命令行打通 → 地面测通 → 真机单点，逐级推进。
- `home()`/HOME 微动开关是硬件文档「架构A」(Pi 直连 GPIO) 的台架标定手段，**不在**本次
  飞控链路集成范围。

## 落地清单（sign-off 后）

1. `cuadc_interface_msgs`：加 `ReleasePayload.srv` + `DropStatus.msg`；删 `SetRelay.srv`
   （及 CMakeLists 注册）；相应 `RelayState` 若仅投放用则一并删。
2. `cuadc_interface`：加 `~/release_payload` 服务（RC override 定时序列：override→forward
   run_s→neutral→撤销）+ `~/status/drop` 发布；删 `~/set_relay`/`relay_state`；drop 参数
   （RC 通道/forward/neutral µs/run_s/ack 超时）走 launch。acceptance/precision 测试改用新服务。
3. `cuadc_decision`：`_tick_drop_payload` 改调 `release_payload`；`mission/` 层移除
   `drop_relay_channel`。
4. `docs/reference/drop-mechanism/`：归档硬件子队 3 份脚本 + 部署文档（原样，附 SHA）。
5. `docs/spec/04-interfaces.md`：按上表更新契约行。
6. **验证**：SITL/Gazebo 端到端（`run_search_stress.sh` 全任务，确认投放态改走舵机后仍
   计到 2 次投放、飞机在投放期间悬停 run_s）；接口 acceptance 测试对新服务过。
