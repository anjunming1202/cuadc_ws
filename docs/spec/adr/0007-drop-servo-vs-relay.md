# ADR-0007：投放机构改用 PWM 舵机（`DO_SET_SERVO` 直接驱动），取代 relay 布尔量

> 状态：**已采纳（2026-10-04，与硬件子队对齐）**。Supersedes 04-interfaces.md 中
> 2026-07-15 记录的「投放机构用 `MAV_CMD_DO_SET_RELAY` 布尔开关量、不用 PWM Servo」
> 决定。初版（提议中）曾采用「RC 直通 + 任务期 RC override」，2026-10-03 台架实测
> 后否决，见文末。

## 背景

投放机构由硬件子队负责，已**在真机上验证成功**：电脑经飞控数传/USB 发 MAVLink
指令驱动 SERVO9(=AUX1) 输出 PWM，带动投放舵机。交付物（脚本 + 部署文档）见
`docs/reference/drop-mechanism/`（原样归档，含读取时 SHA-256）；2026-10-03 外场
台架复验脚本见 `local/flight_ctrl_tests/bench_drop.py`。

验证过的链路（其文档「架构B」）：
```
电脑/树莓派 → MAVLink(数传/USB) → Pixhawk → SERVO9(AUX1) PWM → 投放舵机
```
- 指令：`MAV_CMD_DO_SET_SERVO`，servo=9，值是 **PWM 微秒**，有 COMMAND_ACK。
- 执行器是**连续旋转**舵机/齿轮电机：**1500µs=停**、**1100µs=一个方向**、1900µs=另一方向。
  一次投放 = 发 1100µs **持续 run_s(约 1.9–3.33s)**，再**必须回 1500µs 停住**。
- 飞控侧需：`SERVO9_FUNCTION=0`（未分配 PWM 输出，允许 `DO_SET_SERVO` 驱动）、
  `SERVO9_MIN/TRIM/MAX=1000/1500/2000`、`SERVO9_REVERSED=0`、`SERVO_GPIO_MASK=0`。

这与 2026-07 的仓库现状冲突：当时投放原语是 **relay 布尔量**（决策 `_tick_drop_payload`
→ `set_relay` 服务 → 接口层 `MAV_CMD_DO_SET_RELAY`）。交付的真机硬件是 PWM 舵机——
旧假设被实物推翻。

## 决策

1. **投放原语 = `MAV_CMD_DO_SET_SERVO` 定时序列**，直接驱动投放舵机输出口，走硬件子队
   台架验证的同一条飞控/舵机物理链路、同一套飞控参数（`SERVO9_FUNCTION=0`）。
2. **接口层拥有执行器机制**（唯一 MAVLink 属主）：语义化服务 `ReleasePayload` 内部完成
   整段定时序列，**投放物理完成、舵机已回中立后才返回 success**。
3. **决策层保持语义化**：`_tick_drop_payload` 调 `ReleasePayload`（一次调用=一次投放），
   决策不认识舵机编号/PWM/时长。
4. **移除 relay 投放路径**：删 `SetRelay.srv`/`~/set_relay`/`RelayState`/`relay_state`
   （已被实物推翻的死路径），04-interfaces.md 契约行同步更新。
5. **执行器参数属接口层**：舵机输出口编号、forward/neutral µs、run_s、ack 超时 = 接口
   节点 launch 参数（默认 = 台架验证值 9 / 1100 / 1500 / run_s）。sim 与 real 用同一套
   （SITL 支持 `DO_SET_SERVO`、`ch9out` 会动，无需物理舵机）。
6. **armed 守卫**：未解锁不由软件驱动投放；台架测试经 `allow_disarmed_drop` 参数显式
   放行（飞行前关掉）。

## 执行模型（关键：飞机等投放完成 + 连续舵机必须停住）

舵机物理投放需要 run_s。`ReleasePayload` 服务处理如下（与现有 arm/takeoff 同样跑在 spin
线程、阻塞等 ACK 的模式一致——遥测在独立 `_io_thread` 上发布，不受影响）：
1. `DO_SET_SERVO(drop_servo, forward_us)`，等 ACK；未确认则补发一次停并返回失败。
2. 等 run_s（投放物理动作期间）。
3. `DO_SET_SERVO(drop_servo, neutral_us)`——**把连续舵机停在中立**。`DO_SET_SERVO` 无
   超时回落，停不住比投放失败更危险：未确认就再补发一次，并如实返回失败。
4. 上述完成才返回 `success=True`。

于是决策 `_dispatch_and_await` 在 `DROP_PAYLOAD` 态**一直等到 success**；该态**不发任何
运动指令 = 飞机在 GUIDED 原地悬停**，直到投放物理完成才转下一阶段（重搜/去侦察/RTL）。

## 契约变更（04-interfaces.md）

| 能力 | 通道 | 类型 | 底层 |
|---|---|---|---|
| ~~投放机构输出~~ | ~~`~/set_relay`~~ | ~~SetRelay~~ | ~~DO_SET_RELAY（2026-07-15，本 ADR 取代）~~ |
| 投放执行 | service `~/release_payload` | `ReleasePayload.srv`（空请求 → bool success, string message；2026-10-06 起带 `port`，见修订） | `DO_SET_SERVO` 定时序列（forward run_s→neutral，每步等 ACK），投放完成才返回 |
| 投放状态反馈 | `~/status/drop` | `DropStatus`（bool actuated, 时间戳） | 确认投放序列已执行 |

## 修订：两个单向舵机按 PWM 选口（2026-10-06，硬件子队确认）

SERVO9 上接的不是一个双向舵机，而是左右两个**单向**舵机：**1100µs 驱动左口、1900µs 驱动右口、
1500µs 两个都停**。原先「1100 = 放料方向」实际一直只放左口。

- `ReleasePayload` 请求带口号 `port`（0 左 / 1 右）；接口层按口号查 PWM（`cuadc_physical/airframe.yaml`
  每个 `drop_ports` 项的 `release_us`），口号越界拒绝；`DropStatus` 带回口号。决策仍不认识 PWM。
- 本投用哪个口只在决策 `MissionContext.drop_port` 算一次（从 `first_drop_port` 起轮换），瞄准杠杆臂和放料
  共用；放料失败重试仍是同一口。
- 仿真按 `status/drop` 的口号脱钩对应水瓶（原按进 DROP_PAYLOAD 的次序）。

## 被否决：RC 直通 + 任务期 RC override（2026-10-03 台架推翻）

初版决策曾按负责人「除 drop state 听树莓派,其它都听遥控器」的要求，把飞控设
`SERVO9_FUNCTION=RCIN9`（RC 直通、飞手拨杆可手动投放），任务期由接口层发
`RC_CHANNELS_OVERRIDE` 短暂压投放通道。台架实测否决，理由：
- **RC override 到不了 CH9**：`RC_CHANNELS_OVERRIDE` 在 MAVLink1 只有 8 个通道字段，
  9–18 通道是 MAVLink2 扩展；真机数传链路不保证端到端 MAVLink2。
- 需要硬件子队把 `SERVO9_FUNCTION` 从 0 改到 RCIN9 并整条链路复验；而 `=0` +
  `DO_SET_SERVO` 已台架验证通过且每步有 COMMAND_ACK 可查。

**代价（经确认接受）**：`SERVO9_FUNCTION=0` 下飞手拨杆不再直通投放舵机，遥控器失去
手动投放兜底；地面/应急投放改为软件发 `DO_SET_SERVO`（`bench_drop.py` 或
`release_payload` + `allow_disarmed_drop`）。

## 影响 / 已知限制（沿用硬件文档 §14/§15，不夸大能力）

- **ACK≠载荷已脱离**：COMMAND_ACK 只表示飞控接了指令；无称重/角度闭环、MAVLink 路径无
  HOME 微动开关反馈。投放是否成功脱离由机组图传/赛后确认，本层只保证「按验证时序驱动了
  舵机并回中立」。
- **安全**：软件 1500µs 回中立不能替代物理臂展开关/急停（硬件文档 §15）。真机测试按其
  §12 分级（先台架空载 → 挂载 → 整机），命令行打通 → 地面测通 → 真机单点，逐级推进。
- `home()`/HOME 微动开关是硬件文档「架构A」(Pi 直连 GPIO) 的台架标定手段，**不在**本次
  飞控链路集成范围。

## 验证

- SITL/Gazebo 端到端（`run_search_stress.sh` 全任务）：投放态仍计到 2 次投放、飞机在投放
  期间悬停 run_s；接口 acceptance 测试对 `release_payload` 过。
- 真机：2026-10-03 台架已验证 `DO_SET_SERVO` 链路本体（`bench_drop.py`，ack=0）；
  `release_payload` 服务走同链路的台架复验 + 真实 `drop_run_s` 标定仍待做。
