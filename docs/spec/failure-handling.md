# 失效处理

任务中所有失效 / 异常处理路径的**唯一清单**。状态机主线见 [06-planning.md](06-planning.md)；
这里只列"出问题时怎么办"。改任何一条失效逻辑都要同步改这里。

## 原则

- **任何等待都有时限**，到时显式转移，不靠"总超时兜底"。
- **投放按目标处理**：一个目标确认不了 / 对不上就**拉黑**（`rejected_positions`），换已知的下一个
  候选，没有才回 SEARCH；拉黑和已投的位置都不再被选中、也不被宽门改锁。
- **每次失效处理都留痕**：决策节点打一条
  `[FAULT] <代码> state=<状态>[ -> <去向>] <原因>` 的 WARN，同时发到
  `/cuadc_decision/fault_events`（`std_msgs/String`，JSON：`stamp, code, state, next_state, reason,
  drop_count`）。机载录制 `-a` 自动录进 rosbag，地面站可订阅。
- Pi 或 Pi↔飞控链路本身失效时 Pi 无能为力，**靠飞手 RC 接管**（ADR-0011 的夺控流程）。

阈值：任务 FSM 的在 `cuadc_decision/fsm_tuning.{sim,real}.yaml`，接近阶段的在
`cuadc_decision/approaches/cone_descent_ibvs.{sim,real}.yaml`。

## 跨状态守卫（`mission_logic._check_guards`，每 tick 先查，按序）

| 代码 | 条件 | 转移 |
|---|---|---|
| `PILOT_TAKEOVER` | Pi 接管后被上锁（非 RTL 中），或模式变成 GUIDED/RTL 以外 | `PILOT_TAKEOVER`（终态，Pi 停发一切指令） |
| `BATTERY_CRITICAL` | 电量 < `battery_critical_fraction`（0.20） | `RTL_OR_LAND` |
| `FC_FAILSAFE` | 接口层报 failsafe | `RTL_OR_LAND` |
| `DROP_PHASE_TIMEOUT` | 投放阶段（从到达投放区算，两投合计）> `drop_phase_timeout_s`（90s） | 有侦察阶段去侦察，否则 RTL |
| `RECON_PHASE_TIMEOUT` | 侦察扫描 > `recon_phase_timeout_s`（90s） | `RTL_OR_LAND` |

电量守卫要求真机飞控电池监测已配置，否则读数为空、该守卫不生效。

## 各状态

| 状态 | 代码 | 条件 | 处理 |
|---|---|---|---|
| `TAKEOFF` | `TAKEOFF_ABORTED` | 离地前飞手上锁 / 改模式 | 回 `WAIT_PILOT_ARM`（放行保持有效） |
| `TAKEOFF` | `TAKEOFF_TIMEOUT` | 起飞指令被接受后 `takeoff_timeout_s`（20s）仍没到巡航高度 | `RTL_OR_LAND` |
| `NAVIGATE_TO_*` | `NAV_TIMEOUT` | 超过 2×距离/转场速度 + `nav_timeout_margin_s`（10s）仍没进首航点 0.5m | 直接进 SEARCH / SURVEY |
| `SEARCH_DROP_TARGETS` | `SEARCH_EMPTY_RESCAN` | 扫完一整轮仍没有可选目标（没看到，或全部已投 / 已拉黑） | 扫描归零重扫，最多 `search_rescans`（1）轮 |
| `SEARCH_DROP_TARGETS` | `SEARCH_EMPTY_GIVE_UP` | 重扫用完仍没有 | 放弃投放阶段：有侦察去侦察，否则 RTL |
| `APPROACH_DROP_TARGET` | 见下节 | 接近单元放弃本目标 | 拉黑 → 已知下一个候选直接粗对准；没有 → SEARCH |
| `DROP_PAYLOAD` | `RELEASE_FAILED` | `release_payload` 返回失败 | 重发 |
| `DROP_PAYLOAD` | `RELEASE_GAVE_UP` | 失败超过 `release_max_retries`（2）次 | 记为已投（算次数、该位置不再选），继续 |
| `RTL_OR_LAND` | — | 切 RTL 失败 | 一直重试（不降级 LAND） |

**投完一个之后**（含 `RELEASE_GAVE_UP`）：还要投就先用第一轮 SEARCH 的聚类选下一个
（按 `drop_strategy`，排除已投 / 拉黑），**直接粗对准**（3m 飞过去）；没有可选候选才回 SEARCH
从第一个航点重扫。目标位置不准由粗段的宽门改锁和确认超时兜住。

## 接近阶段（`cone_descent_ibvs`）

内部恢复（不离开 APPROACH，不算放弃）：

| 情况 | 处理 |
|---|---|
| 末端短暂丢检测（≤ `max_coast_ticks`） | coast：保持上一测量，不降高 |
| 末端持续丢失 | climb：原地以 `climb_rate_mps` 爬升重捕获；爬到 `reacquire_ceiling_m` 仍没见 → 退回粗段 |
| 已收敛、投放高度附近丢失 | endgame：有界盲收口投放 |
| 粗段到点、门内无检测但 `relock_radius_m`（1.5m）内持续 `relock_ticks` 次看到别的筒 | 改锁过去，记 `RELOCK_WIDE` |
| 到投放高度 `stable_timeout_s`（15s）仍没过投放门（4cm） | 门放宽到 `relaxed_drop_tol_m`（8cm），记 `DROP_GATE_RELAXED`（偏一点投出去比投不出去好：筒外 0.5m 内仍算 B 区） |

放弃本目标（→ 任务 FSM 拉黑）：

| 代码 | 条件 |
|---|---|
| `TARGET_UNCONFIRMED` | 粗段水平到点后 `coarse_confirm_timeout_s`（8s）仍没交接末端（改锁后重新计时） |
| `TARGET_LOST_REPEATEDLY` | 末端退回粗段超过 `max_reacquire`（3）次 |
| `DROP_GATE_TIMEOUT` | 放宽后再 `relaxed_timeout_s`（10s）仍没过门 |

备用的 `stepped_descent_ibvs` 没有放弃路径（切换过去前要补）。

## 只记录、不改任务逻辑

| 代码 | 条件 |
|---|---|
| `PERCEPTION_STALE` | SEARCH / APPROACH / SURVEY 中感知超过 `perception_stale_log_s`（3s）没有新检测消息 |
| `PERCEPTION_RESUMED` | 断流后恢复，带断流时长 |

另：检测超过 0.5s 未更新时决策节点清空检测（`[STALE]`），接近单元随即走丢失路径——防止拿旧帧伺服。

## 已知限制（暂不处理）

- 投放阶段 90s 是两投合计、从到达投放区开始算。带风 / 延迟的仿真里第一投对准就可能用掉 ~50s，
  第二投会被这个总时限截断。
- 侦察凑不齐 `recon_cylinder_count` 时只能等 90s 超时。现在每次都是这样：`color_pad` 检测器只认
  "蓝垫上的白筒"（蓝垫 = 投放区 B 区），侦察筒没有蓝垫、检测不到，覆盖数恒为 0。侦察识别靠图传人工（ADR-0003）。
- Pi 进程崩溃 / 链路断开后飞控停在最后一个 GUIDED 目标悬停，只能飞手接管。
