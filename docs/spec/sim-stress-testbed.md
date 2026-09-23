# 仿真压力测试台：真实条件下的 SEARCH 优化闭环

## 1. 目标与范围

可**复现地量化**"改一版 SEARCH 逻辑是否让投放任务更稳":在**真实条件**
（风扰、传感器噪声、感知时延、相机成像）下跑 Gazebo + **真实 CV** 全流程，
统计**第二次投放成功率**，作为 SEARCH（及后续）算法优化的度量与压力来源。

- **不用桩节点**——扰动一律建模真实情况，注入真实闭环，不给真值加假高斯。
- 不测控制环/接口层本身（那是 `flight_acceptance_test.py` 的事）。
- **固定筒布局**（现有 `field/config.py` 坐标），每轮只变扰动种子——直接复现
  并解决已观察到的第二次投放 run-to-run 波动；度量的是"对扰动的鲁棒性"，
  不是对布局的泛化（随机布场是以后的事）。

## 2. 指标与判定

- **主指标**：第二次投放成功率（N 轮中成功比例）。
- 成功 = 预算内**实际完成两次投放**，判据是 `mission_state` 时间线里
  `DROP_PAYLOAD` 出现 ≥2 次（每进一次 = 完成一次投放）。**不能**只看"到达
  `NAVIGATE_TO_RECON_ZONE`"就算成功：决策的 `_check_guards` 有个投放区任务
  超时守卫，超时会**放弃投放、强制转 `NAVIGATE_TO_RECON_ZONE`**（0~1 次投放）——
  那是失败(`gave_up`)不是成功。
- 结果分三类：`success`(投放≥2) / `gave_up`(到达终态但投放<2，守卫放弃) /
  `timeout`(整轮硬超时都没到终态)。
- 附带：完成耗时、各状态停留时长、失败归因（卡在哪个状态）。
- 结果信号取 `/cuadc_decision/mission_state`（`std_msgs/String`，State 枚举），
  **不侵入决策代码**——压测台是外部观察者。

## 3. 架构：复用现有 launch + 薄编排 + 可配真实感

现有三层（世界 / 任务节点 / 触发，见 `README.md` Quickstart）**不变**；压测台
只在外面套一个编排器，并往里注入可配置的真实感。

### 3.1 无头编排器（`cuadc_sim`）

每轮：起 `gazebo_bringup`(`gui:=false`) + realism parm → 起 `mission_bringup`
(`profile:=sim stop_after_drop:=true`) → 发 `task_start_trigger` → 订阅
`mission_state` 记状态时间线 → 判定 → **按捕获的 PID/进程组干净拆除** → 硬
超时兜底 → 汇总 N 轮成 CSV + 摘要。

### 3.2 真实感注入（默认全关 → 干净 sim 行为不变）

| 层 | 注入点 | 内容 |
|---|---|---|
| 飞控动力学（风） | `config/sim_realism.parm` + `gazebo_bringup` 新增 `sim_realism_params:=<path>` 再叠一个 `--add-param-file` | `SIM_WIND_SPD` / `SIM_WIND_DIR` / `SIM_WIND_TURB` |
| 传感器噪声 | 同一 parm 文件 | `SIM_GPS_*`、`SIM_ACC*_RND`、`SIM_GYR*_RND`、`SIM_BARO_RND`（ArduPilot 自带传感器模型） |
| 感知时延 | **sim-only** `detection_delay_node`（`cuadc_sim`）经压测台 launch 插在感知↔决策之间 | 缓冲延迟转发 `DetectionArray`，建模相机→CV→决策实测时延（值可由 cv-field-test profiling 提供） |

不给 `sim_realism_params`（parm 缺省）+ 零延迟档 = 干净 sim，行为与现在完全一致。

> **感知时延为什么不放进生产节点（B 方案）**：时延注入不改任何生产文件
> （`cv_perception_node`/`cv_perception_bringup`/`mission_bringup` 保持纯净）。
> 压测台专用的 `launch/mission_sim_delayed.launch.py` 用 `SetRemap` 把感知输出
> `/perception/detections` 改道到 `_predelay`（只作用于该 launch 里包含的
> `cv_perception_bringup`），再由 `detection_delay_node` 延迟后发回原话题给决策。
> 生产链 `mission_bringup` 里没有这个 launch、没有延迟节点、没有 remap，真机/正式
> 仿真感知→决策零延迟直连——real 不受污染这一点在 `git diff` 层面可证（生产文件
> 与重构基线逐字节一致）。压测台入口：`run_mission_sim_delayed.sh <delay_ms>`
> （零延迟仍走 `run_mission_sim.sh`）。

> **相机成像噪声（放弃 SDF 路线）**：原计划改外部 `ardupilot_gazebo` 相机传感器
> SDF 加 `<noise>`。实测本机 **gz-sim 8.14 + ogre2 相机不吃 `<noise>`**：注入
> stddev=8 后，两帧（时间戳确不同、即不同渲染）仍逐字节几乎相同（帧间 mean-abs
> diff=0.003，若生效应≈9）。已排除改错文件（同模型磁盘有源码/编译产物多份拷贝，
> gz 只认 `GZ_SIM_RESOURCE_PATH` 上那份）和渲染引擎（ogre2 本支持噪声 pass）。
> 再挖 gz 内部噪声 pass 属外部依赖深坑。成像噪声层**暂跳过**（决策：先做全扰动
> 组合，成像留到真机阶段——真相机本就有真噪声）。若日后要在 sim 补：走仓库内
> 注入（`cv_perception_node` 加 `image_noise_stddev`，检测前对帧加噪，与
> `detection_latency_ms` 同套路），别再走外部 SDF。

## 4. 构建顺序

编排器 + 打分（零扰动先测**当前 SEARCH 成功率基线**、量化波动）→ 风 → 传感器
噪声 → 感知时延 → 相机成像 → **然后**据此改 SEARCH（去离群/确认簇/放宽远筒
选择门/dedup，见 `cuadc_decision/README.md` 与 `mission_logic.py` 现状）。

## 5. 感知时延阈值（标定发现）

用时延层单独扫（其余扰动关、固定布局，每档 3 轮，`--detection-latency-ms`）：

| 时延 | 第二次投放成功率 |
|---|---|
| 0 / 100 / 150ms | 100%（每轮投满 2） |
| 200ms | 33%（拐点） |
| 250 / 300ms | 0% |

**悬崖极陡**：150ms 满分，200ms 起塌，250ms+ 归零——不是缓降。SEARCH 本身不受
影响（簇正常形成、选出目标、进 APPROACH）；塌的是**末端 IBVS**——它是盯检测
`offset` 的 PID 伺服，时延=闭环里的纯滞后(dead-time)，超过约 150ms 就把伺服推向
极限环振荡，对不准到可投、守卫超时放弃。

- **安全预算：感知端到端时延 ≤ 150ms。** 真机实测值由 cv-field-test 的 on-Pi
  profiling 给（CV 计算耗时感知节点 `_publish_perf` 已在回传；相机捕获+图传两段需
  Pi 上测）。
- 决策：真值 ≤150ms → 末端伺服不必改；落进悬崖(150~250ms) → 需硬化末端 IBVS
  抗时延（降增益 / 用速度外推目标做时延补偿 / Smith 预估器等）。

## 6. 坑 / 注意

- 拆除**只按 PID/进程组**，绝不 `pkill` 模式串（自杀教训，见 `docs/lessons.md`）。
- WSL2 下 SITL 相关命令带 `--no-wsl2-network`（见 `docs/lessons.md`）。
- 每轮 Gazebo 冷启约分钟级，N 取 10–20 量级，**每轮必须有硬超时**。
- 每轮换不同扰动种子；布局固定。

## 7. 与其它文档

- SEARCH/聚类现状、感知↔决策契约：`cuadc_decision/README.md`、
  [`05-perception-decision-interface.md`](05-perception-decision-interface.md)。
- 感知时延实测来源：cv-field-test 的 probe profiling（每阶段耗时）。
- 场地/筒布局：[`field-layout.md`](field-layout.md)。
