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

### 1.1 手工扰动红线（防 sim-to-real 过拟合，务必遵守）

扰动是为了**压测鲁棒性**，绝不能变成"算法去拟合一个虚构噪声、结果实机反而失效"。硬规矩：

1. **只注入能对应真实机制、且能测到量值的扰动**：延迟用 cv-field-test profiling 的实测值、
   传感器噪声用 ArduPilot 标定模型/真机日志推的值——**不编数**。
2. **做不 faithfully 的宁可不做**：卷帘快门、机身振动 gz 建不真（见 §3.2 相机成像已放弃），
   硬造假的比留空更坏。真机本有、sim 复现不了的，诚实留空。
3. **姿态噪声不手工加**：开传感器噪声层，经**真实 EKF** 传播出的姿态本身就带真实噪声/延迟，
   感知反投影用的正是这个 EKF 姿态——手工再加高斯是双重计数+造假。
4. **扰动默认全关**：算法先在干净 sim 上按真实物理调好；扰动层只用来看"扛不扛得住"，
   **绝不用来把增益调去抵消某个具体假噪声**。
5. **扰动只许待在 sim-only / opt-in 处**（parm 不传、sim-only 节点），生产/实机路径逐字节不变；
   有真机 `.bin` 日志后拿它对拍，sim 加扰动的行为分布要对得上真机——防"扰动是虚构的"的终极闸。

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

### 3.2 真实感注入总索引（默认全关 → 干净 sim 行为不变）

扰动分三层，各有一张**参数表**做数值的单一事实源（都在 `cuadc_sim/config/`，
sim-only 包）；本节只记角色、注入点和加载方式，数值以表为准：

| 层 | 参数表 | 注入点 / 加载方式 | 旋钮 |
|---|---|---|---|
| 飞控动力学（风） | `sim_realism.parm` | ArduPilot 内建风模型；`gazebo_bringup sim_realism_params:=<path>` 叠 `--add-param-file`，不传=不加载 | `SIM_WIND_SPD` / `SIM_WIND_DIR` / `SIM_WIND_TURB` |
| 传感器噪声 | 同一 parm 文件 | ArduPilot 自带传感器模型（GPS 噪声经真实 EKF 传播，见 §1.1 红线 3） | `SIM_GPS_*`、`SIM_ACC*_RND`、`SIM_GYR*_RND`、`SIM_BARO_RND` |
| ROS 链路（相机→CV 图像时延） | `disturbances.yaml` | **sim-only** `image_delay_node` 插在相机↔感知之间，延迟转发 `Image`、**保留拍照戳**——注入点与真机物理一致(拍照→CV)，并触发位姿-图像时刻不一致（§3.3）。入口 `run_mission_sim_delayed.sh [ms]`（省略 ms=用表值）/ 压测 `--latency-ms` / 量测 `run_measure_reprojection.sh [ms]` | `image_latency_ms` |

生产 `mission_bringup` / 世界 bringup 不读任何一张表；不传 parm + 表值 0 =
干净 sim，行为与现在完全一致。**常驻化路径**：链路延迟真机本来就有，等 Pi 上
实测出基线延迟后把 `image_latency_ms` 填成测量值、delayed 路径转为 sim 默认
路径——在那之前不填猜测值（红线 1）。

### 3.3 反投影的两个延迟效应（区分清楚）

真机延迟对反投影有**两个不同**的影响，图像侧注入一并覆盖：

1. **结果晚到决策**：检测结果延迟送达。对**静止**筒影响小（筒不动，晚到的坐标不算过时），
   主要拖末端 IBVS（见 §5 的 150ms 悬崖）。
2. **反投影用错位姿**（新，正确性修复，非扰动）：反投影须用**图像拍摄时刻**的位姿；若用"处理时刻
   的最新位姿"，运动/转动时世界坐标本身就偏，偏量 ∝ 速度×链路延迟、角速率×延迟×高度 →
   **散点**。这极可能是真机数据集反投影发散的根因（散而非偏 = 输入不同步，非公式错）。
   - **修复（已实现，sim/real 同一份）**：感知缓冲位姿流，按图像 `header.stamp`（=拍照时刻）
     插值取位姿再反投影；`DetectionArray` 附带 `vehicle_pose`（拍照时刻位姿），决策 IBVS 的
     姿态补偿用它（契约见 05 文档）。干净/低延迟时行为不变；有延迟时才补偿。
   - **验证闭环（2026-09-30，`run_measure_reprojection.sh` 动态飞越 1m/s，bias_e）**：
     干净+capture −0.10（本底，见下）；注入 150ms+latest +0.06（=+0.15−0.10，算术吻合）；
     400ms+latest **+0.30**（随注入线性放大，FAIL）；400ms+capture **−0.03（PASS，注入被
     完全补偿）**。旋钮：`pose_at:=latest` 仅证据链 A/B 用，启用即告警。
   - **已知残差——遥测到达滞后 ~0.1s**：位姿消息内容旧、到达 interface 才盖戳（SITL 经
     MAVProxy 转发；真机是串口+解析，应更小）。静态不可见、运动时=v×0.1 的本底偏差。
     精化路径：用 MAVLink `time_boot_ms` 映射盖"测量时刻"戳（真机同理），待做。

> **姿态噪声不单列**：见 §1.1 第 3 条，由传感器噪声层经真实 EKF 自然产生，感知直接用该 EKF 位姿。

> **时延为什么在 sim-only 节点、不在生产节点**：`mission_sim_delayed.launch.py`
> 用 `SetRemap` 把感知的图像订阅改道到 `image_delay_node` 的延迟输出；生产
> `mission_bringup` 不含这套（生产文件与基线逐字节一致）。
> 入口 `run_mission_sim_delayed.sh <delay_ms>`；零延迟走 `run_mission_sim.sh`。

> **相机成像噪声——放弃 sim 注入**：gz-sim 8.14 + ogre2 相机不吃 `<noise>`（注入
> stddev=8，两帧不同渲染却逐字节几乎相同；已排除改错文件/渲染引擎）。成像留到真机阶段
> （真相机本有真噪声）；若日后要在 sim 补，走仓库内 `image_noise_stddev`，别改外部 SDF。

## 4. 构建顺序

编排器 + 打分（零扰动先测**当前 SEARCH 成功率基线**、量化波动）→ 风 → 传感器
噪声 → 感知时延 → **然后**据此改 SEARCH（相机成像层已放弃 sim 注入，见 §3.2）。

## 5. 感知时延阈值（标定发现）

**重扫（2026-09-30，图像延迟法 `--latency-ms`，真机化窄相机+时刻对齐后，每档 5 轮）**：

| 时延 | stepped_descent_ibvs | cone_descent_ibvs |
|---|---|---|
| 0ms | 10/10 | 9/9（一轮基础设施误伤除外） |
| 150ms | **0/5** | **0/5**（250/350 被支配未扫） |

**悬崖比历史更靠前**：旧系统(宽视场云台+无时刻对齐)150ms 还满分，现系统 150ms 两套
approach 同归于尽。失效签名完全一致且**不是丢筒**——检测全程在(miss=0)、SEARCH/粗段
正常，塌的是末端水平伺服：150ms 纯滞后把 PID 推入 ±0.2m 极限环（off 打转、hspeed
0.2-0.3 降不下来），投放门(4cm)永远进不去，守卫超时放弃。历史"0-150ms→100%"作废：
旧相机 FOV 大、瞄准门宽，掩盖了同一滞后。

时刻对齐(ADR-0010 B)已把**测量偏置**消干净（400ms 下反投影 3cm，见 §3.3），剩下的
塌点是纯控制动态——正因如此两套降高策略(阶梯/锥面)在此毫无差别：瓶颈在共享的水平
PID 死区时间，不在竖直调度。

- **安全预算收紧：感知端到端时延必须 ≪150ms**，或末端伺服加时延补偿后重新标定。
  真机实测值由 cv-field-test 的 on-Pi profiling 给。
- 下一步（算法阶段待办）：**测量时效补偿**——检测带拍照戳，tick 时算 age=now−capture，
  用遥测机体速度把量测偏移前推 off_now≈off_capture−v_body·age（只用实测量，共用于两套
  approach，放 approaches/base）。消掉死区里"测量老化"一半后重扫定新悬崖。

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
