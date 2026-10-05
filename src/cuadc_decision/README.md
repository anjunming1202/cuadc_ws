# cuadc_decision

规划/决策模块。任务状态机的设计依据（状态列表、阶段划分、跨状态强制
打断机制）唯一权威版本在
[`docs/spec/06-planning.md`](../../docs/spec/06-planning.md)；与感知、
接口层的边界分别见
[`05-perception-decision-interface.md`](../../docs/spec/05-perception-decision-interface.md)、
[`04-interfaces.md`](../../docs/spec/04-interfaces.md)。本文件不重复
描述这些内容。

代码分层：
- `mission_logic.py`——纯 Python，状态机本身、每个阶段的判断逻辑，不
  依赖 rclpy，可以脱离 ROS2 节点单独单元测试。
- `drop_strategy.py`——投放目标选哪个筒的策略，纯 Python，独立于状态机。
  每个策略是一个函数（签名见文件头），注册进 `STRATEGIES` 字典，改
  `ACTIVE_STRATEGY` 指向哪个就切换/实验哪个策略，不用改 `mission_logic.py`。
- `search_strategy.py`——SEARCH 阶段的可替换算法（逐帧检测的时序聚类、候选提取、
  何时可选目标），从 `mission_logic` 抽出，同 `drop_strategy` 的轻量做法，可调参数在
  同名 `search_strategy.{sim,real}.yaml`（跟着算法放）。
- `approaches/`——投放接近的可替换「阶段策略」，一个 approach 一个模块 + 各自
  的 sim/real config yaml 跟着放（组织同 `cuadc_perception/detectors/`），详见
  下面「投放接近」。
- `mission/`——世界/任务/场景参数的单一真源配置层（非算法参数：区中心、扫描航点、
  一般巡航高度、要投几次、跑哪些阶段、`initial_state`…），sim/real 分文件 + 场景覆盖，
  详见 [`docs/spec/mission-config-layer.md`](../../docs/spec/mission-config-layer.md)。
- `decision_node.py`——唯一碰 ROS2 API 的地方，订阅遥测/检测、调用
  接口层 service、发布位置/速度指令，每个 tick 把数据交给 `mission_logic`
  处理，再把结果翻译回真正的 ROS2 调用。

```
ros2 launch cuadc_decision decision_bringup.launch.py
```

需要配合 `cuadc_interface`（真实/仿真飞控接口）和 `cuadc_perception`
（当前是桩感知）一起起，才能跑出完整闭环。

## Launch 参数

- `tick_rate_hz`（默认 10.0）：状态机 tick 频率。
- `profile`（默认 `sim`）：环境档，选各算法/配置层的 `.sim`/`.real` 文件（见下「配置分层」）。
- `scenario`（默认 `full_mission`）：跑哪种场景，选 `mission/scenarios/<scenario>.yaml`
  ——场景自带 `initial_state`（从哪个 state 起跑）和 `phases`（跑哪些阶段），单阶段隔离
  测试靠它 + `initial_state` 组合实现，不在状态机里加 test-only 分支。首批：`full_mission`、
  `drop_zone_bench`（投放区单练，投完即 RTL + 放宽 SEARCH 选目标门）、`recon_zone_bench`。
  完整设计见 [`docs/spec/mission-config-layer.md`](../../docs/spec/mission-config-layer.md)。

```bash
ros2 launch cuadc_decision decision_bringup.launch.py scenario:=drop_zone_bench
```

## 日志

统一 `self.get_logger()`，加标签前缀、限频（`throttle_duration_sec`）
避免刷屏：

- `[STATE]`：状态切换，每次都打。
- `[PERF]`：耗时监控（`tick()` 单次耗时、感知桩节点单帧生成耗时），约
  1 秒一条，正常情况下不会触发报警级别的输出。
- `[DETECT]`：搜索/进近阶段限频打一行当前看到的候选数量和大概尺寸。

另开一个终端 `ros2 topic echo /cuadc_decision/mission_state` 能拿到干净的、
只有当前状态的实时视图，不用在主日志里找。

## 投放接近（APPROACH）：可替换的「阶段策略」单元

任务状态机（`SEARCH → APPROACH → DROP`，见
[`06-planning.md`](../../docs/spec/06-planning.md)）、状态间的 transition、
以及三条对外接口（感知 `Detection`、接口层运动指令、投放 `release_payload`）是
**固定框架**；`APPROACH_DROP_TARGET` 这一阶段**内部怎么把飞机对到筒上**是
**自由、可替换**的部分。要试新的接近算法，就写一个新的 approach 单元换掉
旧的、transition 不变——这是 `drop_strategy.py` 那套「可插拔一层、独立于
状态转移」的做法用在更大粒度上。

- **approach 单元的接口**：跟 `drop_strategy` 同样轻量——一个 approach 是一组
  `enter/loop/exit`（`loop` 每 tick 吃 `ctx`、吐运动指令 +「对齐好没有 / 该不
  该投」的判定），注册进一个 `name -> approach` dict，一个「当前用哪个」切换/
  实验，不动状态机主干。
- **approach 自己的自由度**：粗/精子流程、先水平后下降、降高时机、末端切 IBVS
  的条件、（要的话）动态调焦、以及**用哪个检测结果**——都在 approach 内部
  决定。检测本身是感知那边的可替换库（见
  [`cuadc_perception/README.md`](../cuadc_perception/README.md)），approach 通过
  `Detection` 话题消费、**不自己跑 CV**。调焦同理：**决定**在 approach、
  **执行**走 item 3 的相机控制服务端点（见
  [`camera-tuning.md`](../../docs/spec/camera-tuning.md)）——一个资源一个持有者。
- **命名**：approach 名字要长、点出算法形态（它编码整套「怎么接近」，不是一个
  小旋钮）。第一份 = **`stepped_descent_ibvs`**（粗接近世界坐标
  PBVS、末端像素 IBVS，决策见
  [ADR-0005](../../docs/spec/adr/0005-terminal-align-visual-servoing.md)）。以后
  有新形态再加（如 `spiral_search_then_ibvs`），不复用一个太泛的名字。
- **运动指令**：粗接近用位置指令（`~/cmd/position_target_local`，现在就在用）；
  末端 IBVS 用 `~/cmd/velocity`（[04](../../docs/spec/04-interfaces.md) 已有，
  **机身系**前/右/下速度）。下视固定相机下像素误差方向天然就是机身水平方向，
  IBVS 输出直接是机身系速度，不用转地理 NED。注意 `GUID_TIMEOUT`（默认 3s
  惯性）：末端按几 Hz 持续发速度，投放前压到近零速再触发 `release_payload`。
  `TickResult` 现在只有 `position_target` + `service_call`，迁移要给它加一个
  **速度指令**字段（内部结构，由 `decision_node` 翻成对应 04 调用）。

## 配置分层：两条正交轴（类别 × sim/real）

参数按**类别**分开、永不合并，各自跟属主放；再按 **sim/real 分文件**做环境隔离。完整
设计（判据、场景组合、隔离测试）见
[`docs/spec/mission-config-layer.md`](../../docs/spec/mission-config-layer.md)，这里只列本包落点：

- **A 世界/任务/场景**（非算法参数）→ `mission/`（`mission.{sim,real}.yaml` + `scenarios/`）。
- **B 各可替换算法**（跟算法源码放、按算法名命名）→ `approaches/<approach>.{sim,real}.yaml`
  （增益/门限/降高/投放容差/杆臂…，及**投放区作业高度**——与投放算法绑定）、
  `search_strategy.{sim,real}.yaml`（SEARCH 聚类/选目标门）；检测器同理在
  `cuadc_perception/detectors/`。
- **B′ 顶层 FSM 本体通用参数**（到点容差/阶段超时/安全守卫）→ `mission_logic.{sim,real}.yaml`。

**sim/real 与 scenario 选择**沿用 `connection` 那套：不靠一个参数默认值二选一，而是
**场景启动脚本**把 `profile`/`scenario` 写死（`run_mission_sim.sh`=sim+full_mission /
`run_mission_sim_drop_zone_bench.sh`=sim+drop_zone_bench / `run_mission_real.sh`=real）。
两者经 `mission_bringup` 传到 `decision_node.__init__`，那里各层 `load` 后调
`mission_logic.configure_approach(profile)` + `configure_mission(world, tuning, search)`
一次性注入（`world` 已含场景覆盖）。同一个 `profile` 也选感知检测器的档。
- **cv_explore 沙盒不强制 yaml**：沙盒里参数用文件里看得见的常量 + README
  记录即可；yaml 提炼是**迁移到生产**才做的规范，沙盒图快、不背这个包袱。

## 迁移边界：哪些写死、哪些自由

- **现状**：`mission_logic._tick_approach_drop_target` 是闭环 **PBVS**（用
  `position_target` 发绝对局部坐标），直接写死在状态机方法里，没抽成可替换
  单元。
- **迁移做两件事**：(1) 把 approach 抽成上面的可替换单元（dict + enter/loop/
  exit + `TickResult` 加速度字段）；(2) 落地第一份
  `stepped_descent_ibvs`（粗段沿用现有 PBVS、末端加 IBVS）。
- **有意写死/耦合的部分**（迁移不动）：任务 FSM 的状态与转移（06）、transition
  契约、三条对外接口。**自由的部分**：approach 内部实现。抽象只做到「approach
  这条已被证明稳定的缝」，**不**预先给「检测 × 控制」的组合建框架——那是
  cv_explore 沙盒用文件夹探索的事，不是生产需求。

## 末端对准的两个关键设计（Gazebo 闭环踩坑后定的）

离线点质量模型抓不到、只有 Gazebo+SITL 真动力学才暴露的两点，都已修正并
验证（桩感知发 bearing，两个筒端到端投放成功、自动 RTL）：

- **全程锁 yaw**：`Set Velocity` 现在带 `yaw_rate=0`（见
  [04](../../docs/spec/04-interfaces.md) 自动朝向行为）。否则飞控默认自动朝向
  会让机身随速度矢量转 yaw、机体前/右轴跟着转，末端成了绕圈而非收敛。
- **末端硬锁关联走世界坐标、伺服走 bearing**：裸 bearing 会随机身俯仰摆动
  （高速交接的减速俯仰能把 bearing 甩出关联门→误判丢锁→靠陈旧值跑飞）；
  关联改到世界坐标最近门（姿态无关、能区分相邻筒、定位偏置在相对量里抵消），
  控制仍用 bearing 姿态补偿偏移（定位无关，[ADR-0005](../../docs/spec/adr/0005-terminal-align-visual-servoing.md)）。
  另外交接要等粗段在目标上方停稳（速度门）才切，不带动量冲进末端。

仍开口：**真实 CV** 端到端被 Block A 卡住（感知在当前 sim 世界检出为 0，见
[`cuadc_perception/README.md`](../cuadc_perception/README.md) 检测器约定）——以上
末端验证用的是桩感知发 bearing。
