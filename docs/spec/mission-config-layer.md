# 任务参数 / 场景配置分层（mission-config-layer）

> 状态：**已实现并验证（2026-09-23）**。结构随负责人 Q&A 定案（见 §9 决策记录）。
> 验证：逻辑等价（扫描航点与旧硬编码字节一致、SEARCH 门/phases 行为）、两场景节点构造+
> 配置注入、yaml 打包进 install；带 GUI 端到端压测台 1 轮 full_mission 成功（2/2 投放→侦察）。
> 决策权责见 [`README.md`](README.md)「决策权责」——架构由人拍板，AI 写文档/实现。

## 1. 背景与要解决的痛点

任务的「世界 / 场地 / 初始条件 / 任务描述」类参数现在**硬编码散落**在
`mission_logic.py` 顶部一堆 `_XXX` 常量里（该文件文件头也自认是「占位假设」），
想「单练某一阶段」只能靠 `initial_state` 跳状态 + `stop_after_drop`，改一个
「投放区飞行高度」这种参数还得多处手动重输——**没有一处改、同环境下所有场景
自动生效的单一真源**。

核心诉求：把这类参数抽成**结构化、按环境隔离**的配置层；**算法内部参数**
（检测器 HSV、approach 的 PID/门、FSM 自己的调参）**保持与算法在一起、不搬远**；
单阶段隔离测试靠**配置 + `initial_state` 组合**实现，**不在任务节点里加 test-only 分支**。

## 2. 设计原则（两条正交的分轴）

**轴一 —— 参数类别（决定放哪、谁是属主，永不合并）**

| 类别 | 内容 | 属主 / 位置 |
|---|---|---|
| **A. 世界 / 任务 / 场景**（非算法参数） | 起飞点场地坐标 `takeoff_point`（场地坐标轴 = 起飞坐标系 前/右，换算只做平移，见 [coordinate-frames.md](coordinate-frames.md)）、区中心与扫描航点样式、**一般巡航/过渡高度**、搜索/侦察覆盖高度（和扫描航点间距一起决定覆盖）、投放/侦察检测分界、要投几次、跑哪些阶段、要找几个筒、`initial_state`、投放 relay 通道 | **新的 mission-config 层**（独立成 `mission/`） |
| **B. 各可替换算法内部** | 检测器 HSV（`detectors/*`）；approach 的 PID/门/交接/降高/投放门 + **投放区作业高度** `approach_alt_m`/`drop_alt_m`（`approaches/*`，与投放算法绑定，见 Q2）；投放目标选择（`drop_strategy.py`）；**SEARCH 的聚类/选目标算法**（新 `search_strategy.py`，见 Q6） | **各自算法源码旁**、用**算法名**命名的 `<算法>.{sim,real}.yaml` |
| **B′. 顶层任务 FSM 自身通用参数** | 到点容差、阶段超时、电量/GPS 守卫阈值、`emulate_pilot_arm`（仅 sim 替飞手解锁）——不属于任何可替换算法，是唯一那个顶层 FSM 本体的机制 | **跟 `mission_logic.py` 放一起**：单一 `fsm_tuning.{sim,real}.yaml`（见 Q5/Q7） |
| **C. 固定物理量**（不属于任何算法、sim/real 共用） | 机体几何（相机位置/安装角、飞控离地、投放口位置与投放次序）、投放舵机接线与时序；规则书目标尺寸（投放/侦察筒高） | **`cuadc_physical` 包**：`airframe.yaml`、`targets.yaml`（单一真源；仿真 SDF 由 `cuadc_sim/field/check_airframe.py` 校验一致） |

**关键原则（Q5）**：**算法相关参数与算法代码放在一起**、且**用算法自己的名字命名参数表**
（换算法时整套参数跟着走、一目了然，如 `stepped_descent_ibvs.*.yaml`）。算法是
重点调试对象，参数抽离**不能**把可调项搬远、影响调试/修改。所以 B 类**紧贴各自算法源码、
按算法名命名**。B′ 是顶层 FSM 本体（**只有一个、不可替换**）的机制参数，不套算法名，用
FSM 自己的名字（`mission_logic`）单一一份。只有 A（本就不是算法参数）独立成 `mission/` 层。

判据：表达**任务意图/比赛规则**（飞多高过渡、投几次、跑哪些区、要找几个筒）→ A；
表达**某可替换算法怎么把活干好** → 该算法源码旁按算法名放（B）；表达**顶层 FSM 本体机制**
（到点判定/阶段预算/安全守卫）→ `mission_logic` 自己的 B′ 表。

**轴二 —— sim / real 显式分文件（环境隔离，负责人 Q1/Q3 定）**

决策侧每一层都拆成 `<层>.sim.yaml` / `<层>.real.yaml` **两份独立文件**，照
`approaches/`、`detectors/` 的 `<name>.<profile>.yaml` 既有样板。**目的是隔离**：调实机
只动 `.real.yaml`、调仿真只动 `.sim.yaml`，**互不波及**（不靠交叉引用、不靠跨包 import
去追求「物理上只有一份」）。值恰好相同的（如比赛规则筒数）就两份各写一遍——刻意用
「按环境分文件」换「环境隔离安全」。`cuadc_sim/field/config.py` 天生只服务仿真世界建图，
就是它自己那份 sim 侧的东西，真机测试碰不到它。

「单一真源」体现在**同一环境内跨场景**：base 是这一环境的唯一定义，各场景**继承**、
只写偏离——不用每建一个场景就重输一遍 base 参数（这才是原始痛点）。

## 3. 现状清点（`mission_logic.py` 硬编码常量 → 归类）

**归 A（进 `mission/` 层）**
- `_CRUISE_ALTITUDE_M`（FSM 导航/扫描高度 = 一般巡航高度；approach 自己的作业高度仍留
  approach yaml，见 Q2——含义不同、各自属主，值恰好相等不强并）
- `_DROP_ZONE_CENTER_WORLD` + `_DROP_SCAN_OFFSETS`（推 `_DROP_SCAN_WAYPOINTS`）
- `_RECON_SCAN_WAYPOINTS_WORLD`（同样参数成中心+offsets 样式）
- `_DROP_CYLINDER_COUNT`(3)、`_RECON_CYLINDER_COUNT`(5)、`_DROP_COUNT_TARGET`(2)
- 投放/侦察检测分界（现 `d.position.y < 45.0` 硬编码的中点）
- `_DROP_RELAY_CHANNEL`
- 「跑哪些阶段」（取代 `stop_after_drop`，见 Q4）

**归 B（进新 `search_strategy.{sim,real}.yaml`，挨着抽出的 SEARCH 算法，见 Q6）**
- `_SEARCH_SETTLE_SECONDS`、`_CLUSTER_MATCH_THRESHOLD_M`、`_MIN_CLUSTER_SAMPLES`、
  `_MIN_DETECTION_CONFIDENCE`
- 新增 `min_candidates_before_select`：从 `len(candidates) >= _DROP_CYLINDER_COUNT` 门里
  拆出的独立旋钮（默认=筒数 3），让场景放宽「看到几个候选就选」而不谎报筒数（见 §5.2）

**归 B′（进 `fsm_tuning.{sim,real}.yaml`，挨着顶层 FSM，见 Q7）**
- `_POSITION_TOLERANCE_M`、`_ALTITUDE_TOLERANCE_M`、`_DROP_PHASE_TIMEOUT_S`、
  `_RECON_PHASE_TIMEOUT_S`、`_BATTERY_CRITICAL_FRACTION`、`_GPS_FIX_OK_THRESHOLD`

**不动**（接口/协议常量、坐标换算）：`_INTERFACE_NS`、service 规格、`_world_xy_to_ned` 等。
`drop_strategy.py` 现有 `_USED_EXCLUSION_THRESHOLD_M` 也是 B 类算法参数，本次先不动它
（它已跟算法在一起，需要时再落 `drop_strategy.{sim,real}.yaml`），避免铺得过散。

## 4. 文件布局与加载

### 4.1 放在 `cuadc_decision`
mission-config 层是**任务逻辑**、要**上真机**（`cuadc_decision` 随伴随机部署，`cuadc_sim`
不会），故放 `cuadc_decision` 内，**不**让它依赖 `cuadc_sim`（符合
[ADR-0001](adr/0001-interface-layer-separate-module.md)）。

### 4.2 布局
```
src/cuadc_decision/cuadc_decision/
  mission_logic.py
  fsm_tuning.sim.yaml       # B′：顶层 FSM 通用参数 sim（挨着 FSM 本体，Q7）
  fsm_tuning.real.yaml      # B′：顶层 FSM 通用参数 real
  search_strategy.py           # B：抽出的 SEARCH 聚类/选目标算法（仿 drop_strategy.py，Q6）
  search_strategy.sim.yaml     # B：SEARCH 算法参数 sim（按算法名命名、挨着算法）
  search_strategy.real.yaml    # B：SEARCH 算法参数 real
  drop_strategy.py             # B：现状不动（选哪个筒）
  approaches/…                 # B：现状不动（含投放区作业高度）
  mission/                     # A：世界/任务/场景（非算法参数，独立成层）
    __init__.py
    config.py                  # MissionConfig + load(profile, scenario)
    mission.sim.yaml           # A：sim 环境 世界/任务 单一真源
    mission.real.yaml          # A：real 环境 世界/任务 单一真源
    scenarios/
      full_mission.yaml        # 全任务（恒等/最小声明）
      drop_zone_bench.yaml     # initial_state + phases=[drop] + SEARCH 简化覆盖
      recon_zone_bench.yaml    # initial_state=INIT + phases=[recon]
```

各层由**各自算法/模块自持自读**，`load_config(profile)` / `load_tuning(profile)` 与
approach 的 `load_config` 完全同构（dataclass 默认兜底、yaml 覆盖）：
- **B（SEARCH）**：`search_strategy.py` 提供 `SearchConfig` + `load_config(profile)` +
  一个 `ACTIVE` 指针（仿 `drop_strategy.py` 的轻量做法，非插件框架；现只一个方法，
  以后有第二个再照 `approaches/` 提升成子文件夹按方法分 yaml）。SEARCH 状态从
  `mission_logic` 抽到这里：聚类归并 + 候选提取 + 「够不够选目标」的门。
- **B′（FSM 本体）**：`mission_logic.py` 提供 `FsmTuning` + `load_tuning(profile)` 读挨着它的
  `fsm_tuning.<profile>.yaml`（单一一份，不套算法名）。
- **A**：`mission/config.py` 提供 `MissionConfig` + `load(profile, scenario)`。

### 4.3 场景 = 覆盖组合，绝不复制（照 SITL `--add-param-file` 分层）
`mission.load(profile, scenario)`：
1. 读 `mission.<profile>.yaml` → 世界/任务参数（world）。
2. 叠加 `scenarios/<scenario>.yaml`：声明该场景 `initial_state` / `phases`，以及极少数
   **意图性覆盖**——包括对 B 类算法旋钮的场景级覆盖（如 bench 对 SEARCH 的
   `min_candidates_before_select`），在场景文件里按目标分段显式列出（如 `search_overrides:`）。

`decision_node` 组装时先各自读基线，再把场景的 override 叠加上去（`dataclasses.replace`）。
这样**各算法旋钮的基线属主始终是挨着它自己那份 yaml**（调试算法照常改它），场景只是
**加载期的显式偏离**，不改写算法 yaml、不破坏 sim/real 隔离。场景文件本身**环境无关**，
sim/真机都能用。

### 4.4 参数如何到达 FSM
启动链已有 `profile`/`initial_state`/`stop_after_drop` 逐层透传。改为新增 `scenario`
（默认 `full_mission`）逐层透传（`mission_bringup`→`decision_bringup`→`decision_node`）：
```
decision_node.__init__:
  profile  = get_parameter('profile')                          # 已有
  scenario = declare_parameter('scenario', 'full_mission')
  world  = mission.load(profile, scenario)                     # A + 场景
  search = search_strategy.load_config(profile)                # B：SEARCH 基线
  search = replace(search, **world.search_overrides)           # 叠加场景偏离
  tuning = mission_logic.load_tuning(profile)                  # B′：FSM 本体基线
  # 用 world.initial_state 建 MissionContext；world / search / tuning 注入 mission_logic
```
- `initial_state` 改由**场景**声明；`stop_after_drop` 由 `phases` 取代（见 Q4）。
- `mission_logic` 去掉模块级常量，改读注入的 `world` / `tuning`，SEARCH 逻辑调 `search_strategy`；
  **顶层状态机结构、守卫链不动**。

### 4.5 与 `field/config.py` 的边界
`cuadc_sim/field/config.py` 仍是**仿真世界建图**的属主（精确区边界、圆筒坐标、B 环/筒壁、
地面、贴图、台架出生点 `DROP_ZONE_BENCH_SPAWN`），**真机不用、碰不到**。mission-config 只
持有**「飞机该怎么飞」所需的任务视图**（区中心推扫描航点、要找几个筒），按 Q1 **各环境自持
一份、不跨包 import、不加交叉引用注释**。sim 侧 `mission.sim.yaml` 取值须与 `field/config.py`
建出的 sim 世界对应（值不对会在压测台端到端暴露），但按决策二者**分别维护**。台架出生点
（sim 世界事实，归 `field/config.py`）与对应场景的 `initial_state`（FSM 入口，归场景 yaml）
是**成对但各自独属**。

## 5. 隔离测试：配置 + initial_state（无 test-only 分支）

### 5.1 组合方式
每个隔离测试 = **一个场景 yaml**（`initial_state` + `phases` + 必要覆盖）。任务节点里
**没有** `if test:` 分支——行为差异全部由配置表达。
- 投放区单练：`drop_zone_bench`，`initial_state=INIT`（照样走放行 + 飞手解锁两道门，真机单点
  测试也用它），起飞→前飞≈1m→搜索→接近→投放，`phases=[drop]` 投完即 RTL；sim 出生点写在
  `drop_zone_bench.sim.yaml`（`takeoff_point: [29, 0]`）。配合 `run_gazebo_drop_zone_bench.sh`（`field/config.py` 的
  `DROP_ZONE_BENCH_SPAWN` 世界）。
- `initial_state` 只用来**跳过不需要的任务阶段**，不能跳过放行和等飞手解锁（ADR-0011）。
- 只对某个环境成立的场景偏离写在可选的 `scenarios/<场景>.<profile>.yaml`（最后叠加），不能写进
  sim/real 共用的场景文件——例如 sim 单练出生点写进共用文件，真机单点测试的航点就会全偏。
- 侦察区单练：`recon_zone_bench`，同样 `initial_state=INIT`，`phases=[recon]`——起飞后直接去侦察（起飞后去哪一区
  由 `phases` 决定）；sim 出生点写在 `recon_zone_bench.sim.yaml`（`field/config.py` 的 `RECON_ZONE_BENCH_SPAWN`
  世界，`run_gazebo_recon_zone_bench.sh`），真机区中心写在 `recon_zone_bench.real.yaml`。

### 5.2 SEARCH 简化靠配置
「看到候选就去投」= 放宽选目标的门。现在门硬编码在 `_tick_search_drop_targets`：
`len(candidates) >= _DROP_CYLINDER_COUNT or scan_complete`。做法：把这个门做成 SEARCH 算法
（`search_strategy`）的独立旋钮 `min_candidates_before_select`（基线默认=筒数 3），
`drop_zone_bench.yaml` 的 `search_overrides` 覆盖成 1，`mission_logic` 里**一行 `if test` 不写**。

### 5.3 「跑哪些阶段」不成为 test-only 分支
`phases`（`[drop]` / `[drop, recon]` / `[recon]`）是**通用阶段选择配置**：投放完成后 FSM 判
`'recon' in phases` 决定去侦察还是 RTL——常规配置分支，不是 test-only 代码（取代 `stop_after_drop`，更一般化）。

### 5.4 不在本层
**投放机构测试**（命令行打通 Pi→飞控投放舵机）是另一个关注点、**另开分支**，不碰本层。

## 6. 场景清单（首批）与启动
| 场景 | initial_state | phases | 用途 |
|---|---|---|---|
| `full_mission` | INIT | [drop, recon] | 正式全任务（sim/真机同场景，脚本选 connection/profile） |
| `drop_zone_bench` | INIT | [drop] | 反复调投放接近+投放；SEARCH 简化；sim 出生点见 `.sim.yaml` |
| `recon_zone_bench` | INIT | [recon] | 反复调侦察覆盖；sim 出生点 / 真机区中心见各自 `.sim/.real.yaml` |

启动仍**一脚本一场景**：`run_mission_sim.sh` / `run_mission_sim_drop_zone_bench.sh` 各自把
`scenario:=…` 写死透传；`profile`/`connection` 仍由脚本区分 sim/真机，与场景**正交**。

## 7. 迁移步骤（sign-off 后执行）
1. 抽 `search_strategy.py`（仿 `drop_strategy.py`）：`SearchConfig` + `load_config(profile)` +
   `ACTIVE`，把 SEARCH 的聚类归并/候选提取/选目标门从 `mission_logic._tick_search_drop_targets`
   搬进来；建挨着它的 `search_strategy.{sim,real}.yaml`（§3 归 B 常量 + 拆出
   `min_candidates_before_select`）。
2. `mission_logic.py`：加 `FsmTuning` + `load_tuning(profile)`；建 `fsm_tuning.{sim,real}.yaml`，
   把 §3 归 B′ 常量搬进去。
3. 建 `mission/`：`config.py`（`MissionConfig` + `load`）+ `mission.{sim,real}.yaml` +
   `scenarios/*.yaml`；把 §3 归 A 常量搬进 `mission.*.yaml`。
4. `mission_logic` 去模块级常量，改读注入的 `world`/`tuning`、SEARCH 调 `search_strategy`
   （顶层状态机/守卫链不动）。
5. `decision_node` 增 `scenario`、按 §4.4 组装注入；launch 链透传；脚本写死 `scenario:=`
   （`drop_zone_bench` 脚本改用场景，不再靠 `stop_after_drop`，见 Q4）。
6. **验证**：
   - 纯逻辑：`mission_logic` 既有单测（脱 ROS）跑通，加载 config 后行为等价。
   - 端到端：`scripts/run_search_stress.sh --trials N`（无头压测，按二次投放成功率判定）确认
     全任务未回归；`drop_zone_bench` 场景手动跑一轮确认单阶段隔离生效。
   - 清场只按 PID/进程组 + `pgrep`→`kill -9`，不用 `pkill` 模式串（`docs/lessons.md`）。

## 8. 非目标
- 不做 god-config；不做插件框架；不引入用不到的旋钮；任务节点里不加 test-only 分支。
- 不动 approach/detector 的 sim/real yaml（B 类，投放区作业高度归它）。
- 不碰投放机构测试（另分支）、不碰并行 `feature/search-sim-hardening` 正改的 sim launch /
  感知 / SITL parm。

## 9. 决策记录（负责人 Q&A 定案）
- **Q1 属主 & 隔离**：不做交叉引用/跨包单一真源；决策侧参数按 **sim/real 显式分两份文件**，
  `field/config.py` 自持 sim 侧一份，real 测试碰不到 sim 参数、反之亦然（隔离优先）。
- **Q2 高度**：投放区作业高度**与投放算法绑定**（留 approach yaml，B 类）；mission 层只持
  **一般巡航/过渡高度**（A 类）。含义不同、各自属主，值相等不强并、不做注入。
- **Q3 FSM 内部调参**：**本次一并抽出**，并 **sim/real 分开**。
- **Q4 `min_candidates_before_select`**：归 **B′**；`stop_after_drop` 语义由场景 `phases` 取代。
- **Q5 算法参数就近放置 + 按算法名命名**：算法是重点调试对象，参数抽离不能影响调试/修改——
  **可替换算法的参数（B）与其源码放在一起、用算法自己的名字命名参数表**（换算法整套跟着走，
  如 `stepped_descent_ibvs.*.yaml`）。只有 A（非算法参数）独立成 `mission/` 层。
- **Q6 SEARCH 参数**：把内联在 `mission_logic` 里的 SEARCH 聚类/选目标**抽成轻量命名模块**
  `search_strategy.py`（仿 `drop_strategy.py`）+ 算法名参数表 `search_strategy.{sim,real}.yaml`。
- **Q7「FSM 内部」= 哪个 FSM**：只有一个——`mission_logic.py` 的**顶层扁平任务状态机**（无子 FSM）。
  它本体的通用参数（到点容差/阶段超时/安全守卫）放**单一** `fsm_tuning.{sim,real}.yaml`
  （FSM 不可替换，用自己的名字、不套算法名）；A 层保持纯粹（世界/任务/场景）。
