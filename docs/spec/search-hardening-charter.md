# SEARCH 加固探索 · 章程与诊断纪要

> **性质**:并行 agent 组的**共享输入 + 工作纪律**。第二次投放失败的修复决策
> 已落 [ADR-0006](adr/0006-approach-relock-gate-vs-search-error.md);本文件留作那轮
> 并行竞争探索的历史记录(失败链诊断 + 三方案过程)。分支:`feature/search-sim-hardening`。

## 0. 背景

投放区任务要投满 2 个筒。当前**第二次投放常失败**。仿真压测台(见
[`sim-stress-testbed.md`](sim-stress-testbed.md))已能**可复现地量化**这个失败,
本轮目标是把它修好。之前在真实素材上研究 SEARCH 收效有限(素材质量/条件与
投放时不匹配),所以改在 sim 里、对着压测台、用真实条件设计。

## 1. 精确失败链(sim 实测,基线 `3d6416f`,固定布局、零扰动)

1. 第 2 次 `SEARCH_DROP_TARGETS` 的簇**欠采样**:2/3 号筒各仅约 3 个样本
   (`[SEARCH]` 日志:`(0.6,32.3)x3; (1.3,32.5)x3`)。`_cluster_mean` 是**朴素平均**,少样本下有偏。
2. 偏了的簇均值 → 锁定的世界坐标离**真实筒约 0.67m**。
3. approach 的 coarse→terminal **handoff** 要求"检测水平误差 ≤ `handoff_off_m`(0.6m)
   且已停稳(`hspeed ≤ 0.4`)"。实测无人机飞到锁定点后 `dist2lock≈0.00`、
   `hspeed≈0.002`(纹丝不动),但 `near_det` 卡在 ~0.67 > 0.6 → **handoff 永不触发** → 悬停。
4. `_check_guards` 的**投放区任务超时守卫**(`_DROP_PHASE_TIMEOUT_S`,墙钟计时)到时
   → **放弃投放、强制转 `NAVIGATE_TO_RECON_ZONE`**(此时只投了 0~1 次)。

一句话:**欠采样簇 → 锁定位置偏 → 卡在 handoff 阈值外 → 悬停 → 守卫放弃**。
是 SEARCH 关联/估计层与 approach handoff 的**交互**失败,不是单点。

## 2. 真实基线(修复打分器后)

| 轮次特征 | 现象 | 判定 |
|---|---|---|
| ~40s 完成 | 第二次也快速收敛、投满 2 次 | success |
| ~106s | 第二次卡在 handoff、撞守卫 | gave_up(drops=1) |

双峰、run-to-run 波动,约 0–40% success。抽样 3 轮无头曾 0/3(全 gave_up,drops=1,
~106s)。**注意**:旧打分器把所有 1-drop 的 106s 轮误报成 success,别信历史"5/5"。

## 3. 度量(唯一裁判)

- 压测台:`./scripts/run_search_stress.sh --trials N [--out x.csv]`,详见
  [`sim-stress-testbed.md`](sim-stress-testbed.md)。
- 指标:`success`= 时间线 `DROP_PAYLOAD` 出现 ≥2 次;`gave_up`= 到终态但 <2;`timeout`。
- **无头跑分**(忠实、近实时);**commit 前 GUI 抽验一轮**。GUI 慢 sim 会让墙钟守卫更严苛、
  感知采样更少(比无头更早卡死)——**GUI 只用来肉眼看行为,别拿 GUI 的成绩当分**。

## 4. 目标 & 范围

- 目标:**固定布局、零扰动下把第二次投放成功率拉到高位**(先干净 sim 稳了,再加扰动压测)。
- **范围由各方案自定并论证**——可攻:关联/估计层(去离群/确认簇/远筒采样/选择门)、
  approach handoff(阈值/coarse 是否该跟检测走而非陈旧世界锁)、守卫预算,或它们的交互。
- 约束:**不用桩**;**真机可迁移**(别 overfit sim 的巧合);改动**别冗余/过度设计**
  (典型反例:一堆用不到的条件分支);production 命名带 `cuadc` 前缀。

## 5. 工作方式(并行竞争 + critic)

- 每个方案一个 worktree + 子分支(off `feature/search-sim-hardening`),各实现一版。
- ⚠️ **完整 Gazebo+SITL 不能并发多开**(抢 5760/14550 端口 + CPU)。**实现可并行,
  压测台评分必须串行**(一次一个 sim,由协调者排队)——别同时起 Gazebo。
- **critic**:一个 agent 专挑刺(欠采样/簇偏/阈值/守卫/真机迁移/过度设计),
  每条质疑都要被**解答 + 压测台验证**,不能空对空。
- **每个方案的交付**:①设计说明(改什么、为什么、预期机理)②实现 diff
  ③压测台成绩(≥N 轮 CSV)④GUI 抽验一轮的结论 ⑤对 critic 每条质疑的答复。
- 收敛:成绩横向比 + critic 结论 → 一份 SEARCH 设计 **ADR**(人拍板)→ 合入。

## 6. 材料(全在 git/repo)

- SEARCH/关联现状:`src/cuadc_decision/cuadc_decision/mission_logic.py`
  (`_update_clusters` / `_cluster_mean` / `_tick_search_drop_targets` / `_check_guards`)。
- approach/handoff:`src/cuadc_decision/cuadc_decision/approaches/coarse_pbvs_then_terminal_ibvs.py`
  (`_coarse` 里的 handoff 条件;`ApproachConfig` 的 `handoff_off_m`/`handoff_speed_mps`/`coarse_radius_m`)。
- 感知↔决策契约:[`05-perception-decision-interface.md`](05-perception-decision-interface.md);
  检测器:`src/cuadc_perception/cuadc_perception/detectors/color_pad.py`。
- 场地/筒布局:[`field-layout.md`](field-layout.md);竞赛规则摘录:`docs/reference/`。
- 之前 search-cv 探索(真实素材,条件不匹配、收效有限,只作教训参考):`feature/search-cv-20260921*`。
- 压测台:[`sim-stress-testbed.md`](sim-stress-testbed.md) + `scripts/search_stress.py`。

## 7. 纪律红线

- 提交前:压测台跑过 + **GUI 抽验过**(保证人随时打开 GUI 抽查不会撞上工具性问题)。
- 清场**只按 PID/进程组 + `pgrep`→`kill -9`**,绝不 `pkill` 模式串(见 `docs/lessons.md`)。
- 不改其它分支(可 merge FROM);worktree 放同级 `cuadc_ws_worktrees/`。
- commit 英文、无署名。
