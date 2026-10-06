"""失效处理路径的单测（纯 Python，不起 ROS）：每条 docs/spec/failure-handling.md 里新增的
超时/放弃/拉黑/下一目标转移都走一遍。时间用假时钟推进。"""

from types import SimpleNamespace as NS

import pytest

from cuadc_decision import drop_strategy
from cuadc_decision import mission_logic as ml
from cuadc_decision import search_strategy
from cuadc_decision.approaches import cone_descent_ibvs as cone
from cuadc_decision.drop_strategy import Candidate
from cuadc_decision.mission import config as mc

S = ml.State


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(ml.time, 'monotonic', c)
    monkeypatch.setattr(cone.time, 'monotonic', c)
    return c


def configure(scenario='full_mission', **tuning):
    world = mc.load('sim', scenario)
    t = ml.load_tuning('sim')
    for k, v in tuning.items():
        setattr(t, k, v)
    ml.configure_mission(world, t, search_strategy.load_config('sim'))
    ml.configure_approach('sim')
    return world


def flying_ctx(clock, state, xy=(0.0, 0.0), h=3.0):
    ctx = ml.MissionContext(current_state=state, state_entered_monotonic=clock.t)
    ctx.armed, ctx.mode, ctx.pi_in_control = True, 'GUIDED', True
    ctx.local_position_xy, ctx.local_position_z = xy, -h
    ctx.horizontal_speed, ctx.vertical_speed = 0.0, 0.0
    ctx.roll_rad = ctx.pitch_rad = 0.0
    ctx.drop_task.phase_entered_monotonic = clock.t
    return ctx


def det(x, y, bearing=(0.0, 0.0), size=0.1):
    return NS(target_type='cylinder', confidence=0.9, position=NS(x=x, y=y),
              apparent_size=size, bearing_x=bearing[0], bearing_y=bearing[1])


def cluster(x, y, size=0.1, n=5):
    return [(x, y, size)] * n


# ---- 起飞 / 转场 ----

def test_takeoff_timeout_goes_rtl(clock):
    configure()
    ctx = flying_ctx(clock, S.TAKEOFF, h=1.0)
    ctx.takeoff_step, ctx.takeoff_wait_since = 'wait_altitude', clock.t
    clock.t += 19.0
    assert ml.tick(ctx).next_state is None
    clock.t += 2.0
    r = ml.tick(ctx)
    assert r.next_state == S.RTL_OR_LAND and r.fault.code == 'TAKEOFF_TIMEOUT'


def test_nav_timeout_starts_search_anyway(clock):
    configure()
    ctx = flying_ctx(clock, S.NAVIGATE_TO_DROP_ZONE, xy=(0.0, 0.0))
    assert ml.tick(ctx).next_state is None          # 定时限：2×31.4/5 + 10 ≈ 22.6s
    clock.t += 22.0
    assert ml.tick(ctx).next_state is None
    clock.t += 1.0
    r = ml.tick(ctx)
    assert r.next_state == S.SEARCH_DROP_TARGETS and r.fault.code == 'NAV_TIMEOUT'


# ---- SEARCH 扫完没目标 ----

def _at_last_scan_waypoint(ctx):
    wp = ml._DROP_SCAN_WAYPOINTS[-1]
    ctx.drop_task.waypoint_index = len(ml._DROP_SCAN_WAYPOINTS) - 1
    ctx.local_position_xy, ctx.local_position_z = wp, -ml._WORLD.search_alt_m


@pytest.mark.parametrize('scenario,give_up_to', [('full_mission', S.NAVIGATE_TO_RECON_ZONE),
                                                  ('drop_zone_bench', S.RTL_OR_LAND)])
def test_search_empty_rescans_then_gives_up(clock, scenario, give_up_to):
    configure(scenario)
    ctx = flying_ctx(clock, S.SEARCH_DROP_TARGETS)
    clock.t += 5.0                                    # 过 settle
    _at_last_scan_waypoint(ctx)
    r = ml.tick(ctx)
    assert r.fault.code == 'SEARCH_EMPTY_RESCAN' and ctx.drop_task.waypoint_index == 0
    _at_last_scan_waypoint(ctx)
    r = ml.tick(ctx)
    assert r.next_state == give_up_to and r.fault.code == 'SEARCH_EMPTY_GIVE_UP'


def test_search_skips_rejected_and_used_candidates(clock):
    configure()
    ctx = flying_ctx(clock, S.SEARCH_DROP_TARGETS)
    ctx.drop_task.clusters = [cluster(31.0, 1.5, 0.08), cluster(32.5, -1.5, 0.10), cluster(34.0, 0.0, 0.12)]
    ctx.drop_task.used_positions = [(31.0, 1.5)]
    ctx.drop_task.rejected_positions = [(32.5, -1.5)]
    clock.t += 5.0
    r = ml.tick(ctx)
    assert r.next_state == S.APPROACH_DROP_TARGET
    assert (ctx.drop_task.selected_target.x, ctx.drop_task.selected_target.y) == (34.0, 0.0)


# ---- 投放后：直接飞下一个候选 / 回 SEARCH ----

def _drop_ctx(clock, clusters):
    ctx = flying_ctx(clock, S.DROP_PAYLOAD, xy=(31.0, 1.5), h=1.0)
    ctx.drop_task.clusters = clusters
    ctx.drop_task.selected_target = Candidate(31.0, 1.5, 0.08)
    ctx.last_service_result = ml.ServiceResult('release_payload', True)
    return ctx


def test_after_drop_goes_straight_to_next_known_candidate(clock):
    configure()
    ctx = _drop_ctx(clock, [cluster(31.0, 1.5, 0.08), cluster(32.5, -1.5, 0.12), cluster(34.0, 0.0, 0.10)])
    r = ml.tick(ctx)
    assert r.next_state == S.APPROACH_DROP_TARGET and ctx.drop_task.drop_count == 1
    assert (ctx.drop_task.selected_target.x, ctx.drop_task.selected_target.y) == (34.0, 0.0)  # 剩余里最小


def test_after_drop_without_candidates_searches_again(clock):
    configure()
    ctx = _drop_ctx(clock, [cluster(31.0, 1.5, 0.08)])
    ctx.drop_task.waypoint_index = 3
    r = ml.tick(ctx)
    assert r.next_state == S.SEARCH_DROP_TARGETS and ctx.drop_task.waypoint_index == 0


def test_release_failures_retry_then_count_as_attempted(clock):
    configure()
    ctx = _drop_ctx(clock, [cluster(31.0, 1.5, 0.08)])
    codes = []
    for _ in range(3):
        ctx.last_service_result = ml.ServiceResult('release_payload', False)
        r = ml.tick(ctx)
        codes.append(r.fault.code)
        if r.next_state is not None:
            break
        assert r.service_call is not None and r.service_call.name == 'release_payload'
    assert codes == ['RELEASE_FAILED', 'RELEASE_FAILED', 'RELEASE_GAVE_UP']
    assert ctx.drop_task.drop_count == 1 and r.next_state == S.SEARCH_DROP_TARGETS


def test_release_requests_port_rotating_from_first_drop_port(clock):
    configure()
    ctx = _drop_ctx(clock, [cluster(31.0, 1.5, 0.08)])
    ctx.last_service_result, ctx.drop_port_start = None, 1
    first = ml.tick(ctx).service_call
    ctx.last_service_result = ml.ServiceResult('release_payload', False)
    retry = ml.tick(ctx).service_call
    assert first.kwargs == {'port': 1} and retry.kwargs == {'port': 1}  # 重试仍是同一口
    ctx.drop_task.drop_count, ctx.last_service_result, ctx.command_dispatched = 1, None, False
    assert ml.tick(ctx).service_call.kwargs == {'port': 0}              # 第二投换另一口


# ---- APPROACH 放弃 -> 拉黑 -> 下一个 ----

def _approach_ctx(clock, target, clusters, xy=None, dets=()):
    ctx = flying_ctx(clock, S.APPROACH_DROP_TARGET, xy=xy or (target.x, target.y), h=3.0)
    ctx.drop_task.clusters = clusters
    ctx.detections = list(dets)
    ctx.drop_task.selected_target = target
    ml._ACTIVE_APPROACH.enter(ctx)
    return ctx


def test_coarse_unconfirmed_target_rejected_then_next_candidate(clock):
    configure()
    ghost = Candidate(32.5, -0.5, 0.08)
    ctx = _approach_ctx(clock, ghost, [cluster(32.5, -0.5, 0.08), cluster(34.0, 0.0, 0.12)])
    assert ml.tick(ctx).next_state is None
    clock.t += 8.5
    r = ml.tick(ctx)
    assert r.fault.code == 'TARGET_UNCONFIRMED'
    assert (32.5, -0.5) in ctx.drop_task.rejected_positions
    assert r.next_state == S.APPROACH_DROP_TARGET
    assert (ctx.drop_task.selected_target.x, ctx.drop_task.selected_target.y) == (34.0, 0.0)


def test_coarse_unconfirmed_without_other_candidates_searches(clock):
    configure()
    ghost = Candidate(32.5, -0.5, 0.08)
    ctx = _approach_ctx(clock, ghost, [cluster(32.5, -0.5, 0.08)])
    ml.tick(ctx)
    clock.t += 8.5
    r = ml.tick(ctx)
    assert r.fault.code == 'TARGET_UNCONFIRMED' and r.next_state == S.SEARCH_DROP_TARGETS


def test_wide_relock_reported_and_ignores_rejected(clock):
    configure()
    ghost = Candidate(32.5, -0.5, 0.08)
    ctx = _approach_ctx(clock, ghost, [], dets=[det(32.5, -1.5)])
    ctx.drop_task.rejected_positions = [(32.5, -1.5)]
    for _ in range(8):
        r = ml.tick(ctx)
        assert r.fault is None                       # 被拉黑的位置不改锁
    ctx.drop_task.rejected_positions = []
    codes = [ml.tick(ctx).fault for _ in range(6)]
    assert any(f is not None and f.code == 'RELOCK_WIDE' for f in codes)


def test_terminal_repeated_loss_gives_up(clock):
    configure()
    a = ml._ACTIVE_APPROACH
    target = Candidate(32.5, -1.5, 0.1)
    ctx = _approach_ctx(clock, target, [])
    ctx.local_position_z = -(a.cfg.reacquire_ceiling_m + 0.1)  # 高于 reacquire_ceiling：持续丢失即退回粗段
    out = None
    for attempt in range(a.cfg.max_reacquire + 1):
        a.state.phase, a.state.prev_off_meas = 'terminal', (0.0, 0.0)
        for _ in range(a.cfg.max_coast_ticks + 2):
            out = a.loop(ctx, now=clock.t)
            if a.state.phase == 'coarse' or out.give_up:
                break
    assert out.give_up is not None and out.give_up[0] == 'TARGET_LOST_REPEATEDLY'


def _terminal_at_drop_alt(clock, herr_fwd):
    configure()
    a = ml._ACTIVE_APPROACH
    target = Candidate(32.5, -1.5, 0.1)
    ctx = _approach_ctx(clock, target, [])
    ctx.local_position_z = -a.cfg.drop_alt_m
    a.state.phase, a.state.mode = 'terminal', 'track'
    depth = a._depth(a.cfg.drop_alt_m)
    off = (a.state.lever[0] + herr_fwd, a.state.lever[1])
    ctx.detections = [det(32.5, -1.5, bearing=(off[1] / depth, -off[0] / depth))]
    return a, ctx


def test_drop_gate_relaxes_then_drops(clock):
    a, ctx = _terminal_at_drop_alt(clock, herr_fwd=0.06)   # 4cm 门过不了，8cm 放宽门能过
    events, ready = [], False
    for _ in range(200):
        out = a.loop(ctx, now=clock.t)
        if out.event:
            events.append(out.event[0])
        if out.drop_ready:
            ready = True
            break
        clock.t += 0.1
    assert events == ['DROP_GATE_RELAXED'] and ready
    assert clock.t - 1000.0 > a.cfg.stable_timeout_s


def test_drop_gate_timeout_gives_up(clock):
    a, ctx = _terminal_at_drop_alt(clock, herr_fwd=0.12)   # 放宽后仍过不了
    out = None
    for _ in range(300):
        out = a.loop(ctx, now=clock.t)
        if out.give_up:
            break
        clock.t += 0.1
    assert out.give_up[0] == 'DROP_GATE_TIMEOUT'
    assert clock.t - 1000.0 > a.cfg.stable_timeout_s + a.cfg.relaxed_timeout_s


# ---- 守卫带原因 ----

def test_guards_carry_fault_codes(clock):
    configure()
    ctx = flying_ctx(clock, S.SEARCH_DROP_TARGETS)
    ctx.mode = 'LOITER'
    assert ml.tick(ctx).fault.code == 'PILOT_TAKEOVER'
    ctx.mode, ctx.battery_fraction = 'GUIDED', 0.1
    assert ml.tick(ctx).fault.code == 'BATTERY_CRITICAL'
    ctx.battery_fraction = None
    clock.t += ml._TUNING.drop_phase_timeout_s + 1
    r = ml.tick(ctx)
    assert r.fault.code == 'DROP_PHASE_TIMEOUT' and r.next_state == S.NAVIGATE_TO_RECON_ZONE


def test_strategy_signature_unchanged():
    c = [Candidate(0, 0, 0.1), Candidate(1, 1, 0.08)]
    assert drop_strategy.ACTIVE_STRATEGY(c, [(1.0, 1.0)], 0) == c[0]


# ---- 2026-10-05 真机暴露的空档：贴着投放高度上方浮动 / 末端久拖 ----

def test_drop_gate_budget_runs_above_drop_alt(clock):
    """在 drop_alt 上方带宽内浮动（从不进入投放分支）也要放宽、到时放弃。"""
    a, ctx = _terminal_at_drop_alt(clock, herr_fwd=0.12)
    ctx.local_position_z = -(a.cfg.drop_alt_m + 0.05)
    events, out = [], None
    for _ in range(300):
        out = a.loop(ctx, now=clock.t)
        if out.event:
            events.append(out.event[0])
        if out.give_up:
            break
        clock.t += 0.1
    assert events == ['DROP_GATE_RELAXED'] and out.give_up[0] == 'DROP_GATE_TIMEOUT'


def test_relaxed_event_reported_while_climbing(clock):
    """放宽恰在丢失爬升时到期，事件也要随那一拍的输出带出去。"""
    a, ctx = _terminal_at_drop_alt(clock, herr_fwd=0.12)
    a.loop(ctx, now=clock.t)                       # 进入带宽，开始计时
    ctx.detections = []
    a.state.miss = a.cfg.max_coast_ticks + 1        # 下一拍走爬升
    clock.t += a.cfg.stable_timeout_s + 0.5
    out = a.loop(ctx, now=clock.t)
    assert out.velocity_target[2] < 0 and out.event[0] == 'DROP_GATE_RELAXED'


def test_terminal_timeout_gives_up(clock):
    """末端一直对不上、也没降到投放高度：本目标末端总时长到点放弃。"""
    configure()
    a = ml._ACTIVE_APPROACH
    target = Candidate(32.5, -1.5, 0.1)
    ctx = _approach_ctx(clock, target, [])
    ctx.local_position_z = -2.0
    a.state.phase, a.state.mode, a.state.terminal_since = 'terminal', 'track', clock.t
    depth = a._depth(2.0)
    ctx.detections = [det(32.5, -1.5, bearing=(0.4 / depth, 0.0))]   # 偏 0.4m，锥面外不降
    out = None
    for _ in range(int(a.cfg.terminal_timeout_s / 0.1) + 20):
        out = a.loop(ctx, now=clock.t)
        if out.give_up:
            break
        clock.t += 0.1
    assert out.give_up[0] == 'TERMINAL_TIMEOUT'
    assert a.cfg.terminal_timeout_s < clock.t - 1000.0 < a.cfg.terminal_timeout_s + 0.5


# ---- 收尾 RTL ----

def test_rtl_commanded_once_then_never_again(clock):
    """decision_node 每拍把 last_service_result 刷成 None（无新结果时）：RTL 只能切一次，
    之后飞手拨模式、落地上锁都不能被抢回 RTL。"""
    configure()
    ctx = flying_ctx(clock, S.RTL_OR_LAND)
    r = ml.tick(ctx)
    assert r.service_call is not None and r.service_call.name == 'set_mode'
    ctx.last_service_result = ml.ServiceResult('set_mode', True)
    assert ml.tick(ctx).service_call is None
    ctx.mode = 'RTL'
    for _ in range(5):
        ctx.last_service_result = None
        assert ml.tick(ctx).service_call is None
    ctx.armed, ctx.mode = False, 'STABILIZE'      # 落地上锁后飞手拨回 STABILIZE 准备再飞
    for _ in range(5):
        ctx.last_service_result = None
        assert ml.tick(ctx).service_call is None


def test_rtl_not_sent_when_fc_already_in_rtl(clock):
    configure()
    ctx = flying_ctx(clock, S.RTL_OR_LAND)
    ctx.mode = 'RTL'                              # 飞控 failsafe 自己切了 RTL
    assert ml.tick(ctx).service_call is None
    ctx.mode = 'LOITER'                           # 空中飞手拨走 -> 夺控，不抢回
    r = ml.tick(ctx)
    assert r.service_call is None and r.next_state == S.PILOT_TAKEOVER
