"""position_lock_descent（ADR-0012）单测：飞控按位置指令理想跟随的简化回路，验证末端估计/降高/
投放门/丢失处理；外加 approach 选择与变体叠加。纯 Python，不起 ROS。"""

import math
from types import SimpleNamespace as NS

from cuadc_decision import approaches
from cuadc_decision import mission_logic as ml
from cuadc_decision.drop_strategy import Candidate

TARGET = (32.0, -1.0)
DT = 0.1


def det(x, y, bearing=(0.0, 0.0)):
    return NS(target_type='cylinder', confidence=0.9, position=NS(x=x, y=y),
              apparent_size=0.1, bearing_x=bearing[0], bearing_y=bearing[1])


def make():
    a = approaches.create_active('sim', name='position_lock_descent')
    ctx = ml.MissionContext(current_state=ml.State.APPROACH_DROP_TARGET)
    ctx.local_position_xy, ctx.local_position_z = TARGET, -a.cfg.approach_alt_m
    ctx.horizontal_speed = ctx.vertical_speed = 0.0
    ctx.roll_rad = ctx.pitch_rad = ctx.yaw_rad = 0.0
    ctx.drop_task.selected_target = Candidate(*TARGET, 0.1)
    a.enter(ctx)
    return a, ctx


def fly(a, ctx, seconds, t=1000.0, dets=lambda i: [det(*TARGET)], follow_xy=True, follow_z=True):
    """飞控理想跟随位置指令；返回 (最后输出, 时刻, 指令轨迹)。"""
    out, trace = None, []
    for i in range(int(round(seconds / DT))):
        ctx.detections = dets(i)
        out = a.loop(ctx, now=t)
        if out.drop_ready or out.give_up:
            return out, t, trace
        if out.position_target is not None:
            trace.append(out.position_target)
            x, y, z = out.position_target
            if follow_xy:
                ctx.local_position_xy = (x, y)
            if follow_z:
                ctx.local_position_z = z
        t += DT
    return out, t, trace


def port_world(a, ctx):
    """当前投放口世界位置（第一个投放口 = 相机偏置 + 杆臂）。"""
    off = (a.cfg.camera_offset_m[0] + a.state.lever[0], a.cfg.camera_offset_m[1] + a.state.lever[1])
    c, s = math.cos(ctx.yaw_rad), math.sin(ctx.yaw_rad)
    return (ctx.local_position_xy[0] + off[0] * c - off[1] * s, ctx.local_position_xy[1] + off[0] * s + off[1] * c)


def test_full_descent_drops_with_port_over_target():
    a, ctx = make()
    out, _, trace = fly(a, ctx, 40.0)
    assert out.drop_ready
    assert a.state.phase == 'terminal' and out.velocity_target is None
    assert out.speed_mps == a.cfg.v_max_mps
    assert abs(-ctx.local_position_z - a.cfg.drop_alt_m) < 1e-6
    px, py = port_world(a, ctx)
    assert math.hypot(px - TARGET[0], py - TARGET[1]) < 0.005
    hs = [-z for _, _, z in trace]
    assert all(h2 <= h1 + 1e-9 for h1, h2 in zip(hs, hs[1:]))   # 一直在跟踪：只降不升


def test_port_offset_rotates_with_yaw():
    a, ctx = make()
    ctx.yaw_rad = math.pi / 2  # 机头朝起飞系 +y
    out, _, _ = fly(a, ctx, 40.0)
    assert out.drop_ready
    px, py = port_world(a, ctx)
    assert math.hypot(px - TARGET[0], py - TARGET[1]) < 0.005


def test_median_estimate_ignores_outliers():
    a, ctx = make()
    outlier = (TARGET[0] + 0.3, TARGET[1])  # 门内但偏 0.3m，每 4 帧一次
    fly(a, ctx, 15.0, dets=lambda i: [det(*outlier)] if i % 4 == 3 else [det(*TARGET)])
    assert a.state.phase == 'terminal'
    assert math.hypot(a.state.est[0] - TARGET[0], a.state.est[1] - TARGET[1]) < 0.01


def test_holds_altitude_while_fc_lags_outside_cone():
    a, ctx = make()
    _, t, _ = fly(a, ctx, 3.0)             # 交接进末端
    assert a.state.phase == 'terminal'
    ctx.local_position_xy = (TARGET[0] + 1.0, TARGET[1])  # 飞控还在 1m 外（锥面外）
    h0 = -ctx.local_position_z
    out, _, trace = fly(a, ctx, 3.0, t=t, follow_xy=False)
    assert out.give_up is None and len(trace) == 30
    assert abs(a.state.h_cmd - h0) < 1e-6 and abs(-ctx.local_position_z - h0) < 1e-6


def test_persistent_loss_climbs_over_estimate_then_back_to_coarse():
    a, ctx = make()
    out, t, _ = fly(a, ctx, 3.5)           # 降到 ~1.6m（还没到投放高度）
    assert not out.drop_ready
    assert a.state.phase == 'terminal' and -ctx.local_position_z < 2.0
    est = a.state.est
    out, _, trace = fly(a, ctx, 3.0, t=t, dets=lambda i: [])   # 爬 1.6→1.9m 约 1s + 短丢 0.5s
    assert out.give_up is None
    assert a.state.phase == 'coarse' and a.state.reacquires == 1
    climb = trace[a.cfg.max_coast_ticks + 1:]
    assert climb and all(-z2 >= -z1 - 1e-9 for (_, _, z1), (_, _, z2) in zip(climb, climb[1:]))
    assert (a.state.locked.x, a.state.locked.y) == est  # 回粗段锁在最后估计上


def test_endgame_drops_when_lost_after_converging_at_drop_alt():
    a, ctx = make()
    out, t, _ = fly(a, ctx, 40.0, dets=lambda i: [det(*TARGET)] if -ctx.local_position_z > 1.05 else [])
    assert out.drop_ready and a.state.mode == 'endgame'


def test_terminal_timeout_when_fc_never_converges():
    a, ctx = make()
    _, t, _ = fly(a, ctx, 3.0)
    ctx.local_position_xy = (TARGET[0] + 1.0, TARGET[1])
    out, t_end, _ = fly(a, ctx, a.cfg.terminal_timeout_s + 5.0, t=t, follow_xy=False)
    assert out.give_up[0] == 'TERMINAL_TIMEOUT'
    assert a.cfg.terminal_timeout_s - 3.5 < t_end - t < a.cfg.terminal_timeout_s  # 从交接起算（交接在前 3s 里）


def test_selection_and_variant_overlay():
    base = approaches.create_active('sim', name='cone_descent_ibvs')
    low = approaches.create_active('sim', name='cone_descent_ibvs', variant='lowgain')
    assert (low.cfg.kp, low.cfg.ki, low.cfg.kd) == (0.6, 0.1, 0.15)
    assert low.cfg.refine_gate_m == base.cfg.refine_gate_m and low.cfg.v_max_mps == base.cfg.v_max_mps
    lock = approaches.create_active('sim', name='position_lock_descent')
    assert lock.cfg.refine_gate_m == base.cfg.refine_gate_m   # 粗段/失效参数沿用 cone 同档
    assert lock.cfg.camera_offset_m == (0.11, 0.0)
    assert lock.cfg.max_coast_ticks > base.cfg.max_coast_ticks   # 位置锁定下短丢保持更久
    high = approaches.create_active('sim', name='position_lock_descent', variant='high')
    assert high.cfg.drop_alt_m == 1.4 and high.cfg.aim_migrate_bot_m > high.cfg.drop_alt_m + high.cfg.drop_alt_band_m
    assert high.cfg.reacquire_ceiling_m > high.cfg.drop_alt_m + 0.5
    ml.configure_approach('sim', 'position_lock_descent')
    assert ml._ACTIVE_APPROACH.name == 'position_lock_descent'
    ml.configure_approach('sim')
    assert ml._ACTIVE_APPROACH.name == approaches.ACTIVE


def test_mission_tick_caps_speed_in_terminal_only(monkeypatch):
    clock = NS(t=1000.0)
    monkeypatch.setattr(ml.time, 'monotonic', lambda: clock.t)
    ml.configure_approach('sim', 'position_lock_descent')
    try:
        a = ml._ACTIVE_APPROACH
        ctx = ml.MissionContext(current_state=ml.State.APPROACH_DROP_TARGET, state_entered_monotonic=clock.t)
        ctx.armed, ctx.mode, ctx.pi_in_control = True, 'GUIDED', True
        ctx.local_position_xy, ctx.local_position_z = (TARGET[0] + 0.8, TARGET[1]), -a.cfg.approach_alt_m
        ctx.horizontal_speed = ctx.vertical_speed = 0.0
        ctx.roll_rad = ctx.pitch_rad = ctx.yaw_rad = 0.0
        ctx.drop_task.phase_entered_monotonic = clock.t
        ctx.drop_task.selected_target = Candidate(*TARGET, 0.1)
        ctx.detections = [det(*TARGET, bearing=(0.0, 0.0))]
        a.enter(ctx)
        ctx.local_position_xy = (TARGET[0] + 1.5, TARGET[1])   # 还没到点：粗段
        r = ml.tick(ctx)
        assert a.state.phase == 'coarse' and r.speed_target_mps == ml._TUNING.search_speed_mps
        ctx.local_position_xy = TARGET                          # 到点、低速、在高度：交接
        ml.tick(ctx)
        clock.t += 0.1
        r = ml.tick(ctx)
        assert a.state.phase == 'terminal' and r.position_target is not None
        assert r.speed_target_mps == a.cfg.v_max_mps
    finally:
        ml.configure_approach('sim')
