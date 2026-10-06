"""侦察覆盖（surveys/）的单测（纯 Python，不起 ROS）：覆盖扫描的几何保证、航点推进与结束、原检测计数
逻辑行为不变、任务 FSM 的 NAVIGATE -> SURVEY -> RTL 交接。"""

import itertools
import math
from types import SimpleNamespace as NS

import cuadc_physical
import pytest

from cuadc_decision import mission_logic as ml
from cuadc_decision import search_strategy
from cuadc_decision import surveys
from cuadc_decision.mission import config as mc
from cuadc_decision.surveys import coverage_scan

S = ml.State


def setup(profile='sim', scenario='full_mission', name=None):
    world = mc.load(profile, scenario)
    tuning = ml.load_tuning(profile)
    survey = surveys.create_active(world, tuning, profile, name)
    ml.configure_mission(world, tuning, search_strategy.load_config(profile), survey)
    return world, tuning, survey


def ctx_at(xy, h, state=S.SURVEY_RECON_ZONE, detections=()):
    ctx = ml.MissionContext(current_state=state)
    ctx.armed, ctx.mode, ctx.pi_in_control = True, 'GUIDED', True
    ctx.local_position_xy, ctx.local_position_z = xy, -h
    ctx.detections = list(detections)
    return ctx


def camera_height(cfg):
    af = cuadc_physical.load_airframe()
    return cfg.scan_alt_m + af.fc_rest_clearance_m - af.camera_below_fc_m


def camera_track(survey):
    """飞控航点 + 相机在机体上的前/右偏置（航向锁在起飞航向）= 相机经过的点。"""
    cf, cr, _ = cuadc_physical.load_airframe().camera_position_m
    return [(x + cf, y + cr) for x, y in survey._waypoints]


def lane_segments(survey):
    wps = camera_track(survey)
    return [(wps[i], wps[i + 1]) for i in range(0, len(wps), 2)]


def dist_to_segment(p, a, b):
    ax, ay = a
    bx, by = b
    t = ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / max((bx - ax) ** 2 + (by - ay) ** 2, 1e-12)
    t = min(1.0, max(0.0, t))
    return math.hypot(p[0] - (ax + t * (bx - ax)), p[1] - (ay + t * (by - ay)))


@pytest.mark.parametrize('profile, scenario', [
    ('sim', 'full_mission'), ('real', 'full_mission'), ('sim', 'recon_zone_bench'), ('real', 'recon_zone_bench')])
def test_every_tube_position_passes_under_a_scan_line(profile, scenario):
    world, _, survey = setup(profile, scenario)
    cfg = coverage_scan.load_config(profile)
    max_offset = cfg.view_offset_ratio * camera_height(cfg)
    cx, cy = world.recon_zone_center
    L, W = world.recon_zone_size
    m = cfg.edge_margin_m
    worst = 0.0
    for fx, fy in itertools.product(
            [cx - L / 2 + m + i * (L - 2 * m) / 40 for i in range(41)],
            [cy - W / 2 + m + j * (W - 2 * m) / 40 for j in range(41)]):
        p = mc.field_to_takeoff(fx, fy, world.takeoff_point)
        worst = max(worst, min(dist_to_segment(p, a, b) for a, b in lane_segments(survey)))
    assert worst <= max_offset + 1e-9


def test_default_rulebook_plan_and_fits_phase_budget():
    _, tuning, survey = setup('sim')
    cfg = coverage_scan.load_config('sim')
    assert survey.lanes == 4                        # 扫描线沿 8m 横边（比沿 5m 纵边航程短）
    assert survey.spacing_m == pytest.approx(1.2)
    assert survey.path_m == pytest.approx(4 * 7.8 + 3 * 1.2)
    # 航程/速度 + 每个端点约 3s 的减速拐弯，留在侦察阶段超时之内
    assert survey.path_m / cfg.scan_speed_mps + 3.0 * len(survey._waypoints) < tuning.recon_phase_timeout_s


def test_scan_starts_at_corner_nearest_takeoff_and_snakes():
    world, _, survey = setup('sim', 'recon_zone_bench')
    wps = camera_track(survey)
    x0 = 55.0 + 0.1 + 4.8 / 8 - world.takeoff_point[0]   # 最靠近起飞点的那条扫描线（相机）
    assert wps[0][0] == pytest.approx(x0)
    assert abs(wps[0][1]) == pytest.approx(3.9)
    for i in range(0, len(wps), 2):                      # 每条线横贯整个筒心范围，方向交替
        assert wps[i][0] == pytest.approx(wps[i + 1][0])
        assert wps[i][1] == pytest.approx(-wps[i + 1][1])
    assert [wps[i][1] for i in range(0, len(wps), 2)] == [wps[0][1], -wps[0][1]] * (len(wps) // 4)


def test_coverage_scan_flies_waypoints_slowly_then_finishes():
    _, _, survey = setup('sim', 'recon_zone_bench')
    cfg = coverage_scan.load_config('sim')
    survey.enter(None)
    for wp in survey._waypoints:
        r = ml.tick(ctx_at((0.0, 0.0), cfg.scan_alt_m))
        assert r.position_target == pytest.approx((wp[0], wp[1], -cfg.scan_alt_m))
        assert r.speed_target_mps == cfg.scan_speed_mps
        r = ml.tick(ctx_at(wp, cfg.scan_alt_m))
    assert r.next_state == S.RTL_OR_LAND


def test_coverage_scan_ignores_detections_and_needs_scan_altitude():
    _, _, survey = setup('sim', 'recon_zone_bench')
    survey.enter(None)
    wp0 = survey._waypoints[0]
    many = [NS(target_type='cylinder', confidence=0.9, position=NS(x=float(i), y=0.0)) for i in range(5)]
    r = ml.tick(ctx_at(wp0, 5.0, detections=many))      # 到点但高度不对：不推进，也不因看到 5 个筒结束
    assert r.next_state is None and r.position_target[:2] == pytest.approx(wp0)


def test_navigate_flies_centerline_to_survey_entry_x_and_hands_over():
    world, _, survey = setup('sim', 'recon_zone_bench')
    entry = survey.entry_point()
    centerline_y = world.recon_zone_center[1] - world.takeoff_point[1]
    r = ml.tick(ctx_at((0.0, 0.0), world.cruise_alt_m, state=S.NAVIGATE_TO_RECON_ZONE))
    assert r.position_target == pytest.approx((entry[0], centerline_y, -world.cruise_alt_m))  # 沿中线进区
    survey._index = 3
    r = ml.tick(ctx_at((entry[0], centerline_y), world.cruise_alt_m, state=S.NAVIGATE_TO_RECON_ZONE))
    assert r.next_state == S.SURVEY_RECON_ZONE
    assert survey.progress() == f'waypoint 0/{len(survey._waypoints)}'   # enter() 清了进度
    r = ml.tick(ctx_at((entry[0], centerline_y), world.cruise_alt_m))  # 进区后横移到扫描起点
    assert r.position_target[:2] == pytest.approx(entry)


def test_recon_timeout_reports_survey_progress(monkeypatch):
    _, tuning, survey = setup('sim', 'recon_zone_bench')
    ctx = ctx_at((0.0, 0.0), 3.0)
    ctx.recon_task.phase_entered_monotonic = 1000.0
    monkeypatch.setattr(ml.time, 'monotonic', lambda: 1000.0 + tuning.recon_phase_timeout_s + 1.0)
    r = ml.tick(ctx)
    assert r.next_state == S.RTL_OR_LAND and r.fault.code == 'RECON_PHASE_TIMEOUT'
    assert survey.progress() in r.fault.reason


# ---- 原检测计数逻辑（survey:=detect_count）行为不变 ----

def test_detect_count_original_behaviour():
    world, tuning, survey = setup('sim', name='detect_count')
    cx, cy = world.recon_zone_center
    wps = [mc.field_to_takeoff(cx + dx, cy + dy, world.takeoff_point) for dx, dy in world.recon_scan_offsets]
    assert survey.entry_point() == wps[0]
    survey.enter(None)
    r = ml.tick(ctx_at(wps[0], world.search_alt_m))          # 到第一个航点 -> 指向第二个
    assert r.position_target == pytest.approx((wps[1][0], wps[1][1], -world.search_alt_m))
    assert r.speed_target_mps == tuning.search_speed_mps
    near = tuning.recon_coverage_match_m * 0.5
    dets = [NS(target_type='cylinder', position=NS(x=float(2 * i), y=0.0)) for i in range(4)]
    dets.append(NS(target_type='cylinder', position=NS(x=near, y=0.0)))   # 跟第一个是同一个筒
    assert ml.tick(ctx_at((0.0, 0.0), 3.0, detections=dets)).next_state is None
    assert survey.progress() == f'covered 4/{world.recon_cylinder_count}'
    r = ml.tick(ctx_at((0.0, 0.0), 3.0, detections=[NS(target_type='cylinder', position=NS(x=20.0, y=0.0))]))
    assert r.next_state == S.RTL_OR_LAND
