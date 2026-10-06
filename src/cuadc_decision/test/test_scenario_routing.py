"""场景 phases 决定起飞后去哪一区（纯 Python，不起 ROS）：单练场景照样从 INIT 走放行 + 飞手解锁，
只靠 phases 跳过不跑的阶段，见 docs/spec/mission-config-layer.md §5。"""

import pytest

from cuadc_decision import mission_logic as ml
from cuadc_decision import search_strategy
from cuadc_decision.mission import config as mc

S = ml.State


def configure(profile, scenario):
    world = mc.load(profile, scenario)
    ml.configure_mission(world, ml.load_tuning(profile), search_strategy.load_config(profile))
    return world


def reached_cruise_ctx(world):
    ctx = ml.MissionContext(current_state=S.TAKEOFF)
    ctx.armed, ctx.mode, ctx.pi_in_control = True, 'GUIDED', True
    ctx.takeoff_step = 'wait_altitude'
    ctx.local_position_xy, ctx.local_position_z = (0.0, 0.0), -world.cruise_alt_m
    return ctx


@pytest.mark.parametrize('profile', ['sim', 'real'])
@pytest.mark.parametrize('scenario, first_zone', [
    ('full_mission', S.NAVIGATE_TO_DROP_ZONE),
    ('drop_zone_bench', S.NAVIGATE_TO_DROP_ZONE),
    ('recon_zone_bench', S.NAVIGATE_TO_RECON_ZONE),
])
def test_takeoff_goes_to_first_phase(profile, scenario, first_zone):
    world = configure(profile, scenario)
    assert world.initial_state == S.INIT
    assert ml.tick(reached_cruise_ctx(world)).next_state == first_zone


@pytest.mark.parametrize('profile, takeoff_point, center', [
    ('sim', (54.0, 0.0), (57.5, 0.0)),    # 近侦察区出生世界
    ('real', (0.0, 0.0), (4.0, 0.0)),     # 现场摆筒：区中心在机头前方
])
def test_recon_bench_scan_starts_near_takeoff(profile, takeoff_point, center):
    world = configure(profile, 'recon_zone_bench')
    assert tuple(world.takeoff_point) == takeoff_point
    assert tuple(world.recon_zone_center) == center
    ctx = reached_cruise_ctx(world)
    ctx.current_state = S.NAVIGATE_TO_RECON_ZONE
    r = ml.tick(ctx)
    entry = r.position_target[:2]
    half = (world.recon_zone_size[0] / 2, world.recon_zone_size[1] / 2)
    assert abs(entry[0] + takeoff_point[0] - center[0]) <= half[0]   # 入口在侦察区内
    assert abs(entry[1] + takeoff_point[1] - center[1]) <= half[1]
    assert max(abs(v) for v in entry) < 6.0                          # 单练入口就在起飞点附近
