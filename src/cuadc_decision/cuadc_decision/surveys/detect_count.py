"""原侦察覆盖逻辑（检测计数）：按 mission 层的 recon_scan_offsets 在 search_alt_m 飞扫描航点，把看到的
筒按位置去重记成「已覆盖」，凑够 recon_cylinder_count 个即结束；扫完最后一个航点就悬停，等侦察阶段超时。
依赖感知检出侦察筒——color_pad 只认蓝垫上的白筒，侦察筒检不到（见 docs/spec/failure-handling.md）。
参数沿用原处：扫描样式/高度/筒数在 mission 层，去重距离在 fsm_tuning，本模块没有自己的 yaml。"""

import math
from dataclasses import dataclass

from cuadc_decision.mission.config import field_to_takeoff
from cuadc_decision.surveys.base import SurveyOutput, at_point

NAME = 'detect_count'


@dataclass
class SurveyConfig:
    """无自有参数（见文件头）。"""


def load_config(profile: str) -> SurveyConfig:
    return SurveyConfig()


def create(config, world, tuning):
    return DetectCount(world, tuning)


class DetectCount:
    name = NAME

    def __init__(self, world, tuning):
        self._world, self._tuning = world, tuning
        cx, cy = world.recon_zone_center
        self._waypoints = [field_to_takeoff(cx + dx, cy + dy, world.takeoff_point)
                           for dx, dy in world.recon_scan_offsets]
        self.enter(None)

    def entry_point(self):
        return self._waypoints[0]

    def enter(self, ctx):
        self._covered = []
        self._index = 0

    def progress(self):
        return f'covered {len(self._covered)}/{self._world.recon_cylinder_count}'

    def describe(self):
        return (f'{len(self._waypoints)} waypoints @{self._world.search_alt_m}m, '
                f'until {self._world.recon_cylinder_count} tubes seen')

    def loop(self, ctx):
        for d in ctx.detections:
            if d.target_type != 'cylinder':
                continue
            if not any(math.hypot(d.position.x - px, d.position.y - py) < self._tuning.recon_coverage_match_m
                       for px, py in self._covered):
                self._covered.append((d.position.x, d.position.y))

        if len(self._covered) >= self._world.recon_cylinder_count:
            return SurveyOutput(done=True)

        alt = self._world.search_alt_m
        index = min(self._index, len(self._waypoints) - 1)
        waypoint = self._waypoints[index]
        if at_point(ctx, waypoint, alt, self._tuning.position_tolerance_m, self._tuning.altitude_tolerance_m):
            if index < len(self._waypoints) - 1:
                self._index = index + 1
                waypoint = self._waypoints[self._index]
        return SurveyOutput(position_target=(waypoint[0], waypoint[1], -alt))
