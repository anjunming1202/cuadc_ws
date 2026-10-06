"""覆盖扫描：不靠检测，在侦察区上空按蛇形扫描线慢速飞一遍，区内任何筒位都会从近正上方经过一次，给图传
看清筒里的标识；飞完即结束，不数看到几个筒。

几何：筒壁（高 0.15m）会挡住筒底，相机偏离筒正上方超过约 view_offset_ratio × 相机离地高 时筒里的标识
就被挡掉一部分。扫描线间距取不超过 2 × 这个偏离量、在筒心可能出现的范围里均分，任意筒心离最近一条扫描线
都不超过它。扫描线沿区的两条边各算一遍，取总航程短的；从离起飞点最近的角开始。"""

import math
from dataclasses import dataclass
from pathlib import Path

import cuadc_physical
import yaml

from cuadc_decision.mission.config import field_to_takeoff
from cuadc_decision.surveys.base import SurveyOutput, at_point

NAME = 'coverage_scan'


@dataclass
class SurveyConfig:
    """字段名 = `coverage_scan.<profile>.yaml` 的键。默认值只是兜底（纯单测/import 期不读文件）。"""
    scan_alt_m: float = 3.0
    scan_speed_mps: float = 1.0
    view_offset_ratio: float = 0.2
    edge_margin_m: float = 0.1
    arrive_tol_m: float = 0.3
    alt_tol_m: float = 0.3


def load_config(profile: str) -> SurveyConfig:
    """从挨着本文件的 `coverage_scan.<profile>.yaml` 读配置。"""
    with open(Path(__file__).parent / f'{NAME}.{profile}.yaml') as fh:
        return SurveyConfig(**(yaml.safe_load(fh) or {}))


def create(config, world, tuning):
    return CoverageScan(config, world)


def plan_waypoints(center, size, max_spacing, margin, origin):
    """场地坐标下的蛇形扫描线端点。center/size = 区中心与 (纵 x, 横 y) 尺寸，margin = 筒心离区边界的最小距离，
    origin = 起飞点。返回 (航程, 航点列表, 扫描线条数, 实际间距)。"""
    cx, cy = center
    ranges = ((cx - size[0] / 2 + margin, cx + size[0] / 2 - margin),
              (cy - size[1] / 2 + margin, cy + size[1] / 2 - margin))
    best = None
    for step in (0, 1):  # 0：扫描线沿 y、按 x 步进；1：沿 x、按 y 步进
        along = 1 - step
        lo, hi = ranges[step]
        n = max(1, math.ceil((hi - lo) / max_spacing - 1e-9))
        lanes = [lo + (i + 0.5) * (hi - lo) / n for i in range(n)]
        if abs(lanes[-1] - origin[step]) < abs(lanes[0] - origin[step]):
            lanes.reverse()
        ends = list(ranges[along])
        if abs(ends[1] - origin[along]) < abs(ends[0] - origin[along]):
            ends.reverse()
        points = []
        for i, s in enumerate(lanes):
            for a in (ends if i % 2 == 0 else ends[::-1]):
                points.append((s, a) if step == 0 else (a, s))
        length = sum(math.dist(p, q) for p, q in zip(points, points[1:]))
        if best is None or length < best[0]:
            best = (length, points, n, (hi - lo) / n)
    return best


class CoverageScan:
    name = NAME

    def __init__(self, config, world):
        self._cfg = config
        af = cuadc_physical.load_airframe()
        camera_height = config.scan_alt_m + af.fc_rest_clearance_m - af.camera_below_fc_m
        self.path_m, points, self.lanes, self.spacing_m = plan_waypoints(
            world.recon_zone_center, world.recon_zone_size, 2 * config.view_offset_ratio * camera_height,
            config.edge_margin_m, world.takeoff_point)
        # 扫描线规划的是相机的轨迹；航向锁在起飞航向，飞控目标 = 相机目标 − 相机在机体上的前/右偏置。
        cam_fwd, cam_right = af.camera_position_m[0], af.camera_position_m[1]
        self._waypoints = [(tx - cam_fwd, ty - cam_right)
                           for tx, ty in (field_to_takeoff(x, y, world.takeoff_point) for x, y in points)]
        self.enter(None)

    def entry_point(self):
        return self._waypoints[0]

    def enter(self, ctx):
        self._index = 0

    def progress(self):
        return f'waypoint {self._index}/{len(self._waypoints)}'

    def describe(self):
        return (f'{self.lanes} lanes, spacing {self.spacing_m:.2f}m @{self._cfg.scan_alt_m}m, '
                f'path {self.path_m:.1f}m @{self._cfg.scan_speed_mps}m/s')

    def loop(self, ctx):
        if self._index < len(self._waypoints) and at_point(
                ctx, self._waypoints[self._index], self._cfg.scan_alt_m, self._cfg.arrive_tol_m, self._cfg.alt_tol_m):
            self._index += 1
        if self._index >= len(self._waypoints):
            return SurveyOutput(done=True)
        x, y = self._waypoints[self._index]
        return SurveyOutput(position_target=(x, y, -self._cfg.scan_alt_m), speed_mps=self._cfg.scan_speed_mps)
