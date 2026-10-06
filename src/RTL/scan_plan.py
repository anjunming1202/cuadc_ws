"""按相机视场把「地面 5×5 m」收成更短的扫描航点。

正下视时，5 m 高一帧大约能盖住约 5.8×3.3 m（960×540、fx/fy 见 rtl.yaml），
所以不必真飞满 5 m 边长：窄边（机头方向，对应图像高度）才需要两三个点。
本文件纯几何，不订 ROS、不取帧。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

_CFG_PATH = Path(__file__).parent / 'rtl.yaml'


@dataclass
class RtlConfig:
    scan_alt_m: float = 5.0
    cover_m: float = 5.0
    overlap: float = 0.25
    home_tol_m: float = 0.5
    arrive_tol_m: float = 0.6
    alt_tol_m: float = 0.6
    dwell_s: float = 2.0
    leg_timeout_s: float = 25.0
    home_timeout_s: float = 90.0
    outlier_m: float = 1.5
    cv_width: int = 960
    cv_height: int = 540
    fx: float = 869.37
    fy: float = 871.03
    fc_rest_clearance_m: float = 0.31
    camera_down_m: float = 0.20
    pad_radius_m: float = 0.4
    lock_xy_m: float = 0.20
    lock_hold_s: float = 0.5      # 连续锁住这么久才开始降
    descent_mps: float = 0.35
    h_lead_m: float = 0.40
    hand_off_alt_m: float = 3.0   # GUIDED 降到这切 LAND，不查水平
    land_timeout_s: float = 45.0
    goto_timeout_s: float = 10.0  # 对不准则边降边校


def load_rtl_config(path: Path | None = None) -> RtlConfig:
    path = path or _CFG_PATH
    data: dict = {}
    with open(path, encoding='utf-8') as fh:
        for raw in fh:
            line = raw.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, val = line.split(':', 1)
            key, val = key.strip(), val.strip()
            if '.' in val:
                data[key] = float(val)
            else:
                data[key] = int(val)
    return RtlConfig(**data)


def camera_agl_m(cfg: RtlConfig) -> float:
    """悬停在 scan_alt 时镜头离地：飞控相对解锁点的高度 + 停机离地 − 相机在 FC 下方。"""
    return cfg.scan_alt_m + cfg.fc_rest_clearance_m - cfg.camera_down_m


def ground_footprint_m(cfg: RtlConfig) -> Tuple[float, float]:
    """一帧正下视在地面上的宽×高（米）。x=前（图像高）、y=右（图像宽）；mount yaw=0。"""
    h = camera_agl_m(cfg)
    fwd = h * cfg.cv_height / cfg.fy   # 全高
    right = h * cfg.cv_width / cfg.fx
    return (fwd, right)


def _centers(span: float, full_fov: float, overlap: float) -> List[float]:
    """用长度为 full_fov 的覆盖条铺满 [-span/2, span/2]，返回条带中心。"""
    half = 0.5 * full_fov
    if full_fov >= span:
        return [0.0]
    usable = full_fov * (1.0 - overlap)
    n = max(2, int(math.ceil(span / usable)))
    lo, hi = -0.5 * span + half, 0.5 * span - half
    if n == 1 or abs(hi - lo) < 1e-6:
        return [0.0]
    return [lo + i * (hi - lo) / (n - 1) for i in range(n)]


def scan_waypoints(cfg: RtlConfig | None = None) -> List[Tuple[float, float]]:
    """起飞系扫描航点 (前, 右)，蛇形，中心在原点。

    视场已经盖住 cover_m 的那一轴只留 0；另一轴按重叠抽稀。
    """
    cfg = cfg or load_rtl_config()
    fwd, right = ground_footprint_m(cfg)
    xs = _centers(cfg.cover_m, fwd, cfg.overlap)
    ys = _centers(cfg.cover_m, right, cfg.overlap)
    wps: List[Tuple[float, float]] = []
    for i, x in enumerate(xs):
        col = ys if i % 2 == 0 else list(reversed(ys))
        for y in col:
            wps.append((float(x), float(y)))
    return wps
