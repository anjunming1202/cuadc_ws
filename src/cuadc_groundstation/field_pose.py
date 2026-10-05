#!/usr/bin/env python3
"""Ground-side CUADC field pose: FC NED/GPS → 赛道平面 (x forward, y left).

Does not run on the Pi. Each GUIDED rising edge latches origin; leaving
GUIDED parks at H until the next GUIDED.
"""
from __future__ import annotations

import math
import time
from typing import Any

_M_PER_DEG_LAT = 111320.0  # 纬度 1° 约米数；经度再乘 cos(lat)


def is_guided(mode: Any) -> bool:
    """起飞判据：飞控 mode 变为 GUIDED（大小写不敏感，不含 GUIDED_NOGPS）。"""
    if mode is None:
        return False
    return str(mode).strip().upper() == "GUIDED"


def _num(v: Any) -> float | None:
    if isinstance(v, bool) or v is None:
        return None
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def ned_delta_to_field(dn_m: float, de_m: float, yaw0_rad: float) -> tuple[float, float]:
    """NED 增量转到赛道：+x 沿起飞机头，+y 为机头朝前时的左侧。"""
    c, s = math.cos(yaw0_rad), math.sin(yaw0_rad)
    fwd = dn_m * c + de_m * s  # 机头前方
    left = dn_m * s - de_m * c  # 机头左侧
    return fwd, left


def gps_delta_to_ned(lat_deg: float, lon_deg: float, lat0: float, lon0: float) -> tuple[float, float]:
    """相对原点的北/东位移（米）。无 NED 时的兜底。"""
    dn = (lat_deg - lat0) * _M_PER_DEG_LAT
    de = (lon_deg - lon0) * _M_PER_DEG_LAT * math.cos(math.radians(lat0))
    return dn, de


class FieldPoseTracker:
    """每次 GUIDED 上升沿锁原点；退出 GUIDED（RTL 等）钉回 H，下次再锁。"""

    def __init__(self) -> None:
        self.airborne = False  # 当前这一段 GUIDED 已锁过原点
        self._n0: float | None = None
        self._e0: float | None = None
        self._lat0: float | None = None
        self._lon0: float | None = None
        self._yaw0_rad: float = 0.0
        self._last: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def reset(self) -> None:
        self.__init__()

    def update(self, telem: dict[str, Any]) -> dict[str, Any]:
        """Return overlay fields; does not mutate telem."""
        parked = {
            "field_x_m": 0.0,
            "field_y_m": 0.0,
            "field_yaw_deg": 0.0,
            "field_airborne": False,
            "field_src": "pre-guided",
        }
        ned_n = _num(telem.get("ned_n"))
        ned_e = _num(telem.get("ned_e"))
        lat = _num(telem.get("lat_deg"))
        lon = _num(telem.get("lon_deg"))
        yaw_deg = _num(telem.get("yaw_deg"))
        yaw_rad = None if yaw_deg is None else math.radians(yaw_deg)
        has_ned = ned_n is not None and ned_e is not None
        has_gps = lat is not None and lon is not None
        guided = is_guided(telem.get("mode"))

        if self.airborne and not guided:
            self.reset()
            return parked

        if not self.airborne:
            if not guided or not (has_ned or has_gps):
                return parked
            # 本段 GUIDED 开始：当前点 = H，机头 = +x。
            if has_ned:
                self._n0, self._e0 = ned_n, ned_e
            if has_gps:
                self._lat0, self._lon0 = lat, lon
            self._yaw0_rad = 0.0 if yaw_rad is None else yaw_rad
            self.airborne = True  # 本帧相对原点为 0，画在 H

        src = "ned"
        if has_ned and self._n0 is not None:
            dn, de = ned_n - self._n0, ned_e - self._e0
        elif has_gps and self._lat0 is not None:
            dn, de = gps_delta_to_ned(lat, lon, self._lat0, self._lon0)
            src = "gps"
        else:
            return {
                "field_x_m": self._last[0],
                "field_y_m": self._last[1],
                "field_yaw_deg": self._last[2],
                "field_airborne": True,
                "field_src": "hold",
            }

        fx, fy = ned_delta_to_field(dn, de, self._yaw0_rad)
        heading = 0.0
        if yaw_rad is not None:
            heading = math.degrees(yaw_rad - self._yaw0_rad)
        out = {
            "field_x_m": round(fx, 2),
            "field_y_m": round(fy, 2),
            "field_yaw_deg": round(heading % 360.0, 1),
            "field_airborne": True,
            "field_src": src,
        }
        self._last = (out["field_x_m"], out["field_y_m"], out["field_yaw_deg"])
        return out


class AltitudeTrace:
    """最近 window_s 秒的高度采样，给地面折线图。与是否 GUIDED 无关。"""

    def __init__(self, window_s: float = 30.0) -> None:
        self.window_s = window_s
        self._pts: list[tuple[float, float]] = []  # (unix_s, alt_m)

    def update(self, telem: dict[str, Any]) -> None:
        alt = _num(telem.get("alt_m"))
        if alt is None:
            return
        t = _num(telem.get("t"))
        if t is None:
            t = time.time()
        self._pts.append((t, alt))
        cut = t - self.window_s
        self._pts = [p for p in self._pts if p[0] >= cut]

    def snapshot(self) -> list[dict[str, float]]:
        return [{"t": round(t, 3), "alt": round(a, 2)} for t, a in self._pts]


if __name__ == "__main__":
    t = FieldPoseTracker()
    assert t.update({"mode": "STABILIZE", "ned_n": 5, "ned_e": 0, "yaw_deg": 0})["field_x_m"] == 0
    g0 = t.update({"mode": "GUIDED", "ned_n": 5, "ned_e": 0, "yaw_deg": 0})
    assert g0["field_x_m"] == 0 and g0["field_airborne"]
    g1 = t.update({"mode": "GUIDED", "ned_n": 15, "ned_e": 0, "yaw_deg": 0})
    assert g1["field_x_m"] == 10.0
    rtl = t.update({"mode": "RTL", "ned_n": 20, "ned_e": 0, "yaw_deg": 0})
    assert rtl["field_x_m"] == 0 and not rtl["field_airborne"]
    g2 = t.update({"mode": "GUIDED", "ned_n": 20, "ned_e": 0, "yaw_deg": 0})
    assert g2["field_x_m"] == 0 and g2["field_airborne"]
    g3 = t.update({"mode": "GUIDED", "ned_n": 25, "ned_e": 0, "yaw_deg": 0})
    assert g3["field_x_m"] == 5.0
    print("field_pose self-check ok")
