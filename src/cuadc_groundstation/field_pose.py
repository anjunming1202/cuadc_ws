#!/usr/bin/env python3
"""Ground-side field pose: FC NED/GPS → 起飞坐标系（与任务同一定义，见 docs/spec/coordinate-frames.md）。

Does not run on the Pi. 每次解锁上升沿锁原点和机头；上锁后保持，下次解锁重锁。
"""
from __future__ import annotations

import math
import time
from typing import Any

_M_PER_DEG_LAT = 111320.0  # 纬度 1° 约米数；经度再乘 cos(lat)


def is_guided(mode: Any) -> bool:
    """mode 为 GUIDED（大小写不敏感，不含 GUIDED_NOGPS）。"""
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
    """NED 增量转到起飞系 (前, 右)，与 interface_node._ned_to_takeoff 相同。"""
    c, s = math.cos(yaw0_rad), math.sin(yaw0_rad)
    return dn_m * c + de_m * s, -dn_m * s + de_m * c


def gps_delta_to_ned(lat_deg: float, lon_deg: float, lat0: float, lon0: float) -> tuple[float, float]:
    """相对原点的北/东位移（米）。无 NED 时的兜底。"""
    dn = (lat_deg - lat0) * _M_PER_DEG_LAT
    de = (lon_deg - lon0) * _M_PER_DEG_LAT * math.cos(math.radians(lat0))
    return dn, de


class FieldPoseTracker:
    """解锁上升沿锁原点；从未解锁时读数 ≈ 0（钉在 H）。"""

    def __init__(self) -> None:
        self.latched = False
        self._was_armed = False
        self._n0: float | None = None
        self._e0: float | None = None
        self._lat0: float | None = None
        self._lon0: float | None = None
        self._yaw0_rad: float = 0.0
        self._last: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def update(self, telem: dict[str, Any]) -> dict[str, Any]:
        """Return overlay fields; does not mutate telem."""
        ned_n = _num(telem.get("ned_n"))
        ned_e = _num(telem.get("ned_e"))
        lat = _num(telem.get("lat_deg"))
        lon = _num(telem.get("lon_deg"))
        yaw_deg = _num(telem.get("yaw_deg"))
        yaw_rad = None if yaw_deg is None else math.radians(yaw_deg)
        has_ned = ned_n is not None and ned_e is not None
        has_gps = lat is not None and lon is not None
        # 老版 telem_ground 不发 armed：用 GUIDED 近似
        armed = bool(telem["armed"]) if "armed" in telem else is_guided(telem.get("mode"))

        if armed and not self._was_armed and (has_ned or has_gps):
            self._n0, self._e0 = (ned_n, ned_e) if has_ned else (None, None)
            self._lat0, self._lon0 = (lat, lon) if has_gps else (None, None)
            self._yaw0_rad = 0.0 if yaw_rad is None else yaw_rad
            self.latched = True
        self._was_armed = armed and self.latched

        if not self.latched:
            return {
                "field_x_m": 0.0,
                "field_y_m": 0.0,
                "field_yaw_deg": 0.0,
                "field_latched": False,
                "field_src": "pre-arm",
            }

        src = "ned"
        if has_ned and self._n0 is not None:
            dn, de = ned_n - self._n0, ned_e - self._e0
        elif has_gps and self._lat0 is not None:
            dn, de = gps_delta_to_ned(lat, lon, self._lat0, self._lon0)
            src = "gps"
        else:
            x, y, yaw = self._last
            return {"field_x_m": x, "field_y_m": y, "field_yaw_deg": yaw,
                    "field_latched": True, "field_src": "hold"}

        fx, fy = ned_delta_to_field(dn, de, self._yaw0_rad)
        heading = 0.0 if yaw_rad is None else math.degrees(yaw_rad - self._yaw0_rad)
        out = {
            "field_x_m": round(fx, 2),
            "field_y_m": round(fy, 2),
            "field_yaw_deg": round(heading % 360.0, 1),  # 向右转为正
            "field_latched": True,
            "field_src": src,
        }
        self._last = (out["field_x_m"], out["field_y_m"], out["field_yaw_deg"])
        return out


class AltitudeTrace:
    """最近 window_s 秒的高度采样，给地面折线图。与是否解锁无关。"""

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
    assert not t.update({"armed": False, "ned_n": 5, "ned_e": 0, "yaw_deg": 90})["field_latched"]
    a0 = t.update({"armed": True, "ned_n": 5, "ned_e": 0, "yaw_deg": 90})  # 机头朝东
    assert a0["field_x_m"] == 0 and a0["field_latched"]
    a1 = t.update({"armed": True, "ned_n": 4, "ned_e": 10, "yaw_deg": 90})
    assert (a1["field_x_m"], a1["field_y_m"]) == (10.0, 1.0)  # 向东 10 = 前；向南 1 = 右
    rtl = t.update({"armed": True, "mode": "RTL", "ned_n": 5, "ned_e": 3, "yaw_deg": 90})
    assert rtl["field_x_m"] == 3.0  # 换模式不重锁
    landed = t.update({"armed": False, "ned_n": 5, "ned_e": 1, "yaw_deg": 90})
    assert landed["field_x_m"] == 1.0 and landed["field_latched"]  # 上锁后保持
    b0 = t.update({"armed": True, "ned_n": 5, "ned_e": 1, "yaw_deg": 0})
    assert (b0["field_x_m"], b0["field_y_m"]) == (0.0, 0.0)  # 再次解锁重锁
    old = FieldPoseTracker()  # 无 armed 字段：按 GUIDED 近似
    old.update({"mode": "GUIDED", "ned_n": 0, "ned_e": 0, "yaw_deg": 0})
    assert old.update({"mode": "GUIDED", "ned_n": 2, "ned_e": -1, "yaw_deg": 0})["field_y_m"] == -1.0
    print("field_pose self-check ok")
