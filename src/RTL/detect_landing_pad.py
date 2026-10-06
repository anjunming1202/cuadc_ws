"""起降点（蓝底白 H 圆垫）的图像检测。

本文件只做像素结果，不飞、不订 ROS。后面巡回扫描会反复调用
`find_landing_pad_center`，再用 `landing_pad_to_takeoff_xy` 把圆心反投影到
起飞坐标系（前/右，与投放区 Detection.position 同一套，见
docs/spec/coordinate-frames.md）。

取帧（主脚本接上来时）：与投放区 OpenCV 同一路 10 fps **原始**流——
`camera_service` NV12 shm（`ShmFrameSource` / `/tmp/cuadc-cam-raw.shm`），
图不进 ROS、也不走 5602 H.264 先编码再解码。本函数只吃已转好的 BGR 帧。

检测思路（刻意不看白色 H）：
  1. HSV 抠蓝（阈值与投放区蓝布垫相同）；
  2. 只取蓝 mask 的**外轮廓**（RETR_EXTERNAL），H 是垫上的洞，不进外轮廓；
  3. 面积 + 圆度滤噪点，多个候选取面积最大的那块；
  4. 用外轮廓矩当圆心（按外轮廓围成的实心圆算，H 几乎不拽心）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Tuple

import cv2
import numpy as np

_CFG_PATH = Path(__file__).parent / 'landing_pad.yaml'


@dataclass
class LandingPadConfig:
    blue_lo: Tuple[int, int, int] = (100, 80, 50)
    blue_hi: Tuple[int, int, int] = (130, 255, 255)
    proc_w: int = 960
    morph_k: int = 7
    min_area_px: int = 80
    min_circularity: float = 0.35


@dataclass
class LandingPadHit:
    """一帧里认定的起降垫。u/v/半径已换算回原图像素。"""
    u: float
    v: float
    radius_px: float
    circularity: float
    area_px: float
    contour: Any  # 原图坐标系的外轮廓，给调试画框用


def load_landing_pad_config(path: Path | None = None) -> LandingPadConfig:
    path = path or _CFG_PATH
    data: dict = {}
    with open(path, encoding='utf-8') as fh:
        for raw in fh:
            line = raw.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, val = line.split(':', 1)
            key, val = key.strip(), val.strip()
            if val.startswith('[') and val.endswith(']'):
                data[key] = tuple(int(x.strip()) for x in val[1:-1].split(',') if x.strip())
            elif '.' in val:
                data[key] = float(val)
            else:
                data[key] = int(val)
    return LandingPadConfig(**data)


def _morph(mask: np.ndarray, k: int) -> np.ndarray:
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ker)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ker)


def _centroid(contour) -> Optional[Tuple[float, float]]:
    m = cv2.moments(contour)
    if m['m00'] == 0:
        return None
    return (m['m10'] / m['m00'], m['m01'] / m['m00'])


def find_landing_pad_center(
    image_bgr: np.ndarray,
    cfg: Optional[LandingPadConfig] = None,
    debug: Optional[dict] = None,
) -> Optional[LandingPadHit]:
    """从一帧 BGR 图里找起降点蓝圆的圆心。

    帧应由投放区同款 10 fps shm 原流给出（已转 BGR），不要另开编码预览流再解码。
    返回原图像素 (u, v)；找不到或全被滤掉则 None。
    `debug` 若传入空 dict，会填 `mask`（蓝二值）和 `contour`（选中的外轮廓）。
    """
    cfg = cfg or load_landing_pad_config()
    h0, w0 = image_bgr.shape[:2]
    scale = max(1.0, w0 / float(cfg.proc_w))  # 只缩小，不放大
    if scale > 1.0:
        proc = cv2.resize(image_bgr, (int(round(w0 / scale)), int(round(h0 / scale))))
    else:
        proc = image_bgr

    hsv = cv2.cvtColor(proc, cv2.COLOR_BGR2HSV)
    blue = _morph(
        cv2.inRange(hsv, np.array(cfg.blue_lo), np.array(cfg.blue_hi)),
        cfg.morph_k,
    )
    # 只要最外圈：白色 H 是洞，不会当成独立轮廓
    cnts, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None  # (area, circ, cen, r_eq, contour_proc)
    for c in cnts:
        area = cv2.contourArea(c)
        if area < cfg.min_area_px:
            continue
        cen = _centroid(c)
        if cen is None:
            continue
        _, r_enc = cv2.minEnclosingCircle(c)
        circ = min(1.0, area / (math.pi * r_enc * r_enc)) if r_enc > 0 else 0.0
        if circ < cfg.min_circularity:
            continue
        if best is None or area > best[0]:
            r_eq = math.sqrt(area / math.pi)
            best = (area, circ, cen, r_eq, c)

    if debug is not None:
        debug['mask'] = blue
        debug['contour'] = None if best is None else best[4]

    if best is None:
        return None
    area, circ, (u, v), r_eq, c = best
    contour_full = (c.astype(np.float32) * scale).astype(np.int32) if scale != 1.0 else c
    return LandingPadHit(
        u=float(u * scale),
        v=float(v * scale),
        radius_px=float(r_eq * scale),
        circularity=float(circ),
        area_px=float(area * scale * scale),
        contour=contour_full,
    )


def landing_pad_to_takeoff_xy(hit: LandingPadHit, camera, pose) -> Optional[Tuple[float, float]]:
    """像素圆心 → 起飞坐标系地面 (x 前, y 右)。

    与投放区同一条反投影：射线从镜头光心出发（已含飞控到相机的 0.11/0.20 m），
    交地面得到垫心起飞系坐标。调用方应把飞控飞到这个点，不要再减相机偏置。
    """
    from cuadc_perception import geometry
    cam_to_world = geometry.cam_to_world_matrix(camera, pose)
    cam_origin = geometry.camera_origin(camera, pose)
    world_xy, _range = geometry.reproject_to_cylinder_top(
        hit.u, hit.v, camera, cam_origin, cam_to_world, cylinder_height_m=0.0)
    if world_xy is None:
        return None
    return (float(world_xy[0]), float(world_xy[1]))
