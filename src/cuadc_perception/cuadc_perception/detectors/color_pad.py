"""颜色检测器:HSV 彩色分割——蓝色落点垫 + 垫中心白筒的"白盘套蓝盘"同心先验。
颜色比灰度/边缘抗糊、抗光照,是主攻思路;从 cv_explore/color_shape 沙盒移植过来
(见 cuadc_perception/README.md 检测器库约定 + 迁移组织规范)。

纯函数(numpy/cv2 输入输出,不依赖 rclpy,可脱离节点单测),只输出**图像空间**
观测(白筒像素中心 + 视在半径 px + 置信度);反投影、归一化方位、物理半径过滤
都在感知节点侧。

筒定位两路:①"洞法"——蓝 mask 里筒是被蓝垫包住的洞(RETR_CCOMP 子轮廓),几何
最稳;②白 mask 法——洞不闭合(垫被画面切开)时兜底。多目标:每个够大的蓝垫各
出一个观测。

sim/real 只差 HSV 阈值(算法不变):参数由 `<此目录>/color_pad.{sim,real}.yaml`
提供,`load_config(profile)` 读取,场景启动脚本选 profile。契约见
docs/spec/05-perception-decision-interface.md。
"""

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml

from cuadc_perception.detectors.base import ContourObservation

NAME = 'color_pad'


@dataclass
class ColorPadConfig:
    """蓝垫/白筒 HSV 阈值 + 形态学/几何门。字段名 = yaml 键;HSV 用 list
    (OpenCV HSV: H 0-179, S/V 0-255)。默认值只是兜底,正常从 yaml 读。"""
    blue_lo: Tuple[int, int, int] = (95, 40, 110)
    blue_hi: Tuple[int, int, int] = (115, 255, 255)
    white_lo: Tuple[int, int, int] = (0, 0, 165)
    white_hi: Tuple[int, int, int] = (180, 60, 255)
    proc_w: int = 640              # 处理分辨率宽(只降采样;sim 640 原样,真机 1920 降到这)
    morph_k: int = 5               # 蓝 mask 形态学核(proc 尺度)
    min_mat_area_frac: float = 0.01    # 蓝斑占 proc 面积下限,低于视为无目标
    hole_area_lo: float = 0.004        # 筒洞占垫面积下限
    hole_area_hi: float = 0.5          # 筒洞占垫面积上限
    min_tube_ratio: float = 0.10       # 筒半径/垫半径下限:更小的白斑当反光/噪点
    max_tube_offset: float = 0.90      # 筒心到垫心/垫半径上限:贴边白斑不是筒


def load_config(profile: str) -> ColorPadConfig:
    """从 `<此目录>/color_pad.<profile>.yaml` 读配置(profile 通常 'sim'/'real')。"""
    path = Path(__file__).parent / f'{NAME}.{profile}.yaml'
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    return ColorPadConfig(**data)


def _morph(mask, k):
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ker)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ker)


def _centroid(c):
    m = cv2.moments(c)
    return None if m['m00'] == 0 else (m['m10'] / m['m00'], m['m01'] / m['m00'])


def _area_radius(area):
    return math.sqrt(max(area, 0.0) / math.pi)  # 面积等效半径,对新月形不敏感


def detect_cylinders_pixels(image_bgr, cfg: Optional[ColorPadConfig] = None,
                            perf: Optional[dict] = None,
                            debug: Optional[dict] = None) -> List[ContourObservation]:
    """对一帧做 HSV 分割 -> 蓝垫轮廓 -> 垫内白筒定位,返回每个垫上白筒的图像空间
    观测(坐标一律换算回全分辨率)。`perf` 传空 dict 填分阶段耗时(ms);`debug`
    传空 dict 填 `mask`(蓝垫去噪后的二值图)。"""
    cfg = cfg or ColorPadConfig()
    h0, w0 = image_bgr.shape[:2]
    scale = max(1.0, w0 / cfg.proc_w)          # 只降采样(scale>=1),小图原样处理
    if scale > 1.0:
        proc = cv2.resize(image_bgr, (int(round(w0 / scale)), int(round(h0 / scale))))
    else:
        proc = image_bgr
    ph, pw = proc.shape[:2]

    t0 = time.monotonic()
    hsv = cv2.cvtColor(proc, cv2.COLOR_BGR2HSV)
    blue_raw = cv2.inRange(hsv, np.array(cfg.blue_lo), np.array(cfg.blue_hi))
    white_raw = cv2.inRange(hsv, np.array(cfg.white_lo), np.array(cfg.white_hi))
    t1 = time.monotonic()
    blue = _morph(blue_raw, cfg.morph_k)
    white = _morph(white_raw, max(3, cfg.morph_k // 2))
    t2 = time.monotonic()
    # RETR_CCOMP:两层层级,外轮廓(垫)+ 其子轮廓(洞=筒)
    cnts, hier = cv2.findContours(blue, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    t3 = time.monotonic()

    observations = []
    if cnts and hier is not None:
        hier = hier[0]
        min_area = cfg.min_mat_area_frac * pw * ph
        for mi in range(len(cnts)):
            if hier[mi][3] != -1:
                continue  # 只看外轮廓(蓝垫)
            marea = cv2.contourArea(cnts[mi])
            mcen = _centroid(cnts[mi])
            if marea < min_area or mcen is None:
                continue
            mu, mv = mcen
            mr = _area_radius(marea)
            _, renc = cv2.minEnclosingCircle(cnts[mi])
            circ = min(1.0, marea / (math.pi * renc * renc)) if renc > 0 else 0.0
            obs = _tube_on_mat(mi, cnts, hier, white, mu, mv, mr, marea, circ, scale, cfg)
            if obs is not None:
                observations.append(obs)
    t4 = time.monotonic()

    # 阶段名跟其它检测器对齐(节点按固定 key 组装 ~/perf、cuadc_viz 网页解析)。
    if perf is not None:
        perf['segment_ms'] = (t1 - t0) * 1000.0
        perf['morphology_ms'] = (t2 - t1) * 1000.0
        perf['contours_ms'] = (t3 - t2) * 1000.0
        perf['fit_ms'] = (t4 - t3) * 1000.0
    if debug is not None:
        debug['mask'] = blue

    return observations


def _tube_on_mat(mi, cnts, hier, white, mu, mv, mr, marea, circ, scale, cfg) -> Optional[ContourObservation]:
    """在一个蓝垫(外轮廓 mi)上定位白筒。洞法优先、白 mask 法兜底,过尺寸/居中门。
    返回换算回全分辨率的 ContourObservation(circularity 字段承载置信度),无筒返回 None。"""
    tube = None  # (u, v, radius_px, contour_proc)
    best_a = 0.0
    for i in range(len(cnts)):  # ①洞法:mat 的子轮廓里最大、在垫内、面积合理者
        if hier[i][3] != mi:
            continue
        a = cv2.contourArea(cnts[i])
        hc = _centroid(cnts[i])
        if a <= 0 or hc is None or not (cfg.hole_area_lo * marea <= a <= cfg.hole_area_hi * marea):
            continue
        if math.hypot(hc[0] - mu, hc[1] - mv) > mr or a <= best_a:
            continue
        best_a = a
        tube = (hc[0], hc[1], _area_radius(a), cnts[i])
    if tube is None:  # ②白 mask 法兜底(洞不闭合时):垫内最大白斑
        inside = np.zeros_like(white)
        cv2.circle(inside, (int(mu), int(mv)), int(mr), 255, -1)
        wc, _ = cv2.findContours(cv2.bitwise_and(white, inside), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if wc:
            wb = max(wc, key=cv2.contourArea)
            wa = cv2.contourArea(wb)
            wcen = _centroid(wb)
            if wa > 0 and wcen is not None:
                tube = (wcen[0], wcen[1], max(_area_radius(wa), 2.0), wb)
    if tube is None:
        return None

    wu, wv, wr, wcontour = tube
    if wr / max(mr, 1e-6) < cfg.min_tube_ratio or math.hypot(wu - mu, wv - mv) / max(mr, 1e-6) > cfg.max_tube_offset:
        return None  # speck/贴边白斑不算筒
    conc = 1.0 - min(1.0, math.hypot(wu - mu, wv - mv) / max(mr, 1e-6))  # 同心度
    conf = 0.5 * circ + 0.5 * conc
    contour_full = (wcontour.astype(np.float32) * scale).astype(np.int32) if scale != 1.0 else wcontour
    return ContourObservation(u=wu * scale, v=wv * scale, radius_px=wr * scale,
                              circularity=float(conf), contour=contour_full)
