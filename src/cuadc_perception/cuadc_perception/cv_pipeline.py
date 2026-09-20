"""传统 CV 检测器（第一个检测器）：分割 -> 形态学去噪 -> 轮廓提取 -> 图像空间
拟合。纯函数（numpy/cv2 输入输出，不依赖 rclpy，可脱离节点单测），只输出
**图像空间**的观测（筒的像素中心 + 视在半径 px + 置信度）；反投影、归一化
方位、物理半径过滤都不在这里，在 geometry.py + 感知节点侧组装——这样检测器
与相机内参/姿态/sim-real 解耦，可以整批换检测算法（见
cuadc_perception/README.md 的检测器库约定）。

契约（apparent_size/confidence 语义）见 docs/spec/05-perception-decision-interface.md。
"""

import math
import time
from dataclasses import dataclass
from typing import List, Optional

import cv2


@dataclass
class ContourObservation:
    """检测器对一个候选轮廓的图像空间观测。contour 原样保留给节点画调试图用。"""
    u: float             # 像素中心 x
    v: float             # 像素中心 y
    radius_px: float     # 等效面积半径(px)，反投影后换算物理半径 / 画标注用
    circularity: float   # 面积/外接圆面积，置信度用
    contour: object      # cv2 轮廓，画调试图用


def detect_cylinders_pixels(image_bgr, morph_kernel_px: int = 5,
                            perf: Optional[dict] = None,
                            debug: Optional[dict] = None) -> List[ContourObservation]:
    """对一帧图像做分割/去噪/轮廓/图像拟合，返回每个候选轮廓的图像空间观测。
    `perf` 传一个空 dict 会被填上分阶段耗时（毫秒）；`debug` 传一个空 dict 会
    被填上 `mask`（去噪后的二值图）。反投影 + 物理半径过滤 + accept/reject 在
    节点侧（需要相机内参和高度），不在这里。"""
    t0 = time.monotonic()
    mask = _segment(image_bgr)
    t1 = time.monotonic()
    mask = _denoise(mask, morph_kernel_px)
    t2 = time.monotonic()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    t3 = time.monotonic()

    observations = []
    for contour in contours:
        obs = _fit_contour_image(contour)
        if obs is not None:
            observations.append(obs)
    t4 = time.monotonic()

    # 阶段名是权威来源——cuadc_viz/web/app.js 按固定 key 解析 ~/perf。'fit_ms'
    # 只到图像拟合为止；反投影耗时由节点补进 'fit_reproject_ms'（保持网页那
    # 几个 stat 的 key 不变）。改/加/删阶段名要跟网页一起改。
    if perf is not None:
        perf['segment_ms'] = (t1 - t0) * 1000.0
        perf['morphology_ms'] = (t2 - t1) * 1000.0
        perf['contours_ms'] = (t3 - t2) * 1000.0
        perf['fit_ms'] = (t4 - t3) * 1000.0

    if debug is not None:
        debug['mask'] = mask

    return observations


def _segment(image_bgr):
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return mask


def _denoise(mask, kernel_px):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def _fit_contour_image(contour) -> Optional[ContourObservation]:
    """图像空间拟合：外接圆只用来算 circularity（形状规则度、置信度用）；
    物理尺寸用等效面积半径 sqrt(area/pi)——斜视时色块是细长月牙形，外接圆
    半径会被撑爆，等效面积半径对拉长不敏感。物理半径的合理性过滤放节点做
    （需要反投影出的斜距），这里只出图像量。为什么这么选见
    cuadc_perception/README.md。"""
    area = cv2.contourArea(contour)
    if area <= 0:
        return None
    (u, v), enclosing_radius_px = cv2.minEnclosingCircle(contour)
    if enclosing_radius_px <= 0:
        return None
    area_equiv_radius_px = math.sqrt(area / math.pi)
    circularity = area / (math.pi * enclosing_radius_px ** 2)
    return ContourObservation(u=u, v=v, radius_px=area_equiv_radius_px,
                              circularity=circularity, contour=contour)
