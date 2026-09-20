"""检测器共享类型。放在 base(无依赖)避免和注册表 __init__ 循环导入。"""

from dataclasses import dataclass


@dataclass
class ContourObservation:
    """检测器对一个候选筒的图像空间观测——所有检测器的统一输出(见
    cuadc_perception/README.md 检测器库约定)。反投影/归一化方位/物理半径
    过滤都在感知节点侧,不在检测器里。"""
    u: float             # 像素中心 x
    v: float             # 像素中心 y
    radius_px: float     # 等效面积半径(px);反投影后换算物理半径 / 画标注用
    circularity: float   # 置信度(各检测器可赋不同语义,节点当 confidence 用)
    contour: object      # cv2 轮廓,画调试图用
