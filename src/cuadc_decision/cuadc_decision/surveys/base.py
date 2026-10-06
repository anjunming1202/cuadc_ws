"""Survey 返回契约 + 共用小工具。放 base（无依赖）避免和注册表 __init__ 循环导入——所有 survey 都吐
同一种 SurveyOutput，任务 FSM 统一消费（位置指令 + 是否覆盖完）。"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class SurveyOutput:
    position_target: Optional[Tuple[float, float, float]] = None  # (x,y,z) 起飞坐标系(前/右/下)
    speed_mps: Optional[float] = None  # 水平限速；None = 任务按区域默认（search_speed_mps）
    done: bool = False                 # 覆盖完：任务 FSM 结束侦察阶段


def at_point(ctx, xy, alt, xy_tol, alt_tol):
    """水平到点且到了该高度。"""
    if ctx.local_position_xy is None or ctx.local_position_z is None:
        return False
    return (math.hypot(ctx.local_position_xy[0] - xy[0], ctx.local_position_xy[1] - xy[1]) <= xy_tol
            and abs(-ctx.local_position_z - alt) <= alt_tol)
