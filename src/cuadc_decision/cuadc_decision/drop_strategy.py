"""投放目标选择策略。每个策略是一个函数，加新策略直接在这里写一个新函数、
注册进 STRATEGIES，改 ACTIVE_STRATEGY 指向哪个就切换哪个，不用改
mission_logic.py 或状态机。

策略函数签名: (candidates, used_positions, drop_index) -> Optional[Candidate]
drop_index 是第几次投放（0=第一次，1=第二次）——是否区分第一次/第二次、
要不要排除 used_positions，由具体策略自己决定，不是外部强加的。
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class Candidate:
    x: float
    y: float
    size: float


_USED_EXCLUSION_THRESHOLD_M = 0.6  # 候选离已投过的位置多近算"同一个筒"


def _exclude_used(candidates: List[Candidate], used_positions: List[Tuple[float, float]]) -> List[Candidate]:
    return [c for c in candidates
            if not any(math.hypot(c.x - px, c.y - py) < _USED_EXCLUSION_THRESHOLD_M
                       for px, py in used_positions)]


def size_class_radius(target: Candidate, candidates: List[Candidate], radii, bounds) -> float:
    """目标是哪个投放筒（返回真实半径）。候选正好 len(radii) 个时按测得半径排名对号——不怕测值整体偏大偏小；
    否则按测得半径对 bounds 分档。radii 从小到大。"""
    if len(candidates) == len(radii):
        order = sorted(candidates, key=lambda c: c.size)
        i = min(range(len(order)), key=lambda k: math.hypot(order[k].x - target.x, order[k].y - target.y))
        return radii[i]
    return radii[min(sum(target.size > b for b in bounds), len(radii) - 1)]


def smallest_first(candidates, used_positions, drop_index) -> Optional[Candidate]:
    """默认策略：排除已投过的位置，选视在尺寸最小的（命中分最高）。"""
    remaining = _exclude_used(candidates, used_positions)
    return min(remaining, key=lambda c: c.size) if remaining else None


def largest_first(candidates, used_positions, drop_index) -> Optional[Candidate]:
    """备选：小筒命中率不够时的退路，选最大的换稳定命中。"""
    remaining = _exclude_used(candidates, used_positions)
    return max(remaining, key=lambda c: c.size) if remaining else None


STRATEGIES = {
    'smallest_first': smallest_first,
    'largest_first': largest_first,
}

ACTIVE_STRATEGY = STRATEGIES['smallest_first']  # 改这一行切换/加新策略实验
