"""扫描期间垫心散点 → 一个起飞系目标。

每帧最多一个点，反投影有姿态/高度噪声，所以不用单帧、用中位数丢掉野值
再取均值。点太少就原样平均；一个都没有则退回原点（调用方决定要不要降）。
"""
from __future__ import annotations

import math
from statistics import median
from typing import Iterable, List, Optional, Tuple

Xy = Tuple[float, float]


def consensus_xy(
    points: Iterable[Xy],
    outlier_m: float = 1.5,
) -> Optional[Xy]:
    """散点集合中心。返回 (前, 右)；空输入 → None。"""
    pts: List[Xy] = [(float(x), float(y)) for x, y in points]
    if not pts:
        return None
    mx = median(p[0] for p in pts)
    my = median(p[1] for p in pts)
    kept = [p for p in pts if math.hypot(p[0] - mx, p[1] - my) <= outlier_m]
    if not kept:
        kept = pts
    n = len(kept)
    return (sum(p[0] for p in kept) / n, sum(p[1] for p in kept) / n)
