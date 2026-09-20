"""Approach 返回契约。放 base(无依赖)避免和注册表 __init__ 循环导入——
所有 approach 都吐同一种 ApproachOutput,任务 FSM 统一消费(位置或速度指令 +
是否可投),decision_node 再翻成 position_target / velocity_target。"""

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class ApproachOutput:
    position_target: Optional[Tuple[float, float, float]] = None  # (x,y,z) 局部 NED,粗段
    velocity_target: Optional[Tuple[float, float, float]] = None  # (前,右,下) 机体系速度,末端
    drop_ready: bool = False
