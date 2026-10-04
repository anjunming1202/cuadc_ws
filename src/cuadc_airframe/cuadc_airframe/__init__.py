"""机体固定几何的单一真源（airframe.yaml），感知/决策/仿真共用，见该文件头。"""
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import yaml


@dataclass(frozen=True)
class DropPort:
    name: str
    position_m: Tuple[float, float]  # 相对飞控（前/右）


@dataclass(frozen=True)
class Airframe:
    fc_rest_clearance_m: float
    camera_position_m: Tuple[float, float, float]  # 相对飞控（前/右/下）
    camera_mount_rad: Tuple[float, float, float]   # roll/pitch/yaw
    drop_ports: Tuple[DropPort, ...]                # 按投放次序

    @property
    def camera_below_fc_m(self) -> float:
        return self.camera_position_m[2]

    def lever_arms_from_camera(self) -> Tuple[Tuple[float, float], ...]:
        """各投放口相对相机光心的水平偏置（前/右），按投放次序——末端瞄准用的杠杆臂。"""
        cf, cr = self.camera_position_m[0], self.camera_position_m[1]
        return tuple((p.position_m[0] - cf, p.position_m[1] - cr) for p in self.drop_ports)


def load() -> Airframe:
    with open(Path(__file__).parent / 'airframe.yaml', encoding='utf-8') as fh:
        d = yaml.safe_load(fh)
    return Airframe(
        fc_rest_clearance_m=float(d['fc_rest_clearance_m']),
        camera_position_m=tuple(float(v) for v in d['camera']['position_m']),
        camera_mount_rad=tuple(float(v) for v in d['camera']['mount_rad']),
        drop_ports=tuple(DropPort(p['name'], tuple(float(v) for v in p['position_m']))
                         for p in d['drop_ports']),
    )
