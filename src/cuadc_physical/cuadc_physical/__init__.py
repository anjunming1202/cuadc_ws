"""固定物理配置的单一真源：机体（airframe.yaml）与比赛目标尺寸（targets.yaml），见各文件头。"""
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import yaml


@dataclass(frozen=True)
class DropPort:
    name: str
    position_m: Tuple[float, float]  # 相对飞控（前/右）


@dataclass(frozen=True)
class DropServo:
    output: int
    forward_us: int
    neutral_us: int
    run_s: float


@dataclass(frozen=True)
class Targets:
    drop_cylinder_height_m: float
    recon_cylinder_height_m: float


@dataclass(frozen=True)
class Airframe:
    fc_rest_clearance_m: float
    camera_position_m: Tuple[float, float, float]  # 相对飞控（前/右/下）
    camera_mount_rad: Tuple[float, float, float]   # roll/pitch/yaw
    drop_ports: Tuple[DropPort, ...]                # 按投放次序
    drop_servo: DropServo

    @property
    def camera_below_fc_m(self) -> float:
        return self.camera_position_m[2]

    def lever_arms_from_camera(self) -> Tuple[Tuple[float, float], ...]:
        """各投放口相对相机光心的水平偏置（前/右），按投放次序——末端瞄准用的杠杆臂。"""
        cf, cr = self.camera_position_m[0], self.camera_position_m[1]
        return tuple((p.position_m[0] - cf, p.position_m[1] - cr) for p in self.drop_ports)


def _read(name):
    with open(Path(__file__).parent / name, encoding='utf-8') as fh:
        return yaml.safe_load(fh)


def load_airframe() -> Airframe:
    d = _read('airframe.yaml')
    return Airframe(
        fc_rest_clearance_m=float(d['fc_rest_clearance_m']),
        camera_position_m=tuple(float(v) for v in d['camera']['position_m']),
        camera_mount_rad=tuple(float(v) for v in d['camera']['mount_rad']),
        drop_ports=tuple(DropPort(p['name'], tuple(float(v) for v in p['position_m']))
                         for p in d['drop_ports']),
        drop_servo=DropServo(**d['drop_servo']),
    )


def load_targets() -> Targets:
    return Targets(**_read('targets.yaml'))
