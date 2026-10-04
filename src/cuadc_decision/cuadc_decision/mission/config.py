"""世界 / 任务 / 场景 参数的单一真源（A 类，非算法参数）——见
docs/spec/mission-config-layer.md。与算法内部参数（检测器 HSV、approach PID/门、SEARCH
聚类阈值、FSM 到点容差等）**分开、永不合并**：那些跟各自算法源码放一起（B / B′）。

分两条轴组织：
- **sim / real 显式分文件**做环境隔离：调实机只动 `mission.real.yaml`、调仿真只动
  `mission.sim.yaml`，互不波及。`cuadc_sim/field/config.py` 是仿真世界建图自己那份，真机
  碰不到；按设计二者分别维护、不跨包 import。
- **场景 = base + 覆盖组合**（照 SITL --add-param-file 分层）：`mission.<profile>.yaml` 是
  该环境的唯一定义，`scenarios/<scenario>.yaml` 只写相对 base 的最小偏离（initial_state /
  phases / 极少数意图性覆盖，如 bench 对 SEARCH 的 search_overrides）——同环境内各场景继承
  base，不重复输入。只对某个环境成立的场景偏离（如 sim 单练世界的出生点）写在可选的
  `scenarios/<scenario>.<profile>.yaml`，最后叠加——不能放进共用的场景文件，否则会带到另一个环境。

坐标约定：区中心/扫描 offsets 用**场地坐标**（x=飞行前进方向、y=侧向，同 field-layout.md），
好跟场地图肉眼核对；`takeoff_point` 是起飞（解锁）点的场地坐标。接口层 local_position 原点
就是起飞点，转 NED 由 mission_logic._world_xy_to_ned 做（减起飞点、x/y 对调）。
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple

import yaml


@dataclass
class MissionWorld:
    """一次任务的世界/任务/场景参数。字段名 = yaml 键。默认值 = 当前开发用固定场地的值
    （兜底，纯单测/import 期不读文件）；正常从 mission.<profile>.yaml + 场景覆盖构造。"""
    takeoff_point: Tuple[float, float] = (0.0, 0.0)  # 起飞(解锁)点的场地坐标 = local_position 原点
    cruise_alt_m: float = 2.0                 # FSM 导航/扫描的一般巡航高度（approach 自己的作业高度在 approach yaml，各自属主）
    drop_zone_center: Tuple[float, float] = (32.5, 0.0)
    drop_scan_offsets: List[Tuple[float, float]] = field(default_factory=lambda: [
        (-1.0, -2.0), (-1.0, 2.0),
        (0.0, 2.0), (0.0, -2.0),
        (1.0, -2.0), (1.0, 2.0),
    ])
    recon_zone_center: Tuple[float, float] = (57.5, 0.0)
    recon_scan_offsets: List[Tuple[float, float]] = field(default_factory=lambda: [
        (-1.5, -2.0), (-1.5, 2.0),
        (0.0, 2.0), (0.0, -2.0),
        (1.5, -2.0), (1.5, 2.0),
    ])
    recon_cylinder_count: int = 5             # 侦察区覆盖到这么多筒即算完成
    drop_count_target: int = 2                # 投放次数目标
    drop_recon_split_y_m: float = 45.0        # 投放/侦察检测分界（NED east 中点，见 mission_logic 说明）
    initial_state: str = 'INIT'               # FSM 起跑状态（隔离测试由场景改，见 spec §5）
    phases: List[str] = field(default_factory=lambda: ['drop', 'recon'])  # 跑哪些阶段（取代 stop_after_drop）
    search_overrides: Dict = field(default_factory=dict)  # 场景对 SEARCH 算法(search_strategy)参数的覆盖


def _read_yaml(path: Path) -> dict:
    if not path.exists():
        return {}
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


def load(profile: str, scenario: str = 'full_mission') -> MissionWorld:
    """base（该环境的 mission.<profile>.yaml）叠加 scenarios/<scenario>.yaml、再叠加可选的
    scenarios/<scenario>.<profile>.yaml 的键级覆盖，构造 MissionWorld。场景只写偏离，其余继承 base。"""
    here = Path(__file__).parent
    base = _read_yaml(here / f'mission.{profile}.yaml')
    scen = _read_yaml(here / 'scenarios' / f'{scenario}.yaml')
    scen_env = _read_yaml(here / 'scenarios' / f'{scenario}.{profile}.yaml')
    return MissionWorld(**{**base, **scen, **scen_env})
