"""接近策略库:一个 `approaches/` 文件夹收所有 approach,一个方法一个模块,
各自的 sim/real config yaml 同名同目录跟着放(组织同 cuadc_perception 的
`detectors/`,约定见 cuadc_decision/README.md)。任务 FSM(SEARCH->APPROACH->DROP)
与 transition 是固定框架;`APPROACH_DROP_TARGET` 内部怎么把飞机对到筒上是这里的
可替换部分——改 ACTIVE 切换,不动状态机主干。

每个 approach 模块统一提供:`NAME`、`ApproachConfig`、`load_config(profile)`、
`create(config) -> approach 实例`(实例有 `enter/loop`)。约定见文件头 +
docs/spec/adr/0005-terminal-align-visual-servoing.md。
"""

from cuadc_decision.approaches.base import ApproachOutput
from cuadc_decision.approaches import cone_descent_ibvs
from cuadc_decision.approaches import stepped_descent_ibvs
# 常用类型 re-export,方便脱离节点单测直接构造。
from cuadc_decision.approaches.stepped_descent_ibvs import (
    ApproachConfig, SteppedDescentIbvs)

APPROACHES = {
    stepped_descent_ibvs.NAME: stepped_descent_ibvs,
    cone_descent_ibvs.NAME: cone_descent_ibvs,
}
ACTIVE = cone_descent_ibvs.NAME  # 改这行切换/实验(stepped=阶梯基线 10/10, cone=锥面连续降高)


def create_active(profile=None, config=None):
    """建当前选定 approach 的实例。给 profile 就从它的 yaml 读配置;给 config 直接
    用;都不给用 dataclass 默认(纯单测/import 期,不读文件)。"""
    mod = APPROACHES[ACTIVE]
    if config is None and profile is not None:
        config = mod.load_config(profile)
    return mod.create(config)
