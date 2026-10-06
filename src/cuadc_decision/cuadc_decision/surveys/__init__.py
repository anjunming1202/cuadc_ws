"""侦察覆盖策略库：一个 `surveys/` 文件夹收所有侦察区覆盖方法，一个方法一个模块，各自的 sim/real
config yaml 同名同目录跟着放（组织同 `approaches/`）。任务 FSM 的 NAVIGATE_TO_RECON_ZONE ->
SURVEY_RECON_ZONE -> RTL 是固定框架；SURVEY 内部怎么飞、什么时候算覆盖完是这里的可替换部分——
改 ACTIVE 或启动参数 survey:= 切换，不动状态机主干。

每个 survey 模块统一提供：`NAME`、`SurveyConfig`、`load_config(profile)`、
`create(config, world, tuning) -> 实例`（实例有 `entry_point/enter/loop/progress/describe`）。
"""

from cuadc_decision.surveys import coverage_scan
from cuadc_decision.surveys import detect_count
from cuadc_decision.surveys.base import SurveyOutput

SURVEYS = {
    coverage_scan.NAME: coverage_scan,
    detect_count.NAME: detect_count,
}
ACTIVE = coverage_scan.NAME  # 默认；现场用启动参数 survey:= 切换，不改这行


def create_active(world, tuning, profile=None, name=None):
    """建 survey 实例（name 缺省 = ACTIVE）。给 profile 就从它的 yaml 读配置，不给用 dataclass 默认
    （纯单测/import 期，不读文件）。"""
    mod = SURVEYS[name or ACTIVE]
    config = mod.load_config(profile) if profile is not None else mod.SurveyConfig()
    return mod.create(config, world, tuning)
