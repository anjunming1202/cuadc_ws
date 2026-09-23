"""SEARCH 阶段的算法：把逐帧的筒检测做时序融合（聚类归并）、提取候选、判定「够不够
选目标」。从 mission_logic 的 SEARCH 状态里抽出来，组织同 drop_strategy.py（轻量、一处
声明、可替换），可调参数按算法名放同名 sim/real yaml（同 approaches/、detectors/）。

感知无状态、不发 track_id，跨帧把同一物理目标的多次检测归并成一个聚类是**决策层**自己
做的（见 docs/spec/05-perception-decision-interface.md）——就在这里。

mission_logic 只调 `observe()`（喂本帧检测）+ `select_target()`（问现在能不能选目标）；
扫描航点导航、状态转移仍归 mission_logic。换 SEARCH 算法：以后有第二种方法再照
approaches/ 提升成子文件夹按方法分 yaml，现在只一个方法，扁平一份即可。
"""

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from cuadc_decision.drop_strategy import Candidate


@dataclass
class SearchConfig:
    """SEARCH 算法可调参数。字段名 = yaml 键。默认值只是兜底（纯单测/import 期不读文件），
    正常从 `search_strategy.<profile>.yaml` 读。"""
    cluster_match_threshold_m: float = 0.6   # 本帧检测离已有聚类均值多近算同一物理目标
    min_cluster_samples: int = 3             # 聚类样本数够这么多才当一个可信候选
    settle_seconds: float = 1.5              # 进 SEARCH 后先攒这么久样本再考虑选目标
    min_detection_confidence: float = 0.5    # 只收置信度够的检测（桩恒发 1.0，真实 CV 会过滤）
    min_candidates_before_select: int = 3    # 候选够这么多才允许选（正式=筒数 3；bench 场景覆盖成 1=看到就投）


def load_config(profile: str) -> SearchConfig:
    """从 `search_strategy.<profile>.yaml` 读配置（profile 通常 'sim'/'real'）。"""
    path = Path(__file__).parent / f'search_strategy.{profile}.yaml'
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    return SearchConfig(**data)


def _cluster_mean(cluster):
    n = len(cluster)
    return (
        sum(p[0] for p in cluster) / n,
        sum(p[1] for p in cluster) / n,
        sum(p[2] for p in cluster) / n,
    )


def _update_clusters(clusters, points, match_threshold):
    """按最近邻把本帧检测归并进已有聚类（同一物理目标的历史采样），没匹配上的新建一个。"""
    for x, y, size in points:
        best = None
        best_dist = match_threshold
        for cluster in clusters:
            cx, cy, _ = _cluster_mean(cluster)
            dist = math.hypot(x - cx, y - cy)
            if dist < best_dist:
                best_dist = dist
                best = cluster
        if best is None:
            clusters.append([(x, y, size)])
        else:
            best.append((x, y, size))


def observe(ctx, cfg, split_y):
    """把本帧筒检测（按置信度过滤、只看投放区那一侧 position.y < split_y）归并进
    ctx.drop_task.clusters。split_y 是投放/侦察检测分界（世界/任务参数，由 mission_logic 传
    入；Detection 本身不带区域信息，用中点粗分即可）。"""
    points = [
        (d.position.x, d.position.y, d.apparent_size)
        for d in ctx.detections
        if d.target_type == 'cylinder'
        and d.confidence >= cfg.min_detection_confidence
        and d.position.y < split_y
    ]
    _update_clusters(ctx.drop_task.clusters, points, cfg.cluster_match_threshold_m)


def select_target(ctx, cfg, scan_complete, now, choose):
    """settle 后：把够样本的聚类做成候选，候选够多（min_candidates_before_select）或已飞完
    整条扫描路径（scan_complete）才允许选目标——不能「随便凑够一个候选就选」，否则可能锁死在
    第一眼看到的筒上、从没飞去比较其它候选（2026-09-11 SITL 实测踩过）。`choose` 是投放目标
    选择策略（drop_strategy），返回选中的 Candidate 或 None。还没到选目标条件时返回 None。"""
    if (now - ctx.state_entered_monotonic) < cfg.settle_seconds:
        return None
    candidates = [
        Candidate(*_cluster_mean(c))
        for c in ctx.drop_task.clusters
        if len(c) >= cfg.min_cluster_samples
    ]
    if len(candidates) >= cfg.min_candidates_before_select or scan_complete:
        return choose(candidates, ctx.drop_task.used_positions, ctx.drop_task.drop_count)
    return None
