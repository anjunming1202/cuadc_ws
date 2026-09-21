"""search_cv 共享脚手架:各 SEARCH-阶段 CV 算法统一的输入(cylinder_dataset 真实
航拍帧)、输出(SearchDetection)、产物(detections.csv + overlays/),方便在并行
worktree 里各做一个算法、横向对比。

真实素材无相机标定、无标注 => 按约定**只出可视化 overlay 人工比**:每帧画出检出
的筒(圈 + 视在半径 px + 置信度) + 按大小排名。SEARCH 关心两件事——(1) 稳定检出
全部筒(多目标、抗斜视/糊)、(2) 分得出筒的大小(加分项:同高度下 radius_px 排名
就是尺寸判别效果)。物理尺寸/世界坐标需要标定,这里不算。

一个算法一个文件夹:`search_cv/<algo>/detect.py` 实现
`detect_frame(bgr, frame) -> List[SearchDetection]`,`__main__` 里调 `run(...)`。
详见 search_cv/README.md。
"""

import csv
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List

import cv2

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))   # 让 `from _shared import dataset` 可用
from _shared import dataset            # noqa: E402


@dataclass
class SearchDetection:
    u: float           # 像素中心 x
    v: float           # 像素中心 y
    radius_px: float   # 视在半径(px):同高度下比大小 = 尺寸判别
    conf: float        # 置信度 [0,1]


def run(detect_frame, out_dir, group=dataset.DEFAULT_GROUP, lo=79, hi=200):
    """跑一个算法过一段帧,出 detections.csv + overlays/。detect_frame 签名:
    (bgr_ndarray, dataset.Frame) -> List[SearchDetection]。lo/hi 是帧号闭区间
    (默认 79–200 = 已知有筒段,可按需放宽)。"""
    _, all_frames = dataset.load_group(group)
    frames = dataset.select(all_frames, lo, hi)
    ov_dir = os.path.join(out_dir, "overlays")
    os.makedirs(ov_dir, exist_ok=True)
    n_with = 0
    with open(os.path.join(out_dir, "detections.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["frame", "n", "rank_by_size", "u", "v", "radius_px", "conf"])
        for fr in frames:
            bgr = dataset.load_image(fr)
            dets = sorted(detect_frame(bgr, fr), key=lambda d: -d.radius_px)  # 大 -> 小
            vis = bgr.copy()
            for rank, d in enumerate(dets):
                w.writerow([fr.num, len(dets), rank, round(d.u, 1), round(d.v, 1),
                            round(d.radius_px, 1), round(d.conf, 3)])
                cv2.circle(vis, (int(d.u), int(d.v)), int(max(d.radius_px, 1)), (0, 0, 255), 3)
                cv2.putText(vis, f"#{rank} r{d.radius_px:.0f} c{d.conf:.2f}",
                            (int(d.u) - 45, int(d.v) - int(d.radius_px) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            if not dets:
                cv2.putText(vis, "no target", (12, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imwrite(os.path.join(ov_dir, f"frame_{fr.num:04d}.jpg"), vis)
            n_with += 1 if dets else 0
    n = len(frames)
    print(f"frames {n} | with-detection {n_with} ({100 * n_with / max(n, 1):.0f}%) "
          f"-> {out_dir}/detections.csv , overlays/")
