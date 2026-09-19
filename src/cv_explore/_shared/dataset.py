"""读 cylinder_dataset 某组的帧 + 位姿。纯 I/O,不含任何检测逻辑。

用法:
    from _shared import dataset            # 或 sys.path 指到 cv_explore 后 import _shared.dataset
    meta, frames = dataset.load_group()    # 默认组
    img = dataset.load_image(frames[100])  # 需要时才读图(懒加载 cv2)
"""
import csv
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

# src/cv_explore/_shared/dataset.py -> parents[3] = 仓库根
REPO_ROOT = Path(__file__).resolve().parents[3]
DATA_DIR = REPO_ROOT / "src" / "cylinder_dataset" / "data"
DEFAULT_GROUP = "cylinder_20260916_01_20260916_180235"  # 本轮主用组


@dataclass
class Frame:
    num: int
    image_path: str
    t_utc: str
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    ned: Tuple[float, float, float]  # (north, east, down)
    lat_deg: float
    lon_deg: float
    rel_alt_m: float
    gps_fix: str
    sats: int
    position_trustworthy: bool
    attitude_age_s: float


def resolve_group(group: str) -> Path:
    """group 可以是组名(在 DATA_DIR 下)或直接给一个路径。"""
    p = Path(group)
    return p if p.is_absolute() or p.exists() else (DATA_DIR / group)


def load_group(group: str = DEFAULT_GROUP) -> Tuple[dict, List[Frame]]:
    """返回 (run_meta.json 内容, 按帧号排序的 Frame 列表)。"""
    gdir = resolve_group(group)
    meta = json.loads((gdir / "run_meta.json").read_text())
    frames: List[Frame] = []
    with open(gdir / "index.csv", newline="") as f:
        for r in csv.DictReader(f):
            frames.append(Frame(
                num=int(r["frame"]),
                image_path=str(gdir / r["image"]),
                t_utc=r["capture_time_utc"],
                roll_deg=float(r["roll_deg"]),
                pitch_deg=float(r["pitch_deg"]),
                yaw_deg=float(r["yaw_deg"]),
                ned=(float(r["ned_x"]), float(r["ned_y"]), float(r["ned_z"])),
                lat_deg=float(r["lat_deg"]),
                lon_deg=float(r["lon_deg"]),
                rel_alt_m=float(r["rel_alt_m"]),
                gps_fix=r["gps_fix"],
                sats=int(r["sats"]),
                position_trustworthy=(r["position_trustworthy"].strip().lower() == "true"),
                attitude_age_s=float(r["attitude_age_s"]),
            ))
    frames.sort(key=lambda fr: fr.num)
    return meta, frames


def load_image(frame: Frame):
    """懒加载 cv2 读图,返回 BGR ndarray。读失败抛异常(不静默返回 None)。"""
    import cv2
    img = cv2.imread(frame.image_path)
    if img is None:
        raise FileNotFoundError(f"cannot read image: {frame.image_path}")
    return img


def select(frames: List[Frame], lo: Optional[int] = None, hi: Optional[int] = None) -> List[Frame]:
    """按帧号闭区间 [lo, hi] 取子集(便于只跑有目标的 79–200 段)。"""
    return [f for f in frames if (lo is None or f.num >= lo) and (hi is None or f.num <= hi)]
