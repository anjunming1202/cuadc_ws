"""<algo> SEARCH 检测器模板。拷这个文件夹改名成你的算法:
    cp -r search_cv/template search_cv/<algo>
实现 detect_frame,跑 `python search_cv/<algo>/detect.py`,看 out/overlays/。
契约见 search_cv/README.md。探索期随便折腾,这里的常量/中间图想加就加。
"""

import sys
from pathlib import Path

import cv2  # noqa: F401  (探索时大概率要用)

# 让 `from search_cv._common import ...` 可用(cv_explore 在 sys.path 上)。
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from search_cv._common import SearchDetection, run  # noqa: E402


def detect_frame(bgr, frame):
    """一帧 -> 该帧检出的筒列表(多目标)。frame 是 _shared.dataset.Frame(带位姿)。
    返回 [SearchDetection(u, v, radius_px, conf), ...];没检到返回 []。"""
    dets = []
    # TODO: 你的算法。示例骨架(什么都没检到):
    #   hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    #   ... 分割/形态学/轮廓/拟合 ...
    #   dets.append(SearchDetection(u=cx, v=cy, radius_px=r, conf=score))
    return dets


if __name__ == "__main__":
    run(detect_frame, out_dir=str(Path(__file__).resolve().parent / "out"))
