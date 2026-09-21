# search_cv —— SEARCH 阶段 CV 算法并行探索（真实素材）

投放区 SEARCH 阶段的 CV：从巡航高度/斜视角**稳定检出全部筒**(多目标、抗糊)，
并**分得出筒的大小**（加分项——3 个投放筒直径 0.15/0.20/0.25m，能排序才不是随机
投）。跟 approach 的 CV 探索一样,在 `cv_explore` 里**一个算法一个文件夹、并行几路
在真实素材上试、可视化横向比**,选出的赢家再迁进 `cuadc_perception/detectors/`。

## 数据 + 评估

- 输入:`cylinder_dataset` 真实航拍帧,经 `_shared/dataset.py` 读(每帧带位姿)。
  帧 79–200 是已知有筒段。真实数据是 gitignore 的本地文件,每个 worktree 里
  `src/cylinder_dataset/data` 是**软链**指向主工作树的那一份。
- 评估:真实素材**无相机标定、无标注** => 约定**只出可视化 overlay 人工比**
  (位置像素 + 视在半径 px 排名 = 尺寸判别效果)。不算物理尺寸/世界坐标。

## 契约(所有算法统一)

一个算法 = 一个文件夹 `search_cv/<algo>/detect.py`,实现:

```python
from search_cv._common import SearchDetection, run   # 或 sys.path 指到 cv_explore 后 import

def detect_frame(bgr, frame) -> list[SearchDetection]:   # frame 是 _shared.dataset.Frame(带位姿)
    ...  # 多目标:每个筒一个 SearchDetection(u, v, radius_px, conf)
    return dets

if __name__ == "__main__":
    run(detect_frame, out_dir="out")     # 出 out/detections.csv + out/overlays/
```

`SearchDetection(u, v, radius_px, conf)` 是**图像空间**量(跟生产
`cuadc_perception/detectors` 的 `ContourObservation` 同形,便于赢家迁移)。

## 并行 worktree 工作流

- 基线分支 `feature/search-cv-20260921`(本脚手架在此)。
- 每个算法一个 worktree + 子分支 `feature/search-cv-20260921-<algo>`
  (`color` / `frst` / `blob`),在 `cuadc_ws_worktrees/` 下。
- 每个 session 在自己的 worktree 里:`cp -r search_cv/template search_cv/<algo>`,
  实现 `detect_frame`,`python search_cv/<algo>/detect.py`,看 `out/overlays/`。
  提交**代码**(`out/` 已 gitignore,不进 git)。
- 三路都跑完后横向比 overlays,选赢家迁进 `detectors/`(照 color_pad 的组织)。

## 起点建议

- **color**:直接借鉴生产 `cuadc_perception/detectors/color_pad.py`(已迁移+精调的
  color_shape,蓝垫锚点 + 垫内白筒,多目标),在真实素材上按需重调 HSV、补尺寸判别。
- **frst**:径向对称(FRST)找圆,颜色无关。可参考 git 历史 `radial_symmetry`(23c5048)。
- **blob**:颜色团块 + 同心配对。可参考 git 历史 `blob_detection`(80b59ef)。
