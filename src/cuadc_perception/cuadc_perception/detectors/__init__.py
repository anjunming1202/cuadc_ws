"""检测器库:一个 `detectors/` 文件夹收所有检测器,一个方法一个模块(文件),
各自的 sim/real config yaml **同名同目录跟着放**(约定 + 迁移组织规范见
cuadc_perception/README.md)。从 cv_explore 沙盒迁移新方法时照 color_pad 这个样板:
只迁有意义的,组织进这里,带上 `<方法>.sim.yaml`/`<方法>.real.yaml`。哪个方法
以后长出一堆附属文件,再单独提升成 `detectors/<方法>/` 子文件夹。

选哪个检测器 = 感知节点 `detector` 参数;sim/real = `profile` 参数。每个检测器
模块提供统一接口:`NAME`、`Config` dataclass、`load_config(profile) -> Config`、
`detect_cylinders_pixels(image_bgr, cfg, perf, debug) -> List[ContourObservation]`。
"""

from cuadc_perception.detectors.base import ContourObservation
from cuadc_perception.detectors import color_pad

# 目前只有一个有意义的检测器(从沙盒迁来的 color_pad)。以后有别的方法值得迁移
# 再照 color_pad 的样板加进这里(一个模块 + 同名 sim/real yaml),改节点 detector
# 参数即可切换,不用改节点代码。
DETECTORS = {
    color_pad.NAME: color_pad,
}
ACTIVE_DEFAULT = color_pad.NAME
