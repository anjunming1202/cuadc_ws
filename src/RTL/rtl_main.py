"""全链路 GUIDED 视觉返航入口：回原点 → 扫 H → 降落，一次拉起三个阶段。

各阶段仍是独立模块（`rtl_go_home` / `rtl_scan` / `rtl_land`），这里只把三个
ROS 节点放进同一个进程，靠已有话题交接（`rtl_start` → `home_arrived` →
`pad_waypoint`）。单独调试某一段仍可 `python3 -m RTL.rtl_*`。

触发：`ros2 topic pub --once /cuadc_rtl/rtl_start std_msgs/msg/Empty`
"""
from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .rtl_go_home import RtlGoHomeNode
from .rtl_land import RtlLandNode
from .rtl_scan import RtlScanNode


def main():
    rclpy.init()
    home = RtlGoHomeNode()
    scan = RtlScanNode()
    land = RtlLandNode()
    home.get_logger().info('RTL pipeline: go_home + scan + land (one process)')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(home)
    executor.add_node(scan)
    executor.add_node(land)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        for node in (land, scan, home):
            shm = getattr(node, '_shm', None)
            if shm is not None:
                shm.stop()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
