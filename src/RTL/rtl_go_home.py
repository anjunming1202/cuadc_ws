"""GUIDED 飞回起飞原点。扫 H、降落都不在这里。

收到 `/cuadc_rtl/rtl_start`（Empty，指令名未定）后，向 (0,0) 发
`position_target_local`。水平进入约 0.5 m（`home_tol_m`）且高度到位后停发指令，
并发 `/cuadc_rtl/home_arrived`——扫描脚本拿这个当「可以悬停收帧」的触发。
"""
from __future__ import annotations

import math
import time
from enum import Enum
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Empty, String
from cuadc_interface_msgs.msg import PositionTarget

from .scan_plan import load_rtl_config
from .topics import CMD_LOCAL, HOME_ARRIVED, LOCAL_POS, RTL_START, STAGE, latched_qos


class Phase(Enum):
    IDLE = 'idle'
    GO_HOME = 'go_home'
    ARRIVED = 'arrived'
    FAIL = 'fail'


class RtlGoHomeNode(Node):
    def __init__(self):
        super().__init__('cuadc_rtl_go_home')
        self._rtl = load_rtl_config()
        self._xy: Optional[Tuple[float, float]] = None
        self._z: Optional[float] = None
        self._phase = Phase.IDLE
        self._deadline = 0.0

        self.create_subscription(PoseStamped, LOCAL_POS, self._on_pos, 10)
        self.create_subscription(Empty, RTL_START, self._on_start, 10)
        self._cmd = self.create_publisher(PositionTarget, CMD_LOCAL, 10)
        latch = latched_qos()
        self._arrived_pub = self.create_publisher(Empty, HOME_ARRIVED, latch)
        self._stage_pub = self.create_publisher(String, STAGE, latch)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'go-home ready: alt={self._rtl.scan_alt_m}m tol={self._rtl.home_tol_m}m '
            f'start={RTL_START}')

    def _on_pos(self, msg):
        p = msg.pose.position
        self._xy = (p.x, p.y)
        self._z = p.z

    def _on_start(self, _msg):
        if self._phase is not Phase.IDLE:
            self.get_logger().warn('rtl_start ignored: already running')
            return
        self._phase = Phase.GO_HOME
        self._deadline = time.monotonic() + self._rtl.home_timeout_s
        self._stage_pub.publish(String(data='go_home'))
        self.get_logger().info('rtl_start: GUIDED -> takeoff origin (not FC RTL)')

    def _near_home(self) -> bool:
        if self._xy is None or self._z is None:
            return False
        horiz = math.hypot(self._xy[0], self._xy[1]) <= self._rtl.home_tol_m
        alt = abs(-self._z - self._rtl.scan_alt_m) <= self._rtl.alt_tol_m
        return horiz and alt

    def _tick(self):
        if self._phase is not Phase.GO_HOME:
            return
        if time.monotonic() > self._deadline:
            self.get_logger().error('go-home timeout')
            self._phase = Phase.FAIL
            return
        alt = self._rtl.scan_alt_m
        self._cmd.publish(PositionTarget(x=0.0, y=0.0, z=float(-alt)))
        if self._near_home():
            self._arrived_pub.publish(Empty())
            self._stage_pub.publish(String(data='home_arrived'))
            self._phase = Phase.ARRIVED
            self.get_logger().info(
                f'within {self._rtl.home_tol_m}m of origin; stop cmd, wait scan on {HOME_ARRIVED}')


def main():
    rclpy.init()
    node = RtlGoHomeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
