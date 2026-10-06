"""原点附近找 H 垫。不负责飞回原点。

只在收到 latched `/cuadc_rtl/home_arrived` 之后才开始（回原点已确认 0.5 m+高度，
这里不再查一遍）。立刻在 (0,0) 悬停 `dwell_s`（2 s）。有圆心发 `pad_waypoint`；
2 s 内没有任何垫则 `set_mode(RTL)`，不发 waypoint。位移巡回代码保留但注释掉。
"""
from __future__ import annotations

import math
import threading
import time
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, String
from cuadc_interface_msgs.msg import PositionTarget
from cuadc_interface_msgs.srv import SetMode
from cuadc_perception import geometry
from cuadc_perception.frame_sources import DEFAULT_RAW_SHM, ShmFrameSource

from .cluster_hits import consensus_xy
from .detect_landing_pad import (
    find_landing_pad_center,
    landing_pad_to_takeoff_xy,
    load_landing_pad_config,
)
from .scan_plan import load_rtl_config
# from .scan_plan import ground_footprint_m, scan_waypoints  # 位移扫描用，已停
from .topics import (
    ATTITUDE,
    CMD_LOCAL,
    HOME_ARRIVED,
    LOCAL_POS,
    PAD_WAYPOINT,
    SET_MODE,
    STAGE,
    latched_qos,
)


class Phase(Enum):
    WAIT_HOME = 'wait_home'   # 只等 home_arrived，0.5 m/高度由回原点段确认
    SCAN = 'scan'
    DONE = 'done'
    FAIL = 'fail'


def sample_pad_xy(frame_bgr, camera, pose, pad_cfg=None) -> Optional[Tuple[float, float]]:
    """一帧 → 起飞系垫心；没垫或射线失败则 None。"""
    if frame_bgr is None or pose is None:
        return None
    hit = find_landing_pad_center(frame_bgr, pad_cfg)
    if hit is None:
        return None
    return landing_pad_to_takeoff_xy(hit, camera, pose)


class RtlScanNode(Node):
    def __init__(self):
        super().__init__('cuadc_rtl_scan')
        self._rtl = load_rtl_config()
        self._pad_cfg = load_landing_pad_config()
        self._camera = geometry.load_camera_params('real')
        # 位移巡回（约 ±0.9 m 三点）先关掉，只在原点悬停收帧。
        # self._waypoints = scan_waypoints(self._rtl)

        self._xy: Optional[Tuple[float, float]] = None
        self._z: Optional[float] = None
        self._quat = None
        self._frame = None
        self._frame_lock = threading.Lock()
        self._shm = None

        self._phase = Phase.WAIT_HOME
        self._dwell_end = 0.0
        self._leg_deadline = 0.0
        self._hits: List[Tuple[float, float]] = []
        self._home_signal = False
        self._rtl_sent = False

        self.create_subscription(PoseStamped, LOCAL_POS, self._on_pos, 10)
        self.create_subscription(Imu, ATTITUDE, self._on_att, 10)
        latch = latched_qos()
        self.create_subscription(Empty, HOME_ARRIVED, self._on_home_arrived, latch)
        self._cmd = self.create_publisher(PositionTarget, CMD_LOCAL, 10)
        self._wp_pub = self.create_publisher(PoseStamped, PAD_WAYPOINT, latch)
        self._stage_pub = self.create_publisher(String, STAGE, latch)
        self._set_mode = self.create_client(SetMode, SET_MODE)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'scan idle until {HOME_ARRIVED}; then hover {self._rtl.dwell_s}s')

    def _on_pos(self, msg):
        p = msg.pose.position
        self._xy = (p.x, p.y)
        self._z = p.z

    def _on_att(self, msg):
        q = msg.orientation
        self._quat = (q.x, q.y, q.z, q.w)

    def _on_home_arrived(self, _msg):
        self._home_signal = True

    def _on_frame(self, frame_bgr):
        with self._frame_lock:
            self._frame = frame_bgr

    def _near_origin(self) -> bool:
        if self._xy is None:
            return False
        return math.hypot(self._xy[0], self._xy[1]) <= self._rtl.home_tol_m

    def _open_shm(self):
        """投放区同款 10 fps 原始 NV12 shm。感知若仍占 shmsrc 会失败。"""
        if self._shm is not None:
            return
        src = ShmFrameSource(
            raw_shm=DEFAULT_RAW_SHM,
            src_w=1920, src_h=1080, src_fps=10,
            out_w=self._rtl.cv_width, out_h=self._rtl.cv_height,
        )
        if src.start(self._on_frame):
            self._shm = src
            self.get_logger().info(f'shm 10fps raw: {DEFAULT_RAW_SHM}')
        else:
            self.get_logger().error('shm 未就绪（先停 cuadc_perception_cv，确认 camera_service）')

    def _pose(self):
        if self._xy is None or self._z is None or self._quat is None:
            return None
        return geometry.VehiclePose(
            position=(self._xy[0], self._xy[1], self._z),
            orientation_xyzw=self._quat,
        )

    def _hold(self, x: float, y: float):
        self._cmd.publish(PositionTarget(
            x=float(x), y=float(y), z=float(-self._rtl.scan_alt_m)))

    def _alt_ok(self) -> bool:
        if self._z is None:
            return False
        return abs(-self._z - self._rtl.scan_alt_m) <= self._rtl.alt_tol_m

    def _collect_if_any(self):
        with self._frame_lock:
            frame = self._frame
        xy = sample_pad_xy(frame, self._camera, self._pose(), self._pad_cfg)
        if xy is not None:
            self._hits.append(xy)

    def _begin_hover(self):
        self._open_shm()
        self._hits.clear()
        now = time.monotonic()
        self._dwell_end = now + self._rtl.dwell_s
        self._leg_deadline = now + self._rtl.leg_timeout_s
        self._phase = Phase.SCAN
        self._stage_pub.publish(String(data='scan'))
        self.get_logger().info(
            f'home_arrived: hover {self._rtl.dwell_s}s (gate already checked by go-home)')

    def _tick(self):
        p = self._phase
        if p is Phase.DONE or p is Phase.FAIL:
            return
        now = time.monotonic()

        if p is Phase.WAIT_HOME:
            if self._home_signal:
                self._begin_hover()
            return

        if now > self._leg_deadline:
            self.get_logger().error('hover/detect timeout')
            self._finish(None)
            return

        # 停在起飞原点收帧，不走位移航点。
        self._hold(0.0, 0.0)
        self._collect_if_any()
        if now >= self._dwell_end:
            self._finish(consensus_xy(self._hits, outlier_m=self._rtl.outlier_m))

        # --- 位移巡回（注释保留，不要删）---
        # if self._wp_i >= len(self._waypoints):
        #     self._publish_waypoint()
        #     return
        # tx, ty = self._waypoints[self._wp_i]
        # self._hold(tx, ty)
        # if not self._arrived_for_dwell:
        #     if self._horiz_ok(tx, ty, self._rtl.arrive_tol_m) and self._alt_ok():
        #         self._arrived_for_dwell = True
        #         self._dwell_end = now + self._rtl.dwell_s
        #     return
        # self._collect_if_any()
        # if now >= self._dwell_end:
        #     self._wp_i += 1
        #     self._arrived_for_dwell = False
        #     self._leg_deadline = time.monotonic() + self._rtl.leg_timeout_s

    def _finish(self, xy: Optional[Tuple[float, float]]):
        """有圆心 → 发 pad_waypoint 给降落；没有 → 飞控 RTL。"""
        self._release_shm()
        if xy is None:
            # 2 s 内一帧垫都没有：这里切飞控 RTL。不发 pad_waypoint，
            # 降落节点继续等 waypoint 是故意的——贴地已交给飞控 RTL，不要再 GUIDED 降。
            self.get_logger().warn(f'no pad hits ({len(self._hits)}); FC RTL (land node stays idle)')
            self._stage_pub.publish(String(data='fc_rtl'))
            self._call_fc_rtl()
            self._phase = Phase.DONE
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'takeoff'
        msg.pose.position.x = xy[0]
        msg.pose.position.y = xy[1]
        msg.pose.position.z = -self._rtl.scan_alt_m
        self._wp_pub.publish(msg)
        self._stage_pub.publish(String(data='pad_waypoint'))
        self._phase = Phase.DONE
        self.get_logger().info(
            f'pad waypoint ({xy[0]:.2f},{xy[1]:.2f}) hits={len(self._hits)}; shm released')

    def _call_fc_rtl(self):
        if self._rtl_sent:
            return
        self._rtl_sent = True
        if not self._set_mode.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('set_mode 不可用，无法 FC RTL')
            self._phase = Phase.FAIL
            return
        req = SetMode.Request()
        req.mode = 'RTL'
        self._set_mode.call_async(req)

    def _release_shm(self):
        if self._shm is not None:
            self._shm.stop()
            self._shm = None


def main():
    rclpy.init()
    node = RtlScanNode()
    try:
        rclpy.spin(node)
    finally:
        if node._shm is not None:
            node._shm.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
