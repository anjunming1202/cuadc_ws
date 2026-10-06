"""飞向扫描给出的垫心 waypoint，GUIDED 下降，飞控对准垫心。

对准方法：镜头相对飞控前约 0.11 m、下约 0.20 m（airframe）。反投影射线从
**镜头**出发，得到垫心在起飞系地面上的坐标——这已经把飞控↔镜头的距离算进去了。
`local_position` 是飞控位置，指令就发这个垫心，**飞控飞到垫心**，不要再减/加 0.11
（减 0.11 会变成相机压圆心，那不是我们要的）。相机那时会在圆心前约 0.11 m，垫半径
0.4 m，仍在垫上。5 m 上连续 `lock_hold_s`（0.5 s）飞控到圆心 ≤ `lock_xy_m`（0.20 m）
才开始降；满 `goto_timeout_s`（10 s）仍未锁也边降边校。3 m 切 LAND **不查水平**。
没扫到垫才飞控 RTL。
触发：`/cuadc_rtl/pad_waypoint`。取帧：投放区同款 10 fps 原始 shm。
"""
from __future__ import annotations

import math
import threading
import time
from enum import Enum
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from cuadc_interface_msgs.msg import PositionTarget
from cuadc_interface_msgs.srv import SetMode
from cuadc_perception import geometry
from cuadc_perception.frame_sources import DEFAULT_RAW_SHM, ShmFrameSource

from .detect_landing_pad import (
    find_landing_pad_center,
    landing_pad_to_takeoff_xy,
    load_landing_pad_config,
)
from .scan_plan import load_rtl_config
from .topics import ARMED, ATTITUDE, CMD_LOCAL, LOCAL_POS, PAD_WAYPOINT, SET_MODE, STAGE, latched_qos


class Phase(Enum):
    WAIT_WP = 'wait_wp'
    GOTO = 'goto'
    DESCEND = 'descend'
    LAND = 'land'
    DONE = 'done'
    FAIL = 'fail'


class RtlLandNode(Node):
    def __init__(self):
        super().__init__('cuadc_rtl_land')
        self._rtl = load_rtl_config()
        self._pad_cfg = load_landing_pad_config()
        self._camera = geometry.load_camera_params('real')

        self._xy: Optional[Tuple[float, float]] = None
        self._z: Optional[float] = None
        self._quat = None
        self._armed: Optional[bool] = None
        self._frame = None
        self._frame_lock = threading.Lock()
        self._shm = None

        self._scan_xy: Optional[Tuple[float, float]] = None  # 扫描给的垫心
        self._pad_xy: Optional[Tuple[float, float]] = None   # 当前用来对准的垫心（可被一帧刷新）
        self._phase = Phase.WAIT_WP
        self._deadline = 0.0
        self._h_cmd: Optional[float] = None
        self._land_sent = False
        self._dt = 0.1
        self._lock_since: Optional[float] = None  # 连续锁住的起始时刻

        latch = latched_qos()
        self.create_subscription(PoseStamped, LOCAL_POS, self._on_pos, 10)
        self.create_subscription(Imu, ATTITUDE, self._on_att, 10)
        self.create_subscription(Bool, ARMED, self._on_armed, 10)
        self.create_subscription(PoseStamped, PAD_WAYPOINT, self._on_waypoint, latch)
        self._cmd = self.create_publisher(PositionTarget, CMD_LOCAL, 10)
        self._stage_pub = self.create_publisher(String, STAGE, latch)
        self._set_mode = self.create_client(SetMode, SET_MODE)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'land idle until {PAD_WAYPOINT}; lock_xy={self._rtl.lock_xy_m}m')

    def _on_pos(self, msg):
        p = msg.pose.position
        self._xy = (p.x, p.y)
        self._z = p.z

    def _on_att(self, msg):
        q = msg.orientation
        self._quat = (q.x, q.y, q.z, q.w)

    def _on_armed(self, msg):
        self._armed = msg.data

    def _on_waypoint(self, msg):
        if self._phase is not Phase.WAIT_WP:
            return
        self._scan_xy = (msg.pose.position.x, msg.pose.position.y)
        self._pad_xy = self._scan_xy
        self._lock_since = None
        self._open_shm()
        self._phase = Phase.GOTO
        self._deadline = time.monotonic() + self._rtl.goto_timeout_s
        self._stage_pub.publish(String(data='land'))
        self.get_logger().info(
            f'pad_waypoint ({self._scan_xy[0]:.2f},{self._scan_xy[1]:.2f}) -> goto')

    def _on_frame(self, frame_bgr):
        with self._frame_lock:
            self._frame = frame_bgr

    def _open_shm(self):
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
            self.get_logger().error('shm 未就绪（扫描应已释放；确认 camera_service）')

    def _pose(self):
        if self._xy is None or self._z is None or self._quat is None:
            return None
        return geometry.VehiclePose(
            position=(self._xy[0], self._xy[1], self._z),
            orientation_xyzw=self._quat,
        )

    def _alt(self) -> Optional[float]:
        if self._z is None:
            return None
        return -self._z

    def _fc_err(self) -> Optional[float]:
        if self._xy is None or self._pad_xy is None:
            return None
        return math.hypot(self._xy[0] - self._pad_xy[0], self._xy[1] - self._pad_xy[1])

    def _locked(self) -> bool:
        err = self._fc_err()
        return err is not None and err <= self._rtl.lock_xy_m

    def _refresh_pad_from_frame(self):
        """下降中看见垫就微更目标；离扫描点太远当误检丢掉。"""
        if self._scan_xy is None:
            return
        with self._frame_lock:
            frame = self._frame
        pose = self._pose()
        if frame is None or pose is None:
            return
        hit = find_landing_pad_center(frame, self._pad_cfg)
        if hit is None:
            return
        xy = landing_pad_to_takeoff_xy(hit, self._camera, pose)
        if xy is None:
            return
        if math.hypot(xy[0] - self._scan_xy[0], xy[1] - self._scan_xy[1]) > self._rtl.outlier_m:
            return
        self._pad_xy = xy

    def _hold(self, x: float, y: float, alt: float):
        self._cmd.publish(PositionTarget(x=float(x), y=float(y), z=float(-alt)))

    def _tick(self):
        p = self._phase
        if p is Phase.WAIT_WP or p is Phase.DONE or p is Phase.FAIL:
            # WAIT_WP：还没收到 pad_waypoint。扫描若没看到垫会切飞控 RTL 且不发这点，本节点空等是正常的。
            return
        now = time.monotonic()
        if p is Phase.GOTO and now > self._deadline:
            self._start_descend(now, '10s no 0.20m lock, descend while aligning')
            return
        if p is Phase.DESCEND and now > self._deadline:
            self.get_logger().error('descend timeout')
            self._phase = Phase.FAIL
            return
        if p is Phase.LAND and now > self._deadline:
            self.get_logger().error('LAND timeout (still armed?)')
            self._phase = Phase.FAIL
            return

        self._refresh_pad_from_frame()
        if self._pad_xy is None:
            return
        fx, fy = self._pad_xy  # 飞控目标 = 垫心；0.11 m 已在反投影光心里，指令不再加减

        if p is Phase.GOTO:
            self._hold(fx, fy, self._rtl.scan_alt_m)
            if self._locked():
                if self._lock_since is None:
                    self._lock_since = now
                elif now - self._lock_since >= self._rtl.lock_hold_s:
                    self._start_descend(now, 'held 0.20m for 0.5s')
            else:
                self._lock_since = None
            return

        if p is Phase.DESCEND:
            # 边降边对准垫心；切 LAND 只看出生高度到 3 m，不要求仍在 0.20 m 内
            alt = self._alt()
            if alt is None or self._h_cmd is None:
                return
            self._h_cmd = min(self._h_cmd, alt)
            self._h_cmd -= self._rtl.descent_mps * self._dt
            floor = max(self._rtl.hand_off_alt_m, alt - self._rtl.h_lead_m)
            self._h_cmd = max(floor, self._h_cmd)
            self._hold(fx, fy, self._h_cmd)
            if alt <= self._rtl.hand_off_alt_m + 0.15:
                self._enter_land()
            return

        if p is Phase.LAND:
            if self._armed is False:
                self._phase = Phase.DONE
                self.get_logger().info('disarmed after LAND')

    def _start_descend(self, now: float, why: str):
        alt = self._alt() or self._rtl.scan_alt_m
        self._h_cmd = alt
        self._phase = Phase.DESCEND
        self._deadline = now + self._rtl.land_timeout_s
        err = self._fc_err()
        self.get_logger().info(
            f'descend ({why}) fc_err={None if err is None else round(err, 2)} alt={alt:.2f}')

    def _enter_land(self):
        # 只看出生高度到 ~3 m，水平再偏也不切 RTL、不等 0.20 m
        self._phase = Phase.LAND
        self._deadline = time.monotonic() + self._rtl.land_timeout_s
        if self._land_sent:
            return
        self._land_sent = True
        if not self._set_mode.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('set_mode 不可用，继续 GUIDED 降')
            self._phase = Phase.DESCEND
            self._land_sent = False
            return
        req = SetMode.Request()
        req.mode = 'LAND'
        self._set_mode.call_async(req)
        self.get_logger().info('hand-off: set_mode LAND')


def main():
    rclpy.init()
    node = RtlLandNode()
    try:
        rclpy.spin(node)
    finally:
        if node._shm is not None:
            node._shm.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
