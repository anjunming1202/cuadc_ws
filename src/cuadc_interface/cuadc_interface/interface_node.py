"""接口层节点：ROS2 <-> MAVLink 桥接，契约见 docs/spec/04-interfaces.md。

线程模型：一个后台线程（_io_main）独占读取 pymavlink 连接
（master.recv_match），是唯一读取者——pymavlink 的连接对象不是为多线程
并发读设计的，两个线程同时 recv_match 会互相抢消息。服务/订阅回调跑在
rclpy 的执行器线程里，只允许"发"（master.mav.xxx_send，底层是 socket
写，跟另一个线程的 socket 读互不冲突），不允许直接调用 recv_match；
需要等待飞控确认（比如 Arm）时，通过 _pending_acks 这个共享字典 +
threading.Event 跟 io 线程同步，不是自己去读。
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from pymavlink import mavutil

from cuadc_interface_msgs.msg import (
    CommandAck,
    DropStatus,
    EkfStatus,
    FailsafeState,
    GpsStatus,
    PositionTarget,
)
from cuadc_interface_msgs.srv import Arm, GotoLocation, ReleasePayload, SetMode, Takeoff
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Bool, Float64, String

# EKF_STATUS_REPORT.flags 判定"位置估计可信"的两个必要位，跟
# cuadc_sim/sitl_smoke_test.py 用的是同一组——早于这两位置位时飞控会
# ACCEPT 解锁/起飞请求，但高度基准不稳，几秒后自动上锁。
_EKF_POS_READY = (mavutil.mavlink.EKF_POS_HORIZ_ABS
                  | mavutil.mavlink.EKF_POS_VERT_ABS)

# 只是粗略的"失效保护激活"信号，system_status 是飞控整体健康状态，不是
# 专门的 failsafe 位域；更精确的判定（区分具体哪种 failsafe：低电/失联/
# GPS）留到实际接联调试时按需细化，见 04-interfaces.md"待验证事项"。
_FAILSAFE_SYSTEM_STATUS = (
    mavutil.mavlink.MAV_STATE_CRITICAL,
    mavutil.mavlink.MAV_STATE_EMERGENCY,
)

# 不是飞控本体的心跳来源：地面站（Mission Planner 经数传连飞控时，飞控会把它的心跳转发到
# 所有口，一样进 fanout）、云台等。只认飞控心跳——否则 armed/mode 会在飞控和地面站的
# "未解锁/STABILIZE" 之间来回跳（2026-10-04 实测），模式确认、夺控判定、起飞点原点全会错。
_NON_VEHICLE_TYPES = frozenset({
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_TYPE_GIMBAL,
    mavutil.mavlink.MAV_TYPE_ADSB,
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
})


def _is_vehicle_heartbeat(msg):
    return (msg.type not in _NON_VEHICLE_TYPES
            and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            and msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_GIMBAL)


# RC_CHANNELS_OVERRIDE：65535 = 忽略该通道（不改其覆盖状态），0 = 释放该通道回遥控器。
# 投放只碰投放那一个 RC 通道，其余全填忽略，绝不动飞控/飞行通道，见 docs/spec/adr/0007。
_RC_OVERRIDE_IGNORE = 65535
_RC_OVERRIDE_RELEASE = 0
_RC_OVERRIDE_CHANNELS = 18
# ArduPilot RC override 有超时（不刷新会回落遥控器），投放期间按此周期重发保活。
_RC_OVERRIDE_REFRESH_S = 0.1


def _quaternion_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class InterfaceNode(Node):

    def __init__(self):
        super().__init__('cuadc_interface')

        self.declare_parameter('connection', 'udp:127.0.0.1:14550')
        self.declare_parameter('link_timeout_sec', 5.0)
        self.declare_parameter('command_ack_timeout_sec', 5.0)
        self.declare_parameter('set_mode_timeout_sec', 5.0)
        self._connection_str = self.get_parameter('connection').value
        self._link_timeout = self.get_parameter('link_timeout_sec').value
        self._command_ack_timeout = self.get_parameter('command_ack_timeout_sec').value
        self._set_mode_timeout = self.get_parameter('set_mode_timeout_sec').value

        # 投放执行器参数（属接口层，见 docs/spec/adr/0007）。投放通道平时归遥控器（飞控设
        # SERVO*_FUNCTION=RCIN<ch> 直通），仅投放时本节点在该 RC 通道发定时 override 接管。
        # 默认 = 硬件子队真机验证值；drop_run_s 必须按验证的投放动作时长设置（真机）。
        self.declare_parameter('drop_rc_channel', 9)       # 投放 RC 通道（1..18），对应飞控 RCIN 直通
        self.declare_parameter('drop_forward_us', 1100)    # 投放（放料）方向 PWM
        self.declare_parameter('drop_neutral_us', 1500)    # 连续舵机停止/中立 PWM
        self.declare_parameter('drop_run_s', 2.0)          # 投放方向持续时长（真机按验证值设）
        self._drop_rc_channel = int(self.get_parameter('drop_rc_channel').value)
        self._drop_forward_us = int(self.get_parameter('drop_forward_us').value)
        self._drop_neutral_us = int(self.get_parameter('drop_neutral_us').value)
        self._drop_run_s = float(self.get_parameter('drop_run_s').value)

        # io 线程独占读写的连接 + 派生状态；服务/订阅回调只读 _mode_ids /
        # _mode_names（连接建立后不再变）和 _state（配合 _state_lock）。
        self._master = None
        self._mode_ids = {}
        self._mode_names = {}
        self._state_lock = threading.Lock()
        self._state = {'connected': False, 'armed': False, 'mode': '', 'yaw': 0.0}
        # 起飞点原点（EKF 原点系 NED）：local_position 发布前减它、position_target_local 下发前加它，
        # 使对外坐标都相对起飞点。飞控 EKF 原点不在起飞点（实机实测 z 偏 0.9–2.1m、xy 偏数米）。
        # 原点 = 最近一次解锁瞬间的位置（同 ArduPilot 在解锁时设 home），上锁后保持（落地后读数仍
        # 相对起飞点，返航误差可量）；从未解锁过时跟随当前位置。
        self._origin_ned = None
        self._raw_ned = None
        self._armed_once = False
        self._seen_heartbeat = False
        self._vehicle_sysid = None
        self._vehicle_compid = None
        self._ack_lock = threading.Lock()
        self._pending_acks = {}  # command id -> threading.Event
        self._ack_results = {}  # command id -> COMMAND_ACK.result
        self._shutdown = threading.Event()

        self._create_services()
        self._create_subscriptions()
        self._create_publishers()

        self._io_thread = threading.Thread(target=self._io_main, daemon=True)
        self._io_thread.start()

    # -- 建图/发布/订阅声明 --

    def _create_services(self):
        """决策 <-> 飞控接口层，双向、要等回应：一次性动作，需确认成不成功。"""
        self.create_service(Arm, '~/arm', self._on_arm)
        self.create_service(SetMode, '~/set_mode', self._on_set_mode)
        self.create_service(Takeoff, '~/takeoff', self._on_takeoff)
        self.create_service(GotoLocation, '~/goto_location', self._on_goto_location)
        self.create_service(ReleasePayload, '~/release_payload', self._on_release_payload)

    def _create_subscriptions(self):
        """决策 -> 飞控接口层，单向、不等回应：持续流式下发的指令。"""
        self.create_subscription(
            PositionTarget, '~/cmd/position_target', self._on_position_target, 10)
        self.create_subscription(
            PositionTarget, '~/cmd/position_target_local', self._on_position_target_local, 10)
        self.create_subscription(TwistStamped, '~/cmd/velocity', self._on_velocity, 10)
        self.create_subscription(Float64, '~/cmd/yaw', self._on_yaw, 10)

    def _create_publishers(self):
        """飞控接口层 -> 决策，单向、不等回应：遥测广播。"""
        self.connected_pub = self.create_publisher(Bool, '~/status/connected', 10)
        self.armed_pub = self.create_publisher(Bool, '~/status/armed', 10)
        self.mode_pub = self.create_publisher(String, '~/status/mode', 10)
        self.failsafe_pub = self.create_publisher(FailsafeState, '~/status/failsafe', 10)
        self.drop_status_pub = self.create_publisher(DropStatus, '~/status/drop', 10)
        self.command_ack_pub = self.create_publisher(CommandAck, '~/status/command_ack', 10)

        self.global_position_pub = self.create_publisher(
            NavSatFix, '~/telemetry/global_position', 10)
        self.local_position_pub = self.create_publisher(
            PoseStamped, '~/telemetry/local_position', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '~/telemetry/velocity', 10)
        self.attitude_pub = self.create_publisher(Imu, '~/telemetry/attitude', 10)
        self.battery_pub = self.create_publisher(BatteryState, '~/telemetry/battery', 10)
        self.gps_status_pub = self.create_publisher(GpsStatus, '~/telemetry/gps_status', 10)
        self.ekf_status_pub = self.create_publisher(EkfStatus, '~/telemetry/ekf_status', 10)

    # -- io 线程：连接管理 + 读循环（唯一调用 recv_match 的地方） --

    def _io_main(self):
        while not self._shutdown.is_set() and rclpy.ok():
            master = self._connect_with_retry(self._connection_str)
            if master is None:
                continue  # _shutdown was set while retrying
            # mavutil.mavlink_connection() 对 UDP 目标总是"成功"返回
            # （UDP 无连接握手，对方端口没人监听也一样能建出这个对象）
            # ——真正确认"连上了"必须等到第一条心跳，同 cuadc_sim 的验证
            # 脚本一致，不能只看 mavlink_connection() 有没有抛异常。
            vehicle_hb = self._wait_vehicle_heartbeat(master)
            if vehicle_hb is None:
                self.get_logger().warn(
                    f'no vehicle heartbeat from {self._connection_str}, retrying ...')
                continue
            # 只跟这个飞控说话、只收它的消息（见 _is_vehicle_heartbeat）。
            self._vehicle_sysid = vehicle_hb.get_srcSystem()
            self._vehicle_compid = vehicle_hb.get_srcComponent()
            master.target_system = self._vehicle_sysid
            master.target_component = self._vehicle_compid

            self._master = master
            self._mode_ids = master.mode_mapping() or {}
            self._mode_names = {v: k for k, v in self._mode_ids.items()}
            with self._state_lock:
                self._state['connected'] = True
            self._safe_publish(self.connected_pub, Bool(data=True))
            self.get_logger().info(f'connected to {self._connection_str}')

            master.mav.request_data_stream_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

            self._read_loop(master)

            with self._state_lock:
                self._state['connected'] = False
            self._safe_publish(self.connected_pub, Bool(data=False))
            self.get_logger().warn('link lost, reconnecting ...')
            self._master = None

    def _wait_vehicle_heartbeat(self, master):
        # 不能用 master.wait_heartbeat()：它返回任意来源的第一条心跳，先到的若是地面站，
        # 此时还不知道飞行器类型，mode_mapping() 为空，之后每次 set_mode 都直接失败。
        deadline = time.monotonic() + self._link_timeout
        while time.monotonic() < deadline and not self._shutdown.is_set():
            msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if msg is not None and _is_vehicle_heartbeat(msg):
                return msg
        return None

    def _connect_with_retry(self, connection):
        # 跟 cuadc_sim 的一次性验证脚本不同：这是常驻桥接节点，连不上不该
        # 直接抛异常退出，应该一直重试、同时把 connected=False 报出去，
        # 让规划/决策能感知到"链路还没通"，而不是节点直接崩溃。
        attempt = 0
        while not self._shutdown.is_set():
            try:
                return mavutil.mavlink_connection(connection)
            except OSError as e:
                attempt += 1
                if attempt % 10 == 1:
                    self.get_logger().warn(f'connect to {connection} failed: {e}, retrying ...')
                self._shutdown.wait(2.0)
        return None

    def _read_loop(self, master):
        last_msg_time = time.monotonic()
        while not self._shutdown.is_set() and rclpy.ok():
            msg = master.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                if time.monotonic() - last_msg_time > self._link_timeout:
                    return
                continue
            if msg.get_srcSystem() != self._vehicle_sysid:
                continue  # 地面站等其它系统转发过来的消息
            last_msg_time = time.monotonic()
            handler = self._HANDLERS.get(msg.get_type())
            if handler is not None:
                handler(self, master, msg)

    def _safe_publish(self, publisher, msg):
        # io 线程和 rclpy 关闭时序是独立的：节点关闭过程中 io 线程可能还
        # 卡在上一轮循环里，这时候 publish 会因为 context 已失效而抛
        # RCLError。main() 会先 set _shutdown 再 join 这个线程，缩小了
        # 窗口，但读 recv_match 的阻塞调用不能立刻中断，完全消除不了这
        # 个竞态，所以这里兜底吞掉"关闭过程中的 publish 失败"，不吞真实
        # bug（rclpy.ok() 还是 True 的话说明不是关闭引起的，照样抛出）。
        try:
            publisher.publish(msg)
        except Exception as e:
            if rclpy.ok():
                raise
            self.get_logger().debug(f'publish during shutdown ignored: {e}')

    # -- MAVLink -> ROS2：逐个消息类型的处理 --

    def _handle_heartbeat(self, master, msg):
        if msg.get_srcComponent() != self._vehicle_compid or not _is_vehicle_heartbeat(msg):
            return
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = self._mode_names.get(msg.custom_mode, '')
        with self._state_lock:
            was_armed = self._state['armed']
            first = not self._seen_heartbeat
            self._seen_heartbeat = True
            self._state['armed'] = armed
            self._state['mode'] = mode
            if armed and not was_armed:
                self._armed_once = True
                self._origin_ned = self._raw_ned  # None 时由下一条 LOCAL_POSITION_NED 补上
                if first:
                    # 起来时已在解锁状态（空中重启接口层）：拿不到真正的起飞点，退化为当前位置。
                    self.get_logger().warn(
                        'interface started while ARMED: takeoff origin unknown, using current '
                        'position -- local_position is NOT relative to the takeoff point')
                elif self._origin_ned is not None:
                    self.get_logger().info(
                        'armed: takeoff origin frozen at EKF-frame '
                        f'({self._origin_ned[0]:+.2f}, {self._origin_ned[1]:+.2f}, '
                        f'{self._origin_ned[2]:+.2f})')
        self._safe_publish(self.armed_pub, Bool(data=armed))
        self._safe_publish(self.mode_pub, String(data=mode))

        failsafe = msg.system_status in _FAILSAFE_SYSTEM_STATUS
        reason = mavutil.mavlink.enums['MAV_STATE'][msg.system_status].name
        self._safe_publish(self.failsafe_pub, FailsafeState(active=failsafe, reason=reason))

    def _handle_global_position_int(self, master, msg):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.latitude = msg.lat / 1e7
        fix.longitude = msg.lon / 1e7
        fix.altitude = msg.alt / 1000.0
        self._safe_publish(self.global_position_pub, fix)

    def _handle_local_position_ned(self, master, msg):
        with self._state_lock:
            self._raw_ned = (msg.x, msg.y, msg.z)
            if not self._armed_once or self._origin_ned is None:
                self._origin_ned = self._raw_ned
            ox, oy, oz = self._origin_ned
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = msg.x - ox
        pose.pose.position.y = msg.y - oy
        pose.pose.position.z = msg.z - oz
        self._safe_publish(self.local_position_pub, pose)

        twist = TwistStamped()
        twist.header.stamp = pose.header.stamp
        twist.twist.linear.x = msg.vx
        twist.twist.linear.y = msg.vy
        twist.twist.linear.z = msg.vz
        self._safe_publish(self.velocity_pub, twist)

    def _handle_attitude(self, master, msg):
        # MAV_FRAME_LOCAL_NED 下 yaw 字段是绝对量，_on_position_target_local
        # 锁定当前朝向时需要拿这个值回填——不是给 BODY_OFFSET_NED 用的
        # （那个 frame 下发 0 就够了，不需要读这个）。
        with self._state_lock:
            self._state['yaw'] = msg.yaw
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.angular_velocity.x = msg.rollspeed
        imu.angular_velocity.y = msg.pitchspeed
        imu.angular_velocity.z = msg.yawspeed
        (imu.orientation.x, imu.orientation.y,
         imu.orientation.z, imu.orientation.w) = _quaternion_from_euler(
            msg.roll, msg.pitch, msg.yaw)
        # 加速度计原始数据不在这条消息里，不适用，用 -1 标成"未提供"
        # （sensor_msgs/Imu 的约定写法）。
        imu.linear_acceleration_covariance[0] = -1.0
        self._safe_publish(self.attitude_pub, imu)

    def _handle_sys_status(self, master, msg):
        battery = BatteryState()
        battery.voltage = msg.voltage_battery / 1000.0
        battery.current = msg.current_battery / 100.0 if msg.current_battery >= 0 else float('nan')
        battery.percentage = (
            msg.battery_remaining / 100.0 if msg.battery_remaining >= 0 else float('nan'))
        self._safe_publish(self.battery_pub, battery)

    def _handle_gps_raw_int(self, master, msg):
        status = GpsStatus()
        status.fix_type = msg.fix_type
        status.satellites_visible = msg.satellites_visible
        status.hdop = msg.eph / 100.0
        self._safe_publish(self.gps_status_pub, status)

    def _handle_ekf_status_report(self, master, msg):
        status = EkfStatus()
        status.healthy = (msg.flags & _EKF_POS_READY) == _EKF_POS_READY
        status.status_text = '' if status.healthy else 'position estimate not yet reliable'
        self._safe_publish(self.ekf_status_pub, status)

    def _handle_command_ack(self, master, msg):
        with self._ack_lock:
            event = self._pending_acks.get(msg.command)
            if event is not None:
                self._ack_results[msg.command] = msg.result
                event.set()
        accepted = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        self._safe_publish(
            self.command_ack_pub, CommandAck(command=str(msg.command), result=accepted))

    _HANDLERS = {
        'HEARTBEAT': _handle_heartbeat,
        'GLOBAL_POSITION_INT': _handle_global_position_int,
        'LOCAL_POSITION_NED': _handle_local_position_ned,
        'ATTITUDE': _handle_attitude,
        'SYS_STATUS': _handle_sys_status,
        'GPS_RAW_INT': _handle_gps_raw_int,
        'EKF_STATUS_REPORT': _handle_ekf_status_report,
        'COMMAND_ACK': _handle_command_ack,
    }

    # -- 指令回调：ROS2 -> MAVLink（跑在 rclpy 执行器线程，只发不读） --

    def _send_command_and_wait_ack(self, command, params, timeout):
        master = self._master
        if master is None:
            return False
        event = threading.Event()
        with self._ack_lock:
            self._pending_acks[command] = event
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                command, 0, *params)
            if not event.wait(timeout):
                return False
            with self._ack_lock:
                result = self._ack_results.pop(command, None)
            return result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        finally:
            with self._ack_lock:
                self._pending_acks.pop(command, None)

    def _on_arm(self, request, response):
        # 只发一次、不重试——PreArm 没过时应该拒绝并如实返回，是否重试、
        # 隔多久重试是任务级判断（规划/决策的事），不是接口层该替它决定
        # 的（04-interfaces.md 边界表）。
        ok = self._send_command_and_wait_ack(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (1 if request.arm else 0, 0, 0, 0, 0, 0, 0),
            self._command_ack_timeout)
        response.success = ok
        response.message = '' if ok else 'rejected or no ack (see PreArm/link status)'
        return response

    def _on_set_mode(self, request, response):
        master = self._master
        mode_id = self._mode_ids.get(request.mode)
        if master is None or mode_id is None:
            response.success = False
            return response
        # SET_MODE 是普通消息，不是 COMMAND_LONG，没有 COMMAND_ACK；用
        # HEARTBEAT.custom_mode 确认是否切成功，同 cuadc_sim 里验证过的
        # 做法——ArduPilot 偶尔会丢第一条 SET_MODE，重发直到心跳确认。
        deadline = time.monotonic() + self._set_mode_timeout
        while time.monotonic() < deadline:
            master.set_mode(mode_id)
            time.sleep(0.5)
            with self._state_lock:
                if self._state['mode'] == request.mode:
                    response.success = True
                    return response
        response.success = False
        return response

    def _on_takeoff(self, request, response):
        # ArduCopter GUIDED 模式必须先收到 MAV_CMD_NAV_TAKEOFF 才会真正
        # 离地，Go To Location/Set Velocity 对停在地面的飞机不生效
        # （2026-07-15 SITL 实测确认，见 04-interfaces.md）。
        ok = self._send_command_and_wait_ack(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            (0, 0, 0, 0, 0, 0, request.alt),
            self._command_ack_timeout)
        response.success = ok
        return response

    def _on_goto_location(self, request, response):
        master = self._master
        if master is None:
            response.success = False
            return response
        # SET_POSITION_TARGET_GLOBAL_INT 没有协议层 ack（不是
        # COMMAND_LONG），success 只代表"发出去了"，不代表"到了"——到没
        # 到由规划/决策自己比对遥测位置判断，见 04-interfaces.md。
        master.mav.set_position_target_global_int_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(request.lat * 1e7), int(request.lon * 1e7), request.alt,
            0, 0, 0, 0, 0, 0, 0, 0)
        response.success = True
        return response

    def _send_rc_override(self, channel, value):
        """在单个 RC 通道上发一次 override（其余通道填 65535=忽略，绝不动飞行通道）。
        value=PWM 覆盖该通道；value=0 释放该通道回遥控器。见 docs/spec/adr/0007。"""
        master = self._master
        if master is None:
            return
        chans = [_RC_OVERRIDE_IGNORE] * _RC_OVERRIDE_CHANNELS
        chans[channel - 1] = value
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component, *chans)

    def _on_release_payload(self, request, response):
        # 投放执行：仅本服务被调用时（决策进 DROP_PAYLOAD）在投放 RC 通道上短暂 override，
        # override→forward 跑 drop_run_s → neutral 停 → 释放回遥控器，物理动作完成才返回。
        # 其余任何时刻本节点零 override，投放通道归遥控器（见 docs/spec/adr/0007）。
        # armed 守卫：地面（未解锁）不由软件驱动投放，交给遥控器/人工。
        with self._state_lock:
            armed = self._state['armed']
        if not armed:
            response.success = False
            response.message = 'refused: not armed (disarmed drop is manual/RC per ADR-0007)'
            return response
        if self._master is None:
            response.success = False
            response.message = 'no MAVLink link'
            return response

        ch = self._drop_rc_channel
        # 投放方向持续 drop_run_s，其间按刷新周期重发 override 保活（ArduPilot override 会超时回落）。
        deadline = time.monotonic() + self._drop_run_s
        while time.monotonic() < deadline and not self._shutdown.is_set():
            self._send_rc_override(ch, self._drop_forward_us)
            time.sleep(_RC_OVERRIDE_REFRESH_S)
        # 连续舵机必须先停在中立，再释放回遥控器。
        self._send_rc_override(ch, self._drop_neutral_us)
        time.sleep(_RC_OVERRIDE_REFRESH_S)
        self._send_rc_override(ch, _RC_OVERRIDE_RELEASE)

        self.drop_status_pub.publish(DropStatus(actuated=True))
        response.success = True
        response.message = ''
        return response

    def _on_position_target(self, msg):
        master = self._master
        if master is None:
            return
        # MAV_FRAME_BODY_OFFSET_NED：x/y/z 是相对当前位置和当前朝向的
        # 偏移量（前/右/下），飞控收到指令那一刻用自己最新状态现算，不是
        # 我们这边读一次遥测再加——避免遥测往返延迟导致目标基于过时位置
        # 算出来。yaw=0（这个 frame 下 yaw 也是相对量）就是"不转"，不需要
        # 额外锁定当前朝向；Go To Location 保留绝对坐标 + 默认自动朝向，
        # 刻意跟这个区分开，见 04-interfaces.md"自动朝向行为"一节。
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000101111111000,
            msg.x, msg.y, msg.z,
            0, 0, 0, 0, 0, 0, 0.0, 0)

    def _on_position_target_local(self, msg):
        master = self._master
        if master is None:
            return
        # MAV_FRAME_LOCAL_NED：x/y/z 是相对起飞点的绝对局部坐标，下发前加回起飞点原点换成
        # 飞控的 EKF 原点系。跟 _on_position_target 的机身相对偏移是两回事——这个 frame 下 yaw
        # 字段是绝对量，锁定当前朝向要显式回填当前角度（不能像
        # BODY_OFFSET_NED 那样发 0），不然会落回默认的 WP_YAW_BEHAVIOR。
        with self._state_lock:
            yaw = self._state['yaw']
            origin = self._origin_ned
        if origin is None:
            self.get_logger().warn('position_target_local dropped: no local position yet')
            return
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000101111111000,
            msg.x + origin[0], msg.y + origin[1], msg.z + origin[2],
            0, 0, 0, 0, 0, 0, yaw, 0)

    def _on_velocity(self, msg):
        master = self._master
        if master is None:
            return
        # MAV_FRAME_BODY_OFFSET_NED：vx/vy/vz 是相对当前朝向的机身系速度
        # （前/右/下），不是地理北/东/地——跟 Set Position Target 同样的
        # 理由（遥控/视觉伺服场景天然是机身系直觉）。yaw_rate（rad/s）取自
        # twist.angular.z 并在 type_mask 里标为“有效”（不再忽略）：发 0 就是
        # 锁定当前航向、不自动转向，不发（默认 0）也锁航向。这样 Set Velocity
        # 跟两个 Set Position Target 一样不自动转向——视觉伺服连续发速度时飞控
        # 不再把机头转向飞行方向而绕圈，见 04-interfaces.md"自动朝向行为"一节。
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000011111000111,
            0, 0, 0,
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
            0, 0, 0, 0, msg.twist.angular.z)

    def _on_yaw(self, msg):
        master = self._master
        if master is None:
            return
        # MAV_CMD_CONDITION_YAW 的 param1 单位是度，契约里 ~/cmd/yaw 是
        # 弧度（ROS 惯例），这里要转一次单位；还要归一化到 [0,360)——
        # ArduCopter 收到负数 param1 会直接拒绝该指令（源码校验
        # param1>=0），不归一化的话负角度目标会被飞控静默丢弃。
        yaw_deg = math.degrees(msg.data) % 360.0
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
            yaw_deg, 0, 0, 0, 0, 0, 0)


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceNode()
    try:
        rclpy.spin(node)
    finally:
        # 先叫停 io 线程再拆 rclpy：_shutdown 让线程内部循环在下一次检查
        # 点退出（读循环最多卡 1s 的 recv_match 超时，重连等待最多 2s），
        # join 给够这个窗口，尽量避免线程还在跑的时候 rclpy 的 context
        # 已经失效（_safe_publish 兜的是兜不住的剩余竞态，不是替代这步）。
        node._shutdown.set()
        node._io_thread.join(timeout=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
