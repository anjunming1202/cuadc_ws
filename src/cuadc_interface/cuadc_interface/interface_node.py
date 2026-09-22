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
    EkfStatus,
    FailsafeState,
    GpsStatus,
    PositionTarget,
    RelayState,
)
from cuadc_interface_msgs.srv import Arm, GotoLocation, SetMode, SetRelay, Takeoff
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

        # io 线程独占读写的连接 + 派生状态；服务/订阅回调只读 _mode_ids /
        # _mode_names（连接建立后不再变）和 _state（配合 _state_lock）。
        self._master = None
        self._mode_ids = {}
        self._mode_names = {}
        self._state_lock = threading.Lock()
        self._state = {'connected': False, 'armed': False, 'mode': '', 'yaw': 0.0}
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
        self.create_service(SetRelay, '~/set_relay', self._on_set_relay)

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
        self.relay_state_pub = self.create_publisher(RelayState, '~/status/relay_state', 10)
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
            if master.wait_heartbeat(timeout=self._link_timeout) is None:
                self.get_logger().warn(
                    f'no heartbeat from {self._connection_str}, retrying ...')
                continue

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
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = self._mode_names.get(msg.custom_mode, '')
        with self._state_lock:
            self._state['armed'] = armed
            self._state['mode'] = mode
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
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = msg.x
        pose.pose.position.y = msg.y
        pose.pose.position.z = msg.z
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

    def _on_set_relay(self, request, response):
        ok = self._send_command_and_wait_ack(
            mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
            (request.channel, 1 if request.state else 0, 0, 0, 0, 0, 0),
            self._command_ack_timeout)
        response.success = ok
        if ok:
            self.relay_state_pub.publish(
                RelayState(channel=request.channel, state=request.state))
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
        # MAV_FRAME_LOCAL_NED：x/y/z 是绝对局部坐标（相对起飞点），跟
        # _on_position_target 的机身相对偏移是两回事——这个 frame 下 yaw
        # 字段是绝对量，锁定当前朝向要显式回填当前角度（不能像
        # BODY_OFFSET_NED 那样发 0），不然会落回默认的 WP_YAW_BEHAVIOR。
        with self._state_lock:
            yaw = self._state['yaw']
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000101111111000,
            msg.x, msg.y, msg.z,
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
