"""接口层节点骨架——只声明 docs/spec/04-interfaces.md 里的话题/服务
形状，不含任何 MAVLink/pymavlink 协议实现。契约签字确认后，先按这份
骨架搭桩节点验证规划/决策，再在这里补真实的协议逻辑。"""

import rclpy
from rclpy.node import Node

from cuadc_interface_msgs.msg import (
    CommandAck,
    EkfStatus,
    FailsafeState,
    GpsStatus,
    PositionTarget,
    RelayState,
)
from cuadc_interface_msgs.srv import Arm, GotoLocation, SetMode, SetRelay
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Bool, Float64, String


class InterfaceNode(Node):

    def __init__(self):
        super().__init__('cuadc_interface')

        self.declare_parameter('connection', 'udp:127.0.0.1:14550')

        # 指令：规划/决策 -> 接口层 -> ArduPilot
        self.create_service(Arm, '~/arm', self._on_arm)
        self.create_service(SetMode, '~/set_mode', self._on_set_mode)
        self.create_service(GotoLocation, '~/goto_location', self._on_goto_location)
        self.create_service(SetRelay, '~/set_relay', self._on_set_relay)

        self.create_subscription(
            PositionTarget, '~/cmd/position_target', self._on_position_target, 10)
        self.create_subscription(TwistStamped, '~/cmd/velocity', self._on_velocity, 10)
        self.create_subscription(Float64, '~/cmd/yaw', self._on_yaw, 10)

        # 遥测：ArduPilot -> 接口层 -> 规划/决策
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
        # Attitude 消息类型待实现时定（见 04-interfaces.md），暂用 sensor_msgs/Imu 占位
        self.attitude_pub = self.create_publisher(Imu, '~/telemetry/attitude', 10)
        self.battery_pub = self.create_publisher(BatteryState, '~/telemetry/battery', 10)
        self.gps_status_pub = self.create_publisher(GpsStatus, '~/telemetry/gps_status', 10)
        self.ekf_status_pub = self.create_publisher(EkfStatus, '~/telemetry/ekf_status', 10)

        self.get_logger().warn(
            'cuadc_interface 目前是契约骨架，没有建立 MAVLink 连接，'
            '不会发布任何遥测、指令服务调用会直接抛异常。'
        )

    # -- 指令回调：协议实现待补 --

    def _on_arm(self, request, response):
        raise NotImplementedError

    def _on_set_mode(self, request, response):
        raise NotImplementedError

    def _on_goto_location(self, request, response):
        raise NotImplementedError

    def _on_set_relay(self, request, response):
        raise NotImplementedError

    def _on_position_target(self, msg):
        raise NotImplementedError

    def _on_velocity(self, msg):
        raise NotImplementedError

    def _on_yaw(self, msg):
        raise NotImplementedError


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
