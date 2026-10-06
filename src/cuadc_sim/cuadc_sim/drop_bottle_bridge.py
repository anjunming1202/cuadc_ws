"""sim-only 桥：接口层报一次投放已执行（status/drop，带口号）时，给 Gazebo 里该口的物理水瓶发
DetachableJoint 脱钩（真机同一事件是 release_payload 驱动该口舵机）。只随 mission_bringup(profile=sim)
起——触发源是任务事件，放世界 bringup 会把世界耦合到决策（sim/task 解耦约定）。"""
import os
import subprocess

import cuadc_physical
import rclpy
from cuadc_interface_msgs.msg import DropStatus
from rclpy.node import Node


class DropBottleBridge(Node):
    def __init__(self):
        super().__init__('drop_bottle_bridge')
        self.declare_parameter('model', 'cuadc_iris')
        self.declare_parameter('status_topic', '/cuadc_interface/status/drop')
        # 按口号排的瓶名（模型里的瓶名与机体配置的口名一致）。
        ports = cuadc_physical.load_airframe().drop_ports
        self.declare_parameter('bottles', [p.name for p in ports])
        self._model = self.get_parameter('model').value
        self._bottles = list(self.get_parameter('bottles').value)
        self._dropped = set()
        topic = self.get_parameter('status_topic').value
        self.create_subscription(DropStatus, topic, self._on_drop, 10)
        self.get_logger().info(
            f'drop_bottle_bridge up: watching {topic}, '
            f'bottles by port={self._bottles} on {self._model}')

    def _on_drop(self, msg: DropStatus):
        if not msg.actuated:
            return
        if msg.port >= len(self._bottles):
            self.get_logger().warn(f'drop on unknown port {msg.port}')
        elif msg.port in self._dropped:
            self.get_logger().warn(f'port {msg.port} ({self._bottles[msg.port]}) already dropped')
        else:
            self._dropped.add(msg.port)
            self._detach(self._bottles[msg.port])

    def _detach(self, bottle: str):
        topic = f'/model/{self._model}/drop/{bottle}/detach'
        self.get_logger().info(f'drop actuated -> detach bottle "{bottle}" via {topic}')
        env = dict(os.environ, GZ_VERSION='harmonic')
        # gz.msgs.Empty 到 DetachableJoint 的 detach_topic = 脱钩; 用 CLI 避免额外 gz-transport
        # python 依赖(与 gazebo_bringup 里 fix_camera_pose 同套路)。
        subprocess.Popen(['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', ''],
                         env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main(args=None):
    rclpy.init(args=args)
    node = DropBottleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
