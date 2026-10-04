"""sim-only 桥：决策进 DROP_PAYLOAD 时给 Gazebo 物理水瓶发 DetachableJoint 脱钩
（真机同事件由 interface release_payload 驱动真舵机）。按投放次序 left->right，与
lever_arm 左/右对应。只随 mission_bringup(profile=sim) 起——触发源是任务事件，放
世界 bringup 会把世界耦合到决策（sim/task 解耦约定）。"""
import os
import subprocess

import cuadc_airframe
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_DROP_STATE = 'DROP_PAYLOAD'


class DropBottleBridge(Node):
    def __init__(self):
        super().__init__('drop_bottle_bridge')
        self.declare_parameter('model', 'cuadc_iris')
        self.declare_parameter('state_topic', '/cuadc_decision/mission_state')
        # 脱钩次序 = 投放次序; 与 approach lever_arm 左/右对应(见 ADR-0010 簇②)
        # 脱钩次序 = 机体配置里投放口的投放次序（与决策杠杆臂表同序），模型里的瓶名与口名一致。
        self.declare_parameter('bottles', [p.name for p in cuadc_airframe.load().drop_ports])
        self._model = self.get_parameter('model').value
        self._bottles = list(self.get_parameter('bottles').value)
        self._idx = 0
        self._prev = None
        topic = self.get_parameter('state_topic').value
        self.create_subscription(String, topic, self._on_state, 10)
        self.get_logger().info(
            f'drop_bottle_bridge up: watching {topic}, detach order={self._bottles} on {self._model}')

    def _on_state(self, msg: String):
        s = msg.data
        if s == _DROP_STATE and self._prev != _DROP_STATE:  # 只在进入 DROP 的那一下触发
            if self._idx < len(self._bottles):
                bottle = self._bottles[self._idx]
                self._idx += 1
                self._detach(bottle)
            else:
                self.get_logger().warn('entered DROP_PAYLOAD but all bottles already dropped')
        self._prev = s

    def _detach(self, bottle: str):
        topic = f'/model/{self._model}/drop/{bottle}/detach'
        self.get_logger().info(f'DROP_PAYLOAD -> detach bottle "{bottle}" via {topic}')
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
