"""sim-only 扰动节点：给机体施加风阻力（均值 + 一阶低通阵风）和恒定偏航扭矩。

ArduPilot 的 SIM_WIND_* 在 JSON(Gazebo) 模式下不生效（SIM_JSON.cpp: wind not supported），
风只能在 Gazebo 侧加；偏航扭矩建模电机/桨不完全竖直带来的反扭偏差（真机离地瞬间航向跳 4–9°），
随推力（旋翼转速²，取自 joint_state）缩放：悬停时 = yaw_torque_nm，桨停 = 0。
经 gz ApplyLinkWrench 的 persistent wrench（世界系、作用于 base_link 原点）施加。persistent
wrench 会累加且每条都被逐步求和，所以只发与上次的差值，并每 _CONSOLIDATE 条清一次、下个 tick 重发
总量（同一 tick 里先 clear 再发会被清掉），条目数有界，否则 RTF 会掉到 0.1 以下。
量值来源与红线见 docs/spec/sim-stress-testbed.md §3.2。
"""

import math
import os
import random
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.model_pb2 import Model
from gz.transport13 import Node as GzNode
from rclpy.node import Node

_CONSOLIDATE = 20     # 累计这么多条增量 wrench 后清空重发总量
_MIN_DELTA = (0.02, 0.02, 0.002)  # fx, fy (N), tz (N·m) 变化小于此不发，少攒条目


class SimDisturbanceNode(Node):

    def __init__(self):
        super().__init__('sim_disturbance')
        p = self.declare_parameter
        world = p('world', 'cuadc_field').value
        self._link = p('link', 'cuadc_iris::base_link').value
        self._force = float(p('wind_force_n', 0.0).value)          # 风阻力均值（水平，N）
        self._dir = math.radians(float(p('wind_dir_deg', 0.0).value))  # 风吹向，世界系 x=0°、逆时针为正
        self._gust_std = float(p('gust_std_n', 0.0).value)          # 阵风力标准差（N）
        self._gust_tau = max(0.1, float(p('gust_tau_s', 3.0).value))  # 阵风相关时间（s）
        self._yaw_torque = float(p('yaw_torque_nm', 0.0).value)      # 悬停推力下的偏航扭矩（N·m，绕世界 z）
        self._hover_w2 = float(p('hover_rotor_speed_rad_s', 473.0).value) ** 2  # sim 悬停实测旋翼转速
        model = self._link.split('::')[0]
        hz = max(0.2, float(p('update_hz', 10.0).value))
        self._dt = 1.0 / hz
        self._gust = [0.0, 0.0]
        self._applied = None  # 已施加的 (fx, fy, tz)；None = 刚清空、下个 tick 重发总量
        self._entries = 0
        self._cleared = False

        self._gz = GzNode()
        self._pub = self._gz.advertise(f'/world/{world}/wrench/persistent', EntityWrench)
        self._clear = self._gz.advertise(f'/world/{world}/wrench/clear', Entity)
        self._thrust_ratio = 0.0  # 旋翼转速² 均值 / 悬停值
        self._gz.subscribe(Model, f'/world/{world}/model/{model}/joint_state', self._on_joints)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'[DISTURBANCE] wind {self._force:.2f}N toward {math.degrees(self._dir):.0f}deg '
            f'gust std {self._gust_std:.2f}N tau {self._gust_tau:.1f}s, yaw torque {self._yaw_torque:.3f}Nm')

    def _on_joints(self, msg):
        w2 = [j.axis1.velocity ** 2 for j in msg.joint if j.name.startswith('rotor_')]
        if w2:
            self._thrust_ratio = sum(w2) / len(w2) / self._hover_w2

    def _tick(self):
        a = math.exp(-self._dt / self._gust_tau)  # 一阶 Gauss-Markov：稳态标准差 = gust_std
        b = self._gust_std * math.sqrt(1.0 - a * a)
        self._gust = [a * g + b * random.gauss(0.0, 1.0) for g in self._gust]
        target = (self._force * math.cos(self._dir) + self._gust[0],
                  self._force * math.sin(self._dir) + self._gust[1],
                  self._yaw_torque * self._thrust_ratio)

        ent = Entity()
        ent.name = self._link
        ent.type = Entity.LINK
        if not (self._pub.has_connections() and self._clear.has_connections()):
            return  # 增量施加不能丢消息：等 Gazebo 订阅端连上再发
        if not self._cleared or self._entries >= _CONSOLIDATE:
            self._clear.publish(ent)  # 启动时清掉上一实例留下的；之后定期合并
            self._cleared, self._applied, self._entries = True, None, 0
            return
        applied = self._applied or (0.0, 0.0, 0.0)
        delta = [t - a for t, a in zip(target, applied)]
        if self._applied is not None and all(abs(d) < m for d, m in zip(delta, _MIN_DELTA)):
            return
        msg = EntityWrench()
        msg.entity.CopyFrom(ent)
        msg.wrench.force.x, msg.wrench.force.y = delta[0], delta[1]
        msg.wrench.torque.z = delta[2]
        self._pub.publish(msg)
        self._applied = target
        self._entries += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimDisturbanceNode()
    # gz transport 接管了信号，rclpy 收不到 SIGINT，进程会在 launch 结束后残留并继续推下一个仿真
    # 世界——自己接住信号：清掉施加的 wrench 后直接退出。
    def _stop(*_):
        ent = Entity()
        ent.name = node._link
        ent.type = Entity.LINK
        node._clear.publish(ent)
        os._exit(0)
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    _stop()
