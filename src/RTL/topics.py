"""RTL 脚本共用的话题名。阶段严格串行：上一段发完交接话题，下一段才动。

rtl_start → 回原点 → home_arrived → 悬停找垫 → pad_waypoint → 降落
没垫则 set_mode RTL，不发 pad_waypoint（降落节点一直等）。
"""

INTERFACE = '/cuadc_interface'
LOCAL_POS = f'{INTERFACE}/telemetry/local_position'
ATTITUDE = f'{INTERFACE}/telemetry/attitude'
CMD_LOCAL = f'{INTERFACE}/cmd/position_target_local'

RTL_NS = '/cuadc_rtl'
RTL_START = f'{RTL_NS}/rtl_start'          # Empty，只启动回原点
HOME_ARRIVED = f'{RTL_NS}/home_arrived'    # Empty latched，回原点结束 → 扫描开始
PAD_WAYPOINT = f'{RTL_NS}/pad_waypoint'    # PoseStamped latched，扫描结束 → 降落开始
STAGE = f'{RTL_NS}/stage'                  # String：go_home / scan / land / fc_rtl / done
SET_MODE = f'{INTERFACE}/set_mode'
ARMED = f'{INTERFACE}/status/armed'


def latched_qos():
    """交接话题：后启动的订阅者也能读到上一段的结束信号。"""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
