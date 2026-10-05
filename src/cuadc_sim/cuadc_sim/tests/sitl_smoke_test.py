"""Minimal closed-loop check against a running ArduPilot SITL instance.

Not the project's real interface layer (see docs/spec/adr/0001) -- this
node only exists to prove the simulation loop works: connect over MAVLink,
read telemetry, arm, switch to GUIDED, and send one position-target command.
"""
import time

import rclpy
from rclpy.node import Node
from pymavlink import mavutil


class SitlSmokeTest(Node):

    def __init__(self):
        super().__init__('sitl_smoke_test')
        # MAVProxy's default GCS output (udp:14550) is left free for
        # QGroundControl or another GCS; 14551 is this node's own output,
        # set up via the launch file's `-m "--out 127.0.0.1:14551"`.
        self.declare_parameter('connection', 'udp:127.0.0.1:14551')
        self.declare_parameter('heartbeat_timeout_sec', 120.0)
        self.declare_parameter('telemetry_sample_count', 5)
        self.declare_parameter('run_guided_check', True)
        self.declare_parameter('ekf_ready_timeout_sec', 60.0)
        self.declare_parameter('arm_retry_timeout_sec', 45.0)

        connection = self.get_parameter('connection').value
        heartbeat_timeout = self.get_parameter('heartbeat_timeout_sec').value
        sample_count = self.get_parameter('telemetry_sample_count').value
        run_guided_check = self.get_parameter('run_guided_check').value
        ekf_ready_timeout = self.get_parameter('ekf_ready_timeout_sec').value
        arm_retry_timeout = self.get_parameter('arm_retry_timeout_sec').value

        self.get_logger().info(f'connecting to {connection} ...')
        master = self._connect_with_retry(connection, heartbeat_timeout)

        self.get_logger().info(
            f'waiting for heartbeat (timeout {heartbeat_timeout:.0f}s) ...')
        if master.wait_heartbeat(timeout=heartbeat_timeout) is None:
            raise RuntimeError('no heartbeat received -- is SITL running?')
        self.get_logger().info(
            f'heartbeat ok (sysid={master.target_system}, '
            f'compid={master.target_component})')

        # Without MAVProxy in the loop, nothing has asked SITL to stream
        # telemetry yet -- request it explicitly.
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

        self._read_telemetry(master, sample_count)

        if run_guided_check:
            self._wait_for_ekf_ready(master, ekf_ready_timeout)
            self._guided_check(master, arm_retry_timeout)

        self.get_logger().info('smoke test complete, shutting down')
        rclpy.shutdown()

    def _connect_with_retry(self, connection, timeout):
        # mavlink_connection() can raise immediately if the target isn't up
        # yet (e.g. a tcp:// port with nothing listening) -- SITL is still
        # building/booting when this node starts, so retry until it comes up
        # rather than treating one failed attempt as fatal.
        deadline = time.monotonic() + timeout
        while True:
            try:
                return mavutil.mavlink_connection(connection)
            except OSError as e:
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f'could not connect to {connection} within '
                        f'{timeout:.0f}s: {e}') from e
                self.get_logger().info(f'{e}, retrying ...')
                time.sleep(2.0)

    def _read_telemetry(self, master, sample_count):
        self.get_logger().info(f'reading {sample_count} telemetry samples ...')
        seen = 0
        while seen < sample_count:
            msg = master.recv_match(
                type=['GLOBAL_POSITION_INT', 'SYS_STATUS'],
                blocking=True, timeout=10)
            if msg is None:
                raise RuntimeError('telemetry stream stalled')
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                self.get_logger().info(
                    f'  lat={msg.lat / 1e7:.7f} lon={msg.lon / 1e7:.7f} '
                    f'relative_alt={msg.relative_alt / 1000.0:.2f}m')
                seen += 1
            elif msg.get_type() == 'SYS_STATUS':
                self.get_logger().info(
                    f'  battery={msg.voltage_battery / 1000.0:.2f}V')

    def _wait_for_ekf_ready(self, master, timeout):
        # A non-zero GLOBAL_POSITION_INT fix can appear *before* the EKF has
        # actually set its origin and started trusting GPS -- arming/taking
        # off during that window still gets ACCEPTED, but produces an
        # unstable height reference that trips ArduPilot's own land-detector
        # into auto-disarming a few seconds later ("Disarming motors", no
        # real climb). Observed in practice: EKF_STATUS_REPORT's
        # EKF_POS_HORIZ_ABS/EKF_POS_VERT_ABS flags didn't set until several
        # seconds after GLOBAL_POSITION_INT already showed a non-zero fix.
        # Check those flags directly instead of inferring readiness from
        # position alone.
        self.get_logger().info(
            f'waiting for EKF/GPS to be ready (timeout {timeout:.0f}s) ...')
        required = (mavutil.mavlink.EKF_POS_HORIZ_ABS
                    | mavutil.mavlink.EKF_POS_VERT_ABS)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5)
            if msg is not None and (msg.flags & required) == required:
                self.get_logger().info(
                    'EKF/GPS ready (EKF_POS_HORIZ_ABS and EKF_POS_VERT_ABS set)')
                return
        raise RuntimeError(
            f'EKF position estimate never became reliable within {timeout:.0f}s '
            '(EKF_STATUS_REPORT never set EKF_POS_HORIZ_ABS/EKF_POS_VERT_ABS)')

    def _set_mode_and_confirm(self, master, mode_name, timeout):
        # set_mode() only sends the request; it doesn't confirm the mode
        # switch actually landed before returning. Arming/taking off before
        # the flight controller has genuinely switched modes lets
        # MAV_CMD_NAV_TAKEOFF get ACCEPTED at the command-handler level
        # while the vehicle is still in whatever mode it was in before (e.g.
        # STABILIZE, which needs manual throttle to climb) -- symptom:
        # takeoff "succeeds" but the vehicle never leaves the ground and no
        # motor thrust is ever commanded, followed by an auto-disarm a few
        # seconds later. Confirm via HEARTBEAT.custom_mode before moving on.
        mode_id = master.mode_mapping()[mode_name]
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            master.set_mode(mode_id)
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if hb is not None and hb.custom_mode == mode_id:
                self.get_logger().info(f'confirmed mode switch to {mode_name}')
                return
        raise RuntimeError(
            f'mode never switched to {mode_name} within {timeout:.0f}s')

    def _arm_with_retry(self, master, timeout):
        # A GPS fix alone isn't the full arming-readiness signal -- ArduPilot
        # also checks accelerometer consistency and other AHRS state that
        # can settle a few seconds after the first fix, so a single arm
        # attempt right after _wait_for_ekf_ready can still be rejected.
        # Retry rather than trying to guess every prearm condition.
        deadline = time.monotonic() + timeout
        while True:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
            ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if (ack is not None
                    and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().info('arm accepted')
                    return
                self.get_logger().info(
                    f'arm rejected (result={ack.result}), retrying ...')
            if time.monotonic() >= deadline:
                raise RuntimeError(f'could not arm within {timeout:.0f}s')
            time.sleep(3.0)

    def _guided_check(self, master, arm_retry_timeout):
        self.get_logger().info('arming and switching to GUIDED ...')
        self._set_mode_and_confirm(master, 'GUIDED', timeout=10.0)
        self._arm_with_retry(master, timeout=arm_retry_timeout)
        master.motors_armed_wait()
        self.get_logger().info('armed, sending takeoff to 10m ...')
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, 10)
        self._wait_command_ack(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

        pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
        if pos is None:
            raise RuntimeError('no position available to build a GUIDED target')
        target_lat = pos.lat / 1e7 + 0.00003  # ~3m north
        target_lon = pos.lon / 1e7
        self.get_logger().info(
            f'sending GUIDED position target lat={target_lat:.7f} '
            f'lon={target_lon:.7f} alt=10m ...')
        master.mav.set_position_target_global_int_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(target_lat * 1e7), int(target_lon * 1e7), 10,
            0, 0, 0, 0, 0, 0, 0, 0)
        self.get_logger().info('GUIDED position target sent')

    def _wait_command_ack(self, master, command, timeout=10):
        # Other COMMAND_ACKs (e.g. SET_MESSAGE_INTERVAL from MAVProxy/QGC)
        # can arrive in between; grabbing just the first COMMAND_ACK and
        # bailing out if it's for a different command misreports a real
        # accept as "no ACK received". Keep reading until the matching
        # command shows up or the deadline passes.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=remaining)
            if ack is None:
                break
            if ack.command != command:
                continue
            if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise RuntimeError(f'command {command} rejected: result={ack.result}')
            self.get_logger().info(f'command {command} accepted')
            return
        raise RuntimeError(f'no ACK received for command {command}')


def main(args=None):
    rclpy.init(args=args)
    try:
        SitlSmokeTest()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
