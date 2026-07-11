"""Closed-loop flight verification against a running ArduPilot instance.

Not the project's real interface layer (see docs/spec/adr/0001) -- this
node only exists to prove the *physical* simulation loop works, as
opposed to sitl_smoke_test.py which only proves the *protocol* loop
works (commands get ACCEPTED). Backend-agnostic: it only talks MAVLink,
so pointing `connection` at a sitl_bringup or a gazebo_bringup instance
exercises the same assertions against different physics -- see
src/cuadc_sim/README.md.

Flow: connect -> wait EKF/GPS ready -> GUIDED + arm -> takeoff -> assert
the vehicle actually climbs to the target altitude (within tolerance)
and holds it for a sustained window without disarming -> LAND -> assert
it comes back down and disarms itself on the ground (not mid-air).
"""
import time

import rclpy
from rclpy.node import Node
from pymavlink import mavutil


class FlightAcceptanceTest(Node):

    def __init__(self):
        super().__init__('flight_acceptance_test')
        # Same dual-output convention as sitl_smoke_test.py: 14550 stays
        # free for QGroundControl, 14551 is reserved for our own scripts
        # by both bringup launch files' `-m "--out 127.0.0.1:14551"`.
        self.declare_parameter('connection', 'udp:127.0.0.1:14551')
        self.declare_parameter('heartbeat_timeout_sec', 120.0)
        self.declare_parameter('ekf_ready_timeout_sec', 60.0)
        self.declare_parameter('arm_retry_timeout_sec', 45.0)
        self.declare_parameter('target_altitude_m', 10.0)
        self.declare_parameter('altitude_tolerance_m', 0.5)
        self.declare_parameter('climb_timeout_sec', 30.0)
        self.declare_parameter('hold_duration_sec', 5.0)
        self.declare_parameter('land_timeout_sec', 60.0)

        connection = self.get_parameter('connection').value
        heartbeat_timeout = self.get_parameter('heartbeat_timeout_sec').value
        ekf_ready_timeout = self.get_parameter('ekf_ready_timeout_sec').value
        arm_retry_timeout = self.get_parameter('arm_retry_timeout_sec').value
        self.target_altitude = self.get_parameter('target_altitude_m').value
        self.altitude_tolerance = self.get_parameter('altitude_tolerance_m').value
        self.climb_timeout = self.get_parameter('climb_timeout_sec').value
        self.hold_duration = self.get_parameter('hold_duration_sec').value
        self.land_timeout = self.get_parameter('land_timeout_sec').value

        self.get_logger().info(f'connecting to {connection} ...')
        master = self._connect_with_retry(connection, heartbeat_timeout)

        self.get_logger().info(
            f'waiting for heartbeat (timeout {heartbeat_timeout:.0f}s) ...')
        if master.wait_heartbeat(timeout=heartbeat_timeout) is None:
            raise RuntimeError('no heartbeat received -- is the vehicle running?')
        self.get_logger().info(
            f'heartbeat ok (sysid={master.target_system}, '
            f'compid={master.target_component})')

        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

        self._wait_for_ekf_ready(master, ekf_ready_timeout)
        self._set_mode_and_confirm(master, 'GUIDED', timeout=10.0)
        self._arm_with_retry(master, arm_retry_timeout)
        master.motors_armed_wait()

        self._takeoff_and_assert_climb(master)
        self._assert_holds_altitude(master)
        self._land_and_assert_disarm(master)

        self.get_logger().info(
            'flight acceptance test PASSED: climbed to target altitude, '
            'held it, landed, and disarmed on the ground')
        rclpy.shutdown()

    def _connect_with_retry(self, connection, timeout):
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

    def _wait_for_ekf_ready(self, master, timeout):
        # Same rationale as sitl_smoke_test.py: a non-zero position fix can
        # appear before the EKF actually trusts it, and arming/taking off
        # during that window produces an unstable height reference.
        self.get_logger().info(
            f'waiting for EKF/GPS to be ready (timeout {timeout:.0f}s) ...')
        required = (mavutil.mavlink.EKF_POS_HORIZ_ABS
                    | mavutil.mavlink.EKF_POS_VERT_ABS)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5)
            if msg is not None and (msg.flags & required) == required:
                self.get_logger().info('EKF/GPS ready')
                return
        raise RuntimeError(
            f'EKF position estimate never became reliable within {timeout:.0f}s')

    def _set_mode_and_confirm(self, master, mode_name, timeout):
        self.get_logger().info(f'switching to {mode_name} ...')
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
        self.get_logger().info('arming ...')
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

    def _wait_command_ack(self, master, command, timeout=10):
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

    def _next_state(self, master, timeout):
        # Filtering recv_match() to a single type silently discards any
        # other message type that arrives while it's waiting for a match
        # (pymavlink pulls and drops non-matching messages from the
        # stream) -- polling GLOBAL_POSITION_INT and HEARTBEAT separately
        # in the same loop starves whichever one isn't being waited on
        # right now and can miss a disarm event entirely. Poll both in one
        # call instead, so nothing gets silently dropped.
        msg = master.recv_match(
            type=['GLOBAL_POSITION_INT', 'HEARTBEAT'], blocking=True, timeout=timeout)
        if msg is None:
            return None, None
        if msg.get_type() == 'HEARTBEAT':
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            return armed, None
        return None, msg.relative_alt / 1000.0

    def _takeoff_and_assert_climb(self, master):
        # This is the check that would have caught the HAL_WITH_ESC_TELEM
        # bug: that bug got MAV_CMD_NAV_TAKEOFF ACCEPTED every time but the
        # vehicle never actually climbed and auto-disarmed ~10-24s later.
        # sitl_smoke_test.py stops at "command accepted" and would never
        # have noticed -- this loop keeps watching relative_alt afterward.
        self.get_logger().info(
            f'sending takeoff to {self.target_altitude:.1f}m ...')
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, self.target_altitude)
        self._wait_command_ack(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

        self.get_logger().info(
            f'watching climb (timeout {self.climb_timeout:.0f}s) ...')
        deadline = time.monotonic() + self.climb_timeout
        last_log = 0.0
        while time.monotonic() < deadline:
            armed, alt = self._next_state(master, timeout=2)
            if armed is False:
                raise RuntimeError(
                    'vehicle disarmed while climbing -- motors never '
                    'produced real thrust (this is the failure mode of '
                    'the HAL_WITH_ESC_TELEM bug)')
            if alt is None:
                continue
            now = time.monotonic()
            if now - last_log > 1.0:
                self.get_logger().info(f'  relative_alt={alt:.2f}m')
                last_log = now
            if abs(alt - self.target_altitude) <= self.altitude_tolerance:
                self.get_logger().info(
                    f'reached target altitude ({alt:.2f}m)')
                return
        raise RuntimeError(
            f'never reached target altitude {self.target_altitude:.1f}m '
            f'(+/-{self.altitude_tolerance:.1f}m) within '
            f'{self.climb_timeout:.0f}s -- motors likely never produced '
            'real climb thrust')

    def _assert_holds_altitude(self, master):
        self.get_logger().info(
            f'holding for {self.hold_duration:.0f}s to confirm stable flight ...')
        deadline = time.monotonic() + self.hold_duration
        while time.monotonic() < deadline:
            armed, alt = self._next_state(master, timeout=2)
            if armed is False:
                raise RuntimeError('vehicle disarmed while holding altitude')
            if alt is None:
                continue
            if abs(alt - self.target_altitude) > self.altitude_tolerance:
                raise RuntimeError(
                    f'altitude drifted to {alt:.2f}m while holding '
                    f'(target {self.target_altitude:.1f}m '
                    f'+/-{self.altitude_tolerance:.1f}m)')
        self.get_logger().info('altitude held steady')

    def _land_and_assert_disarm(self, master):
        self.get_logger().info('sending LAND ...')
        self._set_mode_and_confirm(master, 'LAND', timeout=10.0)

        self.get_logger().info(
            f'watching descent and disarm (timeout {self.land_timeout:.0f}s) ...')
        deadline = time.monotonic() + self.land_timeout
        last_alt = None
        while time.monotonic() < deadline:
            armed, alt = self._next_state(master, timeout=2)
            if alt is not None:
                last_alt = alt
            if armed is False:
                if last_alt is not None and last_alt > self.altitude_tolerance:
                    raise RuntimeError(
                        f'disarmed mid-air during landing (relative_alt='
                        f'{last_alt:.2f}m) instead of after touching down')
                self.get_logger().info(
                    f'disarmed on the ground (relative_alt={last_alt})')
                return
        raise RuntimeError(
            f'vehicle never disarmed after LAND within {self.land_timeout:.0f}s '
            f'(last relative_alt={last_alt})')


def main(args=None):
    rclpy.init(args=args)
    try:
        FlightAcceptanceTest()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
