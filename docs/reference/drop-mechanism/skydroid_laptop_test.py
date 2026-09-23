"""Laptop -> Pixhawk USB -> SkyDroid bench test.

Install once on the laptop: python -m pip install pymavlink
Set PORT and SERVO before running. This is an unloaded test only.

NOTE (archival): code and values are verbatim from the hardware team's delivery;
the interactive console strings (originally Chinese) are rendered in English here
because the delivered copy arrived with transfer mojibake. See this folder's README.
"""
from pymavlink import mavutil
import time

PORT = "COM12"       # change to the Pixhawk COM port
BAUD = 115200
SERVO = 9            # use the actual SERVOx output number, not motor output
NEUTRAL_US = 1500
FORWARD_US = 1100
REVERSE_US = 1900
RUN_S = 1.975


link = mavutil.mavlink_connection(PORT, baud=BAUD)
link.wait_heartbeat(timeout=15)
print("Pixhawk connected")

def pwm(value):
    link.mav.command_long_send(
        link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, SERVO, value, 0, 0, 0, 0, 0)
    deadline = time.monotonic() + 0.5
    ack = None
    output = None
    while time.monotonic() < deadline:
        msg = link.recv_match(type=["COMMAND_ACK", "SERVO_OUTPUT_RAW"], blocking=True, timeout=0.1)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
            ack = msg.result
        elif msg.get_type() == "SERVO_OUTPUT_RAW":
            output = getattr(msg, f"servo{SERVO}_raw", None)
    print(f"command={value}us ack={ack} pixhawk_output={output}us")

try:
    pwm(NEUTRAL_US)
    print("Sent 1500 us (stop). Enter: <PWM> <seconds>, e.g. '1000 1' or '2000 1'")
    print("Enter 't' for default 1100 us / 3.330 s; 'x' to stop; 'q' to quit")
    while True:
        command = input("> ").strip().lower()
        if command == "q":
            break
        if command == "x":
            pwm(NEUTRAL_US)
            print("STOP: 1500 us")
            continue
        if command == "t":
            pulse, duration = FORWARD_US, RUN_S
        elif command == "p":
            pulse, duration = REVERSE_US, RUN_S
        else:
            try:
                pulse_text, duration_text = command.split()
                pulse, duration = int(pulse_text), float(duration_text)
            except ValueError:
                print("bad format. examples: 1000 1 / 2000 1 / t / x / q")
                continue
        if not 900 <= pulse <= 2100 or not 0.05 <= duration <= 10:
            print("limits: PWM 900-2100 us, time 0.05-10 s")
            continue
        print(f"running {pulse} us for {duration:.3f} s")
        pwm(pulse)
        time.sleep(duration)
        pwm(NEUTRAL_US)
        print("stopped (1500 us); re-check the rotation head position before the next test")
except KeyboardInterrupt:
    pwm(NEUTRAL_US)
    print("stopped; PWM returned to 1500 us")
finally:
    pwm(NEUTRAL_US)
    link.close()
