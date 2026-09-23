"""Raspberry Pi version; requires pigpio and a GPIO home switch."""
import time
import pigpio

PWM_GPIO = 18          # hardware PWM-capable GPIO; change to your wiring
HOME_GPIO = 23         # switch to GND, active low
NEUTRAL_US = 1500
FORWARD_US = 1100
REVERSE_US = 1900
RUN_S = 3.330
HOME_TIMEOUT_S = 6.0

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio daemon is not running")
pi.set_mode(HOME_GPIO, pigpio.INPUT)
pi.set_pull_up_down(HOME_GPIO, pigpio.PUD_UP)

def pwm(us):
    pi.set_servo_pulsewidth(PWM_GPIO, us)

def stop():
    pwm(NEUTRAL_US)

def home():
    pwm(REVERSE_US)
    deadline = time.monotonic() + HOME_TIMEOUT_S
    while pi.read(HOME_GPIO) and time.monotonic() < deadline:
        time.sleep(0.005)
    stop()
    return pi.read(HOME_GPIO) == 0

def test_360():
    if not home():
        raise TimeoutError("home switch not found")
    pwm(FORWARD_US)
    time.sleep(RUN_S)
    stop()

try:
    print("Call test_360() or home(); Ctrl-C stops and then re-home.")
    # test_360()  # uncomment only after unloaded bench checks
except KeyboardInterrupt:
    stop()
finally:
    stop()
    pi.stop()
