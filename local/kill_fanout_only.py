#!/usr/bin/env python3
import os
import signal
import subprocess
import time

for line in subprocess.check_output(["ps", "-eo", "pid,cmd"], text=True).splitlines():
    if "mav_fanout.py" not in line or "python" not in line.split()[1:2] and "python3" not in line:
        if "mav_fanout.py" not in line:
            continue
    if "uart_probe" in line or "kill_fanout" in line:
        continue
    parts = line.split(None, 1)
    if len(parts) < 2 or "mav_fanout.py" not in parts[1]:
        continue
    if not parts[1].lstrip().startswith("python"):
        continue
    pid = int(parts[0])
    os.kill(pid, signal.SIGTERM)
    print("stopped fanout", pid)
    time.sleep(0.5)
