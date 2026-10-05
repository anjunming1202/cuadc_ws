#!/usr/bin/env python3
"""Stop leftover user-session cv_preview_ctl before systemd takes port 5611."""
import os
import signal
import subprocess

for line in subprocess.check_output(["ps", "-eo", "pid,cmd"], text=True).splitlines():
    rest = line.split(None, 1)[1] if " " in line.strip() else ""
    if "kill_home_ctl" in rest:
        continue
    if not rest.lstrip().startswith("python"):
        continue
    if "cv_preview_ctl.py" not in rest:
        continue
    pid = int(line.split()[0])
    if pid == os.getpid():
        continue
    os.kill(pid, signal.SIGTERM)
    print("killed", pid)
