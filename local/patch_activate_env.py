#!/usr/bin/env python3
from pathlib import Path
p = Path("/home/wly/cuadc_ws_worktrees/field-test/scripts/activate_env.sh")
t = p.read_text()
old = "source /opt/ros/humble/setup.bash\n"
new = (
    "if [ -f /opt/ros/jazzy/setup.bash ]; then\n"
    "  source /opt/ros/jazzy/setup.bash\n"
    "elif [ -f /opt/ros/humble/setup.bash ]; then\n"
    "  source /opt/ros/humble/setup.bash\n"
    "else\n"
    "  echo 'activate_env: no ROS at /opt/ros/jazzy or humble' >&2\n"
    "  return 1 2>/dev/null || exit 1\n"
    "fi\n"
)
if old not in t:
    raise SystemExit("pattern missing")
p.write_text(t.replace(old, new, 1))
print("patched")
