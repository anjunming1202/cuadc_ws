#!/bin/bash
# Paste on the Pi (after scp of the two files), or run via deploy_pi_telemetry.ps1 from Windows.
set -e
echo "=== before ==="
systemctl is-active dronelink-telemetry.service || true
journalctl -u dronelink-telemetry.service -n 15 --no-pager || true

if [ -f ~/dronelink-telemetry.py ]; then
  sudo cp ~/dronelink-telemetry.py /usr/local/sbin/dronelink-telemetry.py
  sudo chmod +x /usr/local/sbin/dronelink-telemetry.py
fi
if [ -f ~/dronelink-telemetry.service ]; then
  sudo cp ~/dronelink-telemetry.service /etc/systemd/system/dronelink-telemetry.service
fi

sudo systemctl daemon-reload
sudo systemctl enable dronelink-telemetry.service
sudo systemctl restart dronelink-telemetry.service
sleep 2
echo "=== after ==="
systemctl is-active dronelink-telemetry.service
systemctl status dronelink-telemetry.service --no-pager -l | head -25
journalctl -u dronelink-telemetry.service -n 30 --no-pager
echo "=== unit ExecStart ==="
grep ExecStart /etc/systemd/system/dronelink-telemetry.service
echo "=== ping ground ==="
ping -c 2 -W 1 192.168.50.2 || true
