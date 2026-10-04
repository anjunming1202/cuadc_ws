#!/bin/bash
# Persist fanout at 115200 (TELEM2) and restart. Does not arm the FC.
set -euo pipefail
install -d /etc/systemd/system/cuadc-onboard-fanout.service.d
install -m 0644 /tmp/fanout-baud.conf /etc/systemd/system/cuadc-onboard-fanout.service.d/override.conf
install -m 0644 /tmp/cuadc-onboard-fanout.service /etc/systemd/system/cuadc-onboard-fanout.service
systemctl daemon-reload
systemctl restart cuadc-onboard-fanout.service
sleep 8
systemctl is-active cuadc-onboard-fanout.service
journalctl -u cuadc-onboard-fanout.service -n 12 --no-pager
