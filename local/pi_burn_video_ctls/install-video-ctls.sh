#!/bin/bash
# 把图传/CV 控制脚本装到 Pi（确认后再跑）。需 sudo。不解锁飞控。
set -euo pipefail
OPT=/opt/cuadc-onboard
UNIT=/etc/systemd/system
install -m 0755 /tmp/cv_preview_ctl.py "$OPT/cv_preview_ctl.py"
install -m 0755 /tmp/cv_preview_rtp.py "$OPT/cv_preview_rtp.py"
install -m 0755 /tmp/video_forward_ctl.py "$OPT/video_forward_ctl.py"
install -m 0644 /tmp/cuadc-onboard-cv-preview-ctl.service "$UNIT/"
install -m 0644 /tmp/cuadc-onboard-video-forward-ctl.service "$UNIT/"
install -m 0755 /tmp/apply-profile.sh "$OPT/apply-profile.sh"
systemctl daemon-reload
# 停掉常开旧单元，避免和 ctl 抢端口
systemctl stop cuadc-onboard-cv-preview.service cuadc-onboard-cv-forward.service \
  cuadc-onboard-video-forward.service 2>/dev/null || true
systemctl disable cuadc-onboard-cv-preview.service cuadc-onboard-cv-forward.service \
  cuadc-onboard-video-forward.service 2>/dev/null || true
systemctl enable cuadc-onboard-cv-preview-ctl.service
systemctl enable --now cuadc-onboard-cv-preview-ctl.service
systemctl enable --now cuadc-onboard-video-forward-ctl.service
systemctl restart cuadc-onboard-profile.service
echo "ok: cv-ctl 5611 default off; video-ctl 5612 default on"
systemctl is-active cuadc-onboard-cv-preview-ctl.service \
  cuadc-onboard-video-forward-ctl.service cuadc-onboard-telem.service
