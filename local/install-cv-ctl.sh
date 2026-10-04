#!/usr/bin/env bash
# Install cv-preview-ctl as a boot service (detector stays off until the viewer button).
set -euo pipefail
install -m 0755 /tmp/cv_preview_ctl.py /opt/cuadc-onboard/cv_preview_ctl.py
install -m 0755 /tmp/cv_preview_rtp.py /opt/cuadc-onboard/cv_preview_rtp.py
install -m 0755 /tmp/apply-profile.sh /opt/cuadc-onboard/apply-profile.sh
install -m 0644 /tmp/cuadc-onboard-cv-preview-ctl.service /etc/systemd/system/cuadc-onboard-cv-preview-ctl.service
systemctl daemon-reload
systemctl stop cuadc-onboard-cv-preview.service cuadc-onboard-cv-forward.service || true
systemctl disable cuadc-onboard-cv-preview.service cuadc-onboard-cv-forward.service || true
systemctl enable --now cuadc-onboard-cv-preview-ctl.service
echo "active:"
systemctl is-active cuadc-onboard-cv-preview-ctl.service || true
systemctl is-active cuadc-onboard-cv-preview.service || true
systemctl is-active cuadc-onboard-cv-forward.service || true
echo "enabled ctl:"
systemctl is-enabled cuadc-onboard-cv-preview-ctl.service
