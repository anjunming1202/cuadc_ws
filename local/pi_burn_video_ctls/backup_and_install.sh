#!/bin/bash
# Backup current /opt/cuadc-onboard + related units, then install video/CV ctls.
# Does not arm. Does not touch dronelink-network.
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
  echo "run: sudo $0" >&2
  exit 1
fi

STAMP=$(date +%Y%m%d_%H%M%S)
BACKUP=/home/wly/cuadc-onboard-backup-${STAMP}
OPT=/opt/cuadc-onboard
UNIT=/etc/systemd/system
install -d -o wly -g wly "$BACKUP/units" "$BACKUP/opt-cuadc-onboard"

echo "=== backup to $BACKUP ==="
if [ -d "$OPT" ]; then
  cp -a "$OPT/." "$BACKUP/opt-cuadc-onboard/"
else
  echo "warn: $OPT missing"
fi

UNITS=(
  cuadc-onboard-video-forward.service
  cuadc-onboard-cv-preview.service
  cuadc-onboard-cv-forward.service
  cuadc-onboard-cv-preview-ctl.service
  cuadc-onboard-video-forward-ctl.service
  cuadc-onboard-telem.service
  cuadc-onboard-profile.service
  cuadc-onboard-camera.service
  cuadc-onboard-fanout.service
)
{
  echo "# unit enabled/active snapshot ${STAMP}"
  for u in "${UNITS[@]}"; do
    en=$(systemctl is-enabled "$u" 2>/dev/null || echo missing)
    ac=$(systemctl is-active "$u" 2>/dev/null || echo missing)
    echo "$u enabled=$en active=$ac"
    if [ -f "$UNIT/$u" ]; then
      cp -a "$UNIT/$u" "$BACKUP/units/"
    fi
  done
} > "$BACKUP/unit-state.txt"

# Restore script: copy trees back, re-enable snapshot-ish old forward/cv units.
cat > "$BACKUP/restore.sh" <<'EOF'
#!/bin/bash
# Restore onboard files from this backup directory. Run as root.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$(id -u)" -ne 0 ]; then
  echo "run: sudo $0" >&2
  exit 1
fi
echo "restoring /opt/cuadc-onboard from $HERE"
systemctl stop cuadc-onboard-cv-preview-ctl.service \
  cuadc-onboard-video-forward-ctl.service 2>/dev/null || true
systemctl disable cuadc-onboard-cv-preview-ctl.service \
  cuadc-onboard-video-forward-ctl.service 2>/dev/null || true
cp -a "$HERE/opt-cuadc-onboard/." /opt/cuadc-onboard/
if [ -d "$HERE/units" ]; then
  cp -a "$HERE/units/." /etc/systemd/system/
fi
systemctl daemon-reload
# Put profile consumers back to pre-ctl always-on video+cv if those units exist.
systemctl enable --now cuadc-onboard-video-forward.service 2>/dev/null || true
systemctl enable --now cuadc-onboard-cv-preview.service 2>/dev/null || true
systemctl enable --now cuadc-onboard-cv-forward.service 2>/dev/null || true
systemctl restart cuadc-onboard-profile.service 2>/dev/null || true
echo "restored. see $HERE/unit-state.txt for previous enable/active."
systemctl is-active cuadc-onboard-video-forward.service \
  cuadc-onboard-cv-preview.service cuadc-onboard-telem.service || true
EOF
chmod 0755 "$BACKUP/restore.sh"
chown -R wly:wly "$BACKUP"
echo "backup ok: $BACKUP"
echo "rollback: sudo $BACKUP/restore.sh"
echo

echo "=== install ctls ==="
install -m 0755 /tmp/cv_preview_ctl.py "$OPT/cv_preview_ctl.py"
install -m 0755 /tmp/cv_preview_rtp.py "$OPT/cv_preview_rtp.py"
install -m 0755 /tmp/video_forward_ctl.py "$OPT/video_forward_ctl.py"
install -m 0644 /tmp/cuadc-onboard-cv-preview-ctl.service "$UNIT/"
install -m 0644 /tmp/cuadc-onboard-video-forward-ctl.service "$UNIT/"
install -m 0755 /tmp/apply-profile.sh "$OPT/apply-profile.sh"
if [ -f /tmp/telem_ground.py ]; then
  install -m 0755 /tmp/telem_ground.py "$OPT/telem_ground.py"
fi
systemctl daemon-reload
systemctl stop cuadc-onboard-cv-preview.service cuadc-onboard-cv-forward.service \
  cuadc-onboard-video-forward.service 2>/dev/null || true
systemctl disable cuadc-onboard-cv-preview.service cuadc-onboard-cv-forward.service \
  cuadc-onboard-video-forward.service 2>/dev/null || true
systemctl enable --now cuadc-onboard-cv-preview-ctl.service
systemctl enable --now cuadc-onboard-video-forward-ctl.service
systemctl restart cuadc-onboard-profile.service
systemctl restart cuadc-onboard-telem.service 2>/dev/null || true
echo "ok: cv-ctl 5611 default off; video-ctl 5612 default on"
systemctl is-active cuadc-onboard-cv-preview-ctl.service \
  cuadc-onboard-video-forward-ctl.service cuadc-onboard-telem.service
echo "BACKUP=$BACKUP"
