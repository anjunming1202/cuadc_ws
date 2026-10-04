#!/bin/bash
# User-readable snapshot of current official onboard paths (no sudo).
# Rollback still needs: sudo $BACKUP/restore.sh
set -euo pipefail
STAMP=$(date +%Y%m%d_%H%M%S)
BACKUP=/home/wly/cuadc-onboard-backup-${STAMP}
OPT=/opt/cuadc-onboard
UNIT=/etc/systemd/system
mkdir -p "$BACKUP/units" "$BACKUP/opt-cuadc-onboard"

echo "=== backup to $BACKUP ==="
if [ -d "$OPT" ]; then
  cp -a "$OPT/." "$BACKUP/opt-cuadc-onboard/"
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
    if [ -d "$UNIT/${u}.d" ]; then
      cp -a "$UNIT/${u}.d" "$BACKUP/units/"
    fi
  done
} > "$BACKUP/unit-state.txt"

if [ -r /etc/cuadc-onboard/profile ]; then
  cp -a /etc/cuadc-onboard/profile "$BACKUP/profile"
fi

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
if [ -f "$HERE/profile" ]; then
  install -d /etc/cuadc-onboard
  cp -a "$HERE/profile" /etc/cuadc-onboard/profile
fi
systemctl daemon-reload
# Re-enable the always-on consumers that were running before video-ctl.
systemctl enable --now cuadc-onboard-video-forward.service 2>/dev/null || true
systemctl enable --now cuadc-onboard-cv-preview.service 2>/dev/null || true
systemctl enable --now cuadc-onboard-cv-forward.service 2>/dev/null || true
systemctl restart cuadc-onboard-profile.service 2>/dev/null || true
echo "restored. see $HERE/unit-state.txt for previous enable/active."
systemctl is-active cuadc-onboard-video-forward.service \
  cuadc-onboard-cv-preview.service cuadc-onboard-telem.service || true
EOF
chmod 0755 "$BACKUP/restore.sh"
echo "$BACKUP" > /tmp/cuadc-onboard-last-backup
echo "BACKUP_OK $BACKUP"
ls -la "$BACKUP" "$BACKUP/opt-cuadc-onboard" "$BACKUP/units"
echo "--- unit-state ---"
cat "$BACKUP/unit-state.txt"
echo "rollback: sudo $BACKUP/restore.sh"
