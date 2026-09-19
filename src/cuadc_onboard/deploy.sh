#!/usr/bin/env bash
# 幂等把 cuadc_onboard 装到 Pi 系统路径并 enable 底座。在 Pi 上以 root 跑。
#   scripts/ -> /opt/cuadc-onboard/    units -> /etc/systemd/system/
#   /etc/cuadc-onboard/profile 缺省才写（不覆盖现场设置）
# enable 的是底座（fanout/camera）+ profile applier；消费者由 profile 决定。
# 绝不碰 dronelink-network / eth0-100m —— 现场 SSH 骑在 AP/以太网上。
# 迁移旧的仓库外自启（disable dronelink 的 telemetry/video）是单独一步，见末尾提示。
set -euo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPT=/opt/cuadc-onboard
ETC=/etc/cuadc-onboard
UNIT_DIR=/etc/systemd/system
PROFILE_ARG="${1:-}"   # 可选：none|ground-monitor|mission（仅在 profile 文件不存在时写入）

if [ "$(id -u)" -ne 0 ]; then
  echo "run as root: sudo $0 [profile]" >&2
  exit 1
fi

install -d "$OPT" "$ETC"
install -m 0755 "$SRC_DIR/scripts/mav_fanout.py"     "$OPT/mav_fanout.py"
install -m 0755 "$SRC_DIR/scripts/telem_ground.py"   "$OPT/telem_ground.py"
install -m 0755 "$SRC_DIR/systemd/apply-profile.sh"  "$OPT/apply-profile.sh"

for u in "$SRC_DIR"/systemd/*.service; do
  install -m 0644 "$u" "$UNIT_DIR/$(basename "$u")"
done

if [ ! -f "$ETC/profile" ]; then
  echo "${PROFILE_ARG:-none}" > "$ETC/profile"
  echo "wrote $ETC/profile = ${PROFILE_ARG:-none}"
else
  echo "kept existing $ETC/profile = $(cat "$ETC/profile")"
fi

systemctl daemon-reload
systemctl enable cuadc-onboard-fanout.service \
                 cuadc-onboard-camera.service \
                 cuadc-onboard-profile.service

echo
echo "installed + enabled base (fanout, camera) + profile applier."
echo "profile = $(cat "$ETC/profile")  (edit $ETC/profile, then restart cuadc-onboard-profile)"
echo
echo "扇出必须对『刚上电的飞控』开串口，不能热交接（见 ADR-0004），所以启用后请整机断电重启一次。"
echo "迁移旧的仓库外自启（单独、可回退，绝不动 network/eth0）："
echo "  systemctl disable --now dronelink-telemetry.service dronelink-video.service"
