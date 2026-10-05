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
install -m 0755 "$SRC_DIR/scripts/camera_service.py" "$OPT/camera_service.py"
install -m 0755 "$SRC_DIR/scripts/cv_preview_ctl.py" "$OPT/cv_preview_ctl.py"
install -m 0755 "$SRC_DIR/systemd/apply-profile.sh"  "$OPT/apply-profile.sh"
# CV 检测/预览的代码在 cuadc_perception + cuadc_physical(cv_preview_ctl 按 PYTHONPATH 引用,
# 见 ADR-0008),不拷到 /opt。

for u in "$SRC_DIR"/systemd/*.service; do
  install -m 0644 "$u" "$UNIT_DIR/$(basename "$u")"
done
# 单元文件是唯一真源：现场手写的扇出 drop-in 会静默盖掉 ExecStart(波特率)，收敛掉。
rm -f "$UNIT_DIR/cuadc-onboard-fanout.service.d/override.conf"
rmdir "$UNIT_DIR/cuadc-onboard-fanout.service.d" 2>/dev/null || true

# 已退役单元(被 cv-preview-ctl 的按钮路径取代):老 Pi 上可能还装着,收敛掉。
for u in cuadc-onboard-cv-preview.service cuadc-onboard-cv-forward.service; do
  systemctl disable --now "$u" 2>/dev/null || true
  rm -f "$UNIT_DIR/$u"
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
                 cuadc-onboard-profile.service \
                 cuadc-onboard-cv-preview-ctl.service
# video-forward 由 ground-monitor profile 拉起;CV 检测/转发由 ctl 默认关、地面按钮才开

echo
echo "installed + enabled base (fanout, camera) + profile applier."
echo "profile = $(cat "$ETC/profile")  (edit $ETC/profile, then restart cuadc-onboard-profile)"
echo
echo "扇出必须对『刚上电的飞控』开串口，不能热交接（见 ADR-0004），所以启用后请整机断电重启一次。"
echo "迁移旧的仓库外自启（单独、可回退，绝不动 network/eth0）："
echo "  systemctl disable --now dronelink-telemetry.service dronelink-video.service"
