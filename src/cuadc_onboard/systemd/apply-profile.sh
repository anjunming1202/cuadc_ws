#!/usr/bin/env bash
# cuadc_onboard 自启薄壳：读 /etc/cuadc-onboard/profile，按名把「消费者」拉起。
# 底座（fanout/camera）各自 enable、独立开机自启，本脚本只负责额外的 profile 消费者。
# 改 profile：编辑该文件后 `systemctl restart cuadc-onboard-profile`（或重启整机）。
set -euo pipefail

PROFILE_FILE=/etc/cuadc-onboard/profile
profile="none"
[ -r "$PROFILE_FILE" ] && profile="$(tr -d '[:space:]' < "$PROFILE_FILE")"
echo "cuadc-onboard profile: ${profile:-none}"

case "${profile:-none}" in
  none)
    : # 只跑底座，消费者全手动
    ;;
  ground-monitor)
    # 外场监看：数传发地面（图传→地面已在 base camera tee 里）
    systemctl start cuadc-onboard-telem.service
    ;;
  mission)
    # 任务侧 bringup 按名引用；任务栈接线前为占位，不阻断底座
    systemctl start cuadc-onboard-mission.service 2>/dev/null \
      || echo "mission bringup not wired yet (task-side bringup 未接线)"
    ;;
  *)
    echo "unknown profile '$profile'; treating as none" >&2
    ;;
esac
