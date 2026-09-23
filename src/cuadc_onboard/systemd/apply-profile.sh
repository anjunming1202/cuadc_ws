#!/usr/bin/env bash
# cuadc_onboard 自启薄壳：读 /etc/cuadc-onboard/profile，按名把「消费者」拉起。
# 底座（fanout/camera）各自 enable、独立开机自启，本脚本只负责额外的 profile 消费者。
# 改 profile：编辑该文件后 `systemctl restart cuadc-onboard-profile`（或重启整机）。
set -euo pipefail

PROFILE_FILE=/etc/cuadc-onboard/profile
profile="none"
[ -r "$PROFILE_FILE" ] && profile="$(tr -d '[:space:]' < "$PROFILE_FILE")"
echo "cuadc-onboard profile: ${profile:-none}"

# 本模块受管的「消费者」单元。切 profile 时先全停、再按 profile 起需要的，
# 使 applier 成为「把状态收敛到 profile」的幂等操作：改完 profile 文件跑
# `systemctl restart cuadc-onboard-profile` 就即时生效，不必重启整机。
# 新增消费者：把其单元加进这里 + 下面对应 profile 的 case 分支。
KNOWN_CONSUMERS=(
  cuadc-onboard-telem.service
  cuadc-onboard-video-forward.service
  cuadc-onboard-cv-preview.service
  cuadc-onboard-cv-forward.service
)

for u in "${KNOWN_CONSUMERS[@]}"; do
  systemctl stop "$u" 2>/dev/null || true
done

case "${profile:-none}" in
  none)
    : # 只跑底座，消费者全手动（相机服务仍在推本地端点，只是没人转发到地面）
    ;;
  ground-monitor)
    # 外场监看：数传 + 干净图传(5603→5600) + CV 标注图传(5604→5606)。
    # 不自启 cv-preview：任务/感知节点(preview_enabled)自己就产出 5604 标注流，独立的
    # cv-preview 会跟它抢单客户端 shm。cv-forward 只把 5604 转到地面(谁产的都转)，因此
    # 与任务/L1 感知同时跑不冲突。相机自检/纯调参需要独立预览时再手动起 cv-preview(ADR-0008)。
    systemctl start cuadc-onboard-telem.service
    systemctl start cuadc-onboard-video-forward.service
    systemctl start cuadc-onboard-cv-forward.service
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
