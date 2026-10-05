#!/usr/bin/env bash
# User-level boot: cv_preview_ctl default off (no sudo). System unit is in the repo for later deploy.sh.
set -euo pipefail
cp /tmp/cv_preview_ctl.py /home/wly/cv_preview_ctl.py
cp /tmp/cv_preview_rtp.py /home/wly/cv_preview_rtp.py
chmod +x /home/wly/cv_preview_ctl.py /home/wly/cv_preview_rtp.py
python3 /tmp/kill_home_ctl.py || true
tmp=$(mktemp)
crontab -l 2>/dev/null | grep -v cv_preview_ctl.py >"$tmp" || true
echo '@reboot sleep 12; /usr/bin/python3 /home/wly/cv_preview_ctl.py --worker /home/wly/cv_preview_rtp.py >>/tmp/cv_preview_ctl.log 2>&1' >>"$tmp"
crontab "$tmp"
rm -f "$tmp"
nohup /usr/bin/python3 /home/wly/cv_preview_ctl.py --worker /home/wly/cv_preview_rtp.py >>/tmp/cv_preview_ctl.log 2>&1 &
sleep 1
echo 'crontab:'
crontab -l
echo 'procs:'
ps -eo pid,cmd | grep python | grep -v grep
echo 'log:'
tail -5 /tmp/cv_preview_ctl.log || true
echo 'linger:'
loginctl show-user wly -p Linger 2>/dev/null || true
