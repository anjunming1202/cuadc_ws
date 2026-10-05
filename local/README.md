# local/ — 现场脚本（不上 Pi 系统路径）

笔记本侧临时脚本与 Pi 烧录包。正式机载代码在 `src/cuadc_onboard/`，
地面 viewer 在 `src/cuadc_groundstation/`。

| 路径 | 用途 |
|---|---|
| `pi_burn_video_ctls/` | 2026-10-04 图传/CV ctl 烧录包 + 备份脚本 |
| `flight_ctrl_tests/` | Pi `~/flight_ctrl_tests` 沙盒：听 14550、遥控解锁、投放、EKF/GPS |
| `install-*.sh` / `kill_*.sh` / `uart_probe.py` 等 | 2026-10-03 串口/扇出排查，勿当常驻服务 |

Pi 正式树备份不在仓库里，在机上：

`/home/wly/cuadc-onboard-backup-20261004_133328`

回档：`sudo …/restore.sh`。见 `src/cuadc_onboard/README.md`。
