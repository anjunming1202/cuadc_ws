# Pi burn pack：video / OpenCV ctl

地面按钮在 `src/cuadc_groundstation/`。本目录是拷到 Pi `/tmp` 再烧进
**正式路径** `/opt/cuadc-onboard` + `/etc/systemd/system` 的文件清单。
脚本权威源仍是 `src/cuadc_onboard/`；改逻辑先改那边再同步本包。

不解锁、不碰 `dronelink-network` / eth0。

## 已于 2026-10-04 烧过

在 **Pi 本机**（已经 SSH 上去就不要再 `ssh 192.168.50.1`）：

```bash
sudo bash /tmp/backup_and_install.sh
```

结果：`video-forward-ctl` 5612 默认开；`cv-preview-ctl` 5611 默认关；
旧常开 `video-forward` / `cv-preview` / `cv-forward` 已 disable。

## 备份在哪（Pi 上）

| 目录 | 说明 |
|---|---|
| **`/home/wly/cuadc-onboard-backup-20261004_133328`** | 该脚本 root 备份，**回档用这份** |
| `/home/wly/cuadc-onboard-backup-20261004_132743` | 安装前用户快照（无 sudo） |

```bash
sudo /home/wly/cuadc-onboard-backup-20261004_133328/restore.sh
```

细节与当前 unit 表见 `src/cuadc_onboard/README.md`。

## 再烧一次

Windows 上把本目录文件 scp 到 `wly@192.168.50.1:/tmp/`（含
`backup_and_install.sh`、两个 ctl `.py`、两个 `.service`、`apply-profile.sh`，
可选 `telem_ground.py`），然后 Pi 上：

```bash
sudo bash /tmp/backup_and_install.sh
```

只装、不再备份：`sudo bash /tmp/install-video-ctls.sh`。
无 sudo 只做家目录快照：`bash /tmp/backup_onboard_user.sh`。
