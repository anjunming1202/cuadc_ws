# cuadc_onboard

机载伴随计算机（树莓派）的**运行时模块**：把飞控串口和相机做成多消费者
可共享的本地端点，并把整条链路做成仓库管理的开机自启。设计与端点契约见
[`docs/spec/onboard-runtime.md`](../../docs/spec/onboard-runtime.md)，此处只讲布局与操作。

**非 colcon 包**（systemd + 脚本），与 `src/dronelink` 一样游离在任务链路外，
但设计成可合回 master、prod/test 共用。

## 布局

```
scripts/   通信程序（能手动跑，不依赖 systemd）
             mav_fanout.py         串口扇出（底座）
             telem_ground.py       数传发送：读扇出 → JSON 发地面 HUD（含 ned_n/ned_e、lat/lon；赛道投影在地面算）
             video_forward_ctl.py  干净图传开关 UDP 5612（默认开 5603→5600）
             cv_preview_ctl.py     OpenCV 开关 UDP 5611（默认关）
             cv_preview_rtp.py     ctl 拉起的检测 worker（5602→5604）
systemd/   自启薄壳
             cuadc-onboard-fanout.service   底座：串口扇出
             cuadc-onboard-camera.service   底座：相机 tee（本地 5602 + 5603）
             cuadc-onboard-telem.service    消费者：数传（ground-monitor profile 拉起）
             cuadc-onboard-video-forward-ctl.service  图传 ctl（profile 拉起，默认开）
             cuadc-onboard-cv-preview-ctl.service     CV ctl（底座 enable，默认关）
             cuadc-onboard-video-forward.service      旧常开单元（改用 ctl，勿同开）
             cuadc-onboard-cv-preview.service / cv-forward.service  旧常开（改用 ctl）
             cuadc-onboard-profile.service  oneshot applier：按 profile 起消费者
             apply-profile.sh               applier 逻辑（读 /etc/cuadc-onboard/profile）
deploy.sh  幂等安装到 Pi 系统路径并 enable 底座 + profile applier + cv-preview-ctl
```

## 端点契约（消费者连这些，不碰硬件）

| 资源 | 端点 | 协议 |
|---|---|---|
| 飞控 MAVLink | `udpin:127.0.0.1:14550` / `14551` | MAVLink（扇出主动推流，绑定即收；等同 mavproxy `--out`） |
| 相机本地帧 | `udp:127.0.0.1:5602` | H.264/RTP（CV/采集/调参读帧） |
| 相机控制口 | `udpin:127.0.0.1:5610` | JSON（`{ae,exposure_us,gain,af,lens,…}` 运行时改拍摄参数） |
| 图传转发输入 | `udp:127.0.0.1:5603` | H.264/RTP（video-forward 读它转地面） |
| CV 预览本地 | `udp:127.0.0.1:5604` | H.264/RTP（cv-preview 产出；cv-forward 转地面） |
| 图传→地面 | `192.168.50.2:5600` | H.264/RTP（video-forward / video_forward_ctl） |
| CV 预览→地面 | `192.168.50.2:5606` | H.264/RTP（cv_preview_ctl 内转发） |
| 图传开关（地面→机载） | `udp:0.0.0.0:5612` | JSON `{cmd:on\|off\|status}` |
| OpenCV 开关（地面→机载） | `udp:0.0.0.0:5611` | JSON `{cmd:on\|off\|status}` |
| 地面数据总线 | `udp:127.0.0.1:5700` | JSON（含 `topic`） |
| GCS | `192.168.50.2:14560` | MAVLink |

消费两个主要端点的最小示例（都不碰硬件，与底座并行）：

```python
# 飞控遥测：把 ('/dev/ttyAMA0', 57600) 换成端点即可，下游不变
mavutil.mavlink_connection("udpin:127.0.0.1:14550")

# 相机帧（cv2）：本机只装了 openh264dec，用它解码（avdec_h264 不在，会开管线失败）
cv2.VideoCapture(
    'udpsrc port=5602 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" '
    "! rtpjitterbuffer latency=50 ! rtph264depay ! h264parse ! openh264dec "
    "! videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false max-buffers=2",
    cv2.CAP_GSTREAMER)
```

## 自启两层

- **底座**（持硬件、产端点）无条件开机自启：mav_fanout（串口扇出）+
  camera-service（独占相机 → tee 本地 5602 + 5603；监听 5610 运行时改曝光/对焦，
  gst 应用非静态 gst-launch）。
- **消费者**（只连端点、不碰硬件）由 profile 选择性自启：`/etc/cuadc-onboard/profile`
  写 `none` / `ground-monitor` / `mission`，oneshot applier 按名拉起。`ground-monitor`
  起 **数传 + video-forward-ctl（默认开 5603→5600）**；OpenCV 由底座 enable 的
  **cv-preview-ctl** 听地面按钮，**默认关**。改 profile 后
  `systemctl restart cuadc-onboard-profile` 即时生效。`none` 时底座在跑、本地端点有数据，但地面什么都没有。

## 部署 / 迁移（在 Pi 上）

```bash
sudo ./deploy.sh ground-monitor   # 装脚本/单元、写 profile（缺省才写）、enable 底座 + applier
```

`deploy.sh` 只装+enable 本模块，**不碰任何 dronelink 服务**。现场只改图传/CV
开关时，也可用 `local/pi_burn_video_ctls/backup_and_install.sh`（先备份再装 ctl）。
迁移旧的仓库外自启是单独一步——**只 disable 要替换的两个**，不删文件、可回退：

```bash
sudo systemctl disable --now dronelink-telemetry.service dronelink-video.service
# 绝不动 dronelink-network / eth0-100m —— SSH 骑在 AP/以太网上
```

**扇出必须对『刚上电的飞控』开串口，不能热交接（见 ADR-0004），迁移启用后
须整机断电重启一次**，让扇出接一个没被交接搞过的飞控。

## 现场（2026-10-03）

- **波特率**：单元 `--baud 57600`（`SERIAL*_BAUD=57`）。USB 的 115200 不是 TELEM 默认。
- **物理口**：本架 TELEM2 无心跳；改插 **TELEM3** 后扇出出现 `heartbeat sys=1`。图传 UDP 与飞控串口无关。
- **禁止 30s 重开串口**：心跳超时若 `close` 再 `open`，这块飞控 TELEM 会哑到断电。`mav_fanout.py` 改为一直持有 `/dev/ttyAMA0`，只打 `still no heartbeat; keeping serial open`。
- **OpenCV 预览**：`cuadc-onboard-cv-preview-ctl` 开机默认关，地面 viewer 按钮经 UDP 5611 才起检测（省 CPU）。

## 2026-10-04 现场：图传/CV ctl 已装到正式路径

Pi 运行副本：`/opt/cuadc-onboard/`（脚本）+ `/etc/systemd/system/cuadc-onboard-*.service`。
当前 profile：`/etc/cuadc-onboard/profile` = `ground-monitor`。

| 单元 | 口 | 开机默认 | 2026-10-04 13:33 状态 |
|---|---|---|---|
| `cuadc-onboard-video-forward-ctl` | UDP **5612** | 干净图传 **开**（5603→地面 5600） | enabled + active |
| `cuadc-onboard-cv-preview-ctl` | UDP **5611** | OpenCV **关** | enabled + active |
| 旧 `video-forward` / `cv-preview` / `cv-forward` | — | 勿与 ctl 同开 | disabled + inactive |
| fanout / camera / telem / profile | — | 底座+数传照旧 | active（未停网络） |

烧录命令（文件先 scp 到 Pi `/tmp/`，在 **Pi 本机**跑，不要再 `ssh 192.168.50.1`）：

```bash
sudo bash /tmp/backup_and_install.sh
```

仓库里同一套：`local/pi_burn_video_ctls/`（`backup_and_install.sh`、ctl 脚本、unit）。
权威源在本目录 `scripts/` + `systemd/`；烧录包是拷到 `/tmp` 用的副本。

### 回档备份（都在 Pi 用户家目录）

| 目录 | 何时 | 用途 |
|---|---|---|
| **`/home/wly/cuadc-onboard-backup-20261004_133328`** | 安装脚本以 root 跑的那份 | **回档用这个** |
| `/home/wly/cuadc-onboard-backup-20261004_132743` | 安装前用户可读快照 | 备用 |

每份里有当时的 `opt-cuadc-onboard/`、`units/`、`profile`、`unit-state.txt`、`restore.sh`。

```bash
sudo /home/wly/cuadc-onboard-backup-20261004_133328/restore.sh
```

会把 `/opt` 和 unit 拷回去，并重新拉起旧的常开 video/cv 单元。不解锁、不碰 `dronelink-network`。

装完若 5611 被抢：不要留 `/home/wly/cv_preview_ctl.py` 那种家目录副本和正式 ctl 一起听。

数传 JSON 现含 `ned_n`/`ned_e`/`lat_deg`/`lon_deg`（赛道图在**地面**投影，Pi 不算）。
只换数传、不动扇出/网络：

```bash
sudo install -m 0755 /tmp/telem_ground.py /opt/cuadc-onboard/telem_ground.py
sudo systemctl restart cuadc-onboard-telem.service
```

2026-10-04 19:16 已装到本机 `/opt`。HUD 无 `ned_*` 时先确认 Pi 上是这份脚本。

> 状态：底座（扇出 + 相机 tee）与数传消费者、profile、deploy.sh 已就位；
> 扇出多消费者共存已真机验证（2026-09-19）；2026-10-03 TELEM3@57600 地面 HUD 有飞控数据。
> 图传/CV 地面按钮 ctl 已于 2026-10-04 写入正式路径（见上表）。
> ground-bus（5700→地面 topic 转发）暂缓。为何用自写扇出见 ADR-0004。
