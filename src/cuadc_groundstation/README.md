# cuadc_groundstation

地面站(地面笔记本侧)运行时:收机载推来的**图传 + 遥测**,在 localhost 起一个
网页把两者同屏显示。是 `cuadc_onboard` 的**地面对端**——非 ROS、低延迟直路
(与 `cuadc_viz` 的 sim/ROS 仪表盘互补,不是同一个东西)。

**非 colcon 包**(纯 Python + 网页),和 `cuadc_onboard` 一样游离在 ROS 任务链外。

## 布局

```
server.py             收 5600 干净图传 + 5606 CV 预览(本机 ffmpeg 转 MPEG-TS)+ 5701/5601 遥测，
                        在 http://127.0.0.1:8765/ 提供双画面网页 + HUD；
                        并代理机载图传/CV 开关(UDP → Pi 5612 / 5611)
index.html            网页：顶上静态赛道 + 橙机位 + 近 30s 高度折线 + 双视频 + HUD
field_pose.py         机位：FC NED → 起飞坐标系（与任务同一定义，解锁时锁原点）；近 30s 高度缓冲
replay_telemetry.py   机载 mcap/zip → jsonl，并可 UDP 回放到 viewer
replays/              回放数据（如 20261004_152633_suite.jsonl）
send_fake_telemetry.py 无飞控时本机灌假遥测测 HUD / 赛道图（6s 后才变 GUIDED）
DroneLink.sdp          air-link 参考(运行时不读，ffmpeg 用自动生成的 loopback SDP)
start.bat              Windows 启动:从本目录跑 python server.py
```

## 起法(在 Windows 上,不是 WSL)

前提:笔记本连 **DroneLink** AP、静态 IP `192.168.50.2/24`;装好 **Python** 与
**ffmpeg**(`winget install --id Gyan.FFmpeg -e`)。机载端(Pi)已在推流。

1. 双击 `start.bat`(或在本目录 `python server.py`)。**同时只开一份**，两份会抢 UDP 5600/5701 和 ffmpeg，图传会黑。
2. 浏览器打开 **http://127.0.0.1:8765/** → 左干净图传 / 右 CV 预览 + 遥测 HUD。
   HUD 的 **EKF/GPS** 行显示机载 `ekf_ready`（与接口层同一判据）及 `fix`/`sats`。
   打开页面**顶部就是赛道图**（写在 HTML 里，不需要飞控、也不用等 JS）。
   `start.bat` 只是 `cd` 到本目录再 `python server.py`，两者等价。改完代码后要
   **重启 server 并在浏览器 Ctrl+F5 强刷**，只重启 Python 不刷新页面会仍看旧 HTML。
   **机位 = 起飞坐标系**（x 前、y 右，与任务同一定义，见
   [coordinate-frames.md](../../docs/spec/coordinate-frames.md)）：解锁那一刻锁原点（= H）和机头，
   上锁后保持，下次解锁重锁；从未解锁时钉在 H。数传没有 `armed` 字段（老版本/假遥测）时按进入 GUIDED 近似。
   优先 `ned_n`/`ned_e`，没有则用经纬度。机载只回传原始字段。
   再下方是**近 30 秒高度折线**（真实 `alt_m`）。暂不画圆桶。

### 图传 / OpenCV 开关（顶栏按钮）

浏览器 → `server.py` HTTP → Pi UDP JSON（主机默认 `192.168.50.1,10.10.10.1`）：

| 按钮 | HTTP | Pi UDP | 机载脚本 | 开机默认 |
|------|------|--------|----------|----------|
| **图传** | `/video-forward/start\|stop\|status` | **5612** | `video_forward_ctl.py`（5603→5600） | **开** |
| **OpenCV** | `/cv-preview/start\|stop\|status` | **5611** | `cv_preview_ctl.py`（检测+5604→5606） | **关** |

需机载已装对应 ctl 单元。2026-10-04 已烧到 Pi 正式路径（`/opt/cuadc-onboard`，
开机默认：图传开 / OpenCV 关）。备份与回档见
[`cuadc_onboard` README](../cuadc_onboard/README.md)（目录
`/home/wly/cuadc-onboard-backup-20261004_133328`）。烧录包在
`local/pi_burn_video_ctls/`。`--air-ctrl-hosts` 可改 Pi 地址。

无飞控自测 HUD 和赛道图:另开一个终端 `python send_fake_telemetry.py`(默认发 127.0.0.1:5701)。
前约 6 秒 `STABILIZE`（飞机应钉在 H），之后 `GUIDED` 沿 +x 走动。

回放一次真实飞行（viewer 先开着）：

```bash
python replay_telemetry.py convert recordings.zip -o replays/20261004_152633_suite.jsonl
python replay_telemetry.py replay replays/20261004_152633_suite.jsonl
```

`--speed 4` 可四倍速。解锁前钉在 H；该袋约 336 s，156 s 解锁，318 s RTL，高度约 0–5 m。

## 录制(每次运行都可开,和任务解耦)

viewer 顶栏「● 录制」按钮开/停,一键**同时**录两路视频——干净图传
`recordings/<时间戳>.ts` 与 CV 预览(带检测框)`<时间戳>_cv.ts`——外加同名
`<时间戳>.jsonl`(每秒一条遥测)。录制只是两个 remux hub 的**订阅者**,不碰视频
主路、不和任何测试/任务代码耦合——调相机/单点测试/正式任务都照录。
`--record-dir <目录>` 可改落点。端点(前端按钮即调这些):
`POST /record/start[?label=xxx]`、`POST /record/stop`、`GET /record/status`。
录的是地面收到的流(Wi-Fi 副本,可能带丢包);全质量的权威录制在**机载侧**
(`cuadc_onboard/scripts/onboard_record.py`:源头 H.264 tee 5605 + rosbag)。

## 端点

收的端口就是机载端点契约的地面一侧(干净图传 `:5600`、CV 预览 `:5606`、遥测 JSON `:5701`/`:5601`)。
契约单一真源见 [`docs/spec/onboard-runtime.md`](../../docs/spec/onboard-runtime.md),此处不复制。

## 部署

源在仓库(本目录)。地面笔记本上从这里跑即可——把本目录拷到笔记本,或直接在
笔记本的仓库检出里跑 `start.bat`。运行副本只是部署产物,改动一律回到本目录。
