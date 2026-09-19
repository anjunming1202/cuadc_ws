# cuadc_groundstation

地面站(地面笔记本侧)运行时:收机载推来的**图传 + 遥测**,在 localhost 起一个
网页把两者同屏显示。是 `cuadc_onboard` 的**地面对端**——非 ROS、低延迟直路
(与 `cuadc_viz` 的 sim/ROS 仪表盘互补,不是同一个东西)。

**非 colcon 包**(纯 Python + 网页),和 `cuadc_onboard` 一样游离在 ROS 任务链外。

## 布局

```
server.py             收 5600 RTP 视频(经本机 ffmpeg 转 MPEG-TS)+ 5701/5601 遥测 JSON，
                        在 http://127.0.0.1:8765/ 提供网页(视频 + HUD)
index.html / mpegts.js 网页前端(mpegts.js 播放 /stream.ts，HUD 读 /telemetry.json)
send_fake_telemetry.py 无飞控时本机灌假遥测测 HUD
DroneLink.sdp          air-link 参考(运行时不读，ffmpeg 用自动生成的 loopback SDP)
start.bat              Windows 启动:从本目录跑 python server.py
```

## 起法(在 Windows 上,不是 WSL)

前提:笔记本连 **DroneLink** AP、静态 IP `192.168.50.2/24`;装好 **Python** 与
**ffmpeg**(`winget install --id Gyan.FFmpeg -e`)。机载端(Pi)已在推流。

1. 双击 `start.bat`(或在本目录 `python server.py`)。
2. 浏览器打开 **http://127.0.0.1:8765/** → 视频 + 遥测 HUD。

无飞控自测 HUD:另开一个终端 `python send_fake_telemetry.py`(默认发 127.0.0.1:5701)。

## 端点

收的端口就是机载端点契约的地面一侧(图传 `:5600`、遥测 JSON `:5701`/`:5601`)。
契约单一真源见 [`docs/spec/onboard-runtime.md`](../../docs/spec/onboard-runtime.md),此处不复制。

## 部署

源在仓库(本目录)。地面笔记本上从这里跑即可——把本目录拷到笔记本,或直接在
笔记本的仓库检出里跑 `start.bat`。运行副本只是部署产物,改动一律回到本目录。
