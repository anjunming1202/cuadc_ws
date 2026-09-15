# DroneLink

DroneLink is the current Raspberry Pi 5 based low-latency video-link setup used with an Arducam/Sony IMX519 camera and ASUS USB-AC56 (RTL8812AU) 5 GHz Wi-Fi adapters.

Ground display is a localhost browser viewer on the Windows PC (not a
standalone media player). Telemetry overlay on the same page is planned next.

## Current working configuration

The Raspberry Pi 5 captures the IMX519 through libcamera/PiSP at 1920x1080, 30 fps, NV12. GStreamer encodes the video with x264 in H.264 using a 6 Mbps low-latency configuration, packages it as RTP, and sends it over UDP to the Windows ground station at `192.168.50.2:5600`.

The air-side USB-AC56 creates a 5 GHz access point named `DroneLink` on channel 36. The Raspberry Pi uses `192.168.50.1/24`; the Windows USB Wi-Fi adapter uses static address `192.168.50.2/24`.

A separate Ethernet link is used for maintenance and SSH. The Raspberry Pi uses `10.10.10.1/24`, the Windows PC uses `10.10.10.2/24`. The Pi Ethernet link is forced to 100 Mbps full duplex because the tested 1 Gbps link occasionally dropped carrier, while 100 Mbps remained stable.

## Hardware

- Raspberry Pi 5
- Arducam camera based on Sony IMX519, approximately 16 MP native sensor resolution
- AK7375 VCM lens actuator driver installed on the Raspberry Pi
- ASUS USB-AC56 Wi-Fi adapter, RTL8812AU / `88XXau` driver
- Windows ground-station PC with a USB-AC56 Wi-Fi adapter

Autofocus was experimentally enabled and verified to move the AK7375 lens, but the current production video configuration does not use continuous autofocus because the IMX519 autofocus tuning was not calibrated.

## Video pipeline

Current video settings:

- Capture: 1920x1080 at 30 fps
- Raw format: NV12
- Encoder: GStreamer `x264enc`
- Codec: H.264/AVC
- Bitrate: 6000 kbps
- `tune=zerolatency`
- `speed-preset=ultrafast`
- `bframes=0`
- `rc-lookahead=0`
- `sync-lookahead=0`
- `sliced-threads=true`
- Keyframe interval: 30 frames
- Transport: RTP/H.264 over UDP
- Destination: `192.168.50.2:5600`

The systemd service definition is in `raspberrypi/systemd/dronelink-video.service`, with the final video command in `raspberrypi/systemd/dronelink-video.override.conf`.

## Raspberry Pi installation layout

Copy the files to these locations:

```text
raspberrypi/scripts/dronelink-network.sh
    -> /usr/local/sbin/dronelink-network.sh

raspberrypi/hostapd/hostapd.conf
    -> /etc/dronelink/hostapd.conf

raspberrypi/systemd/dronelink-network.service
    -> /etc/systemd/system/dronelink-network.service

raspberrypi/systemd/dronelink-video.service
    -> /etc/systemd/system/dronelink-video.service

raspberrypi/systemd/dronelink-video.override.conf
    -> /etc/systemd/system/dronelink-video.service.d/override.conf

raspberrypi/systemd/eth0-100m.service
    -> /etc/systemd/system/eth0-100m.service
```

After copying, make the network script executable and enable the services:

```bash
sudo chmod +x /usr/local/sbin/dronelink-network.sh
sudo systemctl daemon-reload
sudo systemctl enable dronelink-network.service
sudo systemctl enable dronelink-video.service
sudo systemctl enable eth0-100m.service
```

The Pi's Netplan configuration should give `eth0` the static address `10.10.10.1/24`. The exact Netplan file is not included here because the original machine also contains a private Wi-Fi SSID/password. A safe example is included as `raspberrypi/netplan-example.yaml`.

The current setup also depends on the locally installed RTL8812AU/88XXau driver and the separately built AK7375 kernel module. Those compiled kernel modules are not included in this repository because they are kernel-version dependent.

## Windows ground station

Set the DroneLink USB Wi-Fi adapter to:

```text
IPv4 address: 192.168.50.2
Subnet mask: 255.255.255.0
Default gateway: blank
DNS: blank
```

Connect to:

```text
SSID: DroneLink
Password: 12345678
```

### Localhost viewer

A Python server on the ground PC receives the RTP stream via
`windows/DroneLink.sdp`, remuxes it to MPEG-TS with FFmpeg (stream copy, no
re-encode), and serves a page at `http://127.0.0.1:8765/`. The browser plays
it with vendored `mpegts.js` under `windows/localhost_viewer/`.

Requirements: Python 3 on PATH, and FFmpeg on PATH (same as calibration:
`winget install --id Gyan.FFmpeg -e`).

```cmd
windows\start_localhost_viewer.bat
```

Then open Chrome or Edge at `http://127.0.0.1:8765/`. Keep the console window
open while viewing. Pi still sends to `192.168.50.2:5600`.

Brief Wi-Fi UDP loss (association still up) can desync the remux/MSE player.
The viewer auto-restarts FFmpeg and reconnects the page player.

Manual equivalent:

```cmd
python windows\localhost_viewer\server.py --sdp windows\DroneLink.sdp
```

Layout:

```text
windows/DroneLink.sdp                 RTP receiver description (UDP :5600)
windows/start_localhost_viewer.bat    launcher
windows/localhost_viewer/server.py    HTTP + FFmpeg remux
windows/localhost_viewer/index.html   page
windows/localhost_viewer/mpegts.js    vendored player
```

## Camera calibration

Calibration was performed on frames from the actual 1920x1080 video pipeline using a 9x6 inner-corner checkerboard with approximately 24 mm square size.

Results from the current calibration run:

```text
Image size: 1920x1080
Valid images: 124 of 131
RMS reprojection error: 0.301617 px

fx = 1738.74483
fy = 1742.05062
cx = 972.965849
cy = 520.215881

k1 =  0.09691173
k2 = -0.46843941
p1 = -0.00498614
p2 =  0.00222544
k3 =  1.18879159
```

`calibration/capture_calibration.py` receives the RTP stream through OpenCV/FFmpeg and saves one 1920x1080 frame per second for manual selection.

`calibration/calibrate_camera.py` detects the 9x6 checkerboard and calculates the camera matrix and distortion coefficients.

`calibration/test_undistort.py` loads the saved calibration and displays an original/undistorted comparison with checkerboard guide lines.

The generated `camera_calibration.npz` or `IMX519_1920x1080_calibration.npz` is a machine-specific calibration artifact. It should normally be archived with the project or release if you want the exact calibration available on another computer.

## Calibration capture requirements on Windows

Python, OpenCV and FFmpeg are required. FFmpeg was installed with:

```cmd
winget install --id Gyan.FFmpeg -e
```

OpenCV must report FFmpeg support. Before running the OpenCV RTP capture script in CMD, set:

```cmd
set OPENCV_FFMPEG_CAPTURE_OPTIONS=protocol_whitelist;file,udp,rtp
```

A convenience launcher is provided as `calibration/start_capture.bat`.

## Repository notes

Do not commit private home Wi-Fi credentials. The supplied Netplan file is only an example and deliberately contains no private SSID or password.

The WPA password for the dedicated DroneLink AP is currently present in `hostapd.conf`. If the repository will be public, change it before publishing.

The calibration parameters apply to the tested 1920x1080 camera/video mode. If the camera mode, crop, resolution, lens, or optical setup changes materially, recalibrate the camera.

## 新 Windows 地面站：clone 之后怎么跑通图传

下面只写**地面站 Windows**侧。前提是树莓派端已经按上文装好网络/图传服务，并在推流到 `192.168.50.2:5600`。一台从没配过 DroneLink 的电脑，在 `git clone` 本仓库后按顺序做：

### 1. 硬件

- 准备一块 **ASUS USB-AC56**（或同芯片 RTL8812AU）USB 网卡，插到这台 Windows 上。
- 若系统认不出网卡，先装好对应的 **RTL8812AU / 88XXau Windows 驱动**（以网卡厂商或芯片驱动为准），直到设备管理器里能看到这块无线网卡。

### 2. 软件（装到 PATH 里）

在 PowerShell 或「终端」里执行（有管理员权限更省事）：

```cmd
winget install --id Python.Python.3.12 -e
winget install --id Gyan.FFmpeg -e
```

装完后**新开一个**命令行窗口，确认都能找到：

```cmd
python --version
ffmpeg -version
```

浏览器用 **Chrome 或 Edge**（需支持 MSE 直播；不要用过旧的 IE）。

> 只需看图传、不跑标定的话：**不需要**再装 OpenCV / VLC。标定另见上文「Calibration capture requirements on Windows」。

### 3. 连上 DroneLink 专用 Wi‑Fi

1. 用这块 USB 网卡连接：
   - SSID：`DroneLink`
   - 密码：`12345678`（与 `raspberrypi/hostapd/hostapd.conf` 一致；若现场改过密码，以实际为准）
2. 给**这块 USB Wi‑Fi 适配器**设静态 IPv4（不要设错到家里的日常网卡上）：
   - IP：`192.168.50.2`
   - 子网掩码：`255.255.255.0`
   - 默认网关：空
   - DNS：空

可用 `ping 192.168.50.1` 测一下能否 ping 通树莓派图传网卡。

### 4. 启动 localhost 观看页

在仓库里进入地面站脚本目录并运行：

```cmd
cd <仓库根目录>\src\dronelink\windows
start_localhost_viewer.bat
```

保持弹出的黑色窗口不要关，用浏览器打开：

```text
http://127.0.0.1:8765/
```

能看到画面即表示链路跑通。若状态提示重连，多半是空口短暂丢包，页面会自动恢复；若一直无画面，先确认 Pi 端 `dronelink-video` 服务在跑，以及本机是否真是 `192.168.50.2` 在收 `5600` 端口。

### 5. （可选）维护网线

SSH / 拷文件可用网线：Windows `10.10.10.2/24`，树莓派 `10.10.10.1/24`。这与图传 Wi‑Fi 是两条网，不互相替代；只看图传时可以不插网线。
