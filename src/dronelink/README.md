# DroneLink

DroneLink is the current Raspberry Pi 5 based low-latency video-link setup used with an Arducam/Sony IMX519 camera and ASUS USB-AC56 (RTL8812AU) 5 GHz Wi-Fi adapters.

Ground display is a localhost browser viewer on the Windows PC (not a
standalone media player). The same page shows a telemetry HUD fed by UDP
JSON from the Pi (Pixhawk via pymavlink; no ROS on this path yet).

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

raspberrypi/scripts/dronelink-telemetry.py
    -> /usr/local/sbin/dronelink-telemetry.py

raspberrypi/systemd/dronelink-telemetry.service
    -> /etc/systemd/system/dronelink-telemetry.service

raspberrypi/systemd/eth0-100m.service
    -> /etc/systemd/system/eth0-100m.service
```

After copying, make the network script executable and enable the services:

```bash
sudo chmod +x /usr/local/sbin/dronelink-network.sh
sudo systemctl daemon-reload
sudo systemctl enable dronelink-network.service
sudo systemctl enable dronelink-video.service
sudo systemctl enable dronelink-telemetry.service
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

### 每次开机（日常操作）

树莓派上若已 `enable`：`dronelink-network`、`dronelink-video`、
`dronelink-telemetry`，则 **Pi 不用再敲命令**。地面站每次：

1. USB 网卡已插入；树莓派 / 飞控上电。
2. **只跑一次**（不要再另开 `start_localhost_viewer.bat`，会抢 UDP 端口）：

```cmd
cd /d e:\cuadc_ws\src\dronelink\windows
.\start_dronelink_ground.bat
```

   内部顺序：清残留 FFmpeg → `connect_dronelink.bat` 连 `DroneLink` →
   启动 localhost viewer。黑窗口保持开着。
3. 浏览器打开：**http://127.0.0.1:8765/**

不要同时运行 `send_fake_telemetry.py`（会和真机遥测抢显示）。

**WLAN 2 连得慢（第一次本机建议做一次）：** USB 网卡上常挂着酒店/手机等
「自动连接」配置，Windows 会先扫它们。管理员 PowerShell：

```powershell
cd e:\cuadc_ws\src\dronelink\windows
.\tune_wlan2_dronelink.ps1
```

把 `DroneLink` 提到 WLAN 2 最高优先级，其它配置改成手动，并尽量关适配器省电。
之后日常仍用 `start_dronelink_ground.bat` 主动连（比干等系统扫描快）。
`connect_dronelink.bat` **不会**先 disconnect（避免把正在关联的链路掐掉）。

**把仓库里的遥测脚本同步到 Pi（在 Windows 跑，不要在 Pi 的 bash 里跑）：**

```powershell
cd e:\cuadc_ws\src\dronelink\windows
.\deploy_pi_telemetry.ps1
```

会提示输入 `wly@192.168.50.1` 的 SSH 密码；只更新遥测 `.py` / systemd，
**不**重启 Wi‑Fi AP。

**HUD 状态含义：**

| HUD | 含义 |
|---|---|
| 无数据 | 地面没收到遥测 UDP（Pi 服务没发 / 未部署新脚本 / 端口不对） |
| 等待飞控 | UDP 已到，Pi 还在等 `/dev/ttyAMA0` 心跳 |
| 飞控断开 | UDP 在，串口曾连上后又出错，脚本在重试 |
| 在线 | UDP + 飞控字段正常 |
| 过期 | 曾收到过，超过约 2 s 没新包 |

有画面、无遥测：先看 HUD。
- **无数据** → Pi 没往地面发 UDP（服务未起 / 旧脚本卡在等心跳且从不发包）。
- **等待飞控** → UDP 已通，但 `/dev/ttyAMA0:57600` 收不到 MAVLink heartbeat
  （飞控未上电、接线/TELEM 口、波特率）。视频与遥测是两条独立路径。
  Pi：`journalctl -u dronelink-telemetry.service -n 30`。

若视频黑屏：检查是否开了两个 viewer（`-10048`）、`dronelink-video` 是否
active，以及 `ping 192.168.50.1`。

**扫不到 SSID `DroneLink`（维护网线仍通时）：** 在 Windows 用网线恢复 AP：

```powershell
cd e:\cuadc_ws\src\dronelink\windows
.\fix_pi_ap_over_ethernet.ps1
```

Pi 侧可选：`hostapd` 使用更短 beacon（`beacon_int=50`，见
`raspberrypi/hostapd/hostapd.conf`）；拷到 `/etc/dronelink/hostapd.conf`
后 `sudo systemctl restart dronelink-network`。

**开机行为：** Pi 上三个服务均为 `enable` 后，上电自动起 AP / 图传 / 遥测。
地面 Windows **仍需**每次运行 `start_dronelink_ground.bat`（未做成系统自启）。

### Localhost viewer

A Python server on the ground PC owns air-side UDP ports `5600` (RTP) and
`5601` (legacy telemetry JSON and/or RTCP), demuxes JSON into the HUD, and
forwards RTP/RTCP to a loopback SDP for one shared FFmpeg remux (MPEG-TS →
browser via `mpegts.js`). Preferred telemetry also arrives on `:5701`.

Requirements: Python 3 on PATH, and FFmpeg on PATH (same as calibration:
`winget install --id Gyan.FFmpeg -e`).

Daily launcher (preferred):

```cmd
windows\start_dronelink_ground.bat
```

Viewer only (Wi-Fi already up):

```cmd
windows\start_localhost_viewer.bat
```

Then open Chrome or Edge at `http://127.0.0.1:8765/`. Keep the console window
open while viewing. **Only one viewer process** at a time (duplicate binds →
Windows UDP error `-10048`).

| Stream | Destination | Notes |
|---|---|---|
| Video RTP/H.264 | `192.168.50.2:5600` | Pi → ground; viewer proxies to loopback FFmpeg |
| Shared UDP | `192.168.50.2:5601` | legacy telemetry JSON and/or RTCP; demuxed in viewer |
| Telemetry JSON/UDP | `192.168.50.2:5701` (+ optional `10.10.10.2`) | preferred; Pi dual-sends `5701,5601` |

Brief Wi-Fi UDP loss (association still up) can desync the remux/MSE player.
The viewer auto-restarts FFmpeg and reconnects the page player. Telemetry older
than ~2 s is marked stale on the HUD but the last values remain visible.

Manual equivalent:

```cmd
python windows\localhost_viewer\server.py --sdp windows\DroneLink.sdp
```

Without a Pi, you can exercise the HUD only:

```cmd
python windows\send_fake_telemetry.py
```

Layout:

```text
windows/DroneLink.sdp                 air-link RTP reference (UDP :5600)
windows/start_dronelink_ground.bat    daily: connect Wi-Fi + one viewer
windows/start_localhost_viewer.bat    viewer only
windows/connect_dronelink.bat         force-associate WLAN 2 → DroneLink
windows/tune_wlan2_dronelink.ps1      one-time: priority + demote other SSIDs
windows/deploy_pi_telemetry.ps1       SCP telemetry script/unit to Pi (Windows)
windows/fix_pi_ap_over_ethernet.ps1   restore hostapd via 10.10.10.1 Ethernet
windows/send_fake_telemetry.py        local fake telemetry for UI test
windows/localhost_viewer/server.py    HTTP + AirLink proxy + FFmpeg remux
windows/localhost_viewer/index.html   page + HUD
windows/localhost_viewer/mpegts.js    vendored player
raspberrypi/scripts/fix_telemetry_on_pi.sh   optional paste-on-Pi helper
```

### Telemetry JSON schema (UDP :5701 / also :5601)

Each datagram is one UTF-8 JSON object. Unknown fields are ignored; missing
fields show as "—" on the HUD. The Pi sender emits packets **while waiting
for the FC** (`link` may be `connecting` / `error` / `fc`) so the ground can
tell “no UDP” from “UDP ok, FC still coming up”.

```json
{
  "t": 1710000000.0,
  "link": "fc",
  "alt_m": 12.3,
  "roll_deg": 1.0,
  "pitch_deg": -2.5,
  "yaw_deg": 90.0,
  "mode": "GUIDED",
  "batt_v": 22.1,
  "batt_pct": 85.0
}
```

| Field | Meaning |
|---|---|
| `t` | optional Unix timestamp (seconds) |
| `link` | `connecting` / `fc` / `error` (Pi MAVLink side) |
| `alt_m` | relative altitude in meters (prefer takeoff-relative, not MSL) |
| `roll_deg` / `pitch_deg` / `yaw_deg` | attitude in degrees |
| `mode` | flight mode string |
| `batt_v` / `batt_pct` | battery volts / percent |

Send rate: about **5–20 Hz** is enough. Keep this path **one-way** (Pi → ground);
do not send control commands back on this link.

### Pi telemetry sender (no ROS yet)

Script: `raspberrypi/scripts/dronelink-telemetry.py`  
Unit: `raspberrypi/systemd/dronelink-telemetry.service`

Reads Pixhawk on `/dev/ttyAMA0:57600` via pymavlink. A background thread
sends JSON immediately to `192.168.50.2` and `10.10.10.2` on ports
**`5701` and `5601`** (Ethernet destination fails quietly if unplugged).
systemd starts with no long `ExecStartPre` sleep; the script retries the FC
serial link on its own.

From Windows (preferred refresh path):

```powershell
cd e:\cuadc_ws\src\dronelink\windows
.\deploy_pi_telemetry.ps1
```

Or install on the Pi from a cloned tree:

```bash
sudo apt install -y python3-venv python3-full
python3 -m venv ~/dronelink-venv
~/dronelink-venv/bin/pip install -U pip
~/dronelink-venv/bin/pip install pymavlink

sudo cp raspberrypi/scripts/dronelink-telemetry.py /usr/local/sbin/dronelink-telemetry.py
sudo chmod +x /usr/local/sbin/dronelink-telemetry.py
sudo cp raspberrypi/systemd/dronelink-telemetry.service /etc/systemd/system/
# Edit User= and the /home/<user>/dronelink-venv path if your login is not "wly"
sudo systemctl daemon-reload
sudo systemctl enable --now dronelink-telemetry.service
```

Manual test (stop the service first so the serial port is free):

```bash
sudo systemctl stop dronelink-telemetry.service
~/dronelink-venv/bin/python3 /usr/local/sbin/dronelink-telemetry.py
```

**Serial exclusivity:** only one of `dronelink-telemetry` or ROS
`cuadc_interface` (`run_mission_hardware.sh` → `/dev/ttyAMA0:57600`) may own
the UART. When migrating to ROS later, stop this service and forward
`/cuadc_interface/telemetry/*` with the same JSON schema.

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

### 4. 启动观看页（每次开机）

日常最短步骤见上文「每次开机（日常操作）」。**在 Windows** 执行：

```cmd
cd /d e:\cuadc_ws\src\dronelink\windows
.\start_dronelink_ground.bat
```

保持黑色窗口不要关，浏览器打开 `http://127.0.0.1:8765/`。  
不要同时再开第二个 viewer。`deploy_pi_telemetry.ps1` 也只在 Windows 跑。

树莓派若已 enable 图传 + 遥测服务，一般不用再 SSH。

### 5. （可选）维护网线

SSH / 拷文件可用网线：Windows `10.10.10.2/24`，树莓派 `10.10.10.1/24`。这与图传 Wi‑Fi 是两条网，不互相替代；只看图传时可以不插网线。
