# DroneLink

DroneLink is the current Raspberry Pi 5 based low-latency video-link setup used with an Arducam/Sony IMX519 camera and ASUS USB-AC56 (RTL8812AU) 5 GHz Wi-Fi adapters.

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

Install VLC and place `windows/DroneLink.sdp` on the Desktop. The tested low-latency VLC command is contained in `windows/start_dronelink_vlc.bat`.

The command used is:

```cmd
"C:\Program Files\VideoLAN\VLC\vlc.exe" --network-caching=20 --clock-jitter=0 --clock-synchro=0 --drop-late-frames --skip-frames "%USERPROFILE%\Desktop\DroneLink.sdp"
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
