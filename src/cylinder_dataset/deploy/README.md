# 共存部署：地面站遥测/图传 + 数据集采集 同时进行

目标：数据集采集（拍照 + 记录飞控位姿）进行时，**地面站仍能看到相机图像和
遥测**。两个物理资源本来"一次只能一个进程用"，这里各用一个"单一持有者 +
分发"的办法解决：

- **飞控串口 `/dev/ttyAMA0`**：让 **mavlink-router** 独占，向多个 UDP 端点
  分发原始 MAVLink。dronelink-telemetry 和采集脚本各连一个端点，都不再直接
  开串口。
- **相机**：让 **dronelink-video** 独占（照旧），但在其 GStreamer 管线里加
  一个 `tee`，把同一路 H264/RTP 复制一份到本地端口；采集脚本用 `backend=rtp`
  从本地流取帧，不再开相机。

> 权衡：`rtp` 取到的是图传编码后的一帧（有损 1080p），且相对真实曝光有
> 编码+网络+解码延迟（~百 ms）——同刻性比 rpicam 现拍略松，但换来与地面站
> 共存。要最紧同刻/最高画质就别用共存模式，改用独占模式（停 dronelink 服务、
> `backend=rpicam` 或 `picamera2`），见上级 README。

## 一、装 mavlink-router

按官方文档装（源码编译或包管理），确认有 `/usr/bin/mavlink-routerd`。
MAVProxy 也能顶替（见文末），但常驻服务用 mavlink-router 更稳。

## 二、放置文件（都在 Pi 上，需 sudo）

```bash
sudo mkdir -p /etc/mavlink-router
sudo cp mavlink-router/main.conf                              /etc/mavlink-router/main.conf
sudo cp systemd/mavlink-router.service                        /etc/systemd/system/
sudo mkdir -p /etc/systemd/system/dronelink-telemetry.service.d
sudo cp systemd/dronelink-telemetry.service.d/override.conf   /etc/systemd/system/dronelink-telemetry.service.d/override.conf
sudo mkdir -p /etc/systemd/system/dronelink-video.service.d
sudo cp systemd/dronelink-video.service.d/override.conf       /etc/systemd/system/dronelink-video.service.d/override.conf
```

（路径里的 `wly`/IP/端口按现场核对。dronelink-telemetry 的 ExecStart 若与
`/usr/local/sbin/dronelink-telemetry.py` 现状不符，以现状为准改 override。）

## 三、启用并重启

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now mavlink-router.service
sudo systemctl restart dronelink-telemetry.service   # 改从 udp:127.0.0.1:14551 读
sudo systemctl restart dronelink-video.service        # 加了 tee 到本地 5602
```

自检：

```bash
systemctl is-active mavlink-router dronelink-telemetry dronelink-video
# 串口现在应只被 mavlink-routerd 占用：
sudo fuser /dev/ttyAMA0        # 期望只有 mavlink-routerd 一个 PID
```

## 四、跑采集（不独占串口/相机）

```bash
cd ~/cuadc_ws/src/cylinder_dataset
cp deploy/capture.coexist.yaml capture/capture.yaml   # 共存模式运行配置
cp capture/session.example.yaml capture/session.yaml  # 填外参/圆筒实测

# 手动逐张（注意：共存模式不要加 --passive，脚本要发一次流请求让路由器学到本端）
python3 capture/capture_external.py --session capture/session.yaml
# 定时连拍：加 --interval 2
```

此时地面站的图传（192.168.50.2:5600）和 JSON 遥测叠加照常，采集同时出图+数据。

## 端点对照

| 消费者 | 连接 | 方向 |
|---|---|---|
| mavlink-router | 独占 `/dev/ttyAMA0` | — |
| 采集脚本 | `udp:127.0.0.1:14550` | 双向（自主飞行发指令也走这条） |
| dronelink-telemetry | `udp:127.0.0.1:14551` | 读 |
| 地面站 GCS（可选） | `192.168.50.2:14560` | 双向 |
| 采集取帧 | 本地 RTP `127.0.0.1:5602`（图传 tee） | 读 |
| 地面站图传 | `192.168.50.2:5600`（原路不变） | — |

## MAVProxy 顶替路由器（临时验证用）

```bash
mavproxy.py --master=/dev/ttyAMA0,57600 \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --out=udp:192.168.50.2:14560
```
（udpin/udpout 方向按各消费者微调；长期部署仍建议 mavlink-router + systemd。）

## 待在 Pi 上核实（本地无法测）

1. `mavlink-routerd` 装好、`fuser /dev/ttyAMA0` 只剩它。
2. dronelink-telemetry 从 udp 读后，地面站 JSON 叠加照常。
3. 图传 tee 后地面站 5600 画面正常、且本地 5602 有流。
4. `python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i gstreamer`
   确认 OpenCV 带 GStreamer；否则 `backend=rtp` 打不开流，需改用 FFmpeg+SDP
   方案或给 `rtp_pipeline` 传自定义管线。
5. 采集脚本连 `udp:127.0.0.1:14550` 能收到 ATTITUDE/GPS，`backend=rtp` 能出图。
