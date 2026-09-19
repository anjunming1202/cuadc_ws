# cuadc_onboard

机载伴随计算机（树莓派）的**运行时模块**：把飞控串口和相机做成多消费者
可共享的本地端点，并把整条链路做成仓库管理的开机自启。设计与端点契约见
[`docs/spec/onboard-runtime.md`](../../docs/spec/onboard-runtime.md)，此处只讲布局与操作。

**非 colcon 包**（systemd + 脚本），与 `src/dronelink` 一样游离在任务链路外，
但设计成可合回 master、prod/test 共用。

## 布局

```
scripts/   通信程序（能手动跑，不依赖 systemd）
             mav_fanout.py    串口扇出（底座）
             telem_ground.py  数传发送：读扇出 → JSON 发地面 HUD（消费者）
systemd/   自启薄壳
             cuadc-onboard-fanout.service   底座：串口扇出
             cuadc-onboard-camera.service   底座：相机 tee（本地 5602 + 地面 5600）
             cuadc-onboard-telem.service    消费者：数传（ground-monitor profile 拉起）
             cuadc-onboard-profile.service  oneshot applier：按 profile 起消费者
             apply-profile.sh               applier 逻辑（读 /etc/cuadc-onboard/profile）
deploy.sh  幂等安装到 Pi 系统路径并 enable 底座 + profile applier
```

## 端点契约（消费者连这些，不碰硬件）

| 资源 | 端点 | 协议 |
|---|---|---|
| 飞控 MAVLink | `udpin:127.0.0.1:14550` / `14551` | MAVLink（扇出主动推流，绑定即收；等同 mavproxy `--out`） |
| 相机本地帧 | `udp:127.0.0.1:5602` | H.264/RTP |
| 图传→地面 | `192.168.50.2:5600` | H.264/RTP |
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

- **底座**无条件开机自启：mav_fanout（串口扇出）+ camera（tee 本地 5602 +
  地面 5600）。
- **消费者**由 profile 选择性自启：`/etc/cuadc-onboard/profile` 写
  `none` / `ground-monitor` / `mission`，oneshot applier 按名拉起。日常开发用
  `none`，消费者手动跑；`ground-monitor` 起数传。改 profile 后
  `systemctl restart cuadc-onboard-profile` 或重启整机生效。

## 部署 / 迁移（在 Pi 上）

```bash
sudo ./deploy.sh ground-monitor   # 装脚本/单元、写 profile（缺省才写）、enable 底座 + applier
```

`deploy.sh` 只装+enable 本模块，**不碰任何 dronelink 服务**。迁移旧的仓库外
自启是单独一步——**只 disable 要替换的两个**，不删文件、可回退：

```bash
sudo systemctl disable --now dronelink-telemetry.service dronelink-video.service
# 绝不动 dronelink-network / eth0-100m —— SSH 骑在 AP/以太网上
```

**扇出必须对『刚上电的飞控』开串口，不能热交接（见 ADR-0004），迁移启用后
须整机断电重启一次**，让扇出接一个没被交接搞过的飞控。

> 状态：底座（扇出 + 相机 tee）与数传消费者、profile、deploy.sh 已就位；
> 扇出多消费者共存已真机验证（2026-09-19）。ground-bus（5700→地面 topic
> 转发）暂缓，里程碑链路不经它。为何用自写扇出而非 mavlink-router 见 ADR-0004。
