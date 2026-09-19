# cuadc_onboard

机载伴随计算机（树莓派）的**运行时模块**：把飞控串口和相机做成多消费者
可共享的本地端点，并把整条链路做成仓库管理的开机自启。设计与端点契约见
[`docs/spec/onboard-runtime.md`](../../docs/spec/onboard-runtime.md)，此处只讲布局与操作。

**非 colcon 包**（systemd + 脚本），与 `src/dronelink` 一样游离在任务链路外，
但设计成可合回 master、prod/test 共用。

## 布局

```
scripts/   通信程序（能手动跑，不依赖 systemd）：mav_fanout（串口扇出）/ ground-bus / 数传发送 / 图传转发
systemd/   底座单元 + profile（none/ground-monitor/mission）+ base target
deploy.sh  幂等安装到 Pi 系统路径并 enable 底座
```

## 端点契约（消费者连这些，不碰硬件）

| 资源 | 端点 | 协议 |
|---|---|---|
| 飞控 MAVLink | `udpin:127.0.0.1:14550` / `14551` | MAVLink（扇出主动推流，绑定即收；等同 mavproxy `--out`） |
| 相机本地帧 | `udp:127.0.0.1:5602` | H.264/RTP |
| 图传→地面 | `192.168.50.2:5600` | H.264/RTP |
| 地面数据总线 | `udp:127.0.0.1:5700` | JSON（含 `topic`） |
| GCS | `192.168.50.2:14560` | MAVLink |

## 自启两层

- **底座**无条件开机自启（mav_fanout → camera → ground-bus）。
- **消费者**由 profile 选择性自启：`/etc/cuadc-onboard/profile` 写
  `none` / `ground-monitor` / `mission`。日常开发用 `none`，消费者手动跑。

## 部署 / 迁移（在 Pi 上）

`deploy.sh` 装单元/配置、enable 底座。迁移旧的仓库外自启时**只 disable 要
替换的两个**，不删文件、可回退：

```bash
sudo systemctl disable --now dronelink-telemetry.service dronelink-video.service
# 绝不动 dronelink-network / eth0-100m —— SSH 骑在 AP/以太网上
```

> 状态：搭建中。扇出底座 + 多消费者共存已在真机验证（2026-09-19，两个
> `udpin` 消费者同时读到全套遥测）；接着补相机 tee、数传/图传消费者、profile
> 与 deploy.sh。为什么用自写扇出而非 mavlink-router 见 ADR-0004。
