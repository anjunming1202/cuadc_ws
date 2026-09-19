# 0004. 机载传输底座用自写 MAVLink 扇出，不用 mavlink-router

**Status**: Accepted

## Context

机载运行时（见 [`onboard-runtime.md`](../onboard-runtime.md)）需要一个"单一
持有者 + 分发"的传输底座：一个进程独占飞控串口 `/dev/ttyAMA0`，把 MAVLink
扇出给多个消费者（接口层 `cuadc_interface`、数传发送、数据集采集…），让它们
不再互抢串口。spec 初稿定的是业界常用的 **mavlink-router**（C++ 常驻路由）。

在真机（RaspberryPi5 + 这块 ArduPilot 飞控，串口 57600）上实测发现两件事，
mavlink-router 不打补丁用不了：

1. **串口控制线**：这块飞控只有 DTR 或 RTS 拉高时才发数据，两条都低就哑
   （四组合实测：都低=0 字节，其余=正常）。pyserial 打开串口**默认把两条都
   拉高**，所以 pymavlink 系（dronelink-telemetry）一直读得到；mavlink-router
   不拉高这两条，UART 收计数恒为 0。它的配置里也没有 DTR/RTS 选项。
2. **数据流要持续请求**：这块飞控的数据流要有 GCS 持续请求才会一直发；被动
   路由不请求，流会停。需要底座本身做 keepalive。

## Decision

传输底座改用**自写的主动式 MAVLink 扇出** `mav_fanout.py`（pyserial + 原始
字节透传）：

- 用 pyserial 开串口（默认拉高 DTR/RTS，解决控制线问题）。
- 周期性向飞控发心跳 + 请求数据流（keepalive，喂活数据流）。
- 把串口原始 MAVLink 字节主动推到多个本地 UDP 端点，反向把消费者的字节写回
  串口；不解析、不重编码，全消息类型透传。
- 消费者用 `udpin:127.0.0.1:<port>` 绑定收，语义等同 mavproxy `--out`；即使
  "先读后写"的消费者（如 `cuadc_interface`）也能立刻收到稳定流。

## Consequences

- 我们多维护一个 ~90 行脚本，但它简单、完全可控、复用 Pi 上已有 pymavlink，
  且**零外网安装**（Pi 常无外网，mavlink-router 源码编译/依赖都要外网）。
- DTR/RTS 与 keepalive 问题**在底座里解决一次**，所有消费者走 UDP、不碰串口、
  不受影响。
- 单消费者场景多一跳本地 UDP（延迟 <1ms 可忽略）；真要单消费者也可让消费者
  直接连串口、不经底座（连接串参数切换即可，未锁死）。
- 实测（2026-09-19，真机）：两个 read-first `udpin` 消费者经扇出同时读，各
  收到 810 条/6s 全套遥测（ATTITUDE/GPS/SYS_STATUS…），共存成立。
