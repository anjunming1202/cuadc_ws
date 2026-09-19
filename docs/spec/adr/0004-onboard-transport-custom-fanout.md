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

传输底座改用**自写的主动式 MAVLink 扇出** `mav_fanout.py`，串口侧用 **mavutil**
（与 dronelink-telemetry 同一套经验证的做法）：

- `mavutil.mavlink_connection` 开串口（pyserial 默认拉高 DTR/RTS，解决控制线
  问题）、`wait_heartbeat`、再请求各数据流并周期性重请求（喂活数据流）。
  实测这块飞控只有走这套请求序列才稳定发（单纯 request_data_stream ALL 唤不动）。
- 读到消息后把**原始字节** `get_msgbuf()` 主动推到多个本地 UDP 端点，反向把
  消费者字节写回串口；不重编码，全消息类型透传（转发完整帧，消费者解析干净）。
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
  收到 ~674 条/5s 全套遥测（ATTITUDE/GPS/SYS_STATUS…，recv_errs=0），共存成立。
- **串口交接会把这块飞控弄卡**：一个进程释放串口、另一个重开，飞控 telem 口会
  卡住（收不到心跳），需给飞控断电重启才恢复；proven 的 dronelink-telemetry
  代码一样中招，非本实现的 bug。**推论**：扇出必须做**唯一、常驻**的串口持有者
  ——开机对着刚上电的飞控开一次、之后一直不放；所有消费者只走 UDP 端点，永不
  碰串口，因此正常运行中**没有交接、不会卡**。迁移也因此不能"热交接"，须
  "改配置 + 整机断电重启"让扇出对新鲜飞控开串口（见 onboard-runtime.md 部署）。
