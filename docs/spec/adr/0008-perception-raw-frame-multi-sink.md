# 0008. 感知取原始帧、多 sink 路由（图像不进 ROS；CV 代码收进 cuadc_perception）

**Status**: Accepted

## Context

`cuadc_perception` 的 CV 检测算法（`detectors/`，纯函数）已经和 ROS 解耦，
但**取帧和出口**没有：`cv_perception_node` 从 ROS 话题 `/camera/image_raw`
进图（`cv_bridge`）。这在真机上有两个问题：

1. **没人发 `/camera/image_raw`**——只有仿真里 Gazebo 发。真机要让这个节点
   工作，得新加一座"相机→ROS 图像桥"（读 shm/5602 → 发 `sensor_msgs/Image`）。
2. **就算加了桥，把 1080p 塞进 ROS 正是最贵的一步**，且与 [ADR-0004](0004-onboard-transport-custom-fanout.md)
   "图传走低延迟自定义 RTP 直路、不进 ROS"的取向相悖。

与此同时相机链路（cv-field-test）**已把分流提前到编码前**：`camera_service`
先 tee 出**原始 NV12 到 shm**再编码。`cv_preview.py`（现蹲在 `cuadc_onboard/scripts/`）
已经在读这条 shm、跑**同一个** `color_pad` 检测器——但它**只画框推 RTP、丢弃
检测结果**，决策层拿不到；而且它反向 `import cuadc_perception.detectors`，是**倒
挂的依赖**（onboard 代码寄居着 CV 逻辑）。结果：检测被割裂成两处、真机检测结果
进不了 ROS/bag/决策、CV 代码散落。

## Decision

**重的（像素）永不进 ROS，轻的（检测结果）才进；检测跑一次、结果扇出到多个
sink。所有 CV 代码收进 `cuadc_perception` 一个包。**

- **取帧源抽象**：真机 = **shm 原始帧**（搬 `cv_preview` 那段 `shmsrc→BGR` gst），
  仿真 = `/camera/image_raw` 订阅（Gazebo，现状不变）。由 `profile` 选。图像在
  真机上**从不进入 ROS transport**。
- **多 sink 路由**：一次检测的结果扇出到——(a) `DetectionArray`（ROS，给决策，
  **契约不变**，见 [05-perception-decision-interface.md](../05-perception-decision-interface.md)）；
  (b) 标注预览 → **RTP**（5604→5606，给人看），**不发 ROS Image**。
- **rclpy-free 核心**：`perception_core` 只做 `frame(+pose)→检测→反投影→结果`，
  不依赖 rclpy；`rclpy` 只出现在 ros 取帧源 / ros 结果 sink / 薄壳节点。`detectors/`
  的"并列算法 + per-算法 `<algo>.<profile>.yaml`"结构**原样保留**。
- **代码归位**：`cuadc_onboard` 只留**传输**（`camera_service` 原始 tee + 各 forward）
  和 ground-monitor profile 里**按名引用** perception 预览入口的 systemd 单元
  （同 `mission` profile 引用任务 bringup 的套路）。`cuadc_onboard/scripts/cv_preview.py`
  **删除**，其能力以 perception 里一个**无-ROS 预览模式**存在（shm + 检测器 +
  rtp sink，不订位姿/不反投影/不发 ROS；供相机自检、调参）。

两个已选定的选项（可后续调）：

1. **真机 ROS 结果 + RTP 预览合一个进程**（检测一次、双 sink），不再各检测一遍。
2. `frame_sources` / `sinks` 先做**扁平模块**，等出现第 3 个实现再提成目录
   （对齐 `detectors/` 的目录风格）。

## Consequences

- **真机**：CV 检测结果进 ROS/决策/rosbag——**不需要图像→ROS 桥**（前述缺口
  直接消失）；图像不进 ROS；检测只跑一次（消除 cv_preview / cv_perception 的
  重复）。
- **仿真**：不变——Gazebo→`/camera/image_raw` 取帧源、ROS debug Image 给
  cuadc_viz 仪表盘；契约、决策层、桩感知（`stub_perception`）零改动。
- **依赖方向修正**：onboard → perception（按引用），不再是 CV 代码寄居 onboard。
- **可测性**：核心 rclpy-free，可脱离机器人单测（延续"算法不依赖 rclpy"到整条
  流水线）。
- **代价 / 前置条件**：真机需喂**真实相机内参**（cylinder_dataset 棋盘格标定，
  非 sim 的 205.5）+ `cuadc_interface` 连 `udp:127.0.0.1:14550`（不直连串口，
  避免和底座 `mav_fanout` 抢 `/dev/ttyAMA0`）；ground-monitor 的预览单元依赖
  perception 在 Pi 上可 import（`PYTHONPATH`，沿用今天 cv_preview 的做法，免
  colcon build）。

## Alternatives considered

- **加"相机→ROS 图像桥"、让 `cv_perception_node` 原样吃 `/camera/image_raw`**：
  否决——把 1080p 推进 ROS 是最贵一步、与 ADR-0004 相悖，还要额外再读一遍 shm；
  本决策的核心就是**不翻译图像进 ROS**。
- **cv_preview 与 cv_perception 保持两个独立进程**：否决为默认——检测跑两遍；
  但**预览模式**（无-ROS）作为同一核心的一种接线**保留**，服务相机自检/调参。
- **CV 代码继续留在 `cuadc_onboard`**：否决——倒挂依赖、CV 散落多包。
