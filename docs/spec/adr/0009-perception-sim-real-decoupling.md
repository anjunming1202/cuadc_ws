# 0009. 感知 sim/real 解耦：gstreamer 惰性化 + 相机内参分档

**Status**: Accepted

## Context

ADR-0008 重构后的 `cv_perception_node` 把 sim/real 混在一个节点里，暴露两个真机隐患：

1. `main()` 无条件 `Gst.init()` 跑在创建 rclpy Node **之前** → 段错误（rcl/GLib 初始化冲突），连 sim 都起不来——而 sim 走 ROS 图源、根本不用 gstreamer。
2. 相机内参 `fx/fy/cx/cy` + 挂载角是硬编码的 **sim 默认值**（640×480）写在 sim/real 共享的 launch 里，真机启动脚本又没覆盖 → 真机会拿 sim 内参去反投影 960×540 帧，全错。

## Decision

1. **gstreamer 惰性化**：`import gi` + `Gst.init()` 只在真机真正构造 shm 取帧源 / RTP 预览 sink 时才发生（此时 Node 已存在，不冲突）；sim 完全不加载 gstreamer。
2. **相机内参 + 挂载角迁到 `camera.<profile>.yaml`**，`geometry.load_camera_params(profile)` 按 profile 加载（与 `detectors/*.<profile>.yaml` 同套路），共享 launch 不再有内参默认值。sim = 640×480 Gazebo 值，real = 960×540 标定值。

## Consequences

- sim/real 的数据/配置差异全部是"按 profile 选文件"，无代码分支。
- 取帧仍保留 `frame_source`（ros/shm）一处薄适配分支——源本就不同（Gazebo 发 ROS 话题 / 真相机写 shm），是合理接缝。
- **未决（留待 perception owner）**：`run_mission_real.sh` 尚未设 `frame_source:=shm`，真机取帧链未接管；是否把节点拆成 sim/real 两个入口。
