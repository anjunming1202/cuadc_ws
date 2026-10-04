# 录制离线分析（analyze_recording）

`scripts/analyze_recording.py`：把机载录制（`onboard_record` 产物的 `<时间戳>_<label>_bag/`）
画成一张多面板总览图，并在终端打印文本时间线，供飞后复盘。纯离线、只读，不参与任务闭环。

## 输入与读取

- 参数是 bag 目录（含 `*.mcap`）、单个 `.mcap`，或含多个 `*_bag/` 的目录（批量逐个处理）。
- 用纯 Python `mcap` + `mcap-ros2-support` 读，**不依赖 ROS 环境**：bag 在 Pi 上由 jazzy 录，
  地面 humble 没有 mcap 存储插件，`ros2 bag`/`rosbag2_py` 两边都用不上。消息 schema 自带在
  mcap 里，自定义消息无需编译消息包。schema 为空的消息（`std_msgs/Empty`）按无字段处理。
- 不解码图像话题（体积大、总览图用不上）。

## 时间轴与坐标

- 横轴 = 录制开始（mcap 首条消息 log_time）起的秒数；用 log_time（录制端收到时刻），不用消息 header.stamp。
- 位置/检测按话题原样画，不做坐标换算（与 `local_position` 同一局部系，z 向下）；高度 = `-z`。
  俯视图横轴 y、纵轴 x（x 前 / y 右的起飞系下即解锁时机头朝上）。

## 输出

话题驱动：哪个话题在 bag 里有数据就出对应面板，缺的跳过——同一工具适用飞行测试 suite
（只有遥测）与任务/投放台架（含感知、决策）。

| 面板 | 数据来源 |
|---|---|
| 俯视轨迹（按任务状态分色）+ 检测世界坐标（按置信度着色）+ 位置指令目标点 | local_position、detections、cmd/position_target_local |
| 高度 + 高度指令 | local_position、cmd/position_target_local |
| 水平速度 + 速度上限指令 + 速度指令模长 | telemetry/velocity、cmd/speed、cmd/velocity |
| 姿态 roll/pitch/yaw | telemetry/attitude |
| 检测置信度 + 1 s 窗口检出率 | perception/detections |
| 感知单帧耗时 | cuadc_perception_cv/perf（total_ms） |
| 电池电压 | telemetry/battery |
| 时间线：任务状态 / 飞控模式 / 解锁 + rosout WARN 及以上标记 | mission_state、status/mode、status/armed、rosout |

所有时间面板叠加任务状态色带（无 mission_state 时不画），投放执行（status/drop actuated）画竖线。

- 产物固定存到 `local/analyze/`（`local/` 不入库）：`<录制名>.png` 总览图 + `<录制名>.txt` 文本摘要，
  录制名 = bag 目录名去掉 `_bag`，与同次录制的 `.ts` 同名。`--show` 另弹交互窗口。
- 文本摘要（同时打印到终端）：时长、任务状态序列（进入时刻 + 持续时长）、模式/解锁变化、rosout WARN 及以上、
  按任务状态分组的检测统计（帧数、检出帧占比、置信度/半径中位数）、投放事件。

## 非目标

视频（`.ts`）与 bag 的同步抽帧、网页回放、跨 bag 对比——需要时另议。
