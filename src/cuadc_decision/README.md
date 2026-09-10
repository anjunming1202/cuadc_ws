# cuadc_decision

规划/决策模块。任务状态机的设计依据（状态列表、阶段划分、跨状态强制
打断机制）唯一权威版本在
[`docs/spec/06-planning.md`](../../docs/spec/06-planning.md)；与感知、
接口层的边界分别见
[`05-perception-decision-interface.md`](../../docs/spec/05-perception-decision-interface.md)、
[`04-interfaces.md`](../../docs/spec/04-interfaces.md)。本文件不重复
描述这些内容。

代码分三层：
- `mission_logic.py`——纯 Python，状态机本身、每个阶段的判断逻辑，不
  依赖 rclpy，可以脱离 ROS2 节点单独单元测试。
- `drop_strategy.py`——投放目标选哪个筒的策略，纯 Python，独立于状态机。
  每个策略是一个函数（签名见文件头），注册进 `STRATEGIES` 字典，改
  `ACTIVE_STRATEGY` 指向哪个就切换/实验哪个策略，不用改 `mission_logic.py`。
- `decision_node.py`——唯一碰 ROS2 API 的地方，订阅遥测/检测、调用
  接口层 service、发布位置指令，每个 tick 把数据交给 `mission_logic`
  处理，再把结果翻译回真正的 ROS2 调用。

```
ros2 launch cuadc_decision decision_bringup.launch.py
```

需要配合 `cuadc_interface`（真实/仿真飞控接口）和 `cuadc_perception`
（当前是桩感知）一起起，才能跑出完整闭环。

## Launch 参数

- `tick_rate_hz`（默认 10.0）：状态机 tick 频率。
- `initial_state`（默认 `INIT`）：硬件单点测试用，跳过前面几个状态直接
  从某个状态起步，比如让飞机已经悬停在投放区附近时传
  `initial_state:=NAVIGATE_TO_DROP_ZONE`，不用重新走一遍
  `WAIT_TASK_START`/`ARM_AND_TAKEOFF`。已知限制：`_DROP_PHASE_TIMEOUT_S`
  安全超时只在 `NAVIGATE_TO_DROP_ZONE` 到点后才开始计时，直接跳进
  `SEARCH_DROP_TARGETS`/`APPROACH_DROP_TARGET`/`DROP_PAYLOAD` 不会激活
  这个超时保护，建议单点测试从 `NAVIGATE_TO_DROP_ZONE` 起步。
- `stop_after_drop`（默认 `false`）：硬件单点测试用，两次投放都完成后
  直接切 RTL，不飞去侦察区。

```bash
ros2 launch cuadc_decision decision_bringup.launch.py \
    initial_state:=NAVIGATE_TO_DROP_ZONE stop_after_drop:=true
```

## 日志

统一 `self.get_logger()`，加标签前缀、限频（`throttle_duration_sec`）
避免刷屏：

- `[STATE]`：状态切换，每次都打。
- `[PERF]`：耗时监控（`tick()` 单次耗时、感知桩节点单帧生成耗时），约
  1 秒一条，正常情况下不会触发报警级别的输出。
- `[DETECT]`：搜索/进近阶段限频打一行当前看到的候选数量和大概尺寸。

另开一个终端 `ros2 topic echo /cuadc_decision/mission_state` 能拿到干净的、
只有当前状态的实时视图，不用在主日志里找。
