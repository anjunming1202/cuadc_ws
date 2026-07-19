# cuadc_decision

规划/决策模块。任务状态机的设计依据（状态列表、阶段划分、跨状态强制
打断机制）唯一权威版本在
[`docs/spec/06-planning.md`](../../docs/spec/06-planning.md)；与感知、
接口层的边界分别见
[`05-perception-decision-interface.md`](../../docs/spec/05-perception-decision-interface.md)、
[`04-interfaces.md`](../../docs/spec/04-interfaces.md)。本文件不重复
描述这些内容。

代码分两层：
- `mission_logic.py`——纯 Python，状态机本身、每个阶段的判断逻辑，不
  依赖 rclpy，可以脱离 ROS2 节点单独单元测试。
- `decision_node.py`——唯一碰 ROS2 API 的地方，订阅遥测/检测、调用
  接口层 service、发布位置指令，每个 tick 把数据交给 `mission_logic`
  处理，再把结果翻译回真正的 ROS2 调用。

```
ros2 launch cuadc_decision decision_bringup.launch.py
```

需要配合 `cuadc_interface`（真实/仿真飞控接口）和 `cuadc_perception`
（当前是桩感知）一起起，才能跑出完整闭环。
