# cuadc_perception_msgs

感知 → 决策 检测结果契约用到的自定义 ROS2 消息定义。契约本身（字段
含义、通信模式、职责边界）唯一权威版本在
[`docs/spec/05-perception-decision-interface.md`](../../docs/spec/05-perception-decision-interface.md)，
本包只是该契约的 `.msg` 落地，不在这里重复描述。

独立成包（而不是定义在 `cuadc_perception`/`cuadc_decision` 内部）是
ROS2 的常规做法——自定义消息需要 `ament_cmake` + `rosidl` 生成代码，
`cuadc_perception`/`cuadc_decision` 节点本体走 `ament_python`，两种
构建类型不适合揉进同一个包，处理方式同 `cuadc_interface_msgs`。
