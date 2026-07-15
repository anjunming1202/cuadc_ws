# cuadc_interface_msgs

`cuadc_interface` 接口层节点用到的自定义 ROS2 消息/服务定义。契约本身
（每个类型的用途、话题/服务名、设计原则）唯一权威版本在
[`docs/spec/04-interfaces.md`](../../docs/spec/04-interfaces.md)，本
包只是该契约的 `.msg`/`.srv` 落地，不在这里重复描述。

独立成包（而不是定义在 `cuadc_interface` 内部）是 ROS2 的常规做法——
自定义消息/服务需要 `ament_cmake` + `rosidl` 生成代码，`cuadc_interface`
节点本体走 `ament_python`，两种构建类型不适合揉进同一个包。
