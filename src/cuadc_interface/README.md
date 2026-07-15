# cuadc_interface

接口层：与 ArduPilot 的 MAVLink 通信桥接节点。契约（提供哪些话题/
服务、设计原则、跟规划/决策的边界）唯一权威版本在
[`docs/spec/04-interfaces.md`](../../docs/spec/04-interfaces.md)，
不在这里重复。

## 当前状态

`interface_node.py` 是契约骨架——按 `04-interfaces.md` 声明了全部
话题/服务，但没有 MAVLink 连接、没有协议实现，指令服务调用会直接抛
`NotImplementedError`。这是刻意的：先把接口形状定下来，规划/决策可以
先对着这份骨架搭桩测试，接口层的真实协议实现（pymavlink 连接管理、
编解码）随后再补。

## 外部依赖（不 vendor 进本仓库）

- **`pymavlink`**（`pip3 install --user pymavlink`，不走 apt/rosdep，
  同 `cuadc_sim` 的处理方式，见
  [`docs/spec/working-notes.md`](../../docs/spec/working-notes.md)）。

## 用法

```
ros2 launch cuadc_interface interface_bringup.launch.py
ros2 launch cuadc_interface interface_bringup.launch.py connection:=udp:127.0.0.1:14550
```
