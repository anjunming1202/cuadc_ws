# cuadc_interface

接口层：与 ArduPilot 的 MAVLink 通信桥接节点。契约（提供哪些话题/
服务、设计原则、跟规划/决策的边界）唯一权威版本在
[`docs/spec/04-interfaces.md`](../../docs/spec/04-interfaces.md)，
不在这里重复。

## 当前状态

`interface_node.py` 已实现 `04-interfaces.md` 里的全部话题/服务，
接过真实 SITL 验证过（`set_mode`/`arm`/`goto_location`/`set_relay`
及全部遥测）。

## 外部依赖（不 vendor 进本仓库）

- **`pymavlink`**（`pip3 install --user pymavlink`，不走 apt/rosdep，
  同 `cuadc_sim` 的处理方式，见
  [`docs/spec/working-notes.md`](../../docs/spec/working-notes.md)）。

## 用法

每个终端先 `source scripts/activate_env.sh`（工作区通用的环境准备，
跟 `cuadc_sim` 共用同一份，见该脚本注释），再执行：

```
ros2 launch cuadc_interface interface_bringup.launch.py
ros2 launch cuadc_interface interface_bringup.launch.py connection:=udp:127.0.0.1:14550
```

验收测试（需要先在另一个终端起好上面的节点并连上 SITL/真机，不在
launch 文件里，单独手动跑）：

```
ros2 run cuadc_interface interface_acceptance_test
```

机动测试（更完整的多段路径，覆盖水平/爬升/下降组合、转向、连续换
目标，跑得比验收测试慢，见文件顶部说明）：

```
ros2 run cuadc_interface interface_maneuver_test
```
