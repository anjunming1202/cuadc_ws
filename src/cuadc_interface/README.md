# cuadc_interface

接口层：与 ArduPilot 的 MAVLink 通信桥接节点。契约（提供哪些话题/
服务、设计原则、跟规划/决策的边界）唯一权威版本在
[`docs/spec/04-interfaces.md`](../../docs/spec/04-interfaces.md)，
不在这里重复。

## 当前状态

`interface_node.py` 已实现 `04-interfaces.md` 里的全部话题/服务，
全部指令原语（Arm/Set Mode/Takeoff/Go To Location/两个 Set Position
Target/Set Velocity/Set Yaw；投放原语 2026-09-23 由 relay 改为 Release
Payload，见 [ADR-0007](../../docs/spec/adr/0007-drop-servo-vs-relay.md)）和
全部遥测都过 SITL 验证，见下方四个测试脚本。

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

飞行测试节点（`cuadc_interface/tests/`：起降 `interface_takeoff_land_test`、动作合集
`interface_flight_suite_test`、解锁/接管检查 `interface_arm_test`）都走"放行 + 飞手 RC 解锁"流程，
用 `scripts/run_*_{sim,real}.sh` 起（脚本自带接口层并收尾），见
[`field-test-runbook.md`](../../docs/spec/field-test-runbook.md)。

台架未解锁测舵机：launch 加 `allow_disarmed_drop:=true`（飞之前关掉）。拆桨台架脚本在 [`local/flight_ctrl_tests/`](../../local/flight_ctrl_tests/)：电机只听遥控 arm；投放用 `DO_SET_SERVO` 写 **SERVO9(=AUX1)**，不是 RC9 override。
