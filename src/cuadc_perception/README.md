# cuadc_perception

感知/视觉模块。对外契约（消息类型、话题名、通信模式、职责边界）唯一
权威版本在
[`docs/spec/05-perception-decision-interface.md`](../../docs/spec/05-perception-decision-interface.md)，
本文件不重复描述。

当前只有 `stub_perception_node`——从 `cuadc_sim` 场地的已知圆筒坐标
发布检测结果，不订阅相机，供决策层在真实感知算法完成前先打通全流程
闭环测试。真实 CV 检测（圆筒识别/测距）是后续独立分支的范围，见
`docs/spec/06-planning.md` 之外的项目整体路线（这次分支
`feature/perception-decision-kickoff` 明确不含）。

```
ros2 launch cuadc_perception stub_perception_bringup.launch.py
```
