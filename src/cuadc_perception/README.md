# cuadc_perception

感知/视觉模块。对外契约（消息类型、话题名、通信模式、职责边界）唯一
权威版本在
[`docs/spec/05-perception-decision-interface.md`](../../docs/spec/05-perception-decision-interface.md)，
本文件不重复描述。这份文件专门写 CV 算法本身怎么做的、参数怎么调、
踩过什么坑——会跟着算法一起持续更新，不是写一次就不动的设计文档。

两个感知节点发布同一份契约，决策层不需要区分对面是哪一个：

- `stub_perception_node.py`——从 `cuadc_sim` 场地的已知圆筒坐标直接发
  检测结果，不看相机，供决策层快速迭代（不用起 Gazebo）。
- `cv_pipeline.py` + `cv_perception_node.py`——真实检测算法。
  `cv_pipeline.py` 是纯函数实现（numpy 输入输出，不依赖 rclpy，可以
  脱离节点单测），`cv_perception_node.py` 只做 ROS2 接线（订阅图像/
  遥测、发布检测结果），两者分层跟 `cuadc_decision` 的
  `mission_logic.py`/`decision_node.py` 是同一个原则。

```
ros2 launch cuadc_perception stub_perception_bringup.launch.py
ros2 launch cuadc_perception cv_perception_bringup.launch.py
```

顶层任务节点启动脚本按场景分开，`scripts/run_mission_sim.sh`/
`run_mission_hardware.sh` 固定起真实 CV 感知，`run_mission_stub_sim.sh`
固定起桩感知——不用一个参数默认值二选一（之前是这么做的，默认值选中
了 stub，排查了很久才发现），一个脚本名直接对应一种场景，见仓库根
`README.md` Quickstart 表格。

## 检测器库：可替换的 CV 算法（一处声明，统一输出）

会有多种 CV 算法要试（阈值+形态学、径向对称、以后可能的轻量网络…），组织
约定是「一处声明、统一输出、可替换」：

- 每个 CV 算法是一个**纯函数**：输入一帧图像（+ 该算法自己的参数），输出
  **同一种结果**——筒的**像素中心 `(u,v)` + 视在半径(px) + 置信度**。统一
  输出就是「可替换」的来源；输入参数各算法可以不一样，无所谓。这些函数
  维护在一处（`cuadc_perception` 内的检测器模块），签名一致、可互换。
- 选哪个检测器由**配置**决定——像 `cuadc_decision/drop_strategy.py` 那样一个
  轻量的 `name -> 函数` dict + 一个「当前用哪个」的选择就够，**不做插件
  框架**。下面的六步流水线是第一个检测器。
- **反投影 / 归一化不写进检测器**：像素中心 → 世界坐标（反投影到筒顶平面）、
  像素 → 归一化方位 `bearing_x/y`（除掉内参，见
  [ADR-0005](../../docs/spec/adr/0005-terminal-align-visual-servoing.md)）都放在
  一处共享的 geometry 助手里。感知节点的组装顺序是：**选检测器 → 拿到像素
  结果 → geometry 助手补上 world position + bearing → 填成 `Detection` 发布**。
  好处：检测器保持纯图像空间、与内参/姿态/sim-real 解耦、能脱离节点单测；
  内参/姿态换算只维护一份。
- **与决策的边界（重要）**：决策侧的 approach **不 import 这些 CV 函数**，它
  通过 `/perception/detections` 话题拿 `Detection`（契约见
  [`05-...`](../../docs/spec/05-perception-decision-interface.md)）。所以「换
  CV 算法」= 改**感知这边**的配置；「不同任务阶段用不同检测器」要靠 05 里
  预留的「决策→感知开关」（MVP 不做），不是让决策直接调 CV——这样才守住
  「感知无状态、不知任务阶段、跟决策只隔一条话题」的边界。

## CV 算法：六步流水线（第一个检测器）

对每一帧 `/camera/image_raw` 做（`cv_pipeline.detect_cylinders`）：

1. **分割**：转灰度，`cv2.THRESH_OTSU` 自适应阈值二值化。Otsu 每帧
   自己算阈值，能扛住光照变化，不用写死一个亮度阈值。
2. **形态学去噪**：`MORPH_OPEN`（去散粒噪声）+ `MORPH_CLOSE`（填小洞），
   核大小 `morph_kernel_px`。
3. **轮廓提取**：`cv2.findContours(..., RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)`。
4. **拟合筛选**：`cv2.minEnclosingCircle` 拿轮廓中心（反投影用）；
   物理半径**不**用外接圆半径算，用等效面积半径
   `sqrt(area / π)`——原因见下面"已知问题"第一条，外接圆半径在斜视
   角度下会把半径撑爆到离谱。用换算出的半径按 `min_radius_m`/
   `max_radius_m` 过滤掉明显不合理的候选。
5. **反投影**（像素 → 世界坐标）：像素反投影成相机系射线 → 转机体系
   （固定挂载旋转，`camera_fixed_roll/pitch/yaw_rad`）→ 转世界系（乘
   实时姿态四元数）→ 跟**筒顶所在平面**（不是地面）求交，算出世界
   坐标和斜距。用筒顶而不是地面，是因为色块反映的是筒的顶面，跟地面
   求交会引入正比于视线倾角的系统性偏差；筒高对所有筒都一样
   （`cylinder_height_m`），零成本、不用现场标定。姿态倾斜太大（射线
   接近水平）时这一帧直接丢弃，不是"不准"而是数学上解不出合理交点。
6. **Confidence**：轮廓面积 / 外接圆面积（`circularity`）。越接近 1
   说明形状越规则（越像一个圆），斜视/噪声导致的不规则形状会明显
   偏低。第一版只用这一个信号，没有加权多个因子。

> 迁移说明：当前 `cv_pipeline` 把第 5 步反投影也打包在检测器里（直接返回
> 世界坐标）。按上面「检测器库」约定，迁移会把**反投影 + 归一化方位拆到
> 共享 geometry 助手**、检测器只保留纯图像空间的 1–4/6 步（输出像素中心 +
> 半径 + 置信度），由感知节点再组装成 `Detection`。

反投影用到的相机内参默认值是 sim 实测值（
`~/ardupilot_gazebo/models/gimbal_small_3d/model.sdf`：
`horizontal_fov=2.0rad`，640×480）反推出来的
`fx=fy=(640/2)/tan(1.0)≈205.5px`，`cx=320, cy=240`，真机换真实标定值
覆盖参数即可，不用改代码。

## 关键参数（`cv_perception_bringup.launch.py`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `fx`/`fy`/`cx`/`cy` | 205.5/205.5/320.0/240.0 | 相机内参，sim 实测值 |
| `camera_fixed_roll/pitch/yaw_rad` | 0/1.5708/0 | 云台固定挂载角，**必须**跟 `gazebo_bringup.launch.py` 同名参数保持一致 |
| `cylinder_height_m` | 0.3 | 反投影用的筒高 |
| `min_radius_m`/`max_radius_m` | 0.05/0.18 | 物理半径合理区间，过滤明显不合理的候选 |
| `morph_kernel_px` | 5 | 形态学去噪核大小 |

## 已知问题 / 后续要跟的点

- **斜视时色块是"顶面+可见侧面"的月牙形，不是干净的圆**：外接圆为了
  包住这个月牙形状，半径会被撑得远超真实尺寸（实测过一帧撑到 0.34m，
  远超任何筒的真实半径）——这也是筒下面其实压着一个蓝色计分圆环
  地标（B 区标记）导致的，设计阶段没预料到这一层，靠拿真实 Gazebo
  图像调试才发现。已经改用等效面积半径修掉了（见上面第4步），但
  这只是让半径估计"不离谱"，中心点位置本身还是会偏向月牙可见的
  那一侧——这个偏差随进近角度趋于垂直而自愈（见
  `docs/spec/06-planning.md` 投放区进近逻辑的闭环设计），不在 CV 层
  单独修正。
- **真实检测的可靠性有跑与跑之间的波动**：SITL 连续测试过两轮完整
  投放任务，一轮两次投放都顺利用真实 CV 走完，另一轮第二次搜索没能
  凑够候选、靠 90 秒超时兜底跳过了第二次投放——不是稳定复现的 bug，
  更像是真实噪声/视角导致某些筒有时候没被扫描到。这类"搜索参数/
  扫描航点density够不够"的调优留给 `feature/drop-zone-refinement`
  专项跑多轮统计，不算这次 CV 节点实现的缺陷。
- **反投影旋转约定还没有拿真实图像做过角度精度验证**：`_mount_rotation_matrix`
  的 pitch 符号约定是照着云台 `/gimbal/cmd_pitch` 话题自己的实测惯例
  写的（+1.5708=nadir），内部自洽性已经用合成图像单测验证过，但
  roll/yaw 非零（默认都是 0，还没在实际场景里用到过）的情况还没有
  拿真实图像验证，如果以后云台会用到非零 roll/yaw，要先补验证再信。

## Profiling

流水线分阶段耗时（分割/形态学/轮廓/拟合反投影）发布在
`~/perf`（`diagnostic_msgs/DiagnosticArray`），默认不出现在终端日志里，
需要看的时候自己订阅：

```
ros2 topic echo /cuadc_perception_cv/perf
```
