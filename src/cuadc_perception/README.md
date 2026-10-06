# cuadc_perception

感知/视觉模块。对外契约（消息类型、话题名、通信模式、职责边界）唯一
权威版本在
[`docs/spec/05-perception-decision-interface.md`](../../docs/spec/05-perception-decision-interface.md)，
本文件不重复描述。这份文件专门写 CV 算法本身怎么做的、参数怎么调、
踩过什么坑——会跟着算法一起持续更新，不是写一次就不动的设计文档。

两个感知节点发布同一份契约，决策层不需要区分对面是哪一个：

- `stub_perception_node.py`——从 `cuadc_sim` 场地的已知圆筒坐标直接发
  检测结果，不看相机，供决策层快速迭代（不用起 Gazebo）。
- `detectors/` + `cv_perception_node.py`——真实检测算法。`detectors/` 里
  每个检测器是纯函数实现（numpy 输入输出，不依赖 rclpy，可以脱离节点
  单测），`cv_perception_node.py` 只做 ROS2 接线（订阅图像/遥测、选检测器、
  发布检测结果），两者分层跟 `cuadc_decision` 的
  `mission_logic.py`/`decision_node.py` 是同一个原则。

```
ros2 launch cuadc_perception stub_perception_bringup.launch.py
ros2 launch cuadc_perception cv_perception_bringup.launch.py
```

顶层任务节点启动脚本按场景分开，`scripts/run_mission_sim.sh`/
`run_mission_real.sh` 固定起真实 CV 感知，`run_mission_stub_sim.sh`
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
- 选哪个检测器由**配置**决定——`cv_perception_node` 的 `detector` 参数选，
  **不做插件框架**。目前只有一个有意义的检测器 `color_pad`（从沙盒迁来的），
  以后有别的方法值得迁移再加。
- **组织与迁移规范（`detectors/` 文件夹，一个方法一个模块）**：探索期在
  `cv_explore` 里一个方法一个文件夹随便折腾；**迁移到这里时要组织**——只迁
  有意义的，每个方法落成 `detectors/` 下的**一个模块**（`color_pad.py`），
  它的**可调参数按 sim/real 分成两份 yaml、同名同目录跟着方法放**
  （`color_pad.sim.yaml` / `color_pad.real.yaml`）。config 跟着方法走、不平铺
  在包根（会乱）；yaml 甚至可以在探索期就作为 example 维护、跟着方法一起迁。
  哪个方法以后长出一堆附属文件，再单独提升成 `detectors/<方法>/` 子文件夹。
  **新方法迁移照 `color_pad` 这个样板来。** 每个检测器模块统一提供：`NAME`、
  `Config`、`load_config(profile) -> Config`、
  `detect_cylinders_pixels(image_bgr, cfg, perf, debug) -> [ContourObservation]`
  （`ContourObservation` 在 `detectors/base.py`），注册进 `detectors/__init__.py`
  的 `DETECTORS`。
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

## CV 算法：颜色检测器 `color_pad`（唯一/默认）

从 `cv_explore/color_shape` 沙盒移植（`detectors/color_pad.py`）。靶标是**蓝色
落点垫中心立一个白筒**（sim 里是 `gen_tube_mesh` 的 closed-bottom 筒立在浅蓝盘上，
真实赛场是蓝布垫上白桶——同一种"白盘套蓝盘"结构）。颜色比灰度/边缘**抗糊、
抗光照**，是主攻思路。每帧：

1. **HSV 分割**：蓝垫 `inRange(blue_lo, blue_hi)` + 白筒 `inRange(white_lo, white_hi)`。
2. **形态学去噪**：蓝/白各 open+close（核 `morph_k`）。
3. **蓝垫轮廓**：`RETR_CCOMP` 两层层级——外轮廓=垫，其子轮廓=垫上的洞。
   面积够大的**每个**蓝垫各出一个候选（多目标）。
4. **垫内定筒**：①洞法——蓝 mask 里筒是被垫包住的洞（几何最稳）；②白 mask
   法兜底（筒被画面边缘切开、洞不闭合时）。垫上任何非蓝物（压垫的砖、反光、褶皱）
   都是洞，所以候选须过尺寸/居中门且**像筒顶**：偏红像素少（砖偏红，晒白也偏红；
   白筒在暖光下只偏黄）、洞不细长（反光条/褶皱）、兜底白斑大部分在垫子凸包内（垫外
   水泥地不是筒）；合格者取最大。
   中心用**质心 + 面积等效半径**（斜视时白斑是新月形，外接圆半径会被撑爆，
   面积等效半径 `sqrt(area/π)` 对拉长不敏感）。
5. 输出白筒**像素中心 + 半径(px) + 置信度**（`0.5·垫圆度 + 0.5·同心度`）。
   反投影 / `bearing` / 物理半径过滤都在节点侧（见下面「节点侧组装」）。

**sim/real 只差 HSV 阈值**（算法不变），参数在 `detectors/color_pad.{sim,real}.yaml`：
sim 档是 **Gazebo 采样值**；real 档是真实蓝布垫的 census 值（源自
`cv_explore/color_shape`，1920 素材）。两套都验证过：sim 值上 Gazebo 真 CV
端到端两次投放成功、自动 RTL；real 值在 `cylinder_dataset` 真实航拍帧上检出
（红圈正套白桶）。选哪档 = `profile` 参数（见下面「配置分层」）。

### 节点侧组装（反投影 / bearing / 内参，所有检测器共用）

检测器只出图像空间量；`cv_perception_node` 用共享的 `geometry` 助手补上：像素
反投影成相机系射线 → 转机体系（固定挂载旋转 `camera_fixed_roll/pitch/yaw_rad`）
→ 转世界系（乘实时姿态四元数）→ 跟**筒顶所在平面**（不是地面，否则色块=筒顶
会引入正比于视线倾角的系统偏差；筒高 `cylinder_height_m` 对所有筒一样、零成本）
求交，得世界坐标 + 斜距；再按斜距把像素半径换算物理半径、过 `min/max_radius_m`；
像素 → 归一化方位 `bearing_x/y`（除掉内参，见
[ADR-0005](../../docs/spec/adr/0005-terminal-align-visual-servoing.md)）。姿态倾斜
太大（射线接近水平）时这一帧丢弃——数学上解不出合理交点。

相机内参默认值是 sim 实测值（`~/ardupilot_gazebo/models/gimbal_small_3d/model.sdf`：
`horizontal_fov=2.0rad`，640×480）反推的 `fx=fy=(640/2)/tan(1.0)≈205.5px`，
`cx=320, cy=240`，真机换真实标定值覆盖参数即可，不用改代码。

### 配置分层：`detector` 选方法、`profile` 选 sim/real

- 每个检测器的可调参数放它自己的 `detectors/<方法>.<profile>.yaml`，节点启动时
  按 `detector` + `profile` 参数用 `load_config(profile)` 读进来（一次）。
- **sim/real 沿用场景启动脚本**（`run_mission_sim.sh` / `run_mission_real.sh`
  传 `profile:=sim|real`），不靠一个默认值二选一。**注**：脚本层把 `profile`
  透传下去这一步**还没接线**（待办），现在默认 `profile=sim`。

## 关键参数（`cv_perception_bringup.launch.py`）

检测器**算法参数**（HSV/形态学/几何门）不在这——在
`detectors/<detector>.<profile>.yaml`（跟着方法走）。这里是节点级参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `detector` | `color_pad` | 用哪个检测器（`detectors/__init__.py` 的 `DETECTORS`）|
| `profile` | `sim` | 用检测器的哪档 config（`sim`/`real`），选哪份 yaml |
| `fx`/`fy`/`cx`/`cy` | 205.5/205.5/320.0/240.0 | 相机内参，sim 实测值 |
| `camera_fixed_roll/pitch/yaw_rad` | 0/1.5708/0 | 云台固定挂载角，**必须**跟 `gazebo_bringup.launch.py` 同名参数保持一致 |
| `cylinder_height_m` | 0.3 | 反投影用的筒高 |
| `min_radius_m`/`max_radius_m` | 0.05/0.18 | 物理半径合理区间，过滤明显不合理的候选 |

## 已知问题 / 后续要跟的点

- **斜视时白筒是"顶面+可见内壁"的月牙形，中心会偏向可见那一侧**：半径估计
  用面积等效半径（不是外接圆）已经不受影响（见 color_pad 第4步），但中心点
  本身在斜视时仍偏向月牙可见侧——这个偏差随进近角度趋于垂直而自愈（见
  `docs/spec/06-planning.md` 投放区进近的闭环设计），不在 CV 层单独修正。
  （注：筒坐落的蓝色圆盘曾是灰度检测器的干扰源；现在 color_pad 反过来**拿它
  当锚点**——先定蓝垫、再在垫内找白筒，见上。）
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
