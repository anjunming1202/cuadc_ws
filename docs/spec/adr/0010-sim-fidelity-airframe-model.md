# 0010. 仿真机体保真度：真实几何 / 质量惯量 / 动力学

**Status**: Proposed

现有 sim 机体（外部 `~/ardupilot_gazebo` 的 `iris_with_gimbal`，≈1.98kg，相机挂三轴云台）
与真机三处不符，影响投放对准/末端 IBVS 可信度：相机相对飞控有平移偏置(sim 当 0)、投放口
相对相机有杆臂(`lever_arm_m` 当 0)、质量/推力差一截。据真机实测把 sim 拉近。本表是这些数值的
**单一出处**。坐标系：机体 FRD（前 X / 右 Y / 下 Z），参考点=四旋翼交叉中心。状态：✅已知
· 🅔估计(现用,待实测替换) · ❓待确认。缺的动力学数据先用估计值跑，后续有真机数据随时调。

| 簇 | 参数 | 值 | 状态 | 落地位置 |
|---|---|---|---|---|
| ① 感知几何 | 相机相对 FC (前/右/下) | +0.11 / 0 / +0.20 m | ✅ | `camera.real.yaml`(新增平移) + `geometry.py` 反投影 + SDF 相机 link pose |
| ① | 挂载 roll/pitch/yaw | 0 / 1.5708 / 0 (strapdown) | ✅ | `camera.real.yaml` + SDF(去云台) |
| ① | 内参 real (960×540) | fx869.37 fy871.03 cx486.48 cy260.11 | ✅ | `camera.real.yaml`(不变) |
| ① | 内参 sim (640×480) | fx/fy205.5 cx320 cy240 | ✅ | `camera.sim.yaml`(不变) |
| ② 投放几何 | 左瓶杆臂 [前,右] | [−0.11, −0.04] m | ✅ | `coarse_pbvs_then_terminal_ibvs.real.yaml` `lever_arm_m`(左/右两套) |
| ② | 右瓶杆臂 [前,右] | [−0.11, +0.04] m | ✅ | 同上 |
| ③ 动力学 | 总质量(满载) | 3.5 kg (=空机2.4+2瓶×0.55；半载2.95) | ✅ | vendored SDF：外部 iris(≈1.98)不动，加 lumped payload link 补足 |
| ③ | 满载 CoM | 左右居中、前后偏后 0.5cm(可忽略)、z 略下移 | ✅实测 | payload link `<inertial><pose>`；当前 x≈0 已足够 |
| ③ | 惯量 | iris 值 + payload link 估计 | 🅔 双线摆待测替换 | payload link `<inertia>` |
| ③ | 悬停油门 | 真机 `MOT_THST_HOVER=0.35`；**sim 实测 0.33，已匹配** | ✅实测 | 可直接写 0.35 替代 `MOT_HOVER_LEARN` |
| ③ | ArduPilot 参数 | 真机 `local/scratch/test915.param`(gains 近默认) | ✅实测 | 挑 ATC/PSC/MOT/WPNAV 灌 `mission_defaults.parm` |
| — | 轴距(对角电机轴线) | **545 mm** → 旋翼 ±0.1925/±0.1925 (X) | ✅实测 | SDF 旋翼位(几何重做用) |
| — | 螺旋桨 | **15×5.5 折叠, Ø381mm(r0.19)** | ✅实测 | SDF 桨 visual/collision |
| — | 电机/电池 | 朗宇 4008 380KV / 6S 4000mAh；4电机总电流上限≈60A | ✅实测 | 推力上限参考(TWR) |
| — | FC 相对旋翼中心 | 在正中心(前后偏后 0.5cm) | ✅实测 | SDF 摆放(X≈0) |
| — | 相机 | 树莓派 IMX519, FOV≈80°, 卷帘, 自动对焦 | ✅实测 | sim 相机 fov/内参对齐(待做) |
| — | SITL ArduCopter 版本 | 需=真机 | ❓ | 编译对齐 |

## Notes

- **Decision**：机体模型 vendor 进 `src/cuadc_sim/models/cuadc_iris/`（不再改外部依赖，可复现），
  云台改硬支架固定 nadir 相机，world include 改指向它。
- **三簇正交**：①②与飞行正交、彼此独立且数据已全 → 独立验（① 合成图/单测，② 无头压测台）；
  ③ 内部强耦合（质量↔推力↔悬停、惯量↔角速率增益）须一起动，现用估计值、要 SITL+Gazebo 联验。
- **③ 用估计值先跑**：前提=6S 电池、真机现飞得稳(推重比够)、机不在手拿不到 bin。总质量按 lumped
  payload link 补到满载 3.5kg（外部 iris 模型不改），推力按质量比缩放 `area`、悬停靠 `MOT_HOVER_LEARN`
  自学。有真机参数/推力/惯量后按表逐项替换，不必重做结构。
- **端到端仍耦合**：strapdown 下机体一倾图像就动 → 反投影精度(①)在飞行中取决于姿态(③)，收尾须整机联验。
- **sim 自洽**：SDF 相机渲染位置必须等于 `camera.real.yaml` 平移，否则 sim 自相矛盾（真机只 camera.yaml 生效）。
- **动力学替换路径**：真机参数到手后搬进 parm、单桨最大推力到手后设 `area` 锁推重比 → 悬停油门(目标
  0.35–0.5)自动对上、`MOT_THST_HOVER` 用真机日志学到值。当前估计值先满足这个悬停油门区间即可。
- **投放竖直偏置不测**：圆柱限位竖直下落，不影响水平落点。
- **质量突变不做**：DROP 时质量/CoM 跳变按静态档略过，留后续。
- **真机数据已到(2026-09-29)**：轴距545/桨15×5.5/电机朗宇4008-380KV/6S/CoM近中心/参数文件
  test915.param(含 MOT_THST_HOVER=0.35)。**关键结论：sim 悬停 0.33 ≈ 真机 0.35，推力已匹配，不必重调**；
  gains 近默认。**仍缺**：含悬停的 `.bin` 日志、各部件质量(惯量估计用)、ArduCopter 版本号。
- **几何重做(Phase 3)用实测值**：旋翼 ±0.1925(X, 对角545)、桨 r0.19(15")、相机前0.11/下0.20、FC中心。
  上次几何大改(估计值)出现反投影回归(near_det≈1m, 接近段不收敛)已回退、存 patch;
  用实测值重做时须先定位该回归(疑似相机面内朝向 / 抬高出生点使 EKF 原点上移破坏反投影地平面假设),
  与 Phase 4 落点量测工具一起做最省。
