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
| ③ | 满载 CoM | ≈旋翼中心 x/y≈0、z 略下移 | 🅔 | payload link `<inertial><pose>`；载荷 x≈0(水瓶在中心上方)、y=±0.04 对称 |
| ③ | 惯量 | iris 值 + payload link 估计 | 🅔 双线摆待测替换 | payload link `<inertia>` |
| ③ | 单桨最大推力 | 按质量比缩放 lift-drag `area`(×3.5/1.98≈1.77) | 🅔 推力台/datasheet 待替换 | vendored SDF lift-drag `area` ×4 |
| ③ | ArduPilot 参数 | gazebo-iris 默认 + `MOT_HOVER_LEARN=2` 自学悬停 | 🅔 真机 dump 待替换 | `mission_defaults.parm` |
| — | FC 相对旋翼中心前后 X | 暂按 0 | ❓ | SDF 摆放 |
| — | SITL ArduCopter 版本 | 需=真机(4.7.1?) | ❓ | 编译对齐 |

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
- **待要数据（转飞控/硬件组）**：① 真机完整参数文件；② 电机+桨最大推力(推力台曲线或型号+桨+电池节数)；
  ③ 含悬停的 `.bin` 日志(有则给)；④ ArduCopter 版本号；⑤ 电池质量+位置、单臂+电机质量(给了可算惯量估计)。
