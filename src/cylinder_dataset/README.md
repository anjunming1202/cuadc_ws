# cylinder_dataset

采集「真实圆筒图像 ↔ 同一时刻飞控位置+姿态」配对数据集的工具，用于离线
调试/评估 CV 检测算法。跟 `src/dronelink` 一样，这是一个**并列的测试模块，
不是 colcon 包，也不接入任务链路**（任务链路是 `cuadc_interface` +
`cuadc_perception` + `cuadc_decision`）。

采集程序跑在**树莓派**上（相机 + 飞控都在树莓派侧）。已验证环境：
Pi 5 / Ubuntu / Python 3.12 / `pymavlink` / OpenCV / `rpicam-still`（libcamera）/
飞控串口 `/dev/ttyAMA0:57600`。

## 两种采集方法

图像采集、位姿记录、配置、写盘完全一样（共用 `capture/dataset_core.py`），
只在**谁控制飞控**上不同：

| 脚本 | 谁控制飞控 | 触发拍照 |
|---|---|---|
| `capture/capture_external.py` | **外部**（手持 / 遥控 Stabilize / Mission Planner） | 手动回车逐张，或 `--interval` 定时连拍 |
| `capture/capture_grid.py` | **树莓派自主**（GUIDED 解锁/起飞/走立体格点/返航） | 每到一个格点稳定后自动拍 |

两者都：每次运行单独一个带时间戳的新文件夹；每张图配一份同刻位姿 JSON +
写进 `index.csv`。

## 共用设计

### 室外 GPS 与位置可信度

飞控 `LOCAL_POSITION_NED`（相对起飞/home 的 NED 偏移）来自 EKF（融合
IMU+GPS+气压计，非裸 GPS）。室内无 GPS 时水平位置漂移数米不可用；**室外
GPS 3D 锁定后才可信**。普通 GPS 米级精度，要厘米级需 RTK。每张都记录 GPS
星数/fix、`EKF_STATUS_REPORT` 可信位与 `position_trustworthy`，姿态
（`ATTITUDE`）则始终可靠。
> 可信位判定口径与接口层一致，见
> [`interface_node.py`](../cuadc_interface/cuadc_interface/interface_node.py)
> 里 `_EKF_POS_READY`。

### 同一时刻对应

每个位姿字段记录相对拍照时刻的新鲜度 `msg_age_s`，让「同刻对应」可量化。
优先 **picamera2 常驻相机**抓帧（延迟低、同刻最紧），未装回退 `rpicam-still`。
数据流按 `--rate`（默认 15 Hz）请求，越高越新鲜。

### 两条硬约束

1. **三件东西刚性固连，相机↔IMU 外参标一次并记录**（`session.yaml` 的
   `camera_to_imu`）。飞控给的是 IMU 机体系姿态，数据集要的是相机姿态，
   差一个固定外参；任何相对松动都让数据作废。
2. **场地/部件位置手工测量填进 `session.yaml`**：机架内各部件相对位置、
   圆筒地面中心相对 home 的偏移（或经纬度）、尺寸。飞控上报位置照记，
   但以手测值为地面真值。

### 畸变与压缩

- **畸变**：采集端存**原图（带畸变）**，不矫正。矫正是**下游 CV 流程**的离线
  步骤，用 `capture.yaml` 里 `camera_calibration` 引用的那份标定做；该引用
  原样写进每次运行的 `run_meta.json`，数据集自描述。
- **压缩**：`--format jpg|png`、`--quality`（JPEG 质量），默认 `jpg`/95。

### 相机与图传互斥

`rpicam-still`/picamera2 与 dronelink 的 `dronelink-video` 服务都独占
libcamera，**不能同时跑**。采集前先停图传：`sudo systemctl stop dronelink-video`。
默认 **1920x1080**，与部署图传同分辨率，可直接复用 dronelink 已有相机内参
标定（见 `src/dronelink` README 的 Calibration 一节），无需重标。

### 与地面站共存（不独占串口/相机）

想在采集时**地面站仍能看到图像+遥测**：飞控串口交给 mavlink-router 分发、
图传管线 tee 一路到本地，采集用 `connection: udp:...` + `backend: rtp`。
一整套可直接部署的 mavlink-router 配置 / systemd drop-in / 运行配置 + 步骤见
[`deploy/`](deploy/README.md)。同刻性/画质权衡也在那里说明。

## 文件布局

```text
capture/
  dataset_core.py         共享核心：相机后端 / 位姿监听 / 配置合并 / 写盘
  capture_external.py     方法一：飞控外部控制，只读+拍照
  capture_grid.py         方法二：树莓派自主飞格点+拍照
  capture.example.yaml    共享运行参数 + 相机标定引用（复制成 capture.yaml）
  session.example.yaml    共享物理测量：外参/部件/圆筒/场地（复制成 session.yaml）
  grid.example.yaml       方法二专用：格点矩阵 + 飞行参数（复制成 grid.yaml）
  requirements.txt
```

三个 `*.yaml` 都是模板，复制成同名去 `.example` 的工作副本改，副本已
gitignore。**两类参数**：稳定的（串口/分辨率/后端/格式/输出目录/标定）放
`capture.yaml`，脚本自动加载、不用每次敲；每次不同的走命令行，命令行覆盖
`capture.yaml`。

## 在树莓派上跑

相机和飞控都在 Pi 侧，**把本仓库 clone 进 Pi 在 Pi 上跑**：

```bash
# 在树莓派上（ssh wly@10.10.10.1）
git clone -b feature/cylinder-dataset <repo-url> ~/cuadc_ws
cd ~/cuadc_ws/src/cylinder_dataset

pip3 install --user -r capture/requirements.txt   # pymavlink/PyYAML 通常已装
sudo apt install -y python3-picamera2             # 想要最紧同步则装常驻相机后端
sudo systemctl stop dronelink-video               # 释放相机

cp capture/capture.example.yaml capture/capture.yaml     # 一次定
cp capture/session.example.yaml capture/session.yaml     # 每次采集填外参/圆筒
```

### 方法一：飞控外部控制（手持/遥控/Mission Planner）

只读飞控 + 拍照，不发任何控制指令，**不会 override** 遥控 Stabilize 或
Mission Planner AUTO——可一边用遥控/MP 飞或保持姿态，一边在 Pi 上采集。

```bash
# 让飞控进入飞行状态（手持/遥控/MP 均可）、室外等 GPS 3D 锁定，然后：
python3 capture/capture_external.py --session capture/session.yaml            # 手动逐张，回车拍一张
python3 capture/capture_external.py --session capture/session.yaml --interval 2   # 每 2s 一张，Ctrl-C 停
python3 capture/capture_external.py --session capture/session.yaml --passive      # 字节级只读（需飞控已在推流）
```

单张采集就是不带 `--interval`：SSH 终端按回车拍一张，`q`+回车结束。

### 方法二：树莓派自主飞立体格点

树莓派全程控制飞控，无需遥控/MP 打杆。**务必户外、GPS 3D 锁定、EKF 位置
可信，且遥控开机做安全接管在手**（随时能切出 GUIDED 到 Loiter/RTL）。

```bash
cp capture/grid.example.yaml capture/grid.yaml     # 填格点矩阵大小/间距/高度层/飞行速度/末端动作
python3 capture/capture_grid.py --session capture/session.yaml
#   脚本先做起飞前检查（GPS/EKF/姿态），再要求输入 GO 确认，才解锁起飞；
#   --yes 跳过确认；--grid 指定其它 grid 配置
```

飞行序列：切 GUIDED → 解锁 → 起飞到 `takeoff_alt_m` → 逐个格点（每层蛇形，
到点判据 `arrival_radius_m`，稳定 `settle_time_s` 后拍）→ 末端 `end_action`
（rtl/land/loiter）。每张额外记 `grid`（目标 NED、目标 yaw、是否到点）。

## 要不要像正式任务那样起 bringup？

**不用。** 正式任务是 ROS2 多节点栈（`cuadc_interface`+感知+决策），靠
bringup 起节点、再等一个「开始」topic（比赛时开机自启）。本工具是**独立的
并列测试模块**：单个 Python 脚本自己独占飞控串口 + 相机，把飞控通信、飞行、
拍照、记录全包了，**不起任何 ROS2 节点、不需要开始 topic**。

准备与启动就是上面的步骤：SSH 进 Pi → 确保没有别的进程占着飞控串口
（正式任务/`cuadc_interface` 不能同时在跑，会抢 `/dev/ttyAMA0`）→ 停图传
释放相机 → 填好 `*.yaml` → 直接 `python3 capture/capture_*.py`。方法二会在
解锁前做检查并要人工 `GO` 确认。

> 注意串口独占：方法二自己发控制指令，因此**不能**同时跑方法一或
> `cuadc_interface`（三者都要开 `/dev/ttyAMA0`，只能有一个）。

## 数据存哪 / 怎么取回

数据先落在**树莓派本地 SD 卡**。**每次运行**单独一个带时间戳的新文件夹：

```text
<out>/<session>_<YYYYmmdd_HHMMSS>/       # <out> 默认 ~/cylinder_dataset_out
  run_meta.json           本次运行元数据：method、外参/部件/圆筒(来自 session.yaml)、
                          标定引用、home、分辨率、相机后端、模式/格点配置，只记一份
  images/frame_0001.jpg   图像
  frame_0001.json         该张位姿快照：时间戳、姿态、local/global position、
                          GPS 质量、EKF 可信位、各字段 msg_age_s（方法二另含 grid 目标）
  index.csv               所有帧平铺索引，便于批处理
```

采完拉回地面站备份（在**地面站**上执行，走 SSH）：

```bash
rsync -av wly@10.10.10.1:~/cylinder_dataset_out/ ~/datasets/cylinder_dataset_out/
# 或一次性：scp -r wly@10.10.10.1:~/cylinder_dataset_out ~/datasets/
```
