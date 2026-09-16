# cylinder_dataset

采集「真实圆筒图像 ↔ 同一时刻飞控位置+姿态」配对数据集的工具，用于离线
调试/评估 CV 检测算法。跟 `src/dronelink` 一样，这是一个**并列的测试模块，
不是 colcon 包，也不接入任务链路**（任务链路是 `cuadc_interface` +
`cuadc_perception` + `cuadc_decision`）。

采集程序跑在**树莓派**上（相机 + 飞控都在树莓派侧）。已验证环境：
Pi 5 / Ubuntu / Python 3.12 / `pymavlink` / OpenCV / `rpicam-still`（libcamera）/
飞控串口 `/dev/ttyAMA0:57600`。

## 采集方法（室外 GPS）

飞控**不接电机**，与树莓派、相机一起刚性固连在一块板/架子上。**室外**走
正常起飞流程让飞控进入飞行状态、等 GPS 3D 锁定，然后手持整个架子摆位、
对准圆筒采集。每次触发：抓一张图 + 同一时刻飞控位姿快照，存成配对。

### 为什么强调室外 GPS

飞控的 `LOCAL_POSITION_NED`（相对起飞/home 点的 NED 偏移）来自 EKF。
室内无 GPS 时水平位置靠加速度计二次积分、几十秒漂移数米不可用；**室外
GPS 3D 锁定后 EKF 位置才可信**。脚本每张都记录 GPS 星数/fix 类型和
`EKF_STATUS_REPORT` 可信位（`pos_horiz_abs`/`pos_vert_abs`），并给出
`position_trustworthy`，后期一眼可辨。姿态（`ATTITUDE`）则始终可靠。
> 可信位判定口径与接口层一致，见
> [`interface_node.py`](../cuadc_interface/cuadc_interface/interface_node.py)
> 里 `_EKF_POS_READY`。

### 同一时刻对应

每个位姿字段都记录相对拍照时刻的新鲜度 `msg_age_s`，让「同刻对应」可量化。
连拍模式默认用 **picamera2 常驻相机**抓帧（延迟低、同刻最紧）；未装则回退
`rpicam-still`（每张 spawn 一次，同步略松但已记录 age）。数据流按 `--rate`
（默认 15 Hz）请求，位姿越新鲜。

### 两条硬约束

1. **三件东西必须刚性固连，相机↔IMU 外参标一次并记录**（`session.yaml` 的
   `camera_to_imu`）。飞控给的是 IMU 机体系姿态，数据集要的是相机姿态，
   两者差一个固定外参；任何相对松动都会让数据作废。
2. **场地/部件位置手工测量填进 `session.yaml`**：机架内各部件相对位置、
   圆筒地面中心相对 home 的偏移（或经纬度）、尺寸等。飞控上报的位置照记，
   但以手测值为地面真值。

### 相机与图传互斥

`rpicam-still`/picamera2 与 dronelink 的 `dronelink-video` 服务都独占
libcamera，**不能同时跑**。采集前先停图传：`sudo systemctl stop dronelink-video`。
默认 **1920x1080** 拍摄，与部署图传同分辨率，可直接复用 dronelink 已有的
相机内参标定（见 `src/dronelink` README 的 Calibration 一节），无需重标。

## 在树莓派上跑

相机和飞控都在 Pi 侧，直接**把本仓库 clone 进 Pi 在 Pi 上跑**：

```bash
# 在树莓派上（ssh wly@10.10.10.1）
git clone -b feature/cylinder-dataset <repo-url> ~/cuadc_ws
cd ~/cuadc_ws/src/cylinder_dataset

pip3 install --user -r capture/requirements.txt   # pymavlink/PyYAML 通常已装
# 想要最紧同步则装常驻相机后端：
sudo apt install -y python3-picamera2

sudo systemctl stop dronelink-video               # 释放相机

cp capture/session.example.yaml capture/session.yaml
#   编辑 session.yaml：填相机↔IMU 外参、各部件相对位置、圆筒位置/尺寸

# 走正常起飞流程让飞控进入飞行状态、等 GPS 3D 锁定，然后：
# 手动逐张：
python3 capture/capture_dataset.py --session capture/session.yaml
# 定时连拍（每 2s 一张，Ctrl-C 结束）：
python3 capture/capture_dataset.py --session capture/session.yaml --interval 2
```

常用参数：`--out`（输出根目录，默认 `~/cylinder_dataset_out`）、
`--interval`/`--count`（连拍间隔/张数）、`--backend`（`auto`/`picamera2`/`rpicam`）、
`--rate`（遥测频率）、`--width`/`--height`。

## 数据存哪 / 怎么取回

数据先落在**树莓派本地 SD 卡**（采集时只能在 Pi 上）。**每次运行**单独一个
带时间戳的新文件夹，互不覆盖：

```text
<out>/<session>_<YYYYmmdd_HHMMSS>/
  run_meta.json           本次运行元数据：外参/部件位置/圆筒(来自 session.yaml)、
                          home 位置、分辨率、相机后端、模式等，只记一份
  images/frame_0001.jpg   图像
  frame_0001.json         该张位姿快照：时间戳、姿态、local/global position、
                          GPS 质量、EKF 可信位、各字段 msg_age_s
  index.csv               所有帧平铺索引，便于批处理
```

采完及时拉回地面站备份（在**地面站**上执行，走 SSH）：

```bash
rsync -av wly@10.10.10.1:~/cylinder_dataset_out/ ~/datasets/cylinder_dataset_out/
# 或一次性：scp -r wly@10.10.10.1:~/cylinder_dataset_out ~/datasets/
```
