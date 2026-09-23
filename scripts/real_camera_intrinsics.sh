#!/bin/bash
# 真机相机内参 + 取帧/预览的 ros2 launch 参数——单一真源。被所有"真机跑真实
# CV"的启动脚本 source（run_mission_hardware.sh 的 L3 全栈、run_field_l1_perception.sh
# 的感知单点），避免把标定数字抄进多个脚本又各自漂移。
#
# 为何是 CLI 参数而不是 profile 档 yaml：本项目真机相机只有一台，标定值稳定，
# 用一处 shell 片段承载、launch 参数注入即可，不值得再起一层 profile 配置加载。
#
# 内参对应 CV 处理分辨率 cv_width x cv_height（cv_perception 默认 960x540），
# **不是**标定原图分辨率。标定产物存 src/cuadc_perception/calibration/camera_calibration.npz
# （cylinder_dataset 棋盘格标定，src/dronelink/calibration/calibrate_camera.py 在 Windows 上
# 跑得到，见 feature/cylinder-dataset 分支）。npz 里 camera_matrix 是标定原图（相机原始
# WxH）下的值，换算到 960x540：
#   fx' = fx * 960 / W    fy' = fy * 540 / H
#   cx' = cx * 960 / W    cy' = cy * 540 / H
# （相机 raw = 1920x1080 时即整体 ×0.5。畸变系数在此不用——反投影只吃针孔内参，
#  若要去畸变则在取帧阶段做，另议。）
#
# frame_source=shm：真机从 camera_service 的原始 NV12 shm 直取帧，图不进 ROS
# （ADR-0008）。preview_enabled=true：把标注预览编码推本地 5604，交给 ground-monitor
# 的 cv-forward 转地面（profile=none 时没人转，内核丢弃，无害）。

# shellcheck disable=SC2034  # 由 source 方展开使用
REAL_CV_LAUNCH_ARGS=(
    frame_source:=shm
    preview_enabled:=true
    # cylinder_dataset 棋盘格标定 camera_calibration.npz（标定原图 1920x1080，
    # RMS 0.30）换算到 960x540（×0.5）：
    #   fx 1738.74→869.37  fy 1742.05→871.03  cx 972.97→486.48  cy 520.22→260.11
    fx:=869.37
    fy:=871.03
    cx:=486.48
    cy:=260.11
)
