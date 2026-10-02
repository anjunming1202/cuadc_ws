# 感知按固定频率发布的一帧检测结果。契约唯一权威版本见
# docs/spec/05-perception-decision-interface.md。
#
# header.stamp = 这帧图像的拍照时刻(非处理完成时刻)。
# vehicle_pose = 拍照时刻机体位姿(NED 位置+机体->世界四元数),与 bearing 同刻——
#   IBVS 姿态补偿/高度换算必须用它,不得用下游 tick 时的最新遥测。

std_msgs/Header header
geometry_msgs/Pose vehicle_pose
Detection[] detections
