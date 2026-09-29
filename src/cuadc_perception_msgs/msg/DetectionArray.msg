# 感知模块按固定频率持续发布的一帧检测结果。见
# docs/spec/05-perception-decision-interface.md。
#
# header.stamp = 这帧图像的**拍照时刻**（不是感知处理完成时刻）——下游由此知道
# 测量是何时的。vehicle_pose = 拍照时刻的机体位姿（NED 位置 + 机体->世界四元数，
# 感知按拍照时刻从位姿缓冲插值），与本帧 bearing 同刻：末端 IBVS 的姿态补偿/高度
# 换算必须用它，不能用决策 tick 时的最新遥测（时刻不一致会把补偿本身做错）。

std_msgs/Header header
geometry_msgs/Pose vehicle_pose
Detection[] detections
