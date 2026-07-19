# 单个目标检测结果。感知模块逐帧独立发布，无状态，不做时序累积/跨帧
# 关联。契约唯一权威版本见 docs/spec/05-perception-decision-interface.md，
# 本文件不重复描述设计理由。

string target_type            # 目前只有 "cylinder"，字段先留着占位
geometry_msgs/Point position  # 世界坐标：与接口层 telemetry/local_position
                               # 同一坐标系（相对起飞点的局部坐标），由
                               # 感知内部完成像素->世界坐标转换后再发布
float64 apparent_size         # 视在尺寸（外接圆半径，米），供目标选型
                               # （比如投放区挑最小的）用
float64 confidence
