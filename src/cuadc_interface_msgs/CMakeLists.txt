cmake_minimum_required(VERSION 3.8)
project(cuadc_interface_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PositionTarget.msg"
  "msg/GpsStatus.msg"
  "msg/EkfStatus.msg"
  "msg/FailsafeState.msg"
  "msg/DropStatus.msg"
  "msg/CommandAck.msg"
  "srv/Arm.srv"
  "srv/SetMode.srv"
  "srv/Takeoff.srv"
  "srv/GotoLocation.srv"
  "srv/ReleasePayload.srv"
)

ament_package()
