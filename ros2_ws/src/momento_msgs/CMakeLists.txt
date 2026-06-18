cmake_minimum_required(VERSION 3.8)
project(momento_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MomentoCommand.msg"
  "msg/MomentoMode.msg"
  "msg/MomentoState.msg"
  "srv/MomentoControl.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
