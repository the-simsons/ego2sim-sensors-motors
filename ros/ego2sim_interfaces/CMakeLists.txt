cmake_minimum_required(VERSION 3.8)
project(ego2sim_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScalarStamped.msg" "msg/MotorCommand.msg"
  "msg/ControlMode.msg" "msg/MotorState.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
