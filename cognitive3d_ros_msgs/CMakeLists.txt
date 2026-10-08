cmake_minimum_required(VERSION 3.16)
project(cognitive3d_ros_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Event.msg
  srv/StartSession.srv
  srv/EndSession.srv
  DEPENDENCIES builtin_interfaces
)

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
