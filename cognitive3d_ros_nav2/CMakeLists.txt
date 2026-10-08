cmake_minimum_required(VERSION 3.16)
project(cognitive3d_ros_nav2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(cognitive3d_ros REQUIRED)

add_library(cognitive3d_ros_nav2 SHARED
  src/behavior_tree_translator.cpp
  src/register.cpp
)
target_include_directories(cognitive3d_ros_nav2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(cognitive3d_ros_nav2
  rclcpp rclcpp_lifecycle rclcpp_components nav2_msgs cognitive3d_ros)
target_compile_options(cognitive3d_ros_nav2 PRIVATE -Wall -Wextra)

# The component a container loads, as `cognitive3d_ros::Cognitive3DNode` is cognitive3d_ros's:
# the same node, with the Nav2 translators added at construction.
rclcpp_components_register_nodes(cognitive3d_ros_nav2 "cognitive3d_ros_nav2::Cognitive3DNode")

# The entry point a Nav2 robot runs. Deliberately the same executable NAME as the one
# `cognitive3d_ros` ships, because it is the same node: only the package differs, and the shipped
# launch file resolves which one without anybody having to choose.
add_executable(cognitive3d_node src/node_main.cpp)
target_link_libraries(cognitive3d_node cognitive3d_ros_nav2)
ament_target_dependencies(cognitive3d_node rclcpp rclcpp_lifecycle cognitive3d_ros)
target_compile_options(cognitive3d_node PRIVATE -Wall -Wextra)
install(TARGETS cognitive3d_node DESTINATION lib/${PROJECT_NAME})

# doctor for that node. `ros2 run cognitive3d_ros doctor` runs this one whenever this package is
# installed, as the launch file runs this package's node.
add_executable(doctor src/doctor_main.cpp)
target_link_libraries(doctor cognitive3d_ros_nav2)
ament_target_dependencies(doctor rclcpp rclcpp_lifecycle cognitive3d_ros)
target_compile_options(doctor PRIVATE -Wall -Wextra)
install(TARGETS doctor DESTINATION lib/${PROJECT_NAME})

install(TARGETS cognitive3d_ros_nav2
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/../THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_behavior_tree_translator test/test_behavior_tree_translator.cpp)
  target_link_libraries(test_behavior_tree_translator cognitive3d_ros_nav2)
  ament_target_dependencies(test_behavior_tree_translator rclcpp nav2_msgs cognitive3d_ros)
  # The distro's expected behaviour comes from outside the code under test.
  target_compile_definitions(test_behavior_tree_translator
    PRIVATE C3D_TEST_ROS_DISTRO="$ENV{ROS_DISTRO}")

  # Builds a real node, so it owns its ROS domain for the same reason test_session_control does.
  ament_add_gtest(test_entry_point test/test_entry_point.cpp
    ENV ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  target_link_libraries(test_entry_point cognitive3d_ros_nav2)
  ament_target_dependencies(test_entry_point rclcpp rclcpp_lifecycle nav2_msgs cognitive3d_ros)

  # Loads the component the way a container does: from this package's installed ament index, so
  # the install prefix is put on the path, and with the library opened by class_loader rather than
  # linked.
  ament_add_gtest(test_component test/test_component.cpp
    TIMEOUT 120
    ENV ROS_DOMAIN_ID=90 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
  target_include_directories(test_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_component
    rclcpp rclcpp_lifecycle rclcpp_components lifecycle_msgs nav2_msgs cognitive3d_ros)

  # cognitive3d_ros's installed doctor, with this package's install prefix on the ament index as
  # an installed robot has it.
  get_filename_component(C3D_ROS_PREFIX "${cognitive3d_ros_DIR}/../../.." ABSOLUTE)
  ament_add_test(test_doctor_handoff
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_doctor_handoff.py
      ${C3D_ROS_PREFIX}/lib/cognitive3d_ros/doctor
    TIMEOUT 120
    ENV ROS_DOMAIN_ID=96 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(cognitive3d_ros_nav2)
ament_export_dependencies(rclcpp rclcpp_lifecycle rclcpp_components nav2_msgs cognitive3d_ros)
ament_package()
