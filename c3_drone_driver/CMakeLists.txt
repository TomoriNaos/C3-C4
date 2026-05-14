cmake_minimum_required(VERSION 3.8)
project(c3_drone_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

set(msg_files
  "msg/GimbalMotionCommand.msg"
  "msg/GimbalVisualCommand.msg"
  "msg/GimbalState.msg"
  "msg/TargetObservation.msg"
  "msg/MissionCommand.msg"
  "msg/TargetHint.msg"
  "msg/DroneStatus.msg"
)

set(srv_files
  "srv/SetGimbalMode.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

add_executable(gimbal_controller_node src/gimbal_controller_node.cpp)
ament_target_dependencies(gimbal_controller_node rclcpp)

add_executable(target_processor_node src/target_processor_node.cpp)
ament_target_dependencies(target_processor_node rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(mavlink_bridge_node src/mavlink_bridge_node.cpp)
ament_target_dependencies(mavlink_bridge_node rclcpp)

rosidl_get_typesupport_target(c3_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(gimbal_controller_node ${c3_typesupport_target})
target_link_libraries(target_processor_node ${c3_typesupport_target})
target_link_libraries(mavlink_bridge_node ${c3_typesupport_target})

install(TARGETS
  gimbal_controller_node
  target_processor_node
  mavlink_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
