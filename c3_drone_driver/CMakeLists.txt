cmake_minimum_required(VERSION 3.8)
project(c3_drone_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(px4_msgs QUIET)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

set(msg_files
  "msg/CommandAck.msg"
  "msg/GimbalMotionCommand.msg"
  "msg/GimbalVisualCommand.msg"
  "msg/GimbalState.msg"
  "msg/TargetObservation.msg"
  "msg/MissionCommand.msg"
  "msg/TcDetection.msg"
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

add_executable(gimbal_controller_node src/inner_node/gimbal_controller_node.cpp)
ament_target_dependencies(gimbal_controller_node rclcpp)

add_executable(target_processor_node src/inner_node/target_processor_node.cpp)
ament_target_dependencies(target_processor_node rclcpp std_msgs sensor_msgs geometry_msgs ament_index_cpp)
target_include_directories(target_processor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)

add_library(target_fusion_processor_lib src/target_fusion_processor.cpp)
ament_target_dependencies(target_fusion_processor_lib rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(target_fusion_processor_lib PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_library(pose_estimator_lib src/pose_estimator.cpp)
ament_target_dependencies(pose_estimator_lib rclcpp geometry_msgs)
target_include_directories(pose_estimator_lib PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(pose_estimator_lib Eigen3::Eigen)

add_executable(mavlink_bridge_node src/bridge_node/mavlink_bridge_node.cpp)
ament_target_dependencies(mavlink_bridge_node rclcpp)

add_executable(drone_main_controller_node src/inner_node/drone_main_controller_node.cpp)
ament_target_dependencies(drone_main_controller_node rclcpp geometry_msgs ament_index_cpp lifecycle_msgs)
target_include_directories(drone_main_controller_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(motion_controller_node src/inner_node/motion_controller_node.cpp)
ament_target_dependencies(motion_controller_node rclcpp geometry_msgs)
target_link_libraries(motion_controller_node Eigen3::Eigen)

add_executable(sensor_mock_node src/inner_node/sensor_mock_node.cpp)
ament_target_dependencies(sensor_mock_node rclcpp std_msgs sensor_msgs)

add_executable(gimbal_joint_state_bridge_node src/bridge_node/gimbal_joint_state_bridge_node.cpp)
ament_target_dependencies(gimbal_joint_state_bridge_node rclcpp sensor_msgs)

add_executable(px4_pose_bridge_node src/bridge_node/px4_pose_bridge_node.cpp)
ament_target_dependencies(px4_pose_bridge_node rclcpp geometry_msgs nav_msgs)

if(px4_msgs_FOUND)
  add_executable(offboard_setpoint_bridge_node src/bridge_node/offboard_setpoint_bridge_node.cpp)
  ament_target_dependencies(offboard_setpoint_bridge_node rclcpp geometry_msgs px4_msgs)
endif()

rosidl_get_typesupport_target(c3_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(gimbal_controller_node ${c3_typesupport_target})
target_link_libraries(target_fusion_processor_lib ${c3_typesupport_target})
target_link_libraries(pose_estimator_lib ${c3_typesupport_target})
target_link_libraries(target_processor_node ${c3_typesupport_target})
target_link_libraries(target_processor_node ${OpenCV_LIBS})
target_link_libraries(mavlink_bridge_node ${c3_typesupport_target})
target_link_libraries(drone_main_controller_node ${c3_typesupport_target})
target_link_libraries(drone_main_controller_node ${OpenCV_LIBS})
target_link_libraries(motion_controller_node ${c3_typesupport_target})
target_link_libraries(sensor_mock_node ${c3_typesupport_target})
target_link_libraries(gimbal_joint_state_bridge_node ${c3_typesupport_target})
target_link_libraries(px4_pose_bridge_node ${c3_typesupport_target})
if(px4_msgs_FOUND)
  target_link_libraries(offboard_setpoint_bridge_node ${c3_typesupport_target})
endif()
target_link_libraries(target_processor_node target_fusion_processor_lib)
target_link_libraries(target_fusion_processor_lib pose_estimator_lib)
target_link_libraries(drone_main_controller_node pose_estimator_lib)

install(TARGETS
  gimbal_controller_node
  target_processor_node
  mavlink_bridge_node
  drone_main_controller_node
  motion_controller_node
  sensor_mock_node
  gimbal_joint_state_bridge_node
  px4_pose_bridge_node
  target_fusion_processor_lib
  pose_estimator_lib
  DESTINATION lib/${PROJECT_NAME}
)

if(px4_msgs_FOUND)
  install(TARGETS offboard_setpoint_bridge_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
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
