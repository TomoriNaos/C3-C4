cmake_minimum_required(VERSION 3.8...3.28)

project(lidar_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(rclpy REQUIRED)


add_executable(lidar_depth_node src/lidar_depth_node.cpp)
ament_target_dependencies(lidar_depth_node rclcpp sensor_msgs std_msgs)

install(TARGETS
  lidar_depth_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/mmwave_scan_converter.py
  src/mmwave_detection_debug.py
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
