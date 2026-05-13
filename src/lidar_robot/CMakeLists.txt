cmake_minimum_required(VERSION 3.8)
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

add_executable(lidar_depth_node src/lidar_depth_node.cpp)
ament_target_dependencies(lidar_depth_node rclcpp sensor_msgs std_msgs)

add_executable(mmwave_radar_node src/mmwave_radar_node.cpp)
ament_target_dependencies(mmwave_radar_node rclcpp sensor_msgs std_msgs geometry_msgs)

install(TARGETS
  lidar_depth_node
  mmwave_radar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
