cmake_minimum_required(VERSION 3.8)
project(usv_rviz_dashboard)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(perception_dashboard SHARED
  src/perception_dashboard.cpp
  include/usv_rviz_dashboard/perception_dashboard.hpp
)
target_include_directories(perception_dashboard PUBLIC include)
ament_target_dependencies(perception_dashboard
  pluginlib rclcpp rviz_common sensor_msgs std_msgs)
target_link_libraries(perception_dashboard Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS perception_dashboard DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_package()
