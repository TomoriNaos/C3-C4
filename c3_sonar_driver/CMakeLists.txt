cmake_minimum_required(VERSION 3.8)
project(c3_sonar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

set(msg_files
  "msg/SonarDetect.msg"
  "msg/SonarStatus.msg"
)

set(srv_files
  "srv/SetSonarLifecycle.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

add_executable(communicate_node src/bridge_node/communicate_node.cpp)
ament_target_dependencies(communicate_node rclcpp std_msgs)

add_executable(dave_sonar_adapter_node src/bridge_node/dave_sonar_adapter_node.cpp)
ament_target_dependencies(dave_sonar_adapter_node rclcpp sensor_msgs geometry_msgs)

add_executable(sonar_main_controller_node src/inner_node/sonar_main_controller_node.cpp)
ament_target_dependencies(sonar_main_controller_node rclcpp lifecycle_msgs)

rosidl_get_typesupport_target(c3_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(communicate_node ${c3_typesupport_target})
target_link_libraries(dave_sonar_adapter_node ${c3_typesupport_target})
target_link_libraries(sonar_main_controller_node ${c3_typesupport_target})

install(TARGETS
  communicate_node
  dave_sonar_adapter_node
  sonar_main_controller_node
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
