cmake_minimum_required(VERSION 3.8)
project(usv_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "ONNX Runtime installation root")
if(NOT ONNXRUNTIME_ROOT)
  set(ONNXRUNTIME_ROOT "$ENV{HOME}/onnxruntime-linux-x64-1.23.2")
endif()
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")
set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
if(NOT EXISTS "${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime_cxx_api.h" OR NOT EXISTS "${ONNXRUNTIME_LIBRARY}")
  message(FATAL_ERROR "ONNX Runtime not found. Set ONNXRUNTIME_ROOT to a directory containing include/ and lib/.")
endif()

set(NODE_DEPS
  builtin_interfaces
  gazebo_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  vision_msgs
  visualization_msgs
)

add_executable(gated_camera_recognizer src/gated_camera_recognizer.cpp)
target_include_directories(gated_camera_recognizer PRIVATE include ${OpenCV_INCLUDE_DIRS} ${ONNXRUNTIME_INCLUDE_DIR})
ament_target_dependencies(gated_camera_recognizer rclcpp geometry_msgs sensor_msgs std_msgs tf2_ros vision_msgs)
target_link_libraries(gated_camera_recognizer ${OpenCV_LIBS} ${ONNXRUNTIME_LIBRARY})
set_target_properties(gated_camera_recognizer PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
)

add_executable(gated_slice_fusion_recognizer src/gated_slice_fusion_recognizer.cpp)
target_include_directories(gated_slice_fusion_recognizer PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(gated_slice_fusion_recognizer rclcpp sensor_msgs std_msgs vision_msgs)
target_link_libraries(gated_slice_fusion_recognizer ${OpenCV_LIBS})

add_executable(gated_bev_detector src/gated_bev_detector.cpp)
target_include_directories(gated_bev_detector PRIVATE include)
ament_target_dependencies(gated_bev_detector rclcpp sensor_msgs std_msgs)

add_executable(dynamic_target_controller src/dynamic_target_controller.cpp)
target_include_directories(dynamic_target_controller PRIVATE include)
ament_target_dependencies(dynamic_target_controller ${NODE_DEPS})

add_executable(ais_simulator src/ais_simulator.cpp)
target_include_directories(ais_simulator PRIVATE include)
ament_target_dependencies(ais_simulator ${NODE_DEPS})

add_executable(radar_sonar_tracker src/radar_sonar_tracker.cpp)
target_include_directories(radar_sonar_tracker PRIVATE include)
ament_target_dependencies(radar_sonar_tracker ${NODE_DEPS})

add_executable(c3_multimodal_buffer_fusion src/c3_multimodal_buffer_fusion.cpp)
target_include_directories(c3_multimodal_buffer_fusion PRIVATE include)
ament_target_dependencies(c3_multimodal_buffer_fusion ${NODE_DEPS})

add_executable(uav_patrol_controller src/uav_patrol_controller.cpp)
target_include_directories(uav_patrol_controller PRIVATE include)
ament_target_dependencies(uav_patrol_controller ${NODE_DEPS})

add_executable(wave_buoyancy_node src/wave_buoyancy_node.cpp)
target_include_directories(wave_buoyancy_node PRIVATE include)
ament_target_dependencies(wave_buoyancy_node ${NODE_DEPS})

add_executable(usv_target_follower src/usv_target_follower.cpp)
target_include_directories(usv_target_follower PRIVATE include)
ament_target_dependencies(usv_target_follower ${NODE_DEPS})

add_executable(tracking_evaluator src/tracking_evaluator.cpp)
target_include_directories(tracking_evaluator PRIVATE include)
ament_target_dependencies(tracking_evaluator ${NODE_DEPS})

install(TARGETS
  gated_camera_recognizer
  gated_slice_fusion_recognizer
  gated_bev_detector
  dynamic_target_controller
  ais_simulator
  radar_sonar_tracker
  c3_multimodal_buffer_fusion
  uav_patrol_controller
  wave_buoyancy_node
  usv_target_follower
  tracking_evaluator
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mmwave_scan_converter.py
  scripts/mmwave_detection_debug.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
