cmake_minimum_required(VERSION 3.8)
project(usv_bringup)

# Gazebo Classic on this system imports an older jsoncpp CMake package.
# Keep its policy compatibility while configuring gazebo_ros.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "Minimum policy version for third-party CMake packages")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY
  launch
  worlds
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
