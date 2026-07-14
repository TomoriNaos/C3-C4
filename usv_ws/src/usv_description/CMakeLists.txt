cmake_minimum_required(VERSION 3.8)
project(usv_description)

find_package(ament_cmake REQUIRED)

# The current vessel is defined entirely in Xacro primitives and has no mesh assets.
install(DIRECTORY
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
