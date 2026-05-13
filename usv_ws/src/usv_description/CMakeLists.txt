cmake_minimum_required(VERSION 3.8)
project(usv_description)

find_package(ament_cmake REQUIRED)

# 安装 urdf、meshes、rviz 等所有需要共享的文件
install(DIRECTORY
  urdf
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()