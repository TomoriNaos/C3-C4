# USV Fog Simulation Project

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11

## 安装依赖

```bash
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2
```

## 编译

```bash
cd ~/usv_ws
colcon build --symlink-install
source install/setup.bash
```

## 启动大雾海面仿真

```bash
ros2 launch usv_bringup sim.launch.py
```

无图形界面启动：

```bash
ros2 launch usv_bringup sim.launch.py gui:=false rviz:=false
```

## 主要话题

- 雷达：`/radar/scan`
- 声呐：`/sonar/range`
- 深度相机彩色图：`/depth_camera/image_raw`
- 深度相机深度图：`/depth_camera/depth/image_raw`
- 深度相机点云：`/depth_camera/points`

## 目录说明

- `src/usv_description/urdf/wamv_base.urdf.xacro`：船体、雷达、声呐、深度相机模型源文件。
- `src/usv_description/urdf/wamv.urdf`：由 xacro 生成的静态 URDF。
- `src/usv_description/rviz/default.rviz`：RViz 默认配置。
- `src/usv_bringup/launch/sim.launch.py`：Gazebo、robot_state_publisher、RViz 启动入口。
- `src/usv_bringup/worlds/ocean_fog.world`：大雾海面 Gazebo world。

当前效果：

![USV simulation](image.png)
