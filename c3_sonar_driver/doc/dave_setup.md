# DAVE 集成指南（C3 声纳）

## 1. 范围
本仓库（`c3_sonar_driver`）基于 ROS2 Humble。
DAVE 多波束插件通常构建在 ROS1 Noetic + Gazebo Classic 环境中。

推荐架构：
- DAVE/Gazebo 端：发布多波束点云
- ROS2 端（`c3_sonar_driver`）：`dave_sonar_adapter_node` 将点云转换为 `/sonar/detect` 话题

## 2. 准备 DAVE 源代码仓库

```bash
cd /data/code/c3_sonar
./tools/setup_dave_env.sh
```

如果因网络超时导致 `nps_uw_multibeam_sonar` 克隆失败，请重新运行脚本。

## 3. 构建插件（独立工作空间）
请在 ROS1 Noetic + Gazebo11 环境（原生系统或 Docker）中进行构建。

### 3.1 使用仓库内 Docker（推荐）

```bash
cd /home/wanan/Code/code/c3_sonar

# 1) 准备 DAVE 源码到宿主机工作空间（默认 /data/code/dave_ws，可改 DAVE_WS）
DAVE_WS=/home/wanan/Code/code/dave_ws ./tools/setup_dave_env.sh

# 2) 启动 noetic builder 容器
export DAVE_WS_HOST=/home/wanan/Code/code/dave_ws
export C3_SONAR_HOST=/home/wanan/Code/code/c3_sonar
./tools/docker/dave_noetic/scripts/run_builder.sh
```

进入容器后执行：

```bash
build_multibeam.sh
```

预期的插件库文件：
- `libnps_multibeam_sonar_ray_ros_plugin.so`
- `libgazebo_ros_velodyne_gpu_laser.so`

## 4. 运行 ROS2 端

```bash
cd /home/wanan/Code/code/c3_sonar
colcon build --packages-select c3_sonar_driver
source install/setup.bash
./tools/run_c3_sonar_with_dave.sh
```

## 5. 验证话题

```bash
ros2 topic list | rg sonar
ros2 topic echo /sonar/detect
ros2 topic echo /sonar_link/ship_tx/detect
```

## 6. 当前 URDF 挂钩
`urdf/sonar.urdf.xacro` 中已通过 `enable_dave_plugin` 标志包含 DAVE 插件模板。
