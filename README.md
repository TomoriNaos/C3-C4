# Depth Image to PointCloud2

这是一个 ROS 2 Humble 工作区，用于处理 RGB-D 图像，支持传统暗通道去雾和深度学习去雾两种模式，并发布彩色 `sensor_msgs/msg/PointCloud2` 点云。需要仿真输入时，可通过 launch 参数启动 Gazebo 雾天场景。

## 功能

- `fog_depth_camera.world`：简化的雾天深度相机测试场景。
- `ocean_fog.world`：海雾环境，包含水面、航标和深度相机。
- `depth_camera_dehaze.launch.py`：启动点云处理节点；设置 `gazebo:=true` 时启动 Gazebo 和深度相机仿真，设置 `rviz:=true` 时启动 RViz，设置 `dl:=true` 时启用深度学习去雾。
- `depth_image_to_pointcloud2`：C++ 节点，使用传统暗通道 RGB-D 去雾算法，订阅 Gazebo 相机的 RGB、Depth、CameraInfo，发布去雾后的 `/camera/points`。
- `depth_image_to_pointcloud2_dl`：Python 节点，加载 `models/best.pt` 深度学习模型做 RGB 去雾，订阅和发布的话题、消息类型与 C++ 节点一致。

## 目录结构

```text
src/depth_image_to_pointcloud2/
  depth_image_to_pointcloud2/           Python 深度学习模型定义
  include/                              C++ 节点和传统去雾头文件
  src/                                  C++ 节点和传统去雾实现
  scripts/depth_image_to_pointcloud2_dl_node.py
                                        深度学习去雾 ROS 2 节点
  models/best.pt                        深度学习去雾模型
  launch/depth_camera_dehaze.launch.py  节点启动文件，可选启动 Gazebo 和 RViz
  rviz/pointcloud.rviz                  点云显示配置
  worlds/fog_depth_camera.world         默认简化测试场景
  worlds/ocean_fog.world                海雾场景
```

## 依赖

基础构建和 C++ 节点依赖 ROS 2 Humble、OpenCV、`rclcpp`、`sensor_msgs`、`cv_bridge`。

深度学习去雾模式还需要 Python 版 ROS 2 运行库、NumPy、OpenCV 和 PyTorch。容器 `test` 中已安装 CPU 版 PyTorch：

```bash
python3 -m pip install torch --index-url https://download.pytorch.org/whl/cpu
```

该命令同时安装了 PyTorch 依赖：`typing-extensions`、`sympy`、`networkx`、`fsspec`、`mpmath`。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select depth_image_to_pointcloud2 --symlink-install
source install/setup.bash
```

## 运行

默认不启动 Gazebo 和 RViz，只启动点云处理节点：

```bash
ros2 launch depth_image_to_pointcloud2 depth_camera_dehaze.launch.py
```

启用深度学习去雾模型：

```bash
ros2 launch depth_image_to_pointcloud2 depth_camera_dehaze.launch.py dl:=true
```

启动点云处理节点和 RViz：

```bash
ros2 launch depth_image_to_pointcloud2 depth_camera_dehaze.launch.py rviz:=true
```

启动 Gazebo 仿真，加载默认 `fog_depth_camera.world` 场景：

```bash
ros2 launch depth_image_to_pointcloud2 depth_camera_dehaze.launch.py gazebo:=true
```

将 Gazebo 深度相机 Optical Frame 点云坐标转换为 RViz/ROS Standard Frame 坐标：

```bash
ros2 launch depth_image_to_pointcloud2 depth_camera_dehaze.launch.py gazebo:=true gazebo_tran:=true
```

启动 Gazebo server，不启动 Gazebo 图形客户端：

```bash
ros2 launch depth_image_to_pointcloud2 depth_camera_dehaze.launch.py gazebo:=true gui:=false
```

## Launch 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gazebo` | `false` | 是否启动 Gazebo 仿真；为 `false` 时不启动 `gzserver` 和 `gzclient` |
| `gui` | `true` | 是否启动 `gzclient`；仅在 `gazebo:=true` 时生效 |
| `rviz` | `false` | 是否启动 RViz |
| `show_images` | `true` | 是否显示 OpenCV 图像窗口；无 `DISPLAY` 时 DL 节点会自动关闭窗口，避免影响点云发布 |
| `gazebo_tran` | `false` | 是否将点云坐标从 Gazebo/相机 Optical Frame（X 右、Y 下、Z 前）转换为 RViz/ROS Standard Frame（X 前、Y 左、Z 上）。开启后坐标映射为 `X=z, Y=-x, Z=-y` |
| `dl` | `false` | 是否启用深度学习去雾。为 `false` 时启动 C++ 传统去雾节点；为 `true` 时启动 Python 深度学习去雾节点 |

## 主要话题

| 话题 | 类型 | 来源/用途 |
| --- | --- | --- |
| `/depth_camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo 深度相机 RGB 图像 |
| `/depth_camera/depth/image_raw` | `sensor_msgs/msg/Image` | Gazebo 深度相机深度图 |
| `/depth_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | 去雾和深度补偿后的彩色点云 |

`dl:=false` 和 `dl:=true` 两种模式使用相同的话题和消息类型，便于 RViz、下游节点和 rosbag 复用。

## 去雾实现

传统去雾算法文件与节点文件分开：

- `include/depth_image_to_pointcloud2/classical_dehaze.hpp`
- `include/depth_image_to_pointcloud2/rgbd_image.hpp`
- `src/classical_dehaze.cpp`

深度学习去雾文件单独放置：

- `depth_image_to_pointcloud2/dehaze_model.py`
- `scripts/depth_image_to_pointcloud2_dl_node.py`
- `models/best.pt`

深度学习推理流程参考宿主机项目 `E:\MyProject\C4\dehaze_trainer`：加载 checkpoint，按 `base_channels` 构建 U-Net，将 BGR 图像转 RGB tensor，推理后 clamp 到 `[0, 1]`，再转回 BGR 用于点云颜色。

## Docker 测试记录

在容器 `test` 中测试：

```bash
docker exec test bash -lc 'cd /data && source /opt/ros/humble/setup.bash && colcon build --packages-select depth_image_to_pointcloud2 --symlink-install'
```

构建通过。查看 launch 参数：

```bash
docker exec test bash -lc 'cd /data && source install/setup.bash && ros2 launch depth_image_to_pointcloud2 depth_camera_dehaze.launch.py --show-args'
```

`dl` 参数正常显示。验证传统节点启动：

```bash
docker exec test bash -lc 'cd /data && source install/setup.bash && timeout 8s ros2 launch depth_image_to_pointcloud2 depth_camera_dehaze.launch.py gazebo:=false rviz:=false gui:=false'
```

验证深度学习节点加载模型并启动：

```bash
docker exec test bash -lc 'cd /data && source install/setup.bash && timeout 22s ros2 launch depth_image_to_pointcloud2 depth_camera_dehaze.launch.py dl:=true gazebo:=false rviz:=false gui:=false'
```

日志显示 `DL dehaze model loaded: .../models/best.pt on cpu`。

如果容器里已经有其他 Gazebo 进程占用默认 master 端口，可先设置独立端口：

```bash
export GAZEBO_MASTER_URI=http://127.0.0.1:11445
```

---

我测试时使用的命令（一行命令一个终端窗口）：

```shell
ros2 launch depth_image_to_pointcloud2 depth_camera_dehaze.launch.py dl:=true gazebo:=false rviz:=false gazebo_tran:=true
```

```shell
ros2 launch usv_bringup sim.launch.py rviz:=true
```