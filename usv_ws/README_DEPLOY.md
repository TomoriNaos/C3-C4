把下面内容直接作为 `README.md` 发给对方即可。

```md
# USV 多模态仿真项目部署说明

本项目用于 Gazebo Classic 中的无人艇多模态感知仿真，包含相机、深度相机、毫米波雷达、声呐、无人机仿真、融合、追踪和 RViz2 面板。

本压缩包不包含 `third_party/`、`build/`、`install/`、`log/`、`data/`，这些目录不影响源码完整性，可按本文重新生成。

## 1. 运行环境

必须使用：

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- 能联网的终端
- 建议内存至少 8 GB

将项目文件夹放到家目录下，并命名为 `usv_ws`：

```bash
cd ~
ls usv_ws/src
```

正常情况下，应能看到：

```text
c3_sonar_driver
depth_image_to_pointcloud2
usv_bringup
usv_description
usv_perception
usv_rviz_dashboard
```

确认模型文件也存在：

```bash
ls -lh ~/usv_ws/src/usv_bringup/models/best.onnx
ls -lh ~/usv_ws/src/usv_bringup/models/best1.onnx
```

如果这两个文件不存在，请联系项目发送者补发 `src/usv_bringup/models/`。

## 2. 安装 ROS 2 Humble

如果电脑已经安装 ROS 2 Humble，可跳过本节。

```bash
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

ROS 2 Humble 的官方安装说明见 [ROS 2 Humble 文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)。

## 3. 安装项目依赖

打开终端，执行：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-numpy \
  libopencv-dev \
  qtbase5-dev \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-robot-state-publisher
```

初始化 `rosdep`。若提示已经初始化，属于正常现象：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

## 4. 下载 ONNX Runtime

项目需要 ONNX Runtime 执行相机识别模型。默认 Gazebo 仿真不需要 PX4、Micro-XRCE-DDS，也不需要 `px4_msgs`。

```bash
cd ~/usv_ws
mkdir -p third_party
cd third_party

wget https://github.com/microsoft/onnxruntime/releases/download/v1.24.4/onnxruntime-linux-x64-1.24.4.tgz
tar -xzf onnxruntime-linux-x64-1.24.4.tgz
mv onnxruntime-linux-x64-1.24.4 onnxruntime

cd onnxruntime/lib
ln -sf libonnxruntime.so.1.24.4 libonnxruntime.so.1
ln -sf libonnxruntime.so.1.24.4 libonnxruntime.so
```

检查是否成功：

```bash
ls ~/usv_ws/third_party/onnxruntime/include/onnxruntime/core/session/onnxruntime_cxx_api.h
ls ~/usv_ws/third_party/onnxruntime/lib/libonnxruntime.so
```

项目固定使用 ONNX Runtime `1.24.4`。可从 [官方 v1.24.4 发布页](https://github.com/microsoft/onnxruntime/releases/tag/v1.24.4) 获取该版本。

## 5. 编译项目

```bash
cd ~/usv_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --base-paths src \
  --executor sequential \
  --parallel-workers 1
```

`--base-paths src` 很重要：它只编译项目源码，不会错误编译以后可能下载到 `third_party/` 的其他工程。

编译成功后执行：

```bash
source ~/usv_ws/install/setup.bash
```

## 6. 启动仿真

```bash
cd ~/usv_ws
source /opt/ros/humble/setup.bash
source ~/usv_ws/install/setup.bash

ros2 launch usv_bringup sim.launch.py
```

启动后会打开 Gazebo Classic 和 RViz2。RViz2 中默认加载感知仪表盘、图像、点云、热力图与评估指标。

如果电脑没有图形界面，或通过 SSH 连接服务器：

```bash
ros2 launch usv_bringup sim.launch.py gui:=false rviz:=false
```

## 7. 常见问题

| 问题 | 处理方法 |
| --- | --- |
| `Package 'usv_bringup' not found` | 先完成第 5 节编译，并执行 `source ~/usv_ws/install/setup.bash`。 |
| `ONNX Runtime not found` | 检查第 4 节的 `third_party/onnxruntime` 路径和软链接。 |
| `libonnxruntime.so.1` 找不到 | 重新执行第 4 节最后两条 `ln -sf` 命令，再重新编译。 |
| Gazebo 或 RViz2 无法打开 | 确认本机有桌面环境；SSH 环境请使用 `gui:=false rviz:=false`。 |
| RViz2 出现 `libGL error: failed to create drawable` | 当前启动文件已给 RViz2 单独启用软件 OpenGL；重新编译并 `source install/setup.bash` 后再启动。 |
| RViz2 出现 `inotify_add_watch ... No space left on device` | 这是系统 inotify 监听数不足，通常不影响仿真；可执行 `echo fs.inotify.max_user_watches=524288 | sudo tee /etc/sysctl.d/99-usv-inotify.conf && sudo sysctl --system`。 |
| 编译失败后想重新编译 | 删除 `build/ install/ log/` 后，从第 5 节重新执行。 |
| 是否需要 PX4 | 默认演示不需要。当前 `sim.launch.py` 使用 Gazebo Classic 仿真飞行层；如果以后要启动 `px4_offboard.launch.py`，再安装或克隆官方 `px4_msgs`，有该包时会自动编译 `px4_offboard_bridge`。 |

## 8. 不需要发送的目录

以下目录都可以删除或不发送：

```text
build/
install/
log/
data/
third_party/
```

其中 `build/`、`install/`、`log/` 会在编译时自动生成；`data/` 是可选数据集输出目录。
```

关键点是编译命令使用 `colcon build --base-paths src`，这样即使对方以后下载了 PX4 等代码到 `third_party/`，也不会被误编译。
