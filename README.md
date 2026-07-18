# C3 USV 多模态感知与协同追踪仿真

本仓库是基于 **ROS 2 Humble + Gazebo Classic** 的无人船（USV）与无人机（UAV）协同感知、定位和追踪演示系统。系统包含船载毫米波雷达、声呐、普通/门控相机、深度相机、模拟 AIS、UAV 侦察、YOLO 视觉识别、多模态点云融合、热力图巡航与目标跟踪。

> 本 README 面向第一次拿到仓库的新用户。仓库不包含 `build/`、`install/`、`log/`、`data/` 和 `third_party/`，这些目录均应由使用者在本机生成或下载。

## 1. 平台要求

推荐环境：

| 项目 | 推荐版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 仿真器 | Gazebo Classic 11（通过 `gazebo_ros`） |
| 编译器 | GCC 11 或兼容版本 |
| CMake | 3.22 或更高版本 |
| Python | Python 3.10 |
| 推理运行时 | ONNX Runtime 1.24.4 |

当前演示使用 Gazebo Classic。**不需要**安装 PX4-Autopilot、Micro-XRCE-DDS-Agent 或 Gazebo Harmonic 才能编译和运行本仓库的默认仿真；它们仅用于后续真实 PX4 接口扩展。

## 2. 获取代码

```bash
git clone <你的 GitHub 仓库地址> usv_ws
cd usv_ws
```

以下命令假设当前目录就是工作空间根目录，即包含 `src/` 的目录。

## 3. 安装系统依赖

先安装 ROS 2 Humble Desktop。若尚未安装，请按 [ROS 2 Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 完成安装并重新打开终端。

安装本项目常用的编译、Gazebo、RViz、视觉依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git curl wget \
  python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool \
  libopencv-dev qtbase5-dev \
  ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
  ros-humble-xacro ros-humble-vision-msgs ros-humble-cv-bridge \
  ros-humble-tf2-ros ros-humble-rviz2
```

首次使用 `rosdep` 时执行一次：

```bash
sudo rosdep init
rosdep update
```

然后在工作空间根目录安装 ROS 包依赖：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

如果 `sudo rosdep init` 提示已经初始化，直接继续执行 `rosdep update` 即可。

## 4. 安装 ONNX Runtime（必须）

门控相机、普通/深度相机 YOLO 识别和神经网络热力图都依赖 ONNX Runtime。项目默认从以下目录查找它：

```text
third_party/onnxruntime/include/
third_party/onnxruntime/lib/
```

在工作空间根目录执行：

```bash
mkdir -p third_party
cd third_party
wget https://github.com/microsoft/onnxruntime/releases/download/v1.24.4/onnxruntime-linux-x64-1.24.4.tgz
tar -xzf onnxruntime-linux-x64-1.24.4.tgz
mv onnxruntime-linux-x64-1.24.4 onnxruntime
cd ..
```

完成后应能看到：

```bash
test -f third_party/onnxruntime/include/onnxruntime_cxx_api.h && echo "ONNX Runtime headers: OK"
test -f third_party/onnxruntime/lib/libonnxruntime.so && echo "ONNX Runtime library: OK"
```

如果 ONNX Runtime 安装在其他位置，编译时可指定根目录：

```bash
ONNXRUNTIME_ROOT=/你的/onnxruntime/目录 \
colcon build --executor sequential --parallel-workers 1 --symlink-install
```

### CUDA 可选加速

默认 CPU 版 ONNX Runtime 已可运行。若需要 `neural_execution_provider:=cuda`，请自行下载与本机 NVIDIA 驱动、CUDA 版本匹配的 **ONNX Runtime GPU 1.24.4**，并同样解压/重命名为 `third_party/onnxruntime`。不确定 CUDA 兼容性时，请使用 `neural_execution_provider:=auto` 或 `cpu`。

## 5. 模型文件检查（上传仓库时必须做）

模型应位于 `src/usv_bringup/models/`：

```text
best.onnx
best1.onnx
camera.onnx
gated_camera.onnx
plane.onnx
plane_gated.onnx
pos_confidence_best.onnx
```

其中 `camera.onnx`、`gated_camera.onnx`、`plane.onnx`、`plane_gated.onnx`、`pos_confidence_best.onnx` 是当前未跟踪文件。上传 GitHub 前，仓库维护者必须将它们加入提交；否则其他人无法得到完整的 YOLO 与神经网络热力图能力：

```bash
git add src/usv_bringup/models/camera.onnx \
        src/usv_bringup/models/gated_camera.onnx \
        src/usv_bringup/models/plane.onnx \
        src/usv_bringup/models/plane_gated.onnx \
        src/usv_bringup/models/pos_confidence_best.onnx
```

GitHub 单文件限制为 100 MB。上述模型均应保持小于该限制；若未来模型超过限制，请改用 Git LFS 或 GitHub Release，并在这里提供下载脚本。

## 6. 编译

建议不要在 Conda 环境中编译或运行 ROS/Gazebo。若终端提示符前有 `(base)` 或其他 Conda 环境，请先退出：

```bash
conda deactivate
```

然后在工作空间根目录执行：

```bash
cd /路径/usv_ws
source /opt/ros/humble/setup.bash
colcon build --executor sequential --parallel-workers 1 --symlink-install
source install/setup.bash
```

`--executor sequential --parallel-workers 1` 会减少内存占用，首次编译较慢但更稳定。之后只改动感知和启动代码时，可使用：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select usv_perception usv_rviz_dashboard usv_bringup \
  --executor sequential --parallel-workers 1 --symlink-install
source install/setup.bash
```

每次新开终端，都必须重新执行：

```bash
source /opt/ros/humble/setup.bash
source /路径/usv_ws/install/setup.bash
```

## 7. 启动仿真
heatmap_mode` 可选值：

| 参数 | 作用 |
| --- | --- |
| `heatmap_mode:=vote` | 规则投票热力图，不运行位置置信度网络 |
| `heatmap_mode:=neural` | 使用 `pos_confidence_best.onnx` 生成神经网络热力图 |
| `neural_execution_provider:=auto` | 优先尝试 CUDA，不可用时回退 CPU，推荐默认使用 |
| `neural_execution_provider:=cpu` | 强制使用 CPU ONNX Runtime |
| `neural_execution_provider:=cuda` | 强制使用 CUDA，要求 GPU 版 ONNX Runtime 和 CUDA 环境正确 |

### 7.1 正常传感器模式

正常模式不使用 Gazebo 目标真值来控制无人船。无人船和 UAV 依靠视觉 YOLO、AIS、雷达/声呐融合及全局 tracker 进行巡航和追踪：

```bash
cd /路径/usv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch usv_bringup sim.launch.py \
  heatmap_mode:=neural \
  neural_execution_provider:=auto \
  neural_intra_op_threads:=4 \
  demo_mode:=false



### 7.2 稳定演示模式

演示模式仅用于展示稳定的跟随效果。它允许无人船跟随器读取 Gazebo 中配置的目标真值；传感器、视觉识别、热力图和 tracker 仍会照常运行，但不要将此模式当作真实感知评估结果。

```bash
ros2 launch usv_bringup sim.launch.py \
  heatmap_mode:=neural \
  neural_execution_provider:=auto \
  demo_mode:=true
```

### 7.3 无界面运行

在远程服务器或性能不足的机器上，可关闭 Gazebo 图形界面和 RViz：

```bash
ros2 launch usv_bringup sim.launch.py gui:=false rviz:=false
```

## 8. 系统运行后应看到什么

启动完成后：

1. Gazebo Classic 显示海面、主船、目标船和其他动态物体。
2. RViz2 显示点云、热力图、识别/跟踪结果和多模态仪表盘。
3. 仪表盘的图像区采用可拖动分隔条布局，可用鼠标拖动边框调整每个图像窗口大小。
4. UAV 会根据热力图峰值和视觉检测执行侦察；主船在正常模式下仅依据 tracker 中确认的目标进行追踪。

## 9. 常用诊断命令

另开一个已 source 工作空间的终端：

```bash
cd /路径/usv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

查看所有相关话题：

```bash
ros2 topic list | grep -E 'tracked|heatmap|radar|sonar|ais|uav'
```

查看 tracker 输出：

```bash
ros2 topic echo /tracked_objects_text
```

查看 AIS 模拟目标：

```bash
ros2 topic echo /ais/targets
```

检查雷达和声呐点云发布频率：

```bash
ros2 topic hz /c3/buffer/radar_cloud
ros2 topic hz /c3/buffer/sonar_cloud
```

查看 UAV 状态：

```bash
ros2 topic echo /uav/flight_status
```

话题名可能随配置调整；如某个话题不存在，先用 `ros2 topic list` 查找实际名称。

## 10. 常见问题

### `Package 'usv_bringup' not found`

说明当前终端没有加载工作空间：

```bash
source /opt/ros/humble/setup.bash
source /路径/usv_ws/install/setup.bash
```

若 `install/` 不存在，请先完成第 6 节编译。

### 编译提示找不到 `onnxruntime_cxx_api.h` 或 `libonnxruntime.so`

说明第 4 节未完成，或目录结构不正确。确认：

```bash
ls third_party/onnxruntime/include/onnxruntime_cxx_api.h
ls third_party/onnxruntime/lib/libonnxruntime.so
```

若 ONNX Runtime 在其他目录，使用 `ONNXRUNTIME_ROOT=/实际目录` 重新编译。

### 启动时提示某个 Python 节点 `executable ... not found`

通常是旧的 `install/` 目录未更新。重新编译并重新 source：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select usv_perception usv_bringup \
  --executor sequential --parallel-workers 1 --symlink-install
source install/setup.bash
```

### RViz 提示无法加载 `usv_rviz_dashboard/PerceptionDashboard`

重新编译仪表盘并关闭后重新打开 RViz：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select usv_rviz_dashboard usv_bringup \
  --executor sequential --parallel-workers 1 --symlink-install
source install/setup.bash
```

### Gazebo、`spawn_entity.py` 或 Python 包报错，且正在使用 Conda

退出 Conda 后重新 source ROS 环境：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source /路径/usv_ws/install/setup.bash
```

### CUDA 推理失败

先改用通用回退模式确认系统可运行：

```bash
ros2 launch usv_bringup sim.launch.py \
  heatmap_mode:=neural neural_execution_provider:=auto
```

若仍有问题，强制 CPU：

```bash
ros2 launch usv_bringup sim.launch.py \
  heatmap_mode:=neural neural_execution_provider:=cpu
```


## 11. 目录说明

```text
src/
  usv_bringup/          启动文件、Gazebo 世界、运行配置、ONNX 模型
  usv_description/      主船 URDF/Xacro 和 RViz 配置
  usv_perception/       雷达、声呐、相机、融合、热力图、tracker、UAV/主船控制
  usv_rviz_dashboard/   RViz 多模态可视化面板
  c3_sonar_driver/      C3 声呐消息与驱动接口
  depth_image_to_pointcloud2/ 深度图转点云工具
  px4_msgs/             本仓库随附的 PX4 ROS 消息定义
third_party/            本地外部运行时，默认只需 ONNX Runtime，不上传
build/ install/ log/    colcon 生成目录，不上传
data/                   本地采集数据集，不上传
```

## 12. 许可证与第三方组件

各 ROS 包和第三方模型/运行时可能具有各自许可证。发布或商用前，请分别核对 ROS 2、Gazebo、ONNX Runtime、OpenCV、YOLO 模型及本仓库源码的许可证与数据来源。
