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

## ONNX 识别加速

本机已验证 NVIDIA GPU 可用。检查命令：

```bash
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

当前工程的门控识别节点已改为 C++，通过 OpenCV DNN 加载 ONNX 模型。默认模型放在 `src/usv_bringup/models/best.onnx`，会随 `usv_bringup` 一起安装。`yolo_device: auto` 会优先尝试 OpenCV CUDA DNN；如果本机 OpenCV 没有 CUDA DNN 支持，会自动退回 CPU。

## 启动大雾海面仿真

```bash
ros2 launch usv_bringup sim.launch.py
```

无图形界面启动：

```bash
ros2 launch usv_bringup sim.launch.py gui:=false rviz:=false
```

如果 Gazebo 图形界面很卡，优先这样跑：只开 Gazebo 后端和 RViz。

```bash
ros2 launch usv_bringup sim.launch.py gui:=false rviz:=true
```

如果暂时不需要无人机远距离 ALS：

```bash
ros2 launch usv_bringup sim.launch.py uav:=false
```

## 主要话题

- 毫米波雷达：`/mmwave_radar/scan`
- 窄束声呐：`/sonar/scan`
- 深度相机彩色图：`/depth_camera/image_raw`
- 深度相机深度图：`/depth_camera/depth/image_raw`
- 深度相机点云：`/depth_camera/points`
- ALS 点云：`/als/points`
- 门控相机原始图：`/gated_camera/image_raw`
- 门控相机近距切片：`/gated_camera/slice_near`
- 门控相机中距切片：`/gated_camera/slice_mid`
- 门控相机远距切片：`/gated_camera/slice_far`
- 门控相机组合识别图：`/gated_camera/range_view`
- 门控相机识别结果：`/gated_camera/detections`
- 无人机远距离相机：`/uav/front_camera/image_raw`
- 无人机 ALS 点云：`/uav/als/points`
- 无人机远距离识别图：`/uav/long_range/range_view`
- 无人机远距离识别结果：`/uav/long_range/detections`
- 雷达声呐动态跟踪结果：`/tracked_objects`
- 跟踪文字状态：`/tracked_objects_text`
- 动态仿真目标标记：`/simulated_targets`
- 波浪浮力状态：`/wave_buoyancy/status`

## 采集 ROS 相机话题图片

不要截 Gazebo GUI 窗口，直接保存 ROS 图像话题。

保存船载门控融合图：

```bash
source install/setup.bash
ros2 run usv_perception ros_image_recorder --ros-args \
  -p image_topic:=/gated_camera/range_view \
  -p output_dir:=~/usv_captures/gated \
  -p prefix:=gated \
  -p every_n:=5 \
  -p max_images:=300
```

保存无人机远距离图：

```bash
ros2 run usv_perception ros_image_recorder --ros-args \
  -p image_topic:=/uav/long_range/range_view \
  -p output_dir:=~/usv_captures/uav \
  -p prefix:=uav \
  -p every_n:=3 \
  -p max_images:=300
```

这些图片可以作为后续重新训练或测试识别模型的数据源。

## 当前功能

- 大雾海峡环境：Gazebo world 中配置线性雾效、低对比度光照、海面和航标，目标外观已从方块代理加强为更像船、渔船、浮标和漂浮物的组合模型。
- 波浪和浮力近似：`wave_buoyancy_node` 对无人船施加周期性力/力矩，使船体产生海况扰动。
- 动态目标：`dynamic_target_controller` 通过 `/set_entity_state` 按轨迹直接移动小船、浮标和漂浮障碍物，生成可被相机、雷达和声呐观察到的运动目标。
- 动态跟踪：`radar_sonar_tracker` 从毫米波雷达和声呐扫描中聚类目标，并输出速度估计和 RViz Marker。
- 门控相机识别：`gated_camera_recognizer` 根据相机图像和深度图生成近/中/远距离门控切片；未配置 ONNX 权重时使用轮廓检测 fallback，配置 `.onnx` 权重后通过 OpenCV DNN 调用模型。
- 无人机 ALS 远距离识别：world 中加入 `als_uav`，发布远距离相机和机载 ALS 点云，`uav_patrol_controller` 控制无人机环绕巡航，`uav_long_range_recognizer` 输出远距离识别结果。
- RViz 工作台：默认配置显示 TF、船体、毫米波雷达、声呐、船载 ALS、无人机 ALS、动态目标 Marker、跟踪结果和门控相机图像。

## 目录说明

- `src/usv_description/urdf/wamv_base.urdf.xacro`：船体、雷达、声呐、深度相机模型源文件。
- `src/usv_description/urdf/wamv.urdf`：由 xacro 生成的静态 URDF。
- `src/usv_description/rviz/default.rviz`：RViz 默认配置。
- `src/usv_bringup/launch/sim.launch.py`：Gazebo、robot_state_publisher、RViz 启动入口。
- `src/usv_bringup/worlds/ocean_fog.world`：大雾海面 Gazebo world。
- `src/usv_bringup/config/perception.yaml`：感知、跟踪、门控相机和动态目标参数。
- `src/usv_bringup/models/best.onnx`：默认识别模型，随包安装。
- `src/usv_perception/`：多模态感知节点；波浪浮力、动态目标、雷达声呐跟踪、门控相机识别、无人机巡航和 ROS 图像录制均为 C++。

## 更换 ONNX 模型

默认情况下直接替换这个文件即可：

```text
src/usv_bringup/models/best.onnx
```

然后重新编译安装：

```bash
colcon build --symlink-install
source install/setup.bash
```

如果临时测试另一个模型，不想覆盖默认文件，可以启动时传参：

```bash
ros2 launch usv_bringup sim.launch.py yolo_model_path:=/path/to/your/model.onnx
```

船载门控相机和无人机远距离识别默认共用这个 launch 参数。若以后要分别使用两个模型，改这里：

```text
src/usv_bringup/launch/sim.launch.py
```

`src/usv_bringup/config/perception.yaml` 里仍保留 `yolo_model_path` 字段，但默认留空，由 launch 文件按包内模型路径覆盖。

GPU 可用时保持：

```yaml
yolo_device: auto
half_precision: true
```

程序会优先尝试 OpenCV CUDA DNN；如果当前 OpenCV 不支持 CUDA DNN，会退回 CPU。

当前效果：

![USV simulation](image.png)
