# USV Fog Simulation Project

本工程用于琼州海峡大雾低能见度场景下的无人船多模态感知仿真。系统在 ROS 2 Humble + Gazebo Classic 中融合普通相机、门控相机、深度相机、毫米波雷达、声呐、无人机观测和 AIS，实现海上目标识别、融合跟踪、跟随与避障评估。

## 从零部署

最终上传/交付时只需要保留两个目录：

```text
project_root/
├── usv_ws/    # ROS 2 仿真、感知、融合、跟踪代码
└── yolo/      # 普通相机和门控相机 YOLO 数据集、训练结果
```

下面命令默认你站在 `project_root`，或者把 `USV_WS`、`YOLO_DIR` 换成自己的实际目录：

```bash
export USV_WS=$PWD/usv_ws
export YOLO_DIR=$PWD/yolo
```

推荐环境：

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Classic 11
Python 3.10
ONNX Runtime 1.23.2
```

安装基础依赖：

```bash
sudo apt update
sudo apt install -y \
  git wget python3-pip python3-colcon-common-extensions python3-rosdep \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-rviz2 \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs
```

初始化 `rosdep`：

```bash
sudo rosdep init || true
rosdep update
```

安装 ONNX Runtime。推荐放在用户目录，例如 `~/onnxruntime-linux-x64-1.23.2`：

```bash
cd ~
wget https://github.com/microsoft/onnxruntime/releases/download/v1.23.2/onnxruntime-linux-x64-1.23.2.tgz
tar -xzf onnxruntime-linux-x64-1.23.2.tgz
```

如果 ONNX Runtime 在其他目录，编译时指定：

```bash
colcon build --symlink-install --cmake-args -DONNXRUNTIME_ROOT=/your/onnxruntime/path
```

### VRX / WAM-V 资源

本工程内置了简化 WAM-V URDF 和自定义海上目标，主仿真不直接 `find_package(vrx)`；但新机器经常缺 WAM-V/VRX/Gazebo 海上模型资源，建议部署时下载 VRX。两种方式二选一。

方式 1：完整编译 VRX 工作区：

```bash
mkdir -p ~/vrx_ws/src
cd ~/vrx_ws/src
git clone -b humble https://github.com/osrf/vrx.git
cd ~/vrx_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

方式 2：只下载 VRX Gazebo models：

```bash
mkdir -p ~/.gazebo/models
cd /tmp
wget -q https://github.com/osrf/vrx/archive/humble.tar.gz -O vrx.tar.gz
mkdir -p vrx_models
tar -xzf vrx.tar.gz -C vrx_models --strip-components=2 vrx-humble/models
cp -r vrx_models/* ~/.gazebo/models/
```

如果编译了 VRX，新终端按顺序 source：

```bash
source /opt/ros/humble/setup.bash
source ~/vrx_ws/install/setup.bash
source ${USV_WS}/install/setup.bash
```

获取并编译本工程：

```bash
cd ${USV_WS}
rosdep install --from-paths src --ignore-src -r -y
./scripts/build_clean_env.sh
source install/setup.bash
```

## 快速使用

编译：

```bash
cd ${USV_WS}
./scripts/build_clean_env.sh
source install/setup.bash
```

启动：

```bash
ros2 launch usv_bringup sim.launch.py
```

如果 Gazebo 报 multicast 网卡错误，用本地通信启动：

```bash
./scripts/launch_sim_localhost.sh gui:=false rviz:=true
```

常用参数：

```bash
ros2 launch usv_bringup sim.launch.py target_model:=moving_vessel
ros2 launch usv_bringup sim.launch.py target_model:=small_fishing_boat
ros2 launch usv_bringup sim.launch.py c3_multimodal_fusion:=true
ros2 launch usv_bringup sim.launch.py c3_mmwave:=true
ros2 launch usv_bringup sim.launch.py pseudocolor_gated_yolo:=true
ros2 launch usv_bringup sim.launch.py stf_gated_fusion:=false
ros2 launch usv_bringup sim.launch.py gated_bev_detection:=false
ros2 launch usv_bringup sim.launch.py uav:=false
ros2 launch usv_bringup sim.launch.py ais:=false
```

## 模型与数据

当前模型分工：

| 模型 | 相机类型 | 默认节点 | 实时融合阈值 |
| --- | --- | --- | --- |
| `src/usv_bringup/models/best.onnx` | 普通相机、深度相机 RGB 图像 | `gated_camera_recognizer`、`depth_camera_recognizer` | `confidence_threshold: 0.10` |
| `src/usv_bringup/models/best1.onnx` | 船载门控伪彩色、无人机门控伪彩色 | `pseudocolor_gated_camera_recognizer`、`uav_gated_camera_recognizer` | `confidence_threshold: 0.10` |

两个 ONNX 的类别顺序一致：

```text
0 buoy
1 debris_container
2 fishing_boat
3 floating_obstacle
4 platform
5 vessel
```

点云输出里的 `class_id` 已经和 ONNX 类别顺序保持一致。
所以 `usv_target_follower.follow_class_ids: [5.0, 2.0]` 表示跟随船类目标 `vessel/fishing_boat`。

YOLO 数据和训练结果目录：

```text
../yolo/vessel.v2i.yolov8                       普通相机 Roboflow 数据
../yolo/gated_camera.v3i.yolov8                  门控相机 Roboflow 数据
../yolo/runs/detect/c3_gpu_train_vessel          普通相机训练结果
../yolo/runs/detect/c3_gpu_train_gated_camera    门控相机训练结果
```

当前标注审查结果：

```text
普通相机数据 `../yolo/vessel.v2i.yolov8`
  train: 99 images, 218 boxes
  valid: 28 images, 46 boxes
  test : 15 images, 35 boxes

门控相机数据 `../yolo/gated_camera.v3i.yolov8`
  train: 118 images, 321 boxes
  valid: 34 images, 100 boxes
  test : 17 images, 26 boxes
```

`train/valid/test` 都没有发现空标签、类别越界或归一化 bbox 越界。`valid` 集类别分布：

| 数据集 | buoy | debris_container | fishing_boat | floating_obstacle | platform | vessel |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 普通相机 valid | 3 | 13 | 9 | 7 | 4 | 10 |
| 门控相机 valid | 12 | 28 | 14 | 18 | 10 | 18 |

因此当前门控检测准确率不是“只标了某一个类别/某个模型”得出的；门控数据中 6 类都有样本。

训练过程图可看：

```text
../yolo/runs/detect/c3_gpu_train_vessel/results.png
../yolo/runs/detect/c3_gpu_train_vessel/confusion_matrix_normalized.png
../yolo/runs/detect/c3_gpu_train_vessel/val_batch0_pred.jpg

../yolo/runs/detect/c3_gpu_train_gated_camera/results.png
../yolo/runs/detect/c3_gpu_train_gated_camera/confusion_matrix_normalized.png
../yolo/runs/detect/c3_gpu_train_gated_camera/val_batch0_pred.jpg
```

## 检测准确度

检测准确度不是从训练日志手抄出来的，而是用当前工程里的实际 ONNX 模型直接推理统计：

```text
普通相机:
  模型 src/usv_bringup/models/best.onnx
  数据 ../yolo/vessel.v2i.yolov8/data.yaml
  split valid

门控相机:
  模型 src/usv_bringup/models/best1.onnx
  数据 ../yolo/gated_camera.v3i.yolov8/data.yaml
  split valid
```

评估脚本是 `scripts/evaluate_yolo_onnx_dataset.py`。它读取 ONNX metadata 的类别名，并和 `data.yaml` 的类别名对齐，然后按 IoU=0.50 统计 TP、FP、FN、Precision、Recall 和 F1。

我用当前最新数据集的 `valid` split 重新评估，并按 F1 扫描置信度后选择离线评估阈值：

```text
普通相机 best.onnx:  conf=0.15
门控相机 best1.onnx: conf=0.20
```

当前检测准确度：

| 方法 | 模型 | 数据集 | conf | images | GT | pred | TP | FP | FN | Precision | Recall | F1 |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 普通相机 | `best.onnx` | `vessel.v2i.yolov8` valid | 0.15 | 28 | 46 | 46 | 33 | 13 | 13 | 0.7174 | 0.7174 | 0.7174 |
| 门控相机 | `best1.onnx` | `gated_camera.v3i.yolov8` valid | 0.20 | 34 | 100 | 80 | 55 | 25 | 45 | 0.6875 | 0.5500 | 0.6111 |

实际仿真融合时为了降低漏检，普通相机、深度相机、船载门控和无人机门控都使用 `confidence_threshold: 0.10`，并在 YOLO 空框时启用轮廓旁路作为兜底。

可视化图片：

```text
eval_outputs/current_accuracy/camera_detection_accuracy_summary.jpg
eval_outputs/current_accuracy/normal_camera_accuracy_sheet.jpg
eval_outputs/current_accuracy/gated_camera_accuracy_sheet.jpg
eval_outputs/current_accuracy/camera_threshold_sweep_summary.jpg
eval_outputs/camera_selected_visuals/camera_selected_class_metrics.png
eval_outputs/camera_selected_visuals/camera_selected_detection_examples.png
```

后两张图分别展示较稳定类别的 Precision/Recall 和验证集中的正确检测样例。它们用于直观检查模型是否识别到目标，但不替代上表的六类总体结果。

截图颜色：

```text
绿色 GT 框：被匹配
红色 GT 框：漏检
黄色预测框：正确预测
紫色预测框：误检
```

复现评估：

```bash
cd ${USV_WS}

python3 scripts/evaluate_yolo_onnx_dataset.py \
  --model src/usv_bringup/models/best.onnx \
  --data ${YOLO_DIR}/vessel.v2i.yolov8/data.yaml \
  --split val \
  --conf 0.15 \
  --save-examples 24 \
  --save-dir eval_outputs/current_accuracy/final_normal_camera_best_val_conf0p15

python3 scripts/evaluate_yolo_onnx_dataset.py \
  --model src/usv_bringup/models/best1.onnx \
  --data ${YOLO_DIR}/gated_camera.v3i.yolov8/data.yaml \
  --split val \
  --conf 0.20 \
  --save-examples 24 \
  --save-dir eval_outputs/current_accuracy/final_gated_camera_best1_val_conf0p20
```

评估脚本默认优先保存 FP/FN 较多的诊断样例；增加 `--example-order best` 可优先保存 FP/FN 较少的正确检测样例。两种排序只影响截图顺序，不改变准确率计算。

## 实时输出话题

普通检测结果：

```text
/gated_camera/detections
/gated_camera/detection_points
/gated_camera/detection_details
/gated_camera/annotated
```

门控伪彩色检测结果：

```text
/gated_camera/pseudocolor/detections
/gated_camera/pseudocolor/detection_points
/gated_camera/pseudocolor/detection_details
/gated_camera/pseudocolor/annotated
```

无人机门控检测结果：

```text
/uav/gated_camera/detections
/uav/gated_camera/detection_points
/uav/gated_camera/detection_details
/uav/gated_camera/annotated
```

深度相机 YOLO 检测结果，使用 `best.onnx`：

```text
/depth_camera/detections
/depth_camera/detection_points
/depth_camera/detection_details
/depth_camera/annotated
```

### 原始检测点云字段

普通相机、门控伪彩色、无人机门控、三切片几何识别输出 `sensor_msgs/msg/PointCloud2`。每个点 `point_step=36` 字节，字段全部是 `FLOAT32`：

| 字段 | offset | 含义 |
| --- | ---: | --- |
| `x` | 0 | 目标相对 `base_link` 的前向距离，单位 m |
| `y` | 4 | 目标相对 `base_link` 的横向距离，左正右负，单位 m |
| `z` | 8 | 目标高度，单位 m |
| `intensity` | 12 | 检测置信度，也就是 score/confidence |
| `class_id` | 16 | 类别编号，和 ONNX 类别顺序一致 |
| `bbox_cx` | 20 | 图像 bbox 中心点 x，像素坐标 |
| `bbox_cy` | 24 | 图像 bbox 中心点 y，像素坐标 |
| `bbox_w` | 28 | 图像 bbox 宽度，像素 |
| `bbox_h` | 32 | 图像 bbox 高度，像素 |

`class_id` 编号：

```text
0 buoy
1 debris_container
2 fishing_boat
3 floating_obstacle
4 platform
5 vessel
```

有 bbox 的点云：

```text
/gated_camera/detection_points
/gated_camera/pseudocolor/detection_points
/uav/gated_camera/detection_points
/depth_camera/detection_points
/gated_camera/stf_detection_points
```

`/gated_camera/bev_detection_points` 是 BEV 几何旁路，没有图像检测框，因此 `bbox_cx/bbox_cy/bbox_w/bbox_h` 固定为 `0`。

### C3 缓存融合点云字段

`c3_multimodal_buffer_fusion` 会把毫米波雷达、声呐、视觉融合模态和深度点云统一成标准点云。每个点 `point_step=40` 字节：

| 字段 | offset | 含义 |
| --- | ---: | --- |
| `x` | 0 | 目标/点相对 `base_link` 的前向距离，单位 m |
| `y` | 4 | 横向距离，左正右负，单位 m |
| `z` | 8 | 高度，单位 m |
| `intensity` | 12 | 该点置信度或模态加权强度 |
| `class_id` | 16 | 物体类别 ID；未知时为 `-1` |
| `source_id` | 20 | 来源编号 |
| `bbox_cx` | 24 | 图像 bbox 中心 x；非图像来源为 `0` |
| `bbox_cy` | 28 | 图像 bbox 中心 y；非图像来源为 `0` |
| `bbox_w` | 32 | 图像 bbox 宽度；非图像来源为 `0` |
| `bbox_h` | 36 | 图像 bbox 高度；非图像来源为 `0` |

`source_id` 编号：

```text
1  mmWave radar
2  sonar
4  depth cloud
31 normal camera YOLO
32 gated pseudo-color YOLO
33 gated three-slice geometry
34 BEV geometry
35 UAV gated camera
36 depth camera YOLO
```

C3 缓存池输出：

```text
/c3/buffer/radar_cloud
/c3/buffer/sonar_cloud
/c3/buffer/vision_cloud
/c3/buffer/depth_cloud
/c3/buffer/integrated_cloud
```

热力图、飞控目标、已确认目标和实时指标：

```text
/c3/heatmap/image
/c3/drone/goal
/c3/detected_objects
/c3/perception_metrics
/c3/perception_markers
```

新增的 `detection_details` 是 `std_msgs/String` JSON，尽量详细输出实时检测数据，包括：

```text
stamp
frame_id
output_prefix
image_topic
depth_topic
model_path
backend
detection_input
confidence_threshold
nms_threshold
dehaze_enabled
image_width / image_height
detection_count / point_count
max_model_score
detections[]:
  index
  label
  class_id
  score
  bbox:
    cx/cy/w/h     目标框中心、宽、高，像素坐标
    x1/y1/x2/y2   目标框左上角和右下角，像素坐标
points[]:
  index
  label
  class_id
  score
  x/y/z
  bbox_cx/bbox_cy/bbox_w/bbox_h
```

查看：

```bash
ros2 topic echo /gated_camera/detection_details
ros2 topic echo /gated_camera/pseudocolor/detection_details
ros2 topic echo /uav/gated_camera/detection_details
ros2 topic echo /depth_camera/detection_details
```

旧融合跟踪和评估仍然保留：

```text
/tracked_objects
/tracked_object_poses
/tracked_objects_text
/usv_follow_status
/tracking_metrics
```

## C3 多模态流程

1. 门控相机先模拟近、中、远三个距离门控切片。三个切片分别对应不同距离范围内的目标回波，用距离选通削弱大雾前向散射对远处目标的干扰。

2. 将近、中、远三个门控切片分别放入 RGB 三个通道，合成为伪彩色门控图像。船载门控相机使用 `best1.onnx` 进行 YOLO 识别，输出：

```text
/gated_camera/pseudocolor/detections
/gated_camera/pseudocolor/detection_points
/gated_camera/pseudocolor/detection_details
/gated_camera/pseudocolor/annotated
```

3. 门控 YOLO 识别到目标后，根据目标 bbox 的中心位置、目标距离估计和相机视场角，解算目标相对无人船 `base_link` 的三维坐标，并输出 `PointCloud2` 点云。点云中包含：

```text
x / y / z
intensity
class_id
bbox_cx / bbox_cy / bbox_w / bbox_h
```

4. 三切片几何识别作为门控相机的非 YOLO 旁路。`gated_slice_fusion_recognizer` 直接利用近、中、远切片的亮度和范围关系提取目标，输出：

```text
/gated_camera/stf_detection_points
```

5. 普通相机和深度相机 RGB 分支保留，二者都使用 `best.onnx` 识别目标。普通相机图像先做去雾增强，深度相机分支额外把识别结果作为 C3 语义点输入：

```text
/gated_camera/detection_points
/gated_camera/detection_details
/gated_camera/annotated
/depth_camera/detection_points
/depth_camera/detection_details
/depth_camera/annotated
```

6. BEV 几何旁路保留，用俯视投影方式提取空间聚类目标，输出：

```text
/gated_camera/bev_detection_points
```

7. `c3_multimodal_buffer_fusion` 接收四个模态的数据：毫米波雷达点云、声呐 scan 转换点云、视觉融合模态点云、深度点云。毫米波雷达由 5 个高度、4 个方向的 90 度扇区组成，共 20 路转换点云；声呐由前、右、后、左四个 90 度扇区拼成 360 度覆盖。节点内部使用滑动缓存池按时间戳对齐，分别输出四路对齐后的新点云，再拼接为 `/c3/buffer/integrated_cloud`。

```text
/c3/buffer/radar_cloud
/c3/buffer/sonar_cloud
/c3/buffer/vision_cloud
/c3/buffer/depth_cloud
/c3/buffer/integrated_cloud
```

8. 缓存融合点云会被栅格化成 1 m x 1 m 分辨率概率热力图。当前没有训练好的热力图神经网络，因此默认使用规则方法：多模态点按来源和置信度投票，已确认目标附近会被抑制，避免重复定位同一个物体。热力图最大概率点发布到 `/c3/drone/goal`，无人机优先飞往该位置做二次确认。

9. 二次确认成功后，节点会生成 `detected` 目标结构并发布到 `/c3/detected_objects`。这里的 `object_id` 是第 n 次确认出的目标计数，和 6 类物体的 `class_id` 严格分开。目标名称来自门控相机/深度辅助确认后的语义结果，位置进入 EKF 做动态预测。

10. 30 m 以外优先等待无人机门控相机确认；30 m 以内使用船载门控、普通相机和深度相机 YOLO 的语义结果，并结合深度点云几何支持。近距离不使用 BEV 作为确认来源，BEV 只参与远处热力图投票。

### 360 度覆盖方式

毫米波雷达和声呐采用“多扇区拼接”，不是单个传感器硬开超大视场：

```text
毫米波雷达:
  front/right/back/left 四个 90 度扇区
  h10m/h4m/h1p9m/h1p5m/h1m 五个高度 topic 层级
  共 20 路 /mmwave/<sector>/<height>/detections

声呐:
  /sonar/scan
  /sonar/right_scan
  /sonar/back_scan
  /sonar/left_scan
```

门控相机不默认做 360 度拼接。原因是门控/普通相机涉及相机内参、外参、畸变和 bbox 到空间点的投影，四向拼接会让 YOLO 标注、测距和视角畸变一起变复杂。当前实现采用船载前向门控相机负责近距离精识别，无人机门控相机负责远距离二次确认。

### 热力图来源

现在的 `/c3/heatmap/image` 不是神经网络推理结果，而是规则投票热力图：

```text
1. 把 /c3/buffer/integrated_cloud 投影到 base_link 前方二维网格。
2. 每个格子分辨率为 1 m x 1 m。
3. 雷达、声呐、门控、普通相机、BEV、深度点云按 source_id 赋予不同权重。
4. 检测置信度越高，投票越强。
5. 已经写入 detected 结构的目标附近会被降权，避免无人机反复飞向同一个目标。
6. 选择最大置信度格子的中心作为候选目标点。
```

如果后续训练了神经网络热力图模型，它需要读取四路缓存点云或融合点云，输出 `/c3/nn/heatmap_goal`。开启 `enable_nn_heatmap_bypass: true` 后，神经网络目标会接管规则热力图候选点。

### 无人机飞控链路

当前仿真中的无人机模型叫 `scout_uav`，机载传感器只使用门控相机。控制链路如下：

```text
/c3/heatmap/image
  -> 选最大概率坐标
  -> /c3/drone/goal 和 /mission/goal
  -> uav_patrol_controller
  -> Gazebo /set_entity_state 移动 scout_uav
  -> /uav/gated_camera/detection_points 回传二次确认结果
  -> c3_multimodal_buffer_fusion 写入 /c3/detected_objects
```

优先级：

```text
1. 有新 /c3/drone/goal 时，无人机先飞向热力图候选点。
2. 没有新候选点时，无人机围绕目标区域巡航。
3. 远距离目标大于 30 m 时，C3 融合会优先等待无人机门控相机确认。
4. 近距离目标小于等于 30 m 时，船载门控/普通相机和深度几何支持优先。
```

船载门控 YOLO 默认开启。需要关闭时：

```bash
ros2 launch usv_bringup sim.launch.py pseudocolor_gated_yolo:=false
```

需要关闭新的 C3 缓存融合层时：

```bash
ros2 launch usv_bringup sim.launch.py c3_multimodal_fusion:=false
```

### 神经网络旁路

当前热力图由规则方法生成。后续如果训练了点云热力图网络，可以让网络节点读取：

```text
/c3/buffer/radar_cloud
/c3/buffer/sonar_cloud
/c3/buffer/vision_cloud
/c3/buffer/depth_cloud
/c3/buffer/integrated_cloud
```

网络输出一个 `geometry_msgs/PoseStamped` 到：

```text
/c3/nn/heatmap_goal
```

然后启动：

```bash
ros2 launch usv_bringup sim.launch.py c3_multimodal_fusion:=true
```

并在 `src/usv_bringup/config/perception.yaml` 中设置：

```yaml
c3_multimodal_buffer_fusion:
  ros__parameters:
    enable_nn_heatmap_bypass: true
```

## 检验方法

编译检查：

```bash
cd ${USV_WS}
./scripts/build_clean_env.sh
```

Launch 参数检查：

```bash
source install/setup.bash
ros2 launch usv_bringup sim.launch.py --show-args
```

无界面启动检查：

```bash
ros2 launch usv_bringup sim.launch.py gui:=false rviz:=false
```

查看四模态缓存池：

```bash
ros2 topic echo /c3/buffer/radar_cloud --once
ros2 topic echo /c3/buffer/sonar_cloud --once
ros2 topic echo /c3/buffer/vision_cloud --once
ros2 topic echo /c3/buffer/depth_cloud --once
ros2 topic echo /c3/buffer/integrated_cloud --once
```

查看热力图候选点、无人机目标和已确认目标：

```bash
ros2 topic echo /c3/drone/goal
ros2 topic echo /c3/detected_objects
ros2 topic echo /c3/perception_metrics
```

### 指标输出

`/c3/perception_metrics` 会实时输出 PPT 和实验记录常用指标。评价对象由启动参数 `target_model` 同步给 C3，默认只统计 `moving_vessel`；场景中的浮标、平台等真实模型不会被误算为漏检或假阳性。

```text
detection_precision        目标检测精度，按仿真真值匹配统计
false_positive_rate        误检率
miss_rate                  漏检率
classification_accuracy    已匹配目标的分类准确率
evaluation_scope           指标统计口径，默认 target_models
evaluation_target          当前评价目标，默认 moving_vessel
scene_ground_truth_objects 场景内全部真值数量，仅供参考
ground_truth_objects       当前评价口径内的真值数量
single_frame_processing_ms 单帧处理时间
process_memory_mb          当前进程内存占用
detected_objects           已确认目标累计数量
active_detected_objects    最近仍被观测到的有效目标数量
integrated_points          当前融合点云点数
heatmap_best_probability   热力图最大概率
candidate_x / candidate_y  当前热力图候选坐标
radar_points               当前帧毫米波雷达点数
sonar_points               当前帧声呐点数
normal_camera_points       普通相机 YOLO 点数
gated_camera_points        船载门控 YOLO 点数
uav_gated_points           无人机门控 YOLO 点数
depth_camera_yolo_points   深度相机 YOLO 点数
stf_points                 三切片几何点数
bev_points                 BEV 几何点数
depth_points               深度点云点数
```

当前 vessel 跟踪实验的 PPT 可视化结果保存在：

```text
eval_outputs/c3_ablation/vessel_metrics_dashboard.png
eval_outputs/c3_ablation/vessel_modality_ablation.png
eval_outputs/c3_ablation/vessel_tracking_timeline.png
eval_outputs/c3_ablation/vessel_result.json
eval_outputs/c3_sonar_ablation/target_window_metrics.png
eval_outputs/c3_sonar_ablation/sonar_ablation_overview.png
eval_outputs/c3_sonar_ablation/sonar_sector_health.png
eval_outputs/c3_sonar_ablation/sonar_optimization_before_after.png
eval_outputs/c3_sonar_ablation/live_multimodal_diagnostic.png
eval_outputs/c3_sonar_ablation/sonar_capture.json
```

最新声呐实验采用同一次 25 s 仿真窗口。稳定窗口内 `moving_vessel` 目标召回率为 95.3%、漏检率为 4.7%、分类正确率为 95.3%，平均处理耗时为 3.06 ms；活动误轨迹使窗口检测 Precision 只有 13.2%，因此目标召回率不能被解释为完整场景准确率。声呐最佳帧定位误差为 0.20 m，在同一目标栅格中把融合分数从雷达单模态的 1.092 提高到 1.752，增幅 60.4%。

四扇区 95 帧采样中，front 和 left 的非空回波率分别为 40.0% 和 54.7%；right/back 在当前目标分布下没有有效回波，需要改变目标方位后再验证，不能据此判定传感器失效。`sonar_capture.json` 保存原始统计口径，图片只用于展示。

### 降低 vessel 漏检的优先顺序

1. 补齐 `vessel` 的远距离、小目标、遮挡、逆光和浓雾样本，并确保每张训练图中所有可见 `vessel` 都被标注；漏标会直接把真实目标训练成背景。
2. 按航次而不是随机图片划分训练集和验证集，避免相邻帧泄漏造成虚高准确率；单独报告 `vessel` 的 Precision、Recall 和 PR 曲线。
3. 对远距离图像使用切片推理或提高 YOLO 输入分辨率，使目标在网络输入中保留足够像素；同时加入与真实运行一致的门控伪彩色、雾浓度和重影分布。
4. 使用毫米波雷达热区反投影到相机图像形成 ROI，再对 ROI 做 YOLO 二次检测；它比整幅图盲检更适合远处小船。
5. 保持当前 30 秒 EKF 轨迹窗口，并采用“连续两帧低置信检测也可维持轨迹”的 track-before-detect 策略，减少偶发无框导致的跟丢。
6. 标定相机与雷达外参并检查时间戳。空间偏差超过匹配门限时，即使两个模态都检测到目标，融合层仍会把它们视为不同物体。
7. 最后再调整 `confidence_threshold` 和 NMS。阈值降低只能找回边缘预测，也会增加误检，不能替代补标和数据增强。

`/c3/detected_objects` 中的关键字段：

```text
object_id      已确认目标计数 ID，和类别 class_id 无关
class_id       六类物体类别 ID
name           物体名称
x/y/z          EKF 滤波后的当前位置
vx/vy/vz       EKF 估计速度
predicted_x/y  下一时刻预测位置
confidence     综合置信度
updates        该目标被确认更新的次数
```

## 跟踪目标切换

启动时切换：

```bash
ros2 launch usv_bringup sim.launch.py target_model:=moving_vessel
ros2 launch usv_bringup sim.launch.py target_model:=small_fishing_boat
ros2 launch usv_bringup sim.launch.py target_model:=survey_boat
ros2 launch usv_bringup sim.launch.py target_model:=service_boat
```

当前代码没有实时手动切换目标的服务。运行中会自动在融合轨迹中重选目标，优先选择 `class_id=5(vessel)` 和 `class_id=2(fishing_boat)`，无人机远程发现目标会提高优先级。

## 工程结构

```text
project_root/
├── yolo
│   ├── vessel.v2i.yolov8
│   ├── gated_camera.v3i.yolov8
│   └── runs/detect
└── usv_ws
    ├── Readme.md
    ├── 点云信息.png
    ├── class_reference.png
    ├── docs
    │   ├── gated_non_yolo_recognition.md
    │   ├── stf_dataset_local_usage.md
    │   └── model_reference/class_reference.svg
    ├── eval_outputs/current_accuracy
    │   ├── camera_detection_accuracy_summary.jpg
    │   ├── normal_camera_accuracy_sheet.jpg
    │   ├── gated_camera_accuracy_sheet.jpg
    │   └── camera_threshold_sweep_summary.jpg
    ├── eval_outputs/camera_selected_visuals
    │   ├── camera_selected_class_metrics.png
    │   └── camera_selected_detection_examples.png
    ├── eval_outputs/c3_sonar_ablation
    │   ├── target_window_metrics.png
    │   ├── sonar_ablation_overview.png
    │   ├── sonar_sector_health.png
    │   └── sonar_capture.json
    ├── scripts
    │   ├── build_clean_env.sh
    │   ├── launch_sim_localhost.sh
    │   ├── evaluate_yolo_onnx_dataset.py
    │   ├── test_yolo_onnx_images.py
    │   ├── publish_stf_slices.py
    │   ├── stf_to_yolo_gated.py
    │   └── coco_to_yolo_subset.py
    └── src
        ├── usv_bringup
        ├── usv_description
        ├── usv_perception
        └── depth_image_to_pointcloud2
```

核心文件：

| 路径 | 作用 |
| --- | --- |
| `src/usv_bringup/launch/sim.launch.py` | 主仿真启动 |
| `src/usv_bringup/config/perception.yaml` | 感知、融合、跟随、评估参数 |
| `src/usv_bringup/worlds/ocean_fog.world` | 海上大雾场景 |
| `src/usv_bringup/models/best.onnx` | 普通相机和深度相机 YOLO 模型 |
| `src/usv_bringup/models/best1.onnx` | 船载门控和无人机门控 YOLO 模型 |
| `src/usv_perception/src/gated_camera_recognizer.cpp` | 普通/门控 YOLO 识别、详细检测话题、点云输出 |
| `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp` | 四模态缓存池、点云对齐、热力图、二次确认、`detected` 目标库、EKF 预测和 C3 指标输出 |
| `src/usv_perception/src/radar_sonar_tracker.cpp` | 雷达、声呐、视觉、AIS 多源融合 |
| `src/usv_perception/src/usv_target_follower.cpp` | 本船跟随和避障控制 |
| `src/usv_perception/src/uav_patrol_controller.cpp` | 无人机远程探查，并接收 `/c3/drone/goal` 做二次确认飞行 |
| `src/usv_perception/scripts/mmwave_scan_converter.py` | C3 多高度、多扇区毫米波 scan 到点云转换 |
| `src/usv_perception/src/dynamic_target_controller.cpp` | Gazebo 动态目标运动 |
| `src/usv_perception/src/tracking_evaluator.cpp` | 误检、漏检、CPA/TCPA、ID 切换评估 |
| `src/usv_description/urdf/wamv_base.urdf.xacro` | WAM-V 船体和传感器描述 |
| `src/depth_image_to_pointcloud2/src/depth_image_to_pointcloud2_node.cpp` | RGB-D 去雾和点云输出 |
