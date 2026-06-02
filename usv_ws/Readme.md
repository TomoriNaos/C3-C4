# USV Fog Simulation Project

本工程用于琼州海峡大雾低能见度场景下的无人船多模态感知仿真。系统在 ROS 2 Humble + Gazebo Classic 中融合普通相机、门控相机、深度相机、毫米波雷达、声呐、无人机观测和 AIS，实现海上目标识别、融合跟踪、跟随与避障评估。

## 快速使用

编译：

```bash
cd /home/hu/usv_ws
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
ros2 launch usv_bringup sim.launch.py pseudocolor_gated_yolo:=true
ros2 launch usv_bringup sim.launch.py stf_gated_fusion:=false
ros2 launch usv_bringup sim.launch.py gated_bev_detection:=false
ros2 launch usv_bringup sim.launch.py uav:=false
ros2 launch usv_bringup sim.launch.py ais:=false
```

## 模型与数据

当前模型分工：

| 模型 | 相机类型 | 默认节点 | 当前推荐阈值 |
| --- | --- | --- | --- |
| `src/usv_bringup/models/best.onnx` | 普通相机 | `gated_camera_recognizer` | `confidence_threshold: 0.15` |
| `src/usv_bringup/models/best1.onnx` | 门控相机/伪彩色三切片 | `pseudocolor_gated_camera_recognizer`、`uav_gated_camera_recognizer` | `confidence_threshold: 0.20` |

两个 ONNX 的类别顺序一致：

```text
0 buoy
1 debris_container
2 fishing_boat
3 floating_obstacle
4 platform
5 vessel
```

注意：ONNX 类别序号和融合点云里的 `class_id` 不完全一样。程序会按标签名 remap 到跟踪用全局 ID：

```text
0 vessel
1 fishing_boat
2 buoy
3 floating_obstacle
4 debris_container
5 platform
```

所以 `usv_target_follower.follow_class_ids: [0.0, 1.0]` 表示跟随船类目标 `vessel/fishing_boat`。

训练截图目录：

```text
/home/hu/usv_captures/annotation_raw          普通相机截图
/home/hu/usv_captures/annotation_occlusion    普通相机遮挡截图
/home/hu/usv_captures/pseudocolor_single      门控伪彩色单目标截图
/home/hu/usv_captures/pseudocolor_complex     门控伪彩色复杂场景截图
```

YOLO 数据和训练结果目录：

```text
/home/hu/yolo/vessel.v2i.yolov8               普通相机 Roboflow 数据
/home/hu/yolo/gated_camera.v3i.yolov8         门控相机 Roboflow 数据
/home/hu/yolo/runs/detect/c3_gpu_train_vessel 普通相机训练结果
/home/hu/yolo/runs/detect/c3_gpu_train_gated_camera 门控相机训练结果
```

训练过程图可看：

```text
/home/hu/yolo/runs/detect/c3_gpu_train_vessel/results.png
/home/hu/yolo/runs/detect/c3_gpu_train_vessel/confusion_matrix_normalized.png
/home/hu/yolo/runs/detect/c3_gpu_train_vessel/val_batch0_pred.jpg

/home/hu/yolo/runs/detect/c3_gpu_train_gated_camera/results.png
/home/hu/yolo/runs/detect/c3_gpu_train_gated_camera/confusion_matrix_normalized.png
/home/hu/yolo/runs/detect/c3_gpu_train_gated_camera/val_batch0_pred.jpg
```

## 检测准确度

我用当前最新数据集的 `valid` split 重新评估，并按 F1 扫描置信度后选择：

```text
普通相机 best.onnx:  conf=0.15
门控相机 best1.onnx: conf=0.20
```

当前检测准确度：

| 方法 | 模型 | 数据集 | conf | images | GT | pred | TP | FP | FN | Precision | Recall | F1 |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 普通相机 | `best.onnx` | `vessel.v2i.yolov8` valid | 0.15 | 28 | 46 | 46 | 33 | 13 | 13 | 0.7174 | 0.7174 | 0.7174 |
| 门控相机 | `best1.onnx` | `gated_camera.v3i.yolov8` valid | 0.20 | 34 | 100 | 80 | 55 | 25 | 45 | 0.6875 | 0.5500 | 0.6111 |

可视化图片：

```text
eval_outputs/current_accuracy/camera_detection_accuracy_summary.jpg
eval_outputs/current_accuracy/normal_camera_accuracy_sheet.jpg
eval_outputs/current_accuracy/gated_camera_accuracy_sheet.jpg
eval_outputs/current_accuracy/camera_threshold_sweep_summary.jpg
```

截图颜色：

```text
绿色 GT 框：被匹配
红色 GT 框：漏检
黄色预测框：正确预测
紫色预测框：误检
```

复现评估：

```bash
cd /home/hu/usv_ws

python3 scripts/evaluate_yolo_onnx_dataset.py \
  --model src/usv_bringup/models/best.onnx \
  --data /home/hu/yolo/vessel.v2i.yolov8/data.yaml \
  --split val \
  --conf 0.15 \
  --save-examples 24 \
  --save-dir eval_outputs/current_accuracy/final_normal_camera_best_val_conf0p15

python3 scripts/evaluate_yolo_onnx_dataset.py \
  --model src/usv_bringup/models/best1.onnx \
  --data /home/hu/yolo/gated_camera.v3i.yolov8/data.yaml \
  --split val \
  --conf 0.20 \
  --save-examples 24 \
  --save-dir eval_outputs/current_accuracy/final_gated_camera_best1_val_conf0p20
```

## 实时检测话题

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
  label
  global_class_id
  score
  bbox cx/cy/w/h/x1/y1/x2/y2
points[]:
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
```

融合跟踪和评估：

```text
/tracked_objects
/tracked_object_poses
/tracked_objects_text
/usv_follow_status
/tracking_metrics
```

## 当前融合链路

```text
普通相机:
  /gated_camera/image_raw
  -> gated_camera_recognizer + best.onnx
  -> /gated_camera/detection_points

门控相机:
  /gated_camera/range_view
  -> pseudocolor_gated_camera_recognizer + best1.onnx
  -> /gated_camera/pseudocolor/detection_points

无人机门控:
  /uav/gated_camera/image_raw
  -> uav_gated_camera_recognizer + best1.onnx
  -> /uav/gated_camera/detection_points

几何旁路:
  gated_slice_fusion_recognizer -> /gated_camera/stf_detection_points
  gated_bev_detector -> /gated_camera/bev_detection_points

融合:
  radar_sonar_tracker
  <- radar + sonar + normal camera + gated camera + UAV + AIS + BEV + STF
  -> /tracked_objects_text
```

船载门控 YOLO 默认仍由 `pseudocolor_gated_yolo` 控制。要让船载门控相机 ONNX 进入融合，需要启动：

```bash
ros2 launch usv_bringup sim.launch.py pseudocolor_gated_yolo:=true
```

## 跟踪目标切换

启动时切换：

```bash
ros2 launch usv_bringup sim.launch.py target_model:=moving_vessel
ros2 launch usv_bringup sim.launch.py target_model:=small_fishing_boat
ros2 launch usv_bringup sim.launch.py target_model:=survey_boat
ros2 launch usv_bringup sim.launch.py target_model:=service_boat
```

当前代码没有实时手动切换目标的服务。运行中会自动在融合轨迹中重选目标，优先选择 `class_id=0(vessel)` 和 `class_id=1(fishing_boat)`，无人机远程发现目标会提高优先级。

## 工程结构

```text
/home/hu/usv_ws
├── Readme.md
├── docs
│   ├── gated_non_yolo_recognition.md
│   ├── stf_dataset_local_usage.md
│   └── model_reference/class_reference.svg
├── eval_outputs/current_accuracy
│   ├── camera_detection_accuracy_summary.jpg
│   ├── normal_camera_accuracy_sheet.jpg
│   ├── gated_camera_accuracy_sheet.jpg
│   └── camera_threshold_sweep_summary.jpg
├── scripts
│   ├── build_clean_env.sh
│   ├── launch_sim_localhost.sh
│   ├── evaluate_yolo_onnx_dataset.py
│   ├── test_yolo_onnx_images.py
│   ├── capture_annotation_dataset.sh
│   ├── capture_occlusion_dataset.sh
│   ├── capture_pseudocolor_dataset.sh
│   ├── publish_stf_slices.py
│   ├── stf_to_yolo_gated.py
│   └── coco_to_yolo_subset.py
└── src
    ├── usv_bringup
    ├── usv_description
    ├── usv_perception
    ├── lidar_robot
    └── depth_image_to_pointcloud2
```

核心文件：

| 路径 | 作用 |
| --- | --- |
| `src/usv_bringup/launch/sim.launch.py` | 主仿真启动 |
| `src/usv_bringup/config/perception.yaml` | 感知、融合、跟随、评估参数 |
| `src/usv_bringup/worlds/ocean_fog.world` | 海上大雾场景 |
| `src/usv_bringup/models/best.onnx` | 普通相机模型 |
| `src/usv_bringup/models/best1.onnx` | 门控相机模型 |
| `src/usv_perception/src/gated_camera_recognizer.cpp` | 普通/门控 YOLO 识别、详细检测话题、点云输出 |
| `src/usv_perception/src/radar_sonar_tracker.cpp` | 雷达、声呐、视觉、AIS 多源融合 |
| `src/usv_perception/src/usv_target_follower.cpp` | 本船跟随和避障控制 |
| `src/usv_perception/src/uav_patrol_controller.cpp` | 无人机远程探查 |
| `src/usv_perception/src/dynamic_target_controller.cpp` | Gazebo 动态目标运动 |
| `src/usv_perception/src/tracking_evaluator.cpp` | 误检、漏检、CPA/TCPA、ID 切换评估 |
| `src/usv_description/urdf/wamv_base.urdf.xacro` | WAM-V 船体和传感器描述 |
| `src/lidar_robot/src/mmwave_scan_converter.py` | C3 多高度毫米波转换 |
| `src/depth_image_to_pointcloud2/src/depth_image_to_pointcloud2_node.cpp` | RGB-D 去雾和点云输出 |

## 清理说明

已删除生成性目录，便于上传 git：

```text
build/
install/
log/
scripts/__pycache__/
src/**/__pycache__/
旧的 eval_outputs/archive_*
```

如果重新编译，`build/ install/ log/` 会再次生成；上传前可再次删除。
