# USV Fog Simulation Project

本工程面向琼州海峡大雾低能见度海上通航场景，在 ROS 2 Humble + Gazebo Classic 中模拟无人船、无人机、毫米波雷达、声呐、门控相机、深度相机和 AIS，目标是完成海上目标识别、融合跟踪、跟随和避障评估。

本文档按当前 `/home/hu/usv_ws` 实际文件重新整理，不沿用旧版本模型名。

## 快速启动

编译：

```bash
cd /home/hu/usv_ws
./scripts/build_clean_env.sh
source install/setup.bash
```

启动仿真：

```bash
ros2 launch usv_bringup sim.launch.py
```

如果 Gazebo 提示 `Exception sending a multicast message: No such device`，一般是组播网卡问题，不是源码错误。可用本地通信脚本启动：

```bash
./scripts/launch_sim_localhost.sh gui:=false rviz:=true
```

常用开关：

```bash
ros2 launch usv_bringup sim.launch.py gui:=false rviz:=true
ros2 launch usv_bringup sim.launch.py target_model:=small_fishing_boat
ros2 launch usv_bringup sim.launch.py pseudocolor_gated_yolo:=true
ros2 launch usv_bringup sim.launch.py stf_gated_fusion:=false
ros2 launch usv_bringup sim.launch.py gated_bev_detection:=false
ros2 launch usv_bringup sim.launch.py ais:=false
ros2 launch usv_bringup sim.launch.py uav:=false
```

## 当前模型

两个 ONNX 的 metadata 已重新检查，类别顺序完全相同：

```text
0 buoy
1 debris_container
2 fishing_boat
3 floating_obstacle
4 platform
5 vessel
```

| 文件 | 当前用途 | 默认节点 | 说明 |
| --- | --- | --- | --- |
| `src/usv_bringup/models/best.onnx` | 普通/去雾图像 YOLO | `gated_camera_recognizer` | 输入 `/gated_camera/image_raw`，节点内可做去雾，输出 `/gated_camera/detection_points` |
| `src/usv_bringup/models/best1.onnx` | 三切片伪彩色 YOLO | `pseudocolor_gated_camera_recognizer`、`uav_gated_camera_recognizer` | 输入 `range_view` 伪彩色图，输出 `/gated_camera/pseudocolor/detection_points` 或 `/uav/gated_camera/detection_points` |

注意区分两种 ID：

```text
ONNX 输出类别序号:
  0=buoy, 1=debris_container, 2=fishing_boat, 3=floating_obstacle, 4=platform, 5=vessel

融合点云 PointCloud2 的 class_id:
  0=vessel, 1=fishing_boat, 2=buoy, 3=floating_obstacle, 4=debris_container, 5=platform
```

代码会按标签名把 ONNX 输出 remap 成融合点云的 `class_id`。所以 `usv_target_follower` 里的 `follow_class_ids: [0.0, 1.0]` 表示跟随 `vessel` 和 `fishing_boat`，不是 ONNX 的 `buoy/debris_container`。

本地训练/评估数据集：

```text
/home/hu/yolo/vessel.v1i.yolov8
/home/hu/yolo/gated_camera.v2i.yolov8
```

`gated_camera.v2i.yolov8/data.yaml` 里 Roboflow 写的是 `../train/images`，但本地实际图片在 `gated_camera.v2i.yolov8/train/images`。评估时使用 `--split train`，脚本会正确找到真实目录。

## 识别准确率怎么看

我已重新跑了一轮当前 ONNX 的离线检测评估，结果和可视化截图在：

```text
eval_outputs/current_accuracy/final_accuracy_summary.jpg
eval_outputs/current_accuracy/final_normal_best_on_vessel_conf0p25_sheet.jpg
eval_outputs/current_accuracy/final_pseudocolor_best1_on_gated_conf0p10_sheet.jpg
eval_outputs/current_accuracy/threshold_sweep_summary.jpg
eval_outputs/current_accuracy/final_normal_best_on_vessel_conf0p25/summary.json
eval_outputs/current_accuracy/final_pseudocolor_best1_on_gated_conf0p10/summary.json
```

当前结果：

| 方法 | 模型 | conf | 图像数 | GT 框 | 预测框 | TP | FP | FN | Precision | Recall |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 普通/去雾 YOLO | `best.onnx` | 0.25 | 142 | 299 | 3 | 0 | 3 | 299 | 0.0000 | 0.0000 |
| 伪彩色 YOLO | `best1.onnx` | 0.10 | 105 | 146 | 100 | 29 | 71 | 117 | 0.2900 | 0.1986 |

阈值扫描结果见：

```text
eval_outputs/current_accuracy/threshold_sweep_summary.jpg
```

普通模型在 `0.05-0.35` 阈值内都没有 TP；伪彩色模型选择 `conf=0.10`，是为了在不过度放大误检的情况下提高召回。

截图颜色约定：

```text
绿色 GT 框：被正确匹配
红色 GT 框：漏检
黄色预测框：正确匹配
紫色预测框：误检
```

自己复现普通/去雾模型评估：

```bash
cd /home/hu/usv_ws
python3 scripts/evaluate_yolo_onnx_dataset.py \
  --model src/usv_bringup/models/best.onnx \
  --data /home/hu/yolo/vessel.v1i.yolov8/data.yaml \
  --split train \
  --conf 0.25 \
  --save-examples 24 \
  --save-dir eval_outputs/current_accuracy/final_normal_best_on_vessel_conf0p25
```

自己复现伪彩色模型评估：

```bash
cd /home/hu/usv_ws
python3 scripts/evaluate_yolo_onnx_dataset.py \
  --model src/usv_bringup/models/best1.onnx \
  --data /home/hu/yolo/gated_camera.v2i.yolov8/data.yaml \
  --split train \
  --conf 0.10 \
  --save-examples 24 \
  --save-dir eval_outputs/current_accuracy/final_pseudocolor_best1_on_gated_conf0p10
```

只看若干图片的检测框，不计算 precision/recall：

```bash
python3 scripts/test_yolo_onnx_images.py \
  --model src/usv_bringup/models/best1.onnx \
  --images /home/hu/yolo/gated_camera.v2i.yolov8/train/images \
  --limit 20 \
  --save-dir /tmp/usv_onnx_test_pseudocolor
```

STF 三切片响应、BEV 几何检测和最终融合跟踪不是 YOLO 离线分类器，没有 2D 标注集上的 precision/recall 截图。它们要在仿真中看真值指标：

```bash
ros2 topic echo /gated_camera/stf_fusion/status
ros2 topic echo /gated_camera/bev_detector/status
ros2 topic echo /tracked_objects_text
ros2 topic echo /tracking_metrics
```

`/tracking_metrics` 由 `tracking_evaluator` 发布，用 Gazebo 真值评估融合结果，适合看漏检率、误检率、CPA、TCPA、最小安全距离和 ID 切换。

## 当前识别与融合链路

默认链路：

```text
普通/去雾图像:
  /gated_camera/image_raw + /depth_camera/depth/image_raw
  -> gated_camera_recognizer + best.onnx
  -> /gated_camera/detection_points

STF 风格三切片响应:
  /gated_camera/slice_near + /slice_mid + /slice_far
  -> gated_slice_fusion_recognizer
  -> /gated_camera/stf_detection_points

BEV 几何旁路:
  /depth_camera/depth/image_raw + /depth_camera/camera_info
  -> gated_bev_detector
  -> /gated_camera/bev_detection_points

无人机伪彩色:
  /uav/gated_camera/image_raw + /uav/gated_camera/depth/image_raw
  -> uav_gated_camera_recognizer + best1.onnx
  -> /uav/gated_camera/detection_points

融合:
  radar_sonar_tracker
  <- radar + sonar + normal YOLO + pseudo-color YOLO + STF + BEV + UAV + AIS
  -> /tracked_objects
  -> /tracked_object_poses
  -> /tracked_objects_text
```

船载伪彩色 YOLO 默认关闭：

```text
pseudocolor_gated_yolo:=false
```

原因是当前 `best1.onnx` 召回率仍偏低，直接参与船载融合需要结合雷达/声呐/AIS 一起看；需要测试时再打开：

```bash
ros2 launch usv_bringup sim.launch.py pseudocolor_gated_yolo:=true
```

## 切换跟踪目标

### 启动时切换

当前可靠的切换方式是启动时指定 Gazebo 模型名：

```bash
ros2 launch usv_bringup sim.launch.py target_model:=moving_vessel
ros2 launch usv_bringup sim.launch.py target_model:=small_fishing_boat
ros2 launch usv_bringup sim.launch.py target_model:=survey_boat
ros2 launch usv_bringup sim.launch.py target_model:=service_boat
```

`target_model` 会同时传给：

```text
dynamic_target_controller.tracked_target_name
uav_patrol_controller.target_model_name
```

也就是说，动态目标控制器会把这个模型当作主目标，无人机会优先飞去这个目标附近巡查，并通过 `/uav/remote_target_status` 辅助本船跟随。

当前世界里主要模型名：

| 模型名 | 含义 | 动态运动 | 适合作主跟踪目标 |
| --- | --- | --- | --- |
| `moving_vessel` | 目标船 | 是 | 是 |
| `small_fishing_boat` | 小型渔船 | 是 | 是 |
| `survey_boat` | 调查船 | 是 | 是 |
| `service_boat` | 服务船 | 是 | 是 |
| `fishnet_buoy` | 渔网浮标 | 是 | 一般作为障碍物 |
| `floating_obstacle` | 漂浮障碍物 | 是 | 一般作为障碍物 |
| `drift_debris` | 漂浮碎片 | 是 | 一般作为障碍物 |
| `cargo_ship_far` | 远处货船 | 否 | 可做远目标/AIS 背景 |
| `anchored_tanker` | 锚泊油船 | 否 | 可做静态背景 |
| `research_platform` | 平台 | 否 | 障碍物/背景 |
| `floating_container` | 漂浮集装箱 | 否 | 障碍物 |
| `channel_buoy_north`、`channel_buoy_south` | 航道浮标 | 否 | 障碍物 |
| `navigation_marker_port`、`navigation_marker_starboard` | 航标柱 | 否 | 障碍物 |
| `net_line_a` | 网线/浮标串 | 否 | 障碍物 |

### 能否实时切换

当前代码不能通过已有 ROS 服务或话题“实时手动切换目标模型名”。原因是：

```text
dynamic_target_controller、uav_patrol_controller、usv_target_follower
在启动时 declare_parameter 后保存到成员变量，没有参数更新回调；
所以运行中 ros2 param set target_model_name 不会可靠改变控制逻辑。
```

当前运行中的“切换”只有自动切换：

```text
usv_target_follower 会锁定一个 track_id；
如果目标短时间内还在，继续追它；
如果跟丢或超时，会在 class_id=0(vessel) 或 class_id=1(fishing_boat) 的融合轨迹里重新选更可信目标；
无人机远程目标 uav_remote_scout 会提高被选中的优先级。
```

因此，现在要强制换目标，建议停止 launch 后重新启动：

```bash
ros2 launch usv_bringup sim.launch.py target_model:=survey_boat
```

如果后续要实现真正实时切换，建议新增一个目标选择节点或服务：

```text
/target_selector/set_target
  input: model_name 或 track_id
  actions:
    1. 更新 dynamic_target_controller 的 tracked_target_name
    2. 更新 uav_patrol_controller 的 target_model_name
    3. 通知 usv_target_follower 清空 locked_target_id
    4. 可选：把 follow_class_ids 切到目标类别
```

## 主要话题

```text
/gated_camera/image_raw                  船载门控/普通图像
/gated_camera/dehazed                    去雾后的图像
/gated_camera/range_view                 三切片伪彩色图
/gated_camera/annotated                  普通/去雾 YOLO 标框图
/gated_camera/pseudocolor/annotated      伪彩色 YOLO 标框图
/gated_camera/detection_points           普通/去雾 YOLO 点云
/gated_camera/pseudocolor/detection_points 伪彩色 YOLO 点云
/gated_camera/stf_detection_points       三切片响应目标点云
/gated_camera/bev_detection_points       BEV 几何目标点云
/uav/gated_camera/detection_points       无人机目标点云
/uav/remote_target_status                无人机远程目标提示
/ais/targets                             AIS 仿真目标
/tracked_objects                         融合跟踪 Marker
/tracked_object_poses                    融合跟踪 PoseArray
/tracked_objects_text                    融合跟踪 JSON 状态
/usv_follow_status                       本船跟随/避障状态
/tracking_metrics                        实验评估指标
```

目标点云统一使用 `sensor_msgs/PointCloud2`：

```text
x y z intensity class_id bbox_cx bbox_cy bbox_w bbox_h
```

BEV 几何检测没有 2D 图像框，`bbox_*` 填 0。

## 参数位置

主要后期可调参数都在：

```text
src/usv_bringup/config/perception.yaml
```

常调项：

```text
gated_camera_recognizer.confidence_threshold
pseudocolor_gated_camera_recognizer.confidence_threshold
gated_slice_fusion_recognizer.confidence_threshold
gated_bev_detector.min_range / max_range / min_cluster_points
radar_sonar_tracker.association_gate / track_timeout / min_camera_confidence
usv_target_follower.max_speed / desired_standoff / target_lock_timeout
usv_target_follower.follow_class_ids
tracking_evaluator.match_gate / eval_range
```

## 工作区结构

顶层文件：

| 路径 | 作用 |
| --- | --- |
| `Readme.md` | 当前主说明文档 |
| `image.png` | 旧的说明/参考图片 |
| `点云信息.png` | 点云字段参考图片 |
| `.vscode/c_cpp_properties.json` | VS Code C++ 配置 |
| `.vscode/settings.json` | VS Code 工作区配置 |
| `eval_outputs/current_accuracy/` | 本次生成的准确率结果和截图 |
| `eval_outputs/*/summary.json` | 历史或当前 ONNX 离线评估结果 |

文档：

| 路径 | 作用 |
| --- | --- |
| `docs/stf_dataset_local_usage.md` | `/home/hu/STF_Dataset` 的内容、转换和使用方法 |
| `docs/gated_non_yolo_recognition.md` | 不使用 YOLO 的后续方法，主要写未完成路线 |
| `docs/model_reference/class_reference.svg` | 类别/模型参考图 |

脚本：

| 路径 | 作用 |
| --- | --- |
| `scripts/build_clean_env.sh` | 清理旧 overlay 环境后编译 |
| `scripts/launch_sim_localhost.sh` | 用 localhost Gazebo 通信启动仿真 |
| `scripts/evaluate_yolo_onnx_dataset.py` | 用 YOLO 标注集计算 ONNX precision/recall 并保存示例图 |
| `scripts/test_yolo_onnx_images.py` | 只跑图片检测并保存标框图 |
| `scripts/stf_to_yolo_gated.py` | 把 STF 三切片数据转成 YOLO 格式 |
| `scripts/publish_stf_slices.py` | 离线发布 STF 切片到 ROS 话题 |
| `scripts/capture_annotation_dataset.sh` | 采集普通标注图 |
| `scripts/capture_occlusion_dataset.sh` | 采集遮挡标注图 |
| `scripts/capture_pseudocolor_dataset.sh` | 采集伪彩色标注图 |
| `scripts/coco_to_yolo_subset.py` | COCO 子集转 YOLO 格式辅助脚本 |

`src/usv_bringup`：

| 路径 | 作用 |
| --- | --- |
| `src/usv_bringup/package.xml` | bringup 包声明 |
| `src/usv_bringup/CMakeLists.txt` | 安装 launch/world/config/model |
| `src/usv_bringup/config/perception.yaml` | 感知、跟踪、避障、AIS、评估参数 |
| `src/usv_bringup/launch/sim.launch.py` | 主仿真 launch |
| `src/usv_bringup/launch/annotation_capture.launch.py` | 普通标注采集 launch |
| `src/usv_bringup/launch/annotation_occlusion_capture.launch.py` | 遮挡标注采集 launch |
| `src/usv_bringup/launch/annotation_pseudocolor_capture.launch.py` | 伪彩色标注采集 launch |
| `src/usv_bringup/worlds/ocean_fog.world` | 主海上大雾场景 |
| `src/usv_bringup/worlds/annotation_targets.world` | 标注采集专用场景 |
| `src/usv_bringup/models/best.onnx` | 普通/去雾图像 ONNX |
| `src/usv_bringup/models/best1.onnx` | 伪彩色门控 ONNX |
| `src/usv_bringup/docker/Dockerfile` | 容器环境参考 |

`src/usv_perception`：

| 路径 | 作用 |
| --- | --- |
| `src/usv_perception/package.xml` | 感知包声明 |
| `src/usv_perception/CMakeLists.txt` | 编译所有 C++ 节点并链接 ONNX Runtime/OpenCV |
| `src/usv_perception/include/usv_perception/common.hpp` | 波浪高度、欧拉角转四元数等通用函数 |
| `src/usv_perception/resource/usv_perception` | ament 资源标记 |
| `src/usv_perception/src/gated_camera_recognizer.cpp` | 普通/去雾/伪彩色 YOLO 识别、三切片生成、2D 框转目标点云 |
| `src/usv_perception/src/gated_slice_fusion_recognizer.cpp` | STF 风格三切片响应检测旁路 |
| `src/usv_perception/src/gated_bev_detector.cpp` | 深度图投影 BEV 几何聚类检测 |
| `src/usv_perception/src/radar_sonar_tracker.cpp` | 雷达、声呐、视觉、BEV、STF、无人机、AIS 多源融合跟踪 |
| `src/usv_perception/src/usv_target_follower.cpp` | 本船跟随目标、短时预测和反应式避障 |
| `src/usv_perception/src/uav_patrol_controller.cpp` | 无人机巡航、目标靠近和远程目标提示 |
| `src/usv_perception/src/dynamic_target_controller.cpp` | Gazebo 动态目标航路点运动控制 |
| `src/usv_perception/src/ais_simulator.cpp` | 从 Gazebo 真值生成 AIS JSON 目标 |
| `src/usv_perception/src/tracking_evaluator.cpp` | 跟踪、CPA/TCPA、误检漏检、ID 切换评估 |
| `src/usv_perception/src/wave_buoyancy_node.cpp` | 给 WAM-V 施加简化波浪浮力 |
| `src/usv_perception/src/ros_image_recorder.cpp` | ROS 图像采集保存节点 |

`src/usv_description`：

| 路径 | 作用 |
| --- | --- |
| `src/usv_description/package.xml` | 机器人描述包声明 |
| `src/usv_description/CMakeLists.txt` | 安装 urdf/rviz |
| `src/usv_description/urdf/wamv_base.urdf.xacro` | WAM-V 主 xacro，包含传感器插件 |
| `src/usv_description/urdf/wamv.urdf` | 展开的 URDF 参考 |
| `src/usv_description/rviz/default.rviz` | RViz 默认配置 |

`src/lidar_robot`：

| 路径 | 作用 |
| --- | --- |
| `src/lidar_robot/package.xml` | 雷达/雷达转换包声明 |
| `src/lidar_robot/CMakeLists.txt` | 编译/安装雷达相关节点 |
| `src/lidar_robot/src/lidar_depth_node.cpp` | 激光/深度点云聚类节点 |
| `src/lidar_robot/src/mmwave_scan_converter.py` | C3 多高度毫米波 raw scan 转检测点 |
| `src/lidar_robot/src/mmwave_detection_debug.py` | 毫米波检测调试打印 |
| `src/lidar_robot/src/mmwave_radar_node.cpp` | 毫米波雷达节点源码参考 |
| `src/lidar_robot/launch/sim.launch.py` | lidar_robot 原始仿真 launch |
| `src/lidar_robot/urdf/robot.urdf` | 原始机器人 URDF |
| `src/lidar_robot/urdf/robot.urdf.xacro` | 原始机器人 xacro |
| `src/lidar_robot/urdf/plugins/gazebo_laser_plugin.xacro` | Gazebo 激光插件片段 |
| `src/lidar_robot/urdf/plugins/gazebo_mmwave_plugin.xacro` | Gazebo 毫米波插件片段 |

`src/depth_image_to_pointcloud2`：

| 路径 | 作用 |
| --- | --- |
| `src/depth_image_to_pointcloud2/package.xml` | RGB-D 去雾点云包声明 |
| `src/depth_image_to_pointcloud2/CMakeLists.txt` | 编译 `depth_image_to_pointcloud2` |
| `src/depth_image_to_pointcloud2/include/depth_image_to_pointcloud2/depth_image_to_pointcloud2_node.hpp` | 节点头文件 |
| `src/depth_image_to_pointcloud2/src/depth_image_to_pointcloud2_node.cpp` | RGB-D 去雾、深度补偿、点云输出 |
| `src/depth_image_to_pointcloud2/src/main.cpp` | 节点入口 |
| `src/depth_image_to_pointcloud2/launch/depth_camera_dehaze.launch.py` | 单独测试 RGB-D 去雾 launch |
| `src/depth_image_to_pointcloud2/rviz/pointcloud.rviz` | 点云 RViz 配置 |
| `src/depth_image_to_pointcloud2/worlds/fog_depth_camera.world` | 深度相机雾天测试世界 |
| `src/depth_image_to_pointcloud2/LICENSE` | 许可证文件 |

生成性文件不属于源码，上传 git 前通常不提交：

```text
build/
install/
log/
src/**/__pycache__/
scripts/__pycache__/
eval_outputs/   # 如果不需要保留评估截图，也可不提交
```

## 还没完成的方向

详细未完成方案见：

```text
docs/gated_non_yolo_recognition.md
docs/stf_dataset_local_usage.md
```

当前最需要优先解决的是检测模型本身：`best.onnx` 对当前普通图数据几乎全漏检；`best1.onnx` 通过参数把召回率提高到 0.1986，但误检也上升，仍需要更多标注和训练。跟踪不好时，先看识别和融合输入，不要只调 follower。
