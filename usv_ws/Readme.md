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

门控相机的 ONNX 推理使用 ONNX Runtime C++。如果机器上没有，请下载官方 Linux x64 包：

```bash
cd ~
wget https://github.com/microsoft/onnxruntime/releases/download/v1.23.2/onnxruntime-linux-x64-1.23.2.tgz
tar -xzf onnxruntime-linux-x64-1.23.2.tgz
```

默认工程会查找：

```text
/home/hu/onnxruntime-linux-x64-1.23.2
```

如果放在其他位置，编译时指定：

```bash
colcon build --symlink-install --cmake-args -DONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-1.23.2
```

## 编译

```bash
cd ~/usv_ws
colcon build --symlink-install
source install/setup.bash
```

原始工程备份保留在：

```text
/home/hu/usv_ws_original_backup_20260523_c3_merge
```

## ONNX 识别

当前工程的门控识别节点是 C++，通过 ONNX Runtime 加载 YOLO ONNX 模型，不依赖 OpenCV DNN。默认模型放在 `src/usv_bringup/models/best.onnx`，会随 `usv_bringup` 一起安装。

仿真中的门控相机由两部分近似实现：

- `/gated_camera/image_raw`：Gazebo 普通 RGB 相机图像。
- `/depth_camera/depth/image_raw`：深度相机图像。

`gated_camera_recognizer` 用深度图把 RGB 图像按距离分成近/中/远门控切片，并生成 `/gated_camera/range_view`。这是真实门控相机的仿真近似，不是硬件层面真的由一台 RGB 相机和一台深度相机组成。

当前模型类别顺序见 `src/usv_bringup/config/perception.yaml`：

```yaml
class_names: [buoy, fishing_boat]
```

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

如果暂时不需要无人机 ALS 和门控相机：

```bash
ros2 launch usv_bringup sim.launch.py uav:=false
```

如果只想跑 `usv_ws` 原来的单毫米波雷达链路，不启动 C3 多高度雷达：

```bash
ros2 launch usv_bringup sim.launch.py c3_mmwave:=false
```

如果暂时不需要 RGB-D 去雾点云：

```bash
ros2 launch usv_bringup sim.launch.py rgbd_dehaze:=false
```

如果只想看目标运动和感知结果，不让本船自动跟随目标：

```bash
ros2 launch usv_bringup sim.launch.py usv_follow:=false
```

## 主要话题

- 毫米波雷达：`/mmwave_radar/scan`
- 窄束声呐：`/sonar/scan`
- 深度相机彩色图：`/depth_camera/image_raw`
- 深度相机深度图：`/depth_camera/depth/image_raw`
- 深度相机点云：`/depth_camera/points`
- RGB-D 去雾点云：`/depth_camera/dehazed_points`
- ALS 点云：`/als/points`
- C3 多高度雷达原始扫描：`/radar_10m/raw_scan`、`/radar_4m/raw_scan`、`/radar_1p9m/raw_scan`、`/radar_1p5m/raw_scan`、`/radar_1m/raw_scan`
- C3 多高度雷达检测点云：`/mmwave_10m/detections`、`/mmwave_4m/detections`、`/mmwave_1p9m/detections`、`/mmwave_1p5m/detections`、`/mmwave_1m/detections`
- 门控相机原始图：`/gated_camera/image_raw`
- 门控相机近距切片：`/gated_camera/slice_near`
- 门控相机中距切片：`/gated_camera/slice_mid`
- 门控相机远距切片：`/gated_camera/slice_far`
- 门控相机组合识别图：`/gated_camera/range_view`
- 门控相机标框图：`/gated_camera/annotated`
- 门控相机识别结果：`/gated_camera/detections`
- 门控相机目标点云：`/gated_camera/detection_points`
- 门控相机识别状态：`/gated_camera/status`
- 无人机门控相机原始图：`/uav/gated_camera/image_raw`
- 无人机门控相机深度图：`/uav/gated_camera/depth/image_raw`
- 无人机 ALS 点云：`/uav/als/points`
- 无人机门控相机识别图：`/uav/gated_camera/range_view`
- 无人机门控相机识别结果：`/uav/gated_camera/detections`
- 无人机门控相机目标点云：`/uav/gated_camera/detection_points`
- 无人机远程目标指引：`/uav/remote_target_status`
- 融合动态跟踪 Marker：`/tracked_objects`
- 融合动态跟踪 PoseArray：`/tracked_object_poses`
- 跟踪文字状态：`/tracked_objects_text`
- 本船跟随/避障状态：`/usv_follow_status`
- 动态仿真目标标记：`/simulated_targets`
- 波浪浮力状态：`/wave_buoyancy/status`

## 查看识别结果

启动后查看标框图：

```bash
ros2 run rqt_image_view rqt_image_view /gated_camera/annotated
```

查看识别状态：

```bash
ros2 topic echo /gated_camera/status
```

状态中 `backend=onnxruntime_cpu` 表示正在使用 ONNX Runtime 推理；`max_score` 是当前帧最高模型置信度。

查看 2D 检测结果：

```bash
ros2 topic echo /gated_camera/detections
```

查看目标点云：

```bash
ros2 topic echo /gated_camera/detection_points
```

`/gated_camera/detection_points` 类型是 `sensor_msgs/PointCloud2`。这是给后续程序使用的单一目标结果话题：每个识别目标对应一个点，坐标系为 `base_link`，即以船为中心，同时携带 3D 坐标、检测框、置信度和类别 ID：

```text
x: 目标在船前方的距离，单位 m
y: 目标在船左/右方向的位置，单位 m
z: 目标高度，单位 m
intensity: 识别置信度
class_id: 类别编号，0=buoy，1=fishing_boat
bbox_cx: 检测框中心 x，单位 pixel
bbox_cy: 检测框中心 y，单位 pixel
bbox_w: 检测框宽度，单位 pixel
bbox_h: 检测框高度，单位 pixel
```

文字类别名在 `/gated_camera/detections` 和 `/gated_camera/status` 中查看。后续算法如果只需要数值结果，订阅 `/gated_camera/detection_points` 即可；无人机端订阅 `/uav/gated_camera/detection_points`。两者都会尽量通过 TF 转到 `base_link`，供 `radar_sonar_tracker` 统一融合；RViz 中添加 `PointCloud2` 并选择对应话题，可以看到目标点。

## 采集 ROS 相机话题图片

不要截 Gazebo GUI 窗口，直接保存 ROS 图像话题。

保存船载门控原始图，适合重新标注和训练：

```bash
source install/setup.bash
ros2 run usv_perception ros_image_recorder --ros-args \
  -p image_topic:=/gated_camera/image_raw \
  -p output_dir:=~/usv_captures/gated_raw \
  -p prefix:=gated_raw \
  -p every_n:=3 \
  -p max_images:=300
```

保存无人机门控相机图：

```bash
ros2 run usv_perception ros_image_recorder --ros-args \
  -p image_topic:=/uav/gated_camera/range_view \
  -p output_dir:=~/usv_captures/uav \
  -p prefix:=uav \
  -p every_n:=3 \
  -p max_images:=300
```

这些图片可以作为后续重新训练或测试识别模型的数据源。

## 当前功能

- 大雾海峡环境：Gazebo world 中配置线性雾效、低对比度光照、海面和航标，目标外观已从方块代理加强为更像船、渔船、浮标和漂浮物的组合模型。
- 大范围海域：海面扩大到 2400m 级平面，并加入货船、油船、科研平台、作业船、测量船、集装箱、渔网线和多组航标/浮标作为远近背景目标。
- 波浪和浮力近似：`wave_buoyancy_node` 对无人船施加周期性力/力矩，使船体产生海况扰动。
- 动态目标：`dynamic_target_controller` 通过 `/set_entity_state` 低速移动多个目标；默认 `moving_vessel` 是绿色高亮的主跟踪目标，目标沿多航路点缓慢巡航，不再围绕固定点转圈。
- 多源动态跟踪：`radar_sonar_tracker` 融合毫米波雷达、声呐、船载门控相机目标点云、无人机门控相机目标点云，输出 `/tracked_objects`、`/tracked_object_poses` 和 `/tracked_objects_text`。
- 本船目标跟随和基础避障：`usv_target_follower` 订阅 `/tracked_object_poses` 和 `/tracked_objects_text`，优先选择船类目标，其他前方 track 作为障碍物进行绕行、减速或近距离停车；需要关闭时使用 `usv_follow:=false`。
- 门控相机识别：`gated_camera_recognizer` 根据相机图像和深度图生成近/中/远距离门控切片；配置 `.onnx` 权重后通过 ONNX Runtime 调用模型，并发布标框图、2D 检测结果和目标点云。
- C3 多高度毫米波：从 `c3` 合入 `lidar_robot`，在船体 xacro 中增加 10m、4m、1.9m、1.5m、1m 五个原始雷达传感器，并由 `mmwave_scan_converter.py` 输出带功率、SNR、RCS 等字段的检测点云。
- RGB-D 去雾点云：从 `c3` 合入 `depth_image_to_pointcloud2`，接入当前船载深度相机话题，发布 `/depth_camera/dehazed_points`。
- 无人机远程探查：world 中加入 `als_uav`，发布无人机门控相机 RGB/Depth 和机载 ALS 点云；`uav_patrol_controller` 默认会前出到 `moving_vessel` 附近巡查，并发布 `/uav/remote_target_status` 给本船作为远程目标指引。
- RViz 工作台：默认配置显示 TF、船体、毫米波雷达、声呐、船载 ALS、无人机 ALS、门控识别目标点云、动态目标 Marker、跟踪结果和门控相机图像。

## 后期调参入口

主要参数集中在 `src/usv_bringup/config/perception.yaml`，少数场景和传感器参数在 world 或 launch 文件中。

- 启动开关：`src/usv_bringup/launch/sim.launch.py` 中的 `gui`、`rviz`、`perception`、`dynamic_targets`、`usv_follow`、`uav`、`c3_mmwave`、`rgbd_dehaze`、`yolo_model_path`。
- 海况和地图：`src/usv_bringup/worlds/ocean_fog.world` 中调整雾效 `start/end/density`、海面尺寸、静态船舶/平台/浮标/障碍物位置、无人机相机视场和裁剪距离。
- 目标运动：`perception.yaml` 的 `dynamic_target_controller.update_rate`、`motion_time_scale`、`tracked_target_name`、各目标模型名；具体航路点和每个目标速度在 `src/usv_perception/src/dynamic_target_controller.cpp` 中改。
- 融合跟踪：`radar_sonar_tracker` 的 `radar_topic`、`sonar_topic`、`gated_points_topic`、`uav_points_topic`、`cluster_gap`、`min_cluster_points`、`association_gate`、`max_tracking_range`、`track_timeout`、`min_camera_confidence`。船跑快后容易断 track 时，优先增大 `association_gate` 和 `track_timeout`。
- 本船跟随/避障：`usv_target_follower` 的 `follow_class_id`、`prefer_follow_class`、`max_speed`、`max_yaw_rate`、`desired_standoff`、`yaw_gain`、`speed_gain`、`target_lead_time`、`target_lock_timeout`、`min_chase_speed`、`far_range_speed_boost`、`obstacle_lookahead`、`obstacle_clearance`、`hard_stop_distance`、`avoidance_gain`、`obstacle_slowdown_gain`、`use_model_state_obstacles`、`obstacle_model_names`、`startup_delay`。跟丢就增大 `max_speed`、`speed_gain`、`target_timeout` 或 `target_lock_timeout`；靠目标太近就增大 `desired_standoff`；绕障不明显就增大 `avoidance_gain` 或 `obstacle_clearance`。
- 门控识别：`gated_camera_recognizer` 和 `uav_gated_camera_recognizer` 的 `gate_near/mid/far`、`confidence_threshold`、`process_stride`、`camera_horizontal_fov`、`class_names`、`use_tf_transform`。
- 无人机巡航/远程指引：`uav_patrol_controller` 的 `target_tracking_enabled`、`target_model_name`、`remote_target_topic`、`remote_target_confidence`、`target_follow_backoff`、`target_lateral_sweep`、`patrol_speed`、`altitude`、`camera_pitch`。关闭 `target_tracking_enabled` 后无人机会回到普通航路点巡航。
- 多高度毫米波：`sim.launch.py` 里的 `make_c3_mmwave_converter()` 调整 `max_range`、`horizontal_fov_deg`、`angular_resolution_deg`、`intensity_threshold`、海杂波参数和每个高度的雷达参数。
- RGB-D 去雾点云：`sim.launch.py` 中 `rgbd_dehaze_pointcloud` 的 `frame_stride`、`max_valid_depth`、`dark_channel_radius`、`omega`、`min_transmission`、`depth_compensation_strength`。
- 波浪扰动：`wave_buoyancy_node` 的 `water_level`、`wave_amplitude`、`vertical_force_scale`、`torque_scale`。

## 跟踪与避障流程

1. Gazebo 在 `ocean_fog.world` 中生成大雾海面、浮标、障碍物、目标船和无人机；`dynamic_target_controller` 用 `/set_entity_state` 让 `moving_vessel` 等目标低速沿航路点移动。
2. 船载毫米波雷达和声呐发布扫描，船载/无人机门控相机发布图像和深度图；`gated_camera_recognizer` 用深度门控和 ONNX/备用轮廓检测得到目标框，再把目标中心转成 `base_link` 下的 3D 点云。
3. `radar_sonar_tracker` 对雷达扫描做聚类，加入声呐检测，再融合 `/gated_camera/detection_points` 和 `/uav/gated_camera/detection_points`。它用最近邻关联维护 track，估计位置、速度、置信度、类别和来源。
4. `uav_patrol_controller` 默认锁定 `moving_vessel`，让无人机到目标附近高空巡查，并把目标相对本船的位置和速度发布到 `/uav/remote_target_status`。这相当于仿真中的“无人机远程发现/指挥”通道，避免本船近距传感器一断就跟丢。
5. 跟踪结果有三种输出：`/tracked_objects` 给 RViz 看 Marker，`/tracked_object_poses` 给简单程序读位置，`/tracked_objects_text` 给控制器读 ID、速度、类别、置信度和来源。
6. `usv_target_follower` 优先选择 `follow_class_id=1` 的船类目标；当 `/uav/remote_target_status` 有效时，它会优先锁定无人机远程目标 `id=9001`。控制器会锁定当前目标 ID，并用 `target_lead_time` 做短时前视预测；短时漏检时在 `target_timeout` 内继续按预测方向追，不会马上停船或换目标。
7. 除目标以外，前方进入 `obstacle_lookahead` 和 `obstacle_clearance` 范围的 track 都被当作障碍物。仿真中还默认开启 `use_model_state_obstacles`，把 Gazebo 里的浮标/漂浮物模型作为辅助障碍源，方便先验证避障效果；做真实感知算法评估时可以关掉。
8. 避障控制是反应式规则：障碍在左侧就向右偏，障碍在右侧就向左偏；越近、越靠近航线，偏航越大且速度越低；进入 `hard_stop_distance` 和 `hard_stop_lateral` 时会停车并给出更强的绕行偏航。
9. `usv_target_follower` 最后通过 `/set_entity_state` 移动 `wamv`，同时发布 `world -> base_footprint` TF，让船载传感器坐标随船运动。

查看当前控制判断：

```bash
ros2 topic echo /usv_follow_status
ros2 topic echo /tracked_objects_text
ros2 topic echo /uav/remote_target_status
```

## 可以继续改进的方向

- 训练更合适的海上目标 ONNX 模型：把类别扩展到 `vessel`、`fishing_boat`、`buoy`、`floating_obstacle`、`debris/container`、`platform`，否则控制器只能粗略依赖 `class_id=1` 代表船类目标。
- 把现在的反应式避障升级成局部路径规划，例如 DWA、TEB、速度障碍法 VO/ORCA，或在 ROS2 Nav2 上接入 costmap；这样会比“偏航+减速”更像真实自动避碰。
- 给 `radar_sonar_tracker` 加更正式的滤波器，例如 Kalman/UKF/JPDA，解决多目标靠近时 ID 跳变和速度估计抖动。
- 加入 AIS 仿真和融合：把 MMSI、位置、航速、航向转到 `world/base_link`，与当前 track 关联，提高目标船选择稳定性。
- 增加碰撞评估指标：最近会遇距离 CPA、最近会遇时间 TCPA、最小安全距离、误检率、漏检率、跟踪 ID 切换次数，用这些指标做论文或实验报告更有说服力。
- 做场景集：近距离浮标、横穿渔船、静止漂浮物、远处大船、无人机辅助观测、大雾低能见度等，每个场景记录 rosbag 方便复现实验。

## 数据集需求

当前“几何跟踪”不必须要数据集：毫米波雷达、声呐、深度/ALS 点云只要话题、TF 外参、量程、聚类门限和关联门限正确，就能做目标跟踪。

如果要在大雾下可靠区分船舶、浮标、漂浮障碍物、小型渔船等类别，就需要数据集训练视觉识别模型。建议采集船载和无人机门控/RGB-D 图像，覆盖大雾、晴天、夜间、逆光、不同距离和角度，标注 `vessel`、`fishing_boat`、`buoy`、`floating_obstacle`、`debris/container`、`platform` 等类别，训练 YOLO 后导出 ONNX，并同步更新 `class_names`。

AIS 融合目前还没有实现；如果后续要加，需要仿真或真实 AIS 数据，至少包含 MMSI、经纬度或本地坐标、航速 SOG、航向 COG/heading、时间戳，并转换到 `world` 或 `base_link` 坐标系后再和现有 track 关联。

## 目录说明

- `src/usv_description/urdf/wamv_base.urdf.xacro`：船体、雷达、声呐、深度相机模型源文件；`enable_c3_mmwave` 控制 C3 多高度雷达。
- `src/usv_description/urdf/wamv.urdf`：由 xacro 生成的静态 URDF。
- `src/usv_description/rviz/default.rviz`：RViz 默认配置。
- `src/usv_bringup/launch/sim.launch.py`：Gazebo、robot_state_publisher、RViz、本船跟随和传感器节点启动入口。
- `src/usv_bringup/worlds/ocean_fog.world`：大雾海面 Gazebo world。
- `src/usv_bringup/config/perception.yaml`：感知、跟踪、门控相机和动态目标参数。
- `src/usv_bringup/models/best.onnx`：默认识别模型，随包安装。
- `src/usv_perception/`：多模态感知节点；波浪浮力、动态目标、多源融合跟踪、本船目标跟随、门控相机识别、无人机巡航和 ROS 图像录制均为 C++。
- `src/lidar_robot/`：从 `c3` 合入的毫米波原始扫描转换与调试节点。
- `src/depth_image_to_pointcloud2/`：从 `c3` 合入的 RGB-D 去雾和点云生成节点，已改为支持话题参数。

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

船载门控相机和无人机门控相机识别默认共用这个 launch 参数。若以后要分别使用两个模型，改这里：

```text
src/usv_bringup/launch/sim.launch.py
```

`src/usv_bringup/config/perception.yaml` 里仍保留 `yolo_model_path` 字段，但默认留空，由 launch 文件按包内模型路径覆盖。

训练后建议导出 ONNX：

```bash
yolo export model=best.pt format=onnx imgsz=640 opset=12
```

如果类别改变，必须同步修改：

```text
src/usv_bringup/config/perception.yaml
```

例如当前模型是：

```yaml
class_names: [buoy, fishing_boat]
```

当前效果：

![USV simulation](image.png)
