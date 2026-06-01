# `/home/hu/STF_Dataset` 本地数据说明与使用方法

这个目录是 Seeing Through Fog / STF 的门控相机子集，不是普通 RGB 相机数据。它的核心价值是：一帧数据有 3 张同名灰度门控切片，目标在不同距离切片中的响应不同，可以用来验证“真实 gated NIR 图像下，普通 RGB ONNX 检测器会退化”的问题。

## 当前目录内容

统计结果：

```text
/home/hu/STF_Dataset
├── gated0_rect8                 312 张 PNG，1280x720，8-bit 灰度
├── gated1_rect8                 312 张 PNG，1280x720，8-bit 灰度
├── gated2_rect8                 312 张 PNG，1280x720，8-bit 灰度
├── gated_labels_TMPv2           312 个 KITTI 风格标签，和 gated 图像同名
├── cam_left_labels_TMP          300 个左目普通相机标签
├── labeltool_labels             300 个天气/道路状态 JSON
├── labeltool_labels_refined     300 个修正后的天气/道路状态 JSON
├── calib_gated_bwv.json         gated 相机内参
├── calib_cam_stereo_left.json   左目相机内参
├── calib_cam_stereo_right.json  右目相机内参
└── calib_tf_tree_full.json      传感器外参树
```

总大小约 264MB，共 2152 个文件，其中 PNG 936 个、TXT 612 个、JSON 604 个。

## 三个 gated 切片怎么理解

同一帧会有三张图：

```text
gated0_rect8/2018-02-03_21-48-53_00100.png
gated1_rect8/2018-02-03_21-48-53_00100.png
gated2_rect8/2018-02-03_21-48-53_00100.png
```

它们是同一时刻、不同距离门控窗口的灰度响应。近处反光物、远处建筑/车灯、行人轮廓会在不同切片中强弱不同。为了训练普通 3 通道 YOLO，可以把它们合成为伪彩色：

```text
B = gated2 远距离响应
G = gated1 中距离响应
R = gated0 近距离响应
```

这不是“恢复纹理”，而是把距离切片编码进颜色通道，让检测网络学习三切片响应形态。

## 标注格式

`gated_labels_TMPv2/*.txt` 是类似 KITTI 的一行一目标格式。常见字段可以按下面理解：

```text
class truncation occlusion alpha x1 y1 x2 y2 h w l x y z yaw ...
```

其中：

- `class`：如 `PassengerCar`、`Pedestrian`、`LargeVehicle`、`Obstacle`。
- `x1 y1 x2 y2`：1280x720 gated 图像中的 2D 框。
- `h w l`：3D 尺寸。
- `x y z`：相机坐标系中的目标位置，`z` 基本就是前向距离。
- 后面的布尔值表示不同传感器/标注可见性状态，训练 2D 检测时通常不用。

我抽样统计了 `gated_labels_TMPv2` 中有效 3D 标签：

```text
有效 3D 标签：3433
距离 z：min 2.49m, p10 11.70m, p50 33.68m, p90 63.12m, max 280.10m
主要类别：
  PassengerCar 1730
  Pedestrian 1224
  Obstacle 110
  Vehicle 65
  LargeVehicle 45
```

这个距离分布说明：你的仿真门控切片不应该只覆盖近距离，`near/mid/far` 至少要覆盖约 10m、30m、60m 三段；当前程序使用 `[2,18]`、`[12,42]`、`[32,85]`，和这个子集的大多数目标距离是匹配的。

## 相机内参

`calib_gated_bwv.json` 的 gated 相机内参：

```text
width  = 1280
height = 720
fx = 2322.4
fy = 2322.4
cx = 667.777
cy = 261.144
```

我已经让 `gated_camera_recognizer` 优先订阅 `/gated_camera/camera_info` 做定位投影；如果没有 `camera_info`，才退回到水平视场角估计。这样比之前只靠 `camera_horizontal_fov` 更稳。

注意：仿真船上的门控相机不会直接套用 STF 数据集内参。当前处理顺序是：

1. Gazebo/ROS 发布 `/gated_camera/camera_info` 时，直接使用仿真相机自己的 `fx/fy/cx/cy`。
2. 如果图像被 resize，程序会按图像宽高比例缩放内参。
3. 如果没有 `camera_info`，才使用 `perception.yaml` 里的 `camera_fx/camera_fy/camera_cx/camera_cy`。
4. 如果这些手动内参也没有配置，最后退回到 `camera_horizontal_fov` 估算焦距。

STF 的 `fx=fy=2322.4` 只用于离线发布 STF 切片、理解真实 gated 数据尺度，或以后训练真实 gated 模型时做标定参考；它不是 USV 仿真相机的默认内参。

## 转成 YOLO 训练/测试集

新增脚本：

```bash
./scripts/stf_to_yolo_gated.py \
  --dataset /home/hu/STF_Dataset \
  --out /home/hu/STF_YOLO_pseudo \
  --class-mode mapped \
  --val-every 5
```

输出结构：

```text
/home/hu/STF_YOLO_pseudo
├── images/train/*.jpg
├── images/val/*.jpg
├── labels/train/*.txt
├── labels/val/*.txt
├── data.yaml
└── summary.json
```

`mapped` 模式会把车载 STF 类别统一成：

```text
vehicle, pedestrian, obstacle
```

如果只是想学真实 gated 图像中的“目标性”，不关心车/人/障碍分类，可以用：

```bash
./scripts/stf_to_yolo_gated.py \
  --dataset /home/hu/STF_Dataset \
  --out /home/hu/STF_YOLO_objectness \
  --class-mode generic \
  --val-every 5
```

这会把所有有效框统一成 `gated_object`。这个模式更适合迁移到海上：门控图像纹理本来就弱，先学“门控目标响应 + 框位置”，类别再交给你的仿真海上数据、雷达/AIS 或后级融合。

这里的“转成 YOLO”只是一个方便训练和验证的桥，不代表只能用 YOLO。STF 数据集给工程带来的优化主要有三点：

```text
1. 真实 gated 三切片外观：用来验证普通 RGB ONNX 在真实门控图上会退化。
2. 真实框标签和距离分布：用来确定 near/mid/far 门控范围和目标尺寸先验。
3. 相机标定文件：用来测试“图像框 -> 空间点”的投影公式，但不直接替代仿真相机内参。
```

如果你训练 YOLO，可以用 STF 合成伪彩色图；如果你训练 BEV/Pseudo-LiDAR/3D-CNN，也可以直接读取三个灰度切片和标签，不必生成 JPG。

## 和当前 USV 程序怎么结合

现在保留原来的 ONNX 识别链路：

```text
/gated_camera/image_raw -> gated_camera_recognizer -> /gated_camera/detection_points
```

新增了一个旁路三切片识别链路：

```text
/gated_camera/slice_near
/gated_camera/slice_mid
/gated_camera/slice_far
  -> gated_slice_fusion_recognizer
  -> /gated_camera/stf_detection_points
```

`radar_sonar_tracker` 会额外融合 `/gated_camera/stf_detection_points`。这不会替换原来的 ONNX，只是多给跟踪器一个“更像真实门控切片”的观测来源。

启动时默认开启：

```bash
ros2 launch usv_bringup sim.launch.py
```

如果要临时关闭新增旁路：

```bash
ros2 launch usv_bringup sim.launch.py stf_gated_fusion:=false
```

也可以不用 Gazebo，直接把 STF 切片发布成 ROS 图像话题，测试新增旁路节点：

```bash
source /opt/ros/humble/setup.bash
source /home/hu/usv_ws/install/setup.bash

ros2 run usv_perception gated_slice_fusion_recognizer
```

另开一个终端：

```bash
cd /home/hu/usv_ws
./scripts/publish_stf_slices.py --dataset /home/hu/STF_Dataset --rate 2.0 --limit 30
```

观察输出：

```bash
ros2 topic echo /gated_camera/stf_fusion/status
ros2 topic hz /gated_camera/stf_detection_points
```

`/gated_camera/stf_detection_points` 和原来的 `/gated_camera/detection_points` 保持同样的 `PointCloud2` 字段：

```text
x y z intensity class_id bbox_cx bbox_cy bbox_w bbox_h
```

这保证新增 STF 旁路只是多一个观测来源，不改变原先点云消息的下游使用方式。

## 识别策略建议

不要直接用普通 RGB 图片标注出来的 ONNX 去识别真实 gated 图像。真实门控图像是主动近红外响应，纹理少、对比方式不同，普通 RGB 模型会把很多目标漏掉。

更稳的路线是三层：

1. 继续保留当前普通 ONNX：用于仿真 RGB-like 门控图和可见光较好时的检测。
2. 训练伪彩色三切片 ONNX：输入 `gated0/gated1/gated2` 合成图，先训练 `gated_object` 或 `vehicle/pedestrian/obstacle`，再用你的海上仿真图微调到 `vessel/buoy/floating_obstacle`。
3. 用点云/雷达/声呐/AIS 做定位和类别稳定：门控图像负责给出“哪里有目标”，距离最好由深度、毫米波、声呐或三切片范围响应融合，不要只靠 2D 框猜距离。

如果以后有真实海上 gated 数据，最理想的训练输入不是单张灰度图，而是：

```text
channel 0 = near slice
channel 1 = mid slice
channel 2 = far slice
```

这比把三张图平均成一张灰度图更适合检测，因为距离信息没有被抹掉。

更多不使用 YOLO 的方案，例如 Pseudo-LiDAR、BEV、3D-CNN/Gated3D，见：

```text
docs/gated_non_yolo_recognition.md
```
