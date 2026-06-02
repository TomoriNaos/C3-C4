# `/home/hu/STF_Dataset` 本地数据说明

STF 是真实门控相机数据集子集，主要价值是提供真实 gated NIR 三切片外观、标签和相机标定。它不是海上船舶数据集，不能直接证明当前海上 6 类模型的最终效果。

当前 USV 工程里已经实现的使用方式和开关见 `Readme.md`。本文只保留 STF 数据本身、转换方式和后续未完成用法。

## 目录内容

```text
/home/hu/STF_Dataset
├── gated0_rect8                 312 张 PNG，1280x720，8-bit 灰度
├── gated1_rect8                 312 张 PNG，1280x720，8-bit 灰度
├── gated2_rect8                 312 张 PNG，1280x720，8-bit 灰度
├── gated_labels_TMPv2           312 个 KITTI 风格 gated 标签
├── cam_left_labels_TMP          300 个左目普通相机标签
├── labeltool_labels             300 个天气/道路状态 JSON
├── labeltool_labels_refined     300 个修正后的天气/道路状态 JSON
├── calib_gated_bwv.json         gated 相机内参
├── calib_cam_stereo_left.json   左目相机内参
├── calib_cam_stereo_right.json  右目相机内参
└── calib_tf_tree_full.json      传感器外参树
```

三张 gated 图同名对应同一帧：

```text
gated0_rect8/xxx.png
gated1_rect8/xxx.png
gated2_rect8/xxx.png
```

可理解为近、中、远不同门控响应。用于伪彩色训练时常见合成方式是：

```text
R = gated0
G = gated1
B = gated2
```

这不是恢复纹理，而是把距离门控响应编码进 3 个通道。

## 标签格式

`gated_labels_TMPv2/*.txt` 接近 KITTI 标签格式：

```text
class truncation occlusion alpha x1 y1 x2 y2 h w l x y z yaw ...
```

常用字段：

```text
class:
  PassengerCar / Pedestrian / LargeVehicle / Obstacle 等

x1 y1 x2 y2:
  gated 图像中的 2D 框

h w l:
  3D 尺寸

x y z:
  gated 相机坐标系下目标位置，z 约等于前向距离
```

STF 是道路/车载域，类别和海上任务不一致。建议用于：

```text
1. 学真实 gated 图像风格。
2. 预训练 gated_object/objectness。
3. 校验三切片范围分布和相机投影。
```

不建议直接用于：

```text
1. 最终海上 vessel/fishing_boat/buoy 分类结论。
2. 替代 Gazebo 船载相机内参。
```

## 相机内参

`calib_gated_bwv.json` 中 gated 相机内参约为：

```text
width  = 1280
height = 720
fx = 2322.4
fy = 2322.4
cx = 667.777
cy = 261.144
```

USV 仿真相机不能直接套用 STF 内参。当前程序的定位优先级应这样理解：

```text
1. 仿真中优先使用 ROS 发布的 /gated_camera/camera_info。
2. 如果图像尺寸变化，按图像宽高比例缩放内参。
3. 如果没有 camera_info，才看 perception.yaml 里的手动内参。
4. 如果手动内参也没有，最后退回 camera_horizontal_fov。
```

STF 内参只适合离线 STF 实验和真实 gated 数据训练时使用。

## 转成 YOLO 格式

把三切片合成伪彩色图，并生成 YOLO 标签：

```bash
cd /home/hu/usv_ws
python3 scripts/stf_to_yolo_gated.py \
  --dataset /home/hu/STF_Dataset \
  --out /home/hu/STF_YOLO_pseudo \
  --class-mode mapped \
  --val-every 5
```

`mapped` 会把 STF 类别整理为：

```text
vehicle
pedestrian
obstacle
```

如果只想学“门控图里哪里有实体目标”，用通用 objectness：

```bash
python3 scripts/stf_to_yolo_gated.py \
  --dataset /home/hu/STF_Dataset \
  --out /home/hu/STF_YOLO_objectness \
  --class-mode generic \
  --val-every 5
```

输出：

```text
/home/hu/STF_YOLO_pseudo
├── images/train
├── images/val
├── labels/train
├── labels/val
├── data.yaml
└── summary.json
```

这里的“转 YOLO”只是为了方便训练和验证，不代表后续只能用 YOLO。做 BEV、Pseudo-LiDAR、Gated3D 时，也可以直接读取三张灰度切片和原始标签。

## 离线发布 STF 切片

可以不用 Gazebo，直接发布 STF 图像话题，测试三切片节点：

```bash
source /opt/ros/humble/setup.bash
source /home/hu/usv_ws/install/setup.bash

ros2 run usv_perception gated_slice_fusion_recognizer
```

另开终端：

```bash
cd /home/hu/usv_ws
python3 scripts/publish_stf_slices.py \
  --dataset /home/hu/STF_Dataset \
  --rate 2.0 \
  --limit 30
```

观察：

```bash
ros2 topic echo /gated_camera/stf_fusion/status
ros2 topic hz /gated_camera/stf_detection_points
```

这个测试只能说明节点是否能处理真实三切片响应，不能给出海上 6 类识别准确率。

## 后续未完成用法

STF 可以继续用于下面几件事：

```text
1. 预训练 gated_object 检测器，再用海上仿真图微调 6 类。
2. 训练 Gated3D/3D-CNN，让网络直接学习 near/mid/far 切片轴。
3. 做 Pseudo-LiDAR/BEV 的道路域预训练，再迁移到海上几何目标。
4. 用 STF 的 3D 标签检查图像框到空间点的投影误差。
5. 做真实雾天 gated 图像的去噪、归一化、强度增强实验。
```

要真正服务当前海上跟踪任务，还需要补齐：

```text
海上 gated 标注数据，或更丰富的 Gazebo 三切片仿真数据；
每个目标的 2D/BEV/3D 真值；
不同雾浓度、距离、遮挡、光照下的 rosbag；
与 /tracking_metrics 对齐的离线评估脚本。
```
