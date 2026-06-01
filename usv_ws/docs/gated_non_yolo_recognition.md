# 不只用 YOLO 的门控相机识别路线

当前工程保留原来的 ONNX/YOLO 识别，同时新增两条不替换原方法的旁路：

```text
1. STF 三切片目标响应检测
   /gated_camera/slice_near + slice_mid + slice_far
   -> gated_slice_fusion_recognizer
   -> /gated_camera/stf_detection_points

2. 深度图 BEV 几何聚类检测
   /depth_camera/depth/image_raw + /depth_camera/camera_info
   -> gated_bev_detector
   -> /gated_camera/bev_detection_points
```

两路输出都会进入 `radar_sonar_tracker`，但不会覆盖原来的 `/gated_camera/detection_points`。

所有门控目标点云都保持同一个 `sensor_msgs/PointCloud2` 字段格式，方便下游跟踪器、RViz 和你之前的调试脚本继续使用：

```text
topic:
  /gated_camera/detection_points
  /gated_camera/stf_detection_points
  /gated_camera/bev_detection_points
  /uav/gated_camera/detection_points

fields:
  x          目标相对船体前方距离，m
  y          目标相对船体左右位置，m
  z          目标高度，m
  intensity  置信度
  class_id   类别 ID，0=vessel，1=fishing_boat，2=buoy，3=floating_obstacle，4=debris_container，5=platform/unknown_obstacle
  bbox_cx    检测框中心 x，pixel
  bbox_cy    检测框中心 y，pixel
  bbox_w     检测框宽度，pixel
  bbox_h     检测框高度，pixel
```

BEV 几何检测没有天然 2D 图像框，所以 `bbox_*` 字段填 0；这样格式不变，下游仍然能按 `x/y/z/intensity/class_id` 融合。

## 1. 伪彩色三切片检测

这是最容易落地的一条路。把门控三切片当成 3 个通道：

```text
R = near slice
G = mid slice
B = far slice
```

这样网络看到的不是普通纹理，而是“目标在距离门控上的响应分布”。你现在可以先用脚本生成可标注图片：

```bash
./scripts/capture_pseudocolor_dataset.sh \
  /home/hu/usv_captures/pseudocolor_single \
  /home/hu/usv_captures/pseudocolor_complex \
  96 96
```

输出分两类：

```text
/home/hu/usv_captures/pseudocolor_single   单个物体、不同角度
/home/hu/usv_captures/pseudocolor_complex  多目标、遮挡、复杂场景
```

标注时仍然按可见目标外轮廓框。建议类别先保持和海上任务一致：

```text
vessel
fishing_boat
buoy
floating_obstacle
debris_container
platform
```

训练 YOLO 时，把这两类伪彩色图片和 STF 转出来的 `gated_object`/`vehicle` 数据混合训练，可以让模型先适应 gated 图像风格，再适应海上类别。

这里说“转成 YOLO”不是要求你永远只能用 YOLO，而是把 STF 的真实 gated 三切片和标签整理成一个最容易训练/验证的格式。它有两个用途：

1. 训练伪彩色三切片检测器：输入是 near/mid/far 合成图，输出 2D 框。
2. 学真实 gated 图像的目标响应：先用 `gated_object` 做目标性预训练，再用海上仿真图微调到船、浮标、漂浮物等类别。

如果后面走 BEV、Pseudo-LiDAR 或 3D-CNN，这些 STF 切片和标签仍然可以作为预训练数据，不必绑定 YOLO。

## 2. 伪点云 / Pseudo-LiDAR

如果有深度图和相机内参，可以把每个像素投影到三维空间：

```text
X = (u - cx) * d / fx
Y = (v - cy) * d / fy
Z = d
```

工程里新增的 `gated_bev_detector` 就是这条路线的轻量基线：它把深度图投影到船体坐标，再筛掉水面以下/过高/过远点，最后在 BEV 网格里做连通域聚类，输出目标点。

它不是深度学习模型，不需要训练，优点是能快速验证“只靠船载门控相机 + 深度”是否能给出稳定几何目标。缺点是复杂遮挡和类别识别能力弱。

后续要训练时，可以把投影后的点云保存成 KITTI/OpenPCDet 格式，然后训练：

```text
PointPillars
SECOND
PV-RCNN
CenterPoint
```

训练数据需要：

```text
点云：每帧 N x (x, y, z, intensity)
3D 框：中心 x/y/z、尺寸 l/w/h、yaw、类别
标定：相机内参、外参、坐标系定义
```

你的仿真环境可以从 Gazebo 模型真值生成 3D 框；真实 STF 数据里 `gated_labels_TMPv2` 已有相机系 3D 位置和尺寸，可作为车载预训练参考。

## 3. BEV 检测

BEV 的核心是把点云压到俯视图网格。每个网格可以存：

```text
最大高度
平均高度
点密度
强度均值
不同高度切片占用
```

可训练模型包括：

```text
PIXOR
PointPillars 的 BEV backbone
CenterPoint BEV heatmap
```

为什么适合雾天：透视图里雾和目标会沿视线叠在一起；BEV 里它们在距离轴上分开。雾点通常是稀疏、散乱、近距离漂浮；船体/浮标/障碍物会形成更稳定的空间聚集。

训练步骤：

1. 从仿真导出深度图、相机内参和 Gazebo 真值框。
2. 投影成点云并过滤水面。
3. 栅格化成 BEV 特征。
4. 用真值框生成 BEV heatmap 或 anchor 标签。
5. 训练 CenterPoint/PIXOR 风格网络。
6. 输出 2D BEV 框或中心点，再交给跟踪器。

## 4. 3D-CNN / Gated3D

不要把三张 gated slice 简单看成 RGB，也可以看成一个很薄的体数据：

```text
Tensor = [C=1, D=3, H, W]
```

`D=3` 是距离门控切片轴。可以用：

```text
3D ResNet
3D U-Net
轻量 3D Conv + 2D detection head
```

训练标签可以先用 2D 框；模型输出仍然是 2D 框或分割 mask。比普通 YOLO 的优势是，3D 卷积能学习“沿距离切片的强度变化”，例如雾的连续衰减和硬目标的突变响应。

训练方法：

1. 输入不要存成普通 JPG，而是保存三张原始灰度切片。
2. Dataset 读取时堆叠成 `[3,H,W]` 或 `[1,3,H,W]`。
3. 先做二分类 `objectness` 或分割，再做类别检测。
4. 用 STF 做 gated 风格预训练，用你的仿真海上数据做类别微调。

## 5. 不用 YOLO 时如何训练

可以分三类训练：

1. 监督式 3D 检测  
   需要 3D 框标签。仿真里可由 Gazebo 真值自动生成；真实数据较难。

2. BEV heatmap 检测  
   只需要目标中心和尺寸，标签比完整 3D 框简单。适合你的“目标跟踪中心点”任务。

3. 分割/目标性训练  
   只标 mask 或 2D 框，训练 `objectness`。对真实 gated 图像最稳，因为纹理弱、类别边界不清。

建议实际路线：

```text
第一阶段：STF 三切片 -> gated_object/objectness
第二阶段：仿真伪彩色海上图 -> vessel/buoy/obstacle
第三阶段：深度投影 BEV -> 目标中心点
第四阶段：把视觉检测、BEV 几何、雷达、声呐、AIS 交给 tracker 融合
```

这样比只押注一个 YOLO 模型更稳。

## 6. 如何切换和融合识别方法

当前不是“普通相机”和“伪彩色”二选一，而是多路并行后融合：

```text
普通/去雾图像 ONNX:
  /gated_camera/image_raw
  -> gated_camera_recognizer
  -> /gated_camera/detection_points

伪彩色三切片 ONNX:
  /gated_camera/range_view
  -> pseudocolor_gated_camera_recognizer
  -> /gated_camera/pseudocolor/detection_points

STF 三切片响应旁路:
  /gated_camera/slice_near + slice_mid + slice_far
  -> gated_slice_fusion_recognizer
  -> /gated_camera/stf_detection_points

深度/BEV 几何:
  /depth_camera/depth/image_raw + /depth_camera/camera_info
  -> gated_bev_detector
  -> /gated_camera/bev_detection_points

融合:
  radar_sonar_tracker
  -> /tracked_objects
```

`radar_sonar_tracker` 会给不同来源一个权重：普通 ONNX、伪彩色 ONNX、STF 三切片响应和 BEV 几何各自输出 `intensity` 置信度，融合时再乘来源权重并做轨迹关联。

常用切换方式：

```bash
# 关闭伪彩色 ONNX
ros2 launch usv_bringup sim.launch.py pseudocolor_gated_yolo:=false

# 关闭 STF 三切片旁路
ros2 launch usv_bringup sim.launch.py stf_gated_fusion:=false

# 关闭 BEV 几何旁路
ros2 launch usv_bringup sim.launch.py gated_bev_detection:=false
```

默认模型约定：

```text
src/usv_bringup/models/best.onnx   普通/去雾图像
src/usv_bringup/models/best1.onnx  伪彩色 range_view 图像和无人机 range_view
```

## 7. 模拟真实门控还可以加强什么

真实门控相机主要看主动近红外回波和距离门控响应，纹理不是核心；雾天里纹理本来就会弱。只用伪彩色图训练检测，能基本达到“学会门控图像下哪里像目标”的目的，但类别细分会比普通 RGB 难。

除了深度相机，还可以加强这些模态或物理效果：

```text
1. 门控切片物理模型：不同距离门的曝光窗口、回波衰减、近场后向散射。
2. 近红外材质反射：船体、浮标、木箱、金属杆在 NIR 下亮度不同。
3. 毫米波雷达：远距离船体和大障碍物稳定，抗雾。
4. 声呐：近距离避障和水面/水下障碍补充。
5. AIS：对合作船舶给身份、航速、航向，帮助选择跟踪目标。
6. 时序信息：同一目标连续帧位置更稳定，可用 tracker 或时序网络降低误检。
```

所以推荐路线是：伪彩色图负责视觉检测，深度/BEV负责几何定位，雷达/声呐/AIS负责稳定跟踪。
