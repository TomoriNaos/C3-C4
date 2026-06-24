# 不使用 YOLO 的门控相机后续路线

本文件只写当前还没有完整实现、但值得继续做的识别方法。当前已经实现的普通/去雾 YOLO、伪彩色 YOLO、STF 三切片响应旁路、BEV 几何旁路、融合跟踪和准确率查看方法，统一放在 `Readme.md`。

当前已实现但仍是基线的方法：

```text
gated_slice_fusion_recognizer:
  三切片响应 + 轮廓/面积/范围估计，不是训练模型。

gated_bev_detector:
  深度图投影 + BEV 网格聚类，不输出类别框，只输出几何目标点。

radar_sonar_tracker:
  多源观测融合，但还不是完整 JPDA/UKF/多假设跟踪。

tracking_evaluator:
  能用 Gazebo 真值评估融合跟踪，不等于单个 BEV/STF 检测器的离线 mAP。
```

## 1. 学习型 BEV 检测

当前 `gated_bev_detector` 是手写几何聚类。下一步可以升级为学习型 BEV 检测，例如 PIXOR、CenterPoint 或简化 PointPillars BEV head。

需要的数据：

```text
输入:
  depth image 或伪深度点云
  camera_info 内参
  camera -> base_link 外参

标签:
  Gazebo 真值 3D 框或 BEV 框
  class: vessel / fishing_boat / buoy / obstacle / debris_container / platform
  center_x, center_y, length, width, yaw
```

建议实现步骤：

```text
1. 写 rosbag 或离线导出脚本，保存 depth image、camera_info、model_states。
2. 用 model_states 把每个 Gazebo 模型转成 base_link 下 BEV 真值框。
3. 把深度图投影成点云，再栅格化为 BEV 特征图。
4. 训练 CenterPoint 风格 heatmap：输出目标中心、尺寸和 yaw。
5. 推理节点输出 PointCloud2 或 MarkerArray，再接入 radar_sonar_tracker。
6. 用 /tracking_metrics 和离线 BEV IoU 同时评估。
```

优点：

```text
雾气在前视图里会遮挡纹理，但在 BEV 距离轴上更容易和真实障碍物分开。
```

难点：

```text
需要稳定的 3D/BEV 真值框。
海面目标高度小，水面反射和漂浮噪声会影响点云。
```

## 2. 伪深度点云 / 3D 点云检测

如果只用门控相机和深度图，可以把每个像素投影成点：

```text
X = (u - cx) * d / fx
Y = (v - cy) * d / fy
Z = d
```

然后训练点云检测器：

```text
PointPillars
SECOND
PV-RCNN
CenterPoint 3D
```

需要的数据格式一般接近 KITTI/OpenPCDet：

```text
points:
  N x (x, y, z, intensity)

annotations:
  class, center_x, center_y, center_z, length, width, height, yaw

calibration:
  camera intrinsics
  camera/base_link/world transforms
```

推荐先做仿真数据，因为 Gazebo 可以自动提供模型位置和尺寸。真实 STF 数据虽然有 3D 标签，但它是车载道路域，不是海上船舶域，适合预训练几何/objectness，不适合直接当最终海上模型。

## 3. Gated3D / 三切片 3D-CNN

伪彩色 YOLO 把三张门控切片合成一张 RGB 图，简单但会损失“切片轴”的物理含义。更像门控相机的做法是把输入看成一个薄体数据：

```text
input = [near, mid, far]
shape = [D=3, H, W] 或 [C=1, D=3, H, W]
```

可尝试网络：

```text
轻量 3D Conv + 2D detection head
3D ResNet objectness
3D U-Net segmentation
```

训练方式：

```text
1. 不把三切片先存成 JPG，直接保存 near/mid/far 三张灰度图。
2. Dataset 读取时堆叠成 3D tensor。
3. 第一阶段训练 objectness 或 mask，先让模型学会“哪里是实体目标”。
4. 第二阶段用海上仿真数据训练类别。
5. 推理输出 2D 框或 mask，再用深度/三切片范围估计投影为空间点。
```

优点：

```text
模型能学习沿距离切片的强度变化，理论上比把三切片硬塞成 RGB 更符合门控成像。
```

难点：

```text
需要自己写训练代码和导出 ONNX。
YOLO 现成工具链不能直接训练这种 D 维卷积结构。
```

## 4. 更正式的多目标跟踪

当前 tracker 是轻量关联和轨迹更新，适合仿真演示，但多目标靠近时可能 ID 跳变。

可升级方向：

```text
Kalman / EKF / UKF:
  状态 x, y, vx, vy, heading，可稳定速度估计。

JPDA:
  多个检测和多个 track 近距离交汇时，用概率关联减少 ID switch。

MHT:
  保留多个假设，适合复杂遮挡，但实现复杂。

AIS-guided association:
  AIS 的 MMSI、SOG、COG、heading 可作为强先验，帮助选择目标船。
```

建议先做 UKF/Kalman，再做 AIS 关联增强，最后再考虑 JPDA。

## 5. 训练与评估闭环

不使用 YOLO 时，也必须建立指标闭环，否则很难判断方法是否变好。

推荐指标：

```text
检测:
  BEV IoU / center distance / recall@distance / false positives per frame

跟踪:
  ID switches / track fragmentation / target lost time / average position error

航行安全:
  CPA / TCPA / minimum distance / collision-risk events
```

推荐实验组织：

```text
1. 单目标不同距离。
2. 多目标交叉。
3. 轻遮挡和重遮挡。
4. 大雾低对比。
5. 无人机先发现远处目标。
6. AIS 正常、AIS 延迟、AIS 错误三组。
```

每个场景记录 rosbag，并保存 Gazebo 真值，这样才能复现实验和写论文指标。
