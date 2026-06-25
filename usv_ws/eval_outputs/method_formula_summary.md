# USV/C3 工程数学公式与算法方法汇总

本文汇总当前 `usv_ws` 代码中实际使用的主要数学公式、算法和工程方法，便于写说明书、PPT 和答辩。路径以 `usv_ws` 为根目录。

## 1. 坐标、姿态与相机投影

### 1.1 四元数转偏航角

使用位置：

- `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp`
- `src/usv_perception/src/ais_simulator.cpp`
- `src/usv_perception/src/tracking_evaluator.cpp`
- `src/usv_perception/src/uav_patrol_controller.cpp`
- `src/usv_perception/src/usv_target_follower.cpp`

公式：

```text
yaw = atan2(2 * (w*z + x*y), 1 - 2 * (y^2 + z^2))
```

作用：从 Gazebo/ROS pose 四元数中提取船体或目标的平面航向，用于目标相对坐标、AIS 航向、无人机朝向、跟踪控制。

### 1.2 world 坐标转 base_link 相对坐标

使用位置：

- `src/usv_perception/src/ais_simulator.cpp`
- `src/usv_perception/src/tracking_evaluator.cpp`
- `src/usv_perception/src/usv_target_follower.cpp`
- `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp`

公式：

```text
dx = world_x - usv_x
dy = world_y - usv_y

rel_x =  cos(yaw) * dx + sin(yaw) * dy
rel_y = -sin(yaw) * dx + cos(yaw) * dy
```

作用：把世界坐标下的目标、AIS 或神经网络候选点转到无人船 `base_link`，统一进入跟踪、热力图和控制。

### 1.3 bbox 中心深度解算三维点

使用位置：

- `src/usv_perception/src/gated_camera_recognizer.cpp`

公式：

```text
lateral  = (u - cx) * d / fx
vertical = (v - cy) * d / fy

local_x = d
local_y = -lateral
local_z = -vertical
```

其中 `(u, v)` 是检测框中心，`d` 是 bbox 中心区域深度中位数，`fx/fy/cx/cy` 来自 `CameraInfo` 或视场角回退计算。

视场角回退公式：

```text
fx = image_width / (2 * tan(horizontal_fov / 2))
fy = fx
cx = (image_width  - 1) / 2
cy = (image_height - 1) / 2
```

作用：把门控相机、普通相机、深度相机的 2D 检测框转成目标点云位置。

### 1.4 深度图转点云

使用位置：

- `src/depth_image_to_pointcloud2/src/depth_image_to_pointcloud2_node.cpp`

公式：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth(u, v)
```

作用：把深度图转换为 `PointCloud2`，供深度模态和 BEV 几何旁路使用。

## 2. 门控相机与图像增强

### 2.1 门控三切片权重

使用位置：

- `src/usv_perception/src/gated_camera_recognizer.cpp`

方法：近、中、远三个距离门控切片按距离范围生成响应。每个切片使用以门控中心为均值的高斯距离权重。

公式：

```text
gate_center = (gate_near + gate_far) / 2
sigma = (gate_far - gate_near) / 2.35
weight = exp(-0.5 * ((depth - gate_center) / sigma)^2)
slice = gray * weight * gain
```

伪彩色合成：

```text
R = near_slice
G = mid_slice
B = far_slice
```

作用：保留门控相机“不同距离回波切片”的表达，使 `best1.onnx` 在伪彩色输入上识别目标。

### 2.2 暗通道/透射率去雾

使用位置：

- `src/usv_perception/src/gated_camera_recognizer.cpp`
- `src/depth_image_to_pointcloud2/src/depth_image_to_pointcloud2_node.cpp`

暗通道：

```text
dark(x) = min_c(I_c(x) / A_c)
```

透射率估计：

```text
t(x) = 1 - omega * dark(x)
t(x) = max(t(x), t_min)
```

深度辅助透射率：

```text
t_depth = exp(-beta * depth)
t = min(t, max(t_depth, t_min))
```

图像恢复：

```text
J_c(x) = (I_c(x) - A_c) / max(t(x), t_min) + A_c
output = strength * J + (1 - strength) * I
```

作用：普通相机和深度相机 RGB 输入先增强，再给 `best.onnx` 识别，降低雾气对对比度和颜色的影响。

## 3. YOLO 检测与点云字段

### 3.1 ONNX YOLO 输出解析

使用位置：

- `src/usv_perception/src/gated_camera_recognizer.cpp`
- `scripts/evaluate_yolo_onnx_dataset.py`

方法：

```text
score = objectness * max(class_score)
```

若模型没有 objectness 列：

```text
score = max(class_score)
```

bbox 解码：

```text
x1 = cx - w / 2
y1 = cy - h / 2
x2 = cx + w / 2
y2 = cy + h / 2
```

随后使用 NMS 去除高度重叠框，类别顺序固定为：

```text
0 buoy
1 debris_container
2 fishing_boat
3 floating_obstacle
4 platform
5 vessel
```

### 3.2 轮廓兜底检测

使用位置：

- `src/usv_perception/src/gated_camera_recognizer.cpp`

方法：

```text
mask = (HSV.saturation > 45) AND (HSV.value > 35)
mask -> median blur -> morphology open -> dilate
```

置信度：

```text
score = min(0.92, 0.38 + 18 * contour_area / image_area)
```

作用：当 YOLO 空框时，对高饱和/高亮目标做兜底检测，减少偶发漏检。

### 3.3 检测点云字段

使用位置：

- `src/usv_perception/src/gated_camera_recognizer.cpp`
- `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp`

字段：

```text
x, y, z
intensity/confidence
class_id
source_id
bbox_cx, bbox_cy, bbox_w, bbox_h
```

作用：同时传递几何位置、类别、置信度和原始 2D bbox，方便融合层和可视化层回溯检测来源。

## 4. BEV 与几何旁路

### 4.1 BEV 栅格聚类

使用位置：

- `src/usv_perception/src/gated_bev_detector.cpp`

方法：

1. 深度像素反投影到三维点。
2. 过滤距离、高度、横向范围。
3. 投影到鸟瞰二维栅格。
4. 使用 4 邻域连通域聚类。
5. 对每个聚类输出中心点和置信度。

置信度：

```text
confidence = clamp(0.32 + 0.012 * point_count + 0.035 * cell_count, 0.35, 0.94)
```

作用：在视觉纹理不稳定时，用几何聚集性提供目标候选点。

### 4.2 门控三切片几何融合

使用位置：

- `src/usv_perception/src/gated_slice_fusion_recognizer.cpp`

方法：

```text
max_response = max(near, mid, far)
contrast = max(abs(mid - near), abs(far - mid))
fused = 0.72 * max_response + 0.55 * contrast
```

随后使用 Gaussian blur、Otsu 阈值、开闭运算和轮廓聚类提取目标区域。

作用：不依赖 YOLO 纹理分类，仅利用门控切片响应差异输出几何候选点。

## 5. 毫米波雷达与声呐

### 5.1 毫米波雷达仿真与 scan 转点云

使用位置：

- `src/usv_perception/scripts/mmwave_scan_converter.py`

雷达目标功率：

```text
range_decay(r) = 1 / (1 + alpha * r^2)
beam_gain(theta) = max(0.35, cos(theta))
power = base_rcs * reflectivity * range_decay(r) * beam_gain(theta) + noise
```

方向融合权重：

```text
direction_weight = min_weight + (1 - min_weight) * exp(-distance^2 / (2 * sigma^2))
fused_weight = power * direction_weight
```

SNR：

```text
snr_db = 10 * log10(1 + power * 25)
```

极坐标转点：

```text
horizontal_range = range * cos(elevation)
x = horizontal_range * cos(azimuth)
y = horizontal_range * sin(azimuth)
z = range * sin(elevation)
```

作用：从多高度、多方向毫米波雷达 scan 中生成目标候选点云。

### 5.2 声呐 LaserScan 转点云与三帧融合

使用位置：

- `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp`

单束转换：

```text
x = r * cos(angle + mount_yaw)
y = r * sin(angle + mount_yaw)
z = 0
```

四扇区方向：

```text
front = 0
right = -pi/2
back  = pi
left  = pi/2
```

三帧滑动窗口：每个扇区保留最近 `sonar_fusion_window_frames=3` 帧，融合后再进入 C3 缓存池。

作用：降低单帧声呐稀疏和偶发无回波造成的漏检。

## 6. C3 多模态缓存、热力图与融合

### 6.1 缓存池与时间对齐

使用位置：

- `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp`

方法：

```text
保留最近 buffer_keep_s 秒内数据
取当前融合周期时间 stamp
若 abs(point_stamp - stamp) <= sync_tolerance_s，则参与本轮融合
所有点统一到 base_link 坐标
```

输出四类对齐点云和一个融合点云：

```text
/c3/buffer/radar_cloud
/c3/buffer/sonar_cloud
/c3/buffer/vision_cloud
/c3/buffer/depth_cloud
/c3/buffer/integrated_cloud
```

### 6.2 规则热力图

使用位置：

- `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp`

当前没有训练神经网络热力图，默认使用规则投票热力图；神经网络旁路通过 `enable_nn_heatmap_bypass` 保留。

点投票：

```text
vote = source_weight(source_id) * clamp(confidence, 0.05, 1.0)
```

每个模态在同一栅格内限幅：

```text
cell_score += min(source_cell_vote, heatmap_source_cell_cap)
```

已确认目标抑制：

```text
if distance(point, detected_object) < detected_suppression_radius:
    vote *= 0.08
```

候选点：

```text
candidate = argmax(grid_score)
if max_score < heatmap_threshold:
    no candidate
```

作用：把多源点云压缩成 `1m x 1m` 空间置信度图，选择最可能出现目标的坐标，并避免重复派无人机去已确认目标。

### 6.3 源权重

使用位置：

- `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp`

当前权重：

```text
radar             0.70
sonar             0.60
gated/uav gated   1.25
normal camera     1.05
stf gated         0.85
bev               0.55
depth camera YOLO 1.00
depth point cloud 0.35
```

作用：让带语义的视觉检测权重大于几何/稀疏点云，同时保留毫米波雷达和声呐的远距离、全天候优势。

### 6.4 二次确认与 detected 结构体

使用位置：

- `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp`

方法：

1. 热力图输出最大置信度坐标。
2. 对候选坐标附近 `confirmation_radius` 内的点做加权平均。
3. 根据门控、深度、普通相机或 UAV 视觉点确定语义类别。
4. 若与已有对象距离小于 `object_association_radius`，更新已有对象；否则创建新 `object_id`。

检测对象字段：

```text
object_id: 第 n 个被确认目标的计数 ID
class_id: 六类语义类别 ID
name: 类别名称
x/y/z: EKF 滤波后位置
vx/vy/vz: 速度估计
predicted_x/y: 未来位置
confidence: 综合置信度
```

## 7. 非线性卡尔曼滤波与传统跟踪

### 7.1 C3 detected 目标 EKF

使用位置：

- `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp`

状态量：

```text
x = [px, py, pz, vx, vy, vz]^T
```

预测模型：

```text
px = px + vx * dt
py = py + vy * dt
pz = pz + vz * dt
P  = F * P * F^T + Q
```

其中：

```text
F[0,3] = dt
F[1,4] = dt
F[2,5] = dt
```

观测模型使用极坐标距离、方位角和高度：

```text
h(x) = [sqrt(px^2 + py^2), atan2(py, px), pz]^T
z    = [sqrt(mx^2 + my^2), atan2(my, mx), mz]^T
```

雅可比：

```text
r = sqrt(px^2 + py^2)
q = px^2 + py^2

H[0,0] = px / r
H[0,1] = py / r
H[1,0] = -py / q
H[1,1] =  px / q
H[2,2] = 1
```

更新：

```text
S = H * P * H^T + R
K = P * H^T * S^-1
x = x + K * (z - h(x))
P = (I - K * H) * P
```

作用：对已确认目标做动态定位和未来位置预测。

### 7.2 传统 alpha-beta 跟踪器

使用位置：

- `src/usv_perception/src/radar_sonar_tracker.cpp`

预测：

```text
x = x + vx * dt
y = y + vy * dt
```

更新：

```text
rx = measurement_x - x
ry = measurement_y - y

x  = x  + alpha * rx
y  = y  + alpha * ry
vx = vx + beta * rx / dt
vy = vy + beta * ry / dt
```

参数：

```text
alpha = clamp(0.25 + 0.45 * confidence, 0.25, 0.70)
beta  = clamp(0.08 + 0.22 * confidence, 0.08, 0.30)
```

作用：保留雷达、声呐、视觉、AIS 的传统融合跟踪路径。

## 8. 无人机飞控与船体跟随

### 8.1 无人机远程确认

使用位置：

- `src/usv_perception/src/uav_patrol_controller.cpp`
- `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp`

方法：

1. C3 热力图候选目标距离超过近距离阈值时，发布 `/c3/drone/goal`。
2. 无人机接收目标点，以固定高度和速度向目标点移动。
3. 到达目标附近后，无人机门控相机使用 `best1.onnx` 再次识别。
4. UAV 点云回传到 C3，参与二次确认和 EKF 更新。

目标飞行方向：

```text
heading = atan2(goal_y - uav_y, goal_x - uav_x)
step = min(distance_to_goal, speed / update_rate)
```

### 8.2 无人船目标跟随与避障

使用位置：

- `src/usv_perception/src/usv_target_follower.cpp`

预测瞄准点：

```text
aim_x = target_x + target_vx * lead_time
aim_y = target_y + target_vy * lead_time
lead_distance <= max_lead_distance
```

航向误差：

```text
bearing = atan2(aim_y, aim_x)
yaw_rate = clamp(yaw_gain * bearing, -max_yaw_rate, max_yaw_rate)
```

线速度：

```text
speed = clamp(speed_gain * (range - desired_standoff), 0, max_speed)
```

避障：在前向窗口内计算障碍物横向压力和距离接近度，形成偏航修正；若进入硬停止距离，线速度降为 0。

## 9. AIS 辅助定位

### 9.1 AIS 仿真

使用位置：

- `src/usv_perception/src/ais_simulator.cpp`
- `src/usv_perception/src/radar_sonar_tracker.cpp`

输出内容：

```text
MMSI
relative x/y
vx/vy
SOG = sqrt(vx^2 + vy^2)
COG = atan2(vy, vx)
heading = target yaw
class_id
confidence
```

作用：AIS 不直接产生图像检测框，而是给传统跟踪器提供远距离目标先验，提高目标船选择和 ID 稳定性。

## 10. 评价指标

### 10.1 YOLO 检测指标

使用位置：

- `scripts/evaluate_yolo_onnx_dataset.py`

IoU：

```text
IoU = intersection_area / union_area
```

匹配：同类预测框和 GT 框按置信度排序，若 `IoU >= 0.50` 则记为 TP，否则为 FP；未匹配 GT 为 FN。

```text
Precision = TP / (TP + FP)
Recall    = TP / (TP + FN)
F1        = 2 * Precision * Recall / (Precision + Recall)
```

### 10.2 C3 跟踪评价

使用位置：

- `src/usv_perception/src/tracking_evaluator.cpp`
- `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp`

漏检率和误检率：

```text
miss_rate = FN / (TP + FN)
false_positive_rate = FP / (TP + FP)
```

CPA/TCPA：

```text
tcpa = clamp(-(relative_position dot relative_velocity) / |relative_velocity|^2, 0, horizon)
cpa  = |relative_position + relative_velocity * tcpa|
```

ID 切换：同一个真值目标前后匹配到不同 track ID 时计数加一。

作用：用于汇报目标检测精度、误检率、漏检率、实时性、定位误差和动态跟踪稳定性。

