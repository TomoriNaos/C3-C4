# Target Fusion

> 版本：v1.2.0  
> 更新日期：2026-05-19

## 1. 目标

`TargetFusionProcessor` 负责把 TC 和 GC 的原始数据融合成统一的目标观测，供 `target_processor_node` 使用。

---

## 2. 节点接口

### 输入

- `/tc/detection` -> `c3_drone_driver/msg/TcDetection`
- `/gc/points` -> `sensor_msgs/msg/PointCloud2`

### 输出

- `/target/observation_body` -> `c3_drone_driver/msg/TargetObservation`
- `/gimbal/visual_command` -> `c3_drone_driver/msg/GimbalVisualCommand`

---

## 3. 输入消息格式

### `TcDetection`

当前代码中，`bbox.data` 的约定是：

```text
[x, y, w, h, confidence, target_id, target_type]
```

同时还携带：
- `header.stamp`
- `header.frame_id`
- `cloud`（TC 的点云）

### `GC PointCloud2`

- 只需要有效的 `header.stamp`
- 点云宽高和 xyz 数据必须有效

---

## 4. 配置参数

来自 `config/target_fusion_default.yaml`，核心参数包括：
- 时间同步：`time_sync_threshold_s`、`pending_wait_s`、`buffer_keep_s`
- ROI：`roi_margin_px`、`image_width`、`image_height`
- 深度：`depth_min`、`depth_max`
- 跟踪：`smoothing_alpha`、`max_tracking_loss_frames`
- 置信度融合：`confidence_weight_detection`、`confidence_weight_cluster`、`confidence_weight_stability`

---

## 5. 处理流程

### 5.1 缓冲与同步

- TC 检测和 GC 点云分别进入缓冲区
- 只保留最近 `buffer_keep_s` 时间窗的数据
- `process(now)` 里先清理过期数据，再找最近的 TC / GC 组合
- 若 GC 未在同步窗口内到达：
  - 在 `pending_wait_s` 内继续等
  - 超时后返回丢失结果

### 5.2 ROI 提取

- 根据 bbox 构建 ROI
- 使用 `roi_margin_px` 扩展边界
- 从 TC 点云中遍历 ROI 像素对应点
- 去除无效点和越界深度点
- 点数不足则认为 ROI 无效

### 5.3 融合

- 若 TC 和 GC 都有 ROI 结果，则按噪声方差加权融合质心
- 若只有一路有效，则直接使用该路结果
- 结果会给出 `cluster_quality`

### 5.4 跟踪

- 内部维护一个单目标匀速跟踪器
- 有观测时更新位置、速度和稳定度
- 无观测时只做外推并降低稳定度

### 5.5 输出

- 构造 `TargetObservation`
- 构造 `GimbalVisualCommand`
- 若目标丢失，则只保留观测结果，视觉命令不再下发

---

## 6. 输出消息说明

### `TargetObservation`

当前实现写入的关键字段：
- `header.stamp`
- `t_usec`
- `obs_id`
- `track_id`
- `target_id`
- `frame = FRAME_BODY_DRONE`
- `target_type`
- `position`
- `range`
- `yaw`
- `pitch`
- `confidence`
- `source`（`SOURCE_TC_ONLY` / `SOURCE_GC_ONLY` / `SOURCE_FUSED`）
- `status`（`STATUS_VALID` / `STATUS_LOW_CONF` / `STATUS_LOST`）

### `GimbalVisualCommand`

- `yaw`
- `pitch`
- `confidence`

视觉命令的 yaw/pitch 直接来自观测方位，并乘以 `visual_cmd_gain`。

---

## 7. 处理边界

- 目前是单目标处理，不做多目标跟踪
- `track_id` 在当前实现中是单轨迹语义
- 如果 TC 和 GC 都不可用，`process()` 会返回空

---

## 8. 节点关系

`target_processor_node` 只负责调用 `TargetFusionProcessor`，并把结果发布出去；具体融合逻辑都在 `src/target_fusion_processor.cpp` 中。
