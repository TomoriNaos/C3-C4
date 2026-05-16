# Target Fusion

## 1. 目标

`TargetFusionProcessor` 负责将 TC 和 GC 的原始感知数据融合为统一目标观测，供 `target_processor_node` 调用。

## 2. 输入输出

### 输入

- TC bbox：`[x, y, w, h]`
- TC 点云：`PointCloud2`
- GC 点云：`PointCloud2`
- 时间戳
- 目标类型先验 `target_type_hint`

### 输出

- `TargetObservation`
- `GimbalVisualCommand`
- 丢失状态

## 3. 类职责

### `TargetFusionProcessor`

职责：
- 时间同步
- ROI 裁剪
- 点云降噪
- 双传感器融合
- 目标中心估计
- 跟踪与丢失判定
- 生成观测输出

### 核心接口

```text
updateTcBbox()
updateTcPointCloud()
updateGcPointCloud()
process()
buildObservation()
```

## 4. 处理链路

### 时间同步策略

```text
1. 为 TC 和 GC 各维护一个时间戳有序缓冲区，保留最近 200 ms 数据。
2. 任一新消息到达时，触发最近邻匹配：
   - 在另一传感器缓冲区中查找 |Δt| <= 100 ms 的最近帧。
   - 匹配成功则进入融合流程。
3. 若 TC 先到、GC 未到：
   - 最多额外等待 50~70 ms。
   - 超时后不再阻塞，直接降级输出。
4. 若缓冲区数据超过 200 ms，直接丢弃。
```

### 处理链路

```text
1. bbox 引导 ROI 提取
   将 bbox 投影到 TC 点云，裁剪出 ROI 点云。

2. ROI 预处理
   去无效点、深度限幅、体素降采样。

3. TC/GC 融合
   对齐后的 TC 与 GC 做最近邻/方差加权融合。

4. 目标估计
   聚类得到主簇，计算目标中心、距离和质量指标。

5. 跨帧跟踪
   用 CV 模型做 EKF/Predict-Update，维护轨迹和丢失计数。

6. 输出
   生成 `TargetObservation` 和 `GimbalVisualCommand`。
```

## 5. 节点关系

- `target_processor_node` 负责调用本类



