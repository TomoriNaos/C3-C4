# 任务要求
## 功能要求

```
1. 获取门控相机和传统摄像机返回的数据，包括目标相对于无人机的位姿和物体类型
2. 提供云台控制节点，相机和门控相机作为客户端控制云台旋转实现追踪（如在接近目标时自动调用，控制装载门控相机(下称GC)和传统摄像机（下称TC）的转台旋转，实现对目标的持续跟踪）。云台控制采用yaw+pitch两轴，先做软件限位（可配置上下限与角速度限幅），不做机械细节建模。需要提供控制权仲裁：视觉跟踪优先级高于运动模块前馈控制，避免多端同时写入导致抖动。
3. MAVLink协议+远距离双向通信机制。接收主控发来的目标大致方位和具体命令，把摄像机识别到的数据发回主控
4. 运动模块，通过IMU和EKF2实现自主飞控，控制无人机飞到目标点，至于位姿信息什么的交给主控自己算吧
5. UWB通信能力，用于多无人机间协同（再议，先不做）
6. AIS接收模块，接收母船和同样搭载了AIS的船舶广播的AIS信息，提供给自身运动模块做辅助（再议，非主体部分）
```

## 任务拆解

### 通信机制

#### 1. 与母船间通信

**无人机接收**
- 命令：`start`, `back`, `close`
- 目标点坐标（相对母船）
- 协议：MAVLink + 远距离链路

**无人机发送**
- 目标ID、物体类型ID
- 相对无人机位姿/距离
- 置信度、时间戳、数据来源
- 协议：MAVLink + 远距离链路

---

#### 2. 与云台间通信

**2.1 云台控制命令（topic，仲裁）**

| 角色 | 数据 | 频率 | 说明 |
|------|------|------|------|
| 运动模块 (发送) | yaw, pitch 前馈命令 | 高频 | 根据相对母船位姿计算 |
| 视觉节点 (发送) | yaw, pitch 跟踪命令 | 高频 | 检测到目标时发送 |
| 云台节点 (订阅+仲裁) | - | - | 同时订阅两个topic，执行仲裁 |

**2.2 云台模式切换请求（service）**

| 角色 | 功能 | 说明 |
|------|------|------|
| 门控相机、传统摄像机 (客户端) | 请求模式切换 | 通过service调用 |
| 云台节点 (服务端) | 处理模式切换 | 更新云台工作态 |

**模式定义与转移**
- **默认状态**：跟踪态（执行运动模块前馈命令）
- **状态转移**：
  - `跟踪态 → 检测态`：收到门控或相机的模式切换请求（DETECT）
  - `检测态 → 跟踪态`：视觉数据丢失超过2秒（即：云台节点连续2秒未收到有效视觉跟踪命令）

**2.3 云台状态反馈（topic + query service）**

**发送端**：云台节点  
**订阅端**：运动模块、融合模块  
**坐标系**：yaw、pitch基于无人机机体坐标系

**反馈内容**
- `yaw`, `pitch`：当前角度
- `d_yaw`, `d_pitch`：当前角速度
- `mode`：控制模式（TRACKING / DETECTING）
- `yaw_limit`, `pitch_limit`：是否到达软限位

**用途**：无人机运动模块通过查询云台状态，实时了解云台相对于机体的位姿，用于坐标系变换计算

---

#### 3. 与门控和相机间通信

**3.1 传感器原始数据（topic）**

**传统摄像机(TC)发送**
- 检测框：bbox [x, y, w, h]（像素坐标）
- 检测置信度：confidence [0-1]
- 点云：PointCloud2（摄像机坐标系）
- 时间戳：header.stamp

**门控相机(GC)发送**
- 点云：PointCloud2（摄像机坐标系）
- 时间戳：header.stamp

**处理流程**
1. 数据处理节点订阅TC和GC的点云及检测框
2. 在数据处理节点内（见下一章节）融合处理，输出观测结果
3. 观测结果转换至无人机机体坐标系后发布

**3.2 模式切换请求（service）**

| 客户端 | 调用service | 参数 | 说明 |
|--------|------------|------|------|
| 传统摄像机、门控 | 云台节点 | DETECT / SEARCH / RESET | 触发模式转移 |

---

#### 数据流向（统一版）

```
母船
  ↓ (MAVLink: 目标位置、命令)
无人机-主控
  ├→ 运动模块 ──topic→ 云台节点 (yaw/pitch前馈)
  │
  └→ 摄像机
      ├→ 传统摄像机(TC) ──topic→ 数据处理节点 (bbox+点云)
      │                    └─service→ 云台节点 (模式切换)
      └→ 门控相机(GC) ──topic→ 数据处理节点 (点云)
                           └─service→ 云台节点 (模式切换)
  
  数据处理节点
    ├→ topic: /gimbal/visual_command → 云台节点 (视觉跟踪命令)
    ├→ topic: /target/observation_body → 主控/融合模块
    └→ MAVLink上行模块 → 母船

  云台节点
    ├ (仲裁: 视觉优先级>运动) → 执行者
    ├→ topic: 云台状态 → 运动模块查询 (用于坐标变换)
    └→ 订阅 /gimbal/visual_command (由数据处理节点发布)
```

### 云台节点实现要求

**核心职责**
1. 订阅运动模块和视觉节点的控制命令
2. 根据模式和优先级进行仲裁
3. 施加软限位和速度限幅
4. 输出状态供其他模块查询
5. 处理模式切换请求

---

**1. 输入信号定义**

**1.1 订阅 - 运动模块前馈命令**
- Topic名称：`/gimbal/motion_command`
- 数据结构：
  ```
  header:
    stamp: ROS时间戳
  yaw: float32 [rad]
  pitch: float32 [rad]
  ```
- 说明：持续发送（100+ Hz），即使无目标

**1.2 订阅 - 视觉跟踪命令**
- Topic名称：`/gimbal/visual_command`
- 数据结构：
  ```
  header:
    stamp: ROS时间戳
  yaw: float32 [rad]
  pitch: float32 [rad]
  confidence: float32 [0-1]
  ```
- 说明：仅在检测到目标时发送；无目标时停止发送

**1.3 服务 - 模式切换请求**
- Service名称：`/gimbal/set_mode`
- 请求参数：
  ```
  mode: uint8  # 0=TRACKING, 1=DETECTING
  ```
- 响应：success (bool)

---

**2. 仲裁与控制流程**

**仲裁规则**
- 优先级：视觉 > 运动模块
- 实现方式：云台节点同时订阅两个topic，**当视觉topic在最近2秒内有有效数据时，使用视觉命令；否则使用运动模块命令**
- "有效数据"定义：时间戳在2秒内且置信度达到阈值

```

初始化：
  - 云台回中（yaw=0, pitch=0）
  - 模式 = TRACKING
  - motion_valid_flag = false, visual_valid_flag = false
  - visual_loss_timer = 0

每个控制周期（如 100 Hz）：
    1. 检查模式状态
    2. 若模式为 DETECTING：
        a. 读取最新视觉命令
        b. 如果视觉命令有效（Δt ≤ 2s 且 confidence ≥ 阈值），则目标角度 = 视觉命令
        c. 否则，记录持续无效时间。若连续无效 ≥ 2s，切换模式为 TRACKING，目标角度 = 运动模块前馈
    3. 若模式为 TRACKING：
        a. 检查是否有视觉节点的模式切换请求（DETECT），有则切换模式为 DETECTING，并返回步骤2
        b. 否则，目标角度 = 运动模块前馈
    4. 对目标角度施加限位和角速度限幅
        a. 软限位：
            target_yaw = clip(target_yaw, yaw_min, yaw_max)
            target_pitch = clip(target_pitch, pitch_min, pitch_max)
        
        b. 角速度限幅：
            yaw_rate = (target_yaw - current_yaw) / dt
            yaw_rate = clip(yaw_rate, -yaw_rate_max, +yaw_rate_max)
            target_yaw = current_yaw + yaw_rate * dt
            [同理处理pitch]
        
        c. 输出限位标志（用于后续反馈）
    5. 输出控制量驱动云台
            current_yaw = target_yaw
            current_pitch = target_pitch

    6. 发布状态
    state_msg:
        header.stamp = 当前时间
        yaw = current_yaw
        pitch = current_pitch
        yaw_rate = (current_yaw - last_yaw) / dt
        pitch_rate = (current_pitch - last_pitch) / dt
        mode = 当前模式（TRACKING / DETECTING）
        yaw_at_limit = (current_yaw == yaw_min or current_yaw == yaw_max)
        pitch_at_limit = (current_pitch == pitch_min or current_pitch == pitch_max)
    发布到：`/gimbal/state`
```

### 基于GC和TC的数据处理与融合

**输入**
- TC：检测框 [bbox] + 点云 [PointCloud2]（摄像机坐标系）
- GC：点云 [PointCloud2]（摄像机坐标系）
- 时间戳同步阈值：50～100ms

**输出**
观测结果（用于云台控制和母船通信）：
```
target_observation {
  target_id: int
  position: [x, y, z]          # 相对无人机机体坐标系（单位：米）
  confidence: float32 [0-1]    # 融合置信度
  source: “GC_only” / “TC_only” / “fused”
  timestamp: ROS时间戳
}
```

**处理流程**

```
1.时间同步与数据关联
    用时间戳阈值 50~100ms 同步 TC 检测框与两路点云
    确保 bbox、TC点云、GC点云来自接近的时刻

2.坐标系转换与边界裁剪（检测框引导）
    将 bbox 投影到 TC 点云，提取框内3D点（可加 10~20px margin）
    若点云由深度图生成：直接遍历框内像素，反投影得3D点
    结果：ROI点云（在摄像机坐标系中）

3.ROI点云预处理
    去无效点：NaN / Inf / 深度为0
    深度门限：z_min ~ z_max（按任务配置）
    体素降采样：降低计算量
    可选：统计离群点去除（效果需评估）

4.TC点云降噪（利用GC作为先验）
    GC是低噪声点云，可作为参考
    对 TC 的 ROI 点云：
    a. 找每个点在 GC 中的最近邻
    b. 计算该点到 GC 局部曲面的距离
    c. 距离过大则删除或修正
    结果：降噪后的 ROI 点云

5.聚类与目标选择
    在 ROI 点云上执行欧式聚类或 DBSCAN
    对每个候选簇打分：
      score = w1*点数 + w2*与bbox中心射线接近度 + w3*与上一帧预测接近度
    选最高分簇作为目标主簇
    若无有效簇：标记本帧丢失，由卡尔曼滤波预测

6.计算 3D 中心与质量指标
    对主簇点：使用中位数或截断均值（鲁棒于异常值）
    目标中心：[x_c, y_c, z_c]（摄像机坐标系）
    距离：d = sqrt(x_c² + y_c² + z_c²)
    质量指标：簇的点数、方差等

7.双传感器融合（当两路都有效时）
    使用测量方差加权融合：
      p_fused = (p_TC/σ_TC² + p_GC/σ_GC²) / (1/σ_TC² + 1/σ_GC²)
    若仅一路有效：采用该路数据，置信度打折扣

8.卡尔曼滤波跨帧跟踪
    状态：[x, y, z, vx, vy, vz]（CV恒速模型）
    观测：第 6-7 步得到的 [x_c, y_c, z_c]
    有观测时：EKF update；无观测时：predict 并累计丢失计数
    连续丢失超过阈值（如 30 帧）则轨迹失效

9.输出与发布
    融合置信度 = a*detection_conf + b*cluster_quality + c*track_stability
    发送观测结果：
      - target_position: KF滤波后的 [x, y, z]
      - confidence: 融合置信度
      - source: 数据来源（GC/TC/fused）
      - timestamp: 当前时刻
    
    优先级：优先发布”已跟踪稳定目标”（track_stability > 阈值）
            其次发布单帧高置信度观测
```

**参数配置** (YAML)

```yaml
target_detection:
  # 时间同步阈值
  time_sync_threshold: 0.1  # 秒，100ms
  
  # 深度约束
  depth_min: 0.5    # 米，最小距离
  depth_max: 100.0  # 米，最大距离
  
  # ROI 预处理
  voxel_size: 0.01  # 体素边长，单位米
  
  # 聚类参数
  cluster_tolerance: 0.05    # 聚类距离阈值，单位米
  min_cluster_size: 20       # 最小聚类点数
  
  # 聚类评分权重
  score_weight_count: 0.4                    # w1: 点数权重
  score_weight_ray_distance: 0.3             # w2: 与bbox中心接近度
  score_weight_predict_distance: 0.3         # w3: 与预测位置接近度
  
  # 噪声模型（用于融合和KF）
  tc_measurement_noise: [0.10, 0.10, 0.10]  # TC 的 xyz 测量噪声 (m)
  gc_measurement_noise: [0.05, 0.05, 0.05]  # GC 的 xyz 测量噪声 (m)
  
  # 卡尔曼滤波
  kalman_process_noise: [0.1, 0.1, 0.1, 0.5, 0.5, 0.5]  # [x,y,z,vx,vy,vz]
  max_tracking_loss_frames: 30   # 最多允许丢失帧数
  
  # 置信度评估
  confidence_weight_detection: 0.5    # a: 检测置信度权重
  confidence_weight_cluster: 0.2      # b: 聚类质量权重
  confidence_weight_stability: 0.3    # c: 跟踪稳定性权重
  stability_threshold: 0.7            # 稳定目标的最小置信度
```

---
```
version -v1.0.0
```