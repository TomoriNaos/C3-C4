```markdown
# 无人机视觉跟踪系统设计文档

> 版本：v1.1.0  
> 更新日期：2026-05-15

## 目录

1. [任务要求](#任务要求)
2. [任务拆解](#任务拆解)
   - [1. 与母船间通信](#1-与母船间通信)
   - [2. 与云台间通信](#2-与云台间通信)
   - [3. 与门控和相机间通信](#3-与门控和相机间通信)
3. [数据流向（统一版）](#数据流向统一版)
4. [云台节点实现要求](#云台节点实现要求)
   - [输入信号定义](#输入信号定义)
   - [仲裁与控制流程](#仲裁与控制流程)
5. [基于 GC 和 TC 的数据处理与融合](#基于-gc-和-tc-的数据处理与融合)
   - [输入](#输入)
   - [输出](#输出)
   - [坐标系转换链路](#坐标系转换链路)
   - [处理流程](#处理流程)
   - [节点职责](#节点职责)
   - [参数配置](#参数配置)
6. [无人机主控](#无人机主控)
   - [处理逻辑](#处理逻辑)
   - [位姿估计组件](#位姿估计组件)
7. [运动模块（最小实现）](#运动模块最小实现)

---

## 任务要求

### 功能要求

1. 获取门控相机和传统摄像机返回的数据，包括目标相对于无人机的位姿和物体类型。
2. 提供云台控制节点，相机和门控相机作为客户端控制云台旋转实现追踪（如在接近目标时自动调用，控制装载门控相机（**下称GC**）和传统摄像机（**下称TC**）的转台旋转，实现对目标的持续跟踪。云台控制采用 `yaw + pitch` 两轴，做软件限位（可配置上下限与角速度限幅），但不做机械细节建模。需要提供控制权仲裁：视觉跟踪优先级高于运动模块前馈控制，避免多端同时写入导致抖动。
3. MAVLink 协议 + 远距离双向通信机制。接收主控发来的目标大致方位和具体命令，把摄像机识别到的数据发回主控。
4. 运动模块，通过 IMU 和 EKF2 实现自主飞控，控制无人机飞到目标点，至于位姿信息等交给主控自己计算。
5. UWB 通信能力，用于多无人机间协同（再议，先不做）。
6. AIS 接收模块，接收母船和同样搭载了 AIS 的船舶广播的 AIS 信息，提供给自身运动模块做辅助（再议，非主体部分）。

---

## 任务拆解

### 一、通信机制

#### 1. 与母船间通信

**无人机接收**

- 命令：`start`, `back`, `close`
- 目标点坐标（相对母船坐标系）
- 协议：MAVLink + 远距离链路

**无人机发送**

- 目标 ID、物体类型 ID
- 相对无人机坐标系位姿 / 距离 / 位置
- 置信度、时间戳 etc.
- 协议：MAVLink + 远距离链路

---

#### 2. 与云台间通信

##### 2.1 云台控制命令（Topic，仲裁）

| 角色 | 数据 | 频率 | 说明 |
|------|------|------|------|
| 主控结合运动数据（发送） | `yaw`, `pitch` 前馈命令 | ≥100 Hz | 根据主控下发目标点和当前位姿计算 |
| 视觉节点（发送） | `yaw`, `pitch` 跟踪命令 | 高频 | 检测到目标时发送 |
| 云台节点（订阅 + 仲裁） | - | - | 同时订阅两个 topic，执行仲裁 |

##### 2.2 云台模式切换请求（Service）

| 角色 | 功能 | 说明 |
|------|------|------|
| TC、GC（客户端） | 请求局部接管 | 通过 service 调用，仅在检测模式下生效 |
| 云台节点（服务端） | 处理模式切换 | 更新云台工作态 |

**云台模式**

- **默认状态**：跟踪态（执行运动模块前馈命令）
- **状态转移**：
  - `跟踪态 → 检测态`：主控任务状态机切换到 `DETECTING`，且云台收到视觉接管请求（`DETECT`）
  - `检测态 → 跟踪态`：主控任务状态机切回 `TRACKING`，或连续视觉命令丢失超过 **2 秒**

**触发优先级（与 `mavlink_design.md` 对齐）**

- 主控任务状态机控制全局开关：`TRACKING / DETECTING`
- 视觉节点只做局部接管请求：`DETECT / SEARCH / RESET`
- 若主控处于 `TRACKING`，云台强制回到跟踪态，并忽略视觉接管请求
- 若主控处于 `DETECTING`，云台允许视觉请求接管，否则仍执行运动前馈

##### 2.3 云台状态反馈（Topic + Query Service）

**发送端**：云台节点  
**订阅端**：运动模块、融合模块  
**坐标系**：`yaw`、`pitch` 基于无人机机体坐标系

**反馈内容**

- `yaw`, `pitch`：当前角度
- `d_yaw`, `d_pitch`：当前角速度
- `mode`：控制模式（`TRACKING` / `DETECTING`）
- `yaw_limit`, `pitch_limit`：是否到达软限位

**用途**：无人机运动模块通过查询云台状态，实时了解云台相对于机体的位姿，用于坐标系变换计算。

---

#### 3. 与TC和GC间通信

##### 3.1 传感器原始数据（Topic）

**TC发送**

- 检测框：`bbox [x, y, w, h]`（像素坐标）
- 检测置信度：`confidence [0-1]`
- 点云：`PointCloud2`（摄像机坐标系）
- 时间戳：`header.stamp`

**GC发送**

- 点云：`PointCloud2`（摄像机坐标系）
- 时间戳：`header.stamp`

**处理流程**

1. `target_processor_node` 订阅 TC 和 GC 的点云及检测框。
2. 在节点内部调用 `TargetFusionProcessor` 完成同步、融合和跟踪，具体逻辑见 `doc/target_fusion.md`。
3. 观测结果以 `FRAME_GIMBAL` 发布，交由主控统一转换至 `BODY/NED`。

##### 3.2 模式切换请求（Service）

| 客户端 | 调用 Service | 参数 | 说明 |
|--------|--------------|------|------|
| 传统摄像机、门控 | 云台节点 | `DETECT` / `SEARCH` / `RESET` | 仅申请局部接管，不改变任务主状态 |

---

### 数据流向（统一版）

```text
母船
  ↓ (MAVLink: 目标位置、命令)
无人机主控
  ├→ 运动模块 ──topic→ 飞控接口 (目标点 / 模式 / 返航)
  ├→ 云台节点 ──topic→ yaw/pitch 前馈与状态查询
  ├→ 上行模块 ──MAVLink→ 母船
  ├← 数据处理节点 ──topic→ target_observation_body
  │
  │
  └→ 摄像机
      ├→ 传统摄像机(TC) ──topic→ 数据处理节点 (bbox+点云)
      │                    └─service→ 云台节点 (模式切换)
      └→ 门控相机(GC) ──topic→ 数据处理节点 (点云)
                           └─service→ 云台节点 (模式切换)
  
  数据处理节点
    ├→ topic: /gimbal/visual_command → 云台节点 (视觉跟踪命令)
    ├→ topic: /target/observation_body → 无人机主控
    └→ topic: /mavlink/target_obs → mavlink_bridge_node

  云台节点
    ├ (仲裁: 视觉优先级>运动) → 执行者
    ├→ topic: 云台状态 → 无人机主控 / 运动模块查询 (用于坐标变换)
    └→ 订阅 /gimbal/visual_command (由数据处理节点发布)
```

---

## 二、云台节点

### 核心职责

1. 订阅运动模块和视觉节点的控制命令。
2. 根据模式和优先级进行仲裁。
3. 施加软限位和速度限幅。
4. 输出状态供其他模块查询。
5. 处理模式切换请求。

---

### 输入信号定义

#### 1. 订阅 - 主控模块（代替运动模块）前馈命令

- **Topic 名称**：`/gimbal/motion_command`
- **数据结构**：
  ```text
  header:
    stamp: ROS时间戳
  yaw: float32 [rad]
  pitch: float32 [rad]
  ```
- **说明**：持续发送（≥100 Hz），即使无目标。

#### 2. 订阅 - 视觉跟踪命令

- **Topic 名称**：`/gimbal/visual_command`
- **数据结构**：
  ```text
  header:
    stamp: ROS时间戳
  yaw: float32 [rad]
  pitch: float32 [rad]
  confidence: float32 [0-1]
  ```
- **说明**：仅在检测到目标时发送；无目标时停止发送。

#### 3. 服务 - 模式切换请求

- **Service 名称**：`/gimbal/set_mode`
- **请求参数**：
  ```text
  mode: uint8  # 0=TRACKING, 1=DETECTING
  ```
- **响应**：`success (bool)`

---

### 仲裁与控制流程

#### 仲裁规则

- **优先级**：视觉 > 运动模块
- **实现方式**：云台节点同时订阅两个 topic，满足“视觉有效窗口”时使用视觉命令；否则使用运动模块命令。
- **视觉有效窗口定义**：
  - 视觉命令发布频率建议 `>= 50 Hz`
  - 连续 `100 ms` 未收到视觉命令，判定视觉暂时丢失
  - 连续丢失达到 `2.0 s`，自动回退 `TRACKING`
  - 置信度阈值：`confidence >= conf_min`

#### 伪代码

```text
初始化：
  云台回中（yaw=0, pitch=0）
  模式 = TRACKING
  motion_valid_flag = false, visual_valid_flag = false
  visual_loss_timer = 0

每个控制周期（如 100 Hz）：
  1. 检查模式状态
  2. 若模式为 DETECTING（且主控允许视觉）：
      a. 读取最新视觉命令
      b. 如果视觉命令有效（Δt ≤ 100ms 且 confidence ≥ 阈值），则目标角度 = 视觉命令
      c. 否则，记录持续无效时间。若连续无效 ≥ 2s，切换模式为 TRACKING，目标角度 = 运动模块前馈
  3. 若模式为 TRACKING：
      a. 检查主控全局模式是否为 DETECTING 且存在视觉请求（DETECT），满足则切换模式为 DETECTING，并返回步骤 2
      b. 否则，目标角度 = 运动模块前馈
  4. 对目标角度施加限位和角速度限幅
      a. 软限位：
          target_yaw = clip(target_yaw, yaw_min, yaw_max)
          target_pitch = clip(target_pitch, pitch_min, pitch_max)
      b. 角速度限幅：
          yaw_rate = (target_yaw - current_yaw) / dt
          yaw_rate = clip(yaw_rate, -yaw_rate_max, +yaw_rate_max)
          target_yaw = current_yaw + yaw_rate * dt
          [同理处理 pitch]
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
      发布到：/gimbal/state
```

---

## 三、目标处理节点

### 输入

- **TC**：检测框 `[bbox]` + 点云 `[PointCloud2]`（摄像机坐标系）
- **GC**：点云 `[PointCloud2]`（摄像机坐标系）
- **时间戳同步阈值**：50～100 ms

### 输出

观测结果（用于云台控制和母船通信）：

```text
target_observation {
  target_id: int
  position: [x, y, z]          # 相对云台坐标系（单位：米）
  frame: FRAME_GIMBAL
  confidence: float32 [0-1]    # 融合置信度
  source: “GC_only” / “TC_only” / “fused”
  timestamp: ROS时间戳
}
```

### 坐标系转换链路

```text
TC/GC Camera Frame
  --[T_gimbal_camera: 云台-相机外参(标定)]-->
Gimbal Frame
  --[R_body_gimbal(yaw,pitch): 云台实时角]-->
Body Frame
  --[R_ned_body(q_px4): PX4/EKF2姿态]-->
NED Frame
```

实现约定：

- `T_gimbal_camera`：固定外参矩阵，来自标定文件（如 `config/extrinsics.yaml`）
- `R_body_gimbal`：由云台 yaw + pitch 计算，采用 `Rz(yaw) * Ry(pitch)`（右手系）
- `R_ned_body`：由 PX4/EKF2 姿态四元数得到
- 数据处理节点输出 `FRAME_GIMBAL`；主控负责 `GIMBAL -> BODY -> NED` 转换

### 处理流程

1. 订阅 TC bbox、TC 点云、GC 点云。
2. 按时间戳完成同步与缓存管理。
  - 用时间戳阈值 50~100 ms 同步 TC 检测框与两路点云。确保 bbox、TC 点云、GC 点云来自接近的时刻。
3. 调用 `TargetFusionProcessor` 完成点云融合、目标提取和观测生成。
4. 发布：
   - `/target/observation_body`
   - `/mavlink/target_obs`
   - `/gimbal/visual_command`

   融合置信度 = a * detection_conf + b * cluster_quality + c * track_stability
   发送观测结果（FRAME_GIMBAL）：
     - target_position: KF 滤波后的 [x, y, z]
     - confidence: 融合置信度
     - source: 数据来源（GC/TC/fused）
     - timestamp: 当前时刻
   优先级：优先发布“已跟踪稳定目标”（track_stability > 阈值），其次发布单帧高置信度观测。


### 参数

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
  score_weight_ray_distance: 0.3             # w2: 与 bbox 中心接近度
  score_weight_predict_distance: 0.3         # w3: 与预测位置接近度

  # 噪声模型（用于融合和 KF）
  tc_measurement_noise: [0.10, 0.10, 0.10]  # TC 的 xyz 测量噪声 (m)
  gc_measurement_noise: [0.05, 0.05, 0.05]  # GC 的 xyz 测量噪声 (m)

  # 卡尔曼滤波
  kalman_process_noise: [0.1, 0.1, 0.1, 0.5, 0.5, 0.5]  # [x, y, z, vx, vy, vz]
  max_tracking_loss_frames: 30   # 最多允许丢失帧数

  # 置信度评估
  confidence_weight_detection: 0.5    # a: 检测置信度权重
  confidence_weight_cluster: 0.2      # b: 聚类质量权重
  confidence_weight_stability: 0.3    # c: 跟踪稳定性权重
  stability_threshold: 0.7            # 稳定目标的最小置信度
```

---

## 四、无人机主控

无人机主控作为任务中枢，负责把母船指令、感知结果、云台状态和飞控状态统一起来。

### 核心职责

1. 接收母船消息，解析 `start / back / close / target hint` 等任务命令，并转发给运动模块。
2. 接收数据处理节点输出的目标观测结果（Gimbal Frame），结合当前无人机位姿，完成 `GIMBAL -> BODY -> NED` 转换并生成上行回传消息。
3. 维护当前位姿和状态摘要，向云台、运动模块和上行通信模块提供统一的数据入口。


### 主要输入

- `mavlink_bridge_node` 转来的任务命令和粗目标点。
- `target_processor_node` 输出的 `target_observation_body`（即融合后的目标观测）。
- PX4 / EKF2 提供的当前位姿、航向和健康状态。
- 云台节点反馈的角度状态，用于坐标转换。

### 主要输出

- 运动模块：任务/航点/返航指令。
- 云台节点：视觉跟踪前馈（运动）或目标修正量（视觉）。
- 母船：目标观测结果和无人机状态。

### 处理逻辑

```text
1. 接收并缓存母船命令
   - 从 MAVLink 消息中解析 mission_cmd / target_hint
   - 更新当前任务模式（IDLE / TRANSIT / SEARCH / TRACK / RETURN / ABORT）

2. 获取当前位姿
   - 从 PX4 / EKF2 读取无人机机体位姿
   - 从云台节点读取当前 yaw / pitch
   - 调用位姿估计组件完成 body / camera / gimbal / NED 坐标变换

3. 接收目标观测
   - 读取 target_processor_node 输出的 TargetObservation
   - 按 frame 字段统一转换：GIMBAL -> BODY -> NED
   - 生成用于上行回传的标准观测消息

4. 生成任务输出
   - 有有效目标时，向云台输出跟踪修正量
   - 需要飞行时，向运动模块输出目标点 / 保持 / 返航指令
   - 需要上报时，通过 MAVLink 上行模块发送观测和状态

5. 发布主控状态
   - 输出当前模式、链路状态、飞控状态、视觉状态
   - 为任务状态机提供统一入口
```

---

## 五、运动模块

1. 订阅 `/mission/goal`（主控下发目标点/模式），驱动 PX4 完成飞行。
2. 基于 IMU 与 EKF2 维护无人机基础位姿状态，向主控提供当前飞行状态。
3. 基于“当前位姿 + mission goal”计算云台前馈，发布 `/gimbal/motion_command`（`>=100 Hz`）。
4. 先不展开复杂路径规划、避障和控制律细节，当前仅保留“飞到目标点 / 保持 / 返航”的最小能力。
