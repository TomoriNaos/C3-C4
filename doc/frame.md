# 无人机飞控与视觉跟踪框架

> 版本：v1.2.1  
> 更新日期：2026-05-22

## 1. 作用

本文档描述当前 `c3_drone_driver` 的实际代码结构、节点分工、启动方式和数据流。重点是无人机侧的感知、云台、任务主控和 PX4 Offboard 链路。

---

## 2. 代码结构

```text
c3_drone_driver/
├── src/
│   ├── inner_node/        # 业务主节点
│   └── bridge_node/       # PX4 / MAVLink / 桥接节点
├── msg/                   # 自定义消息
├── srv/                   # 自定义服务
├── config/                # 参数配置
├── launch/                # 启动文件
└── urdf/                  # 机体与云台模型
```

### 2.1 现有节点

| 节点 | 位置 | 作用 |
|---|---|---|
| `sensor_mock_node` | `inner_node` | 仿真输入源，发布 TC/GC 数据和测试任务命令 |
| `target_processor_node` | `inner_node` | 融合 TC/GC，输出目标观测和视觉云台命令 |
| `gimbal_controller_node` | `inner_node` | 云台 yaw/pitch 仲裁、限位和状态输出 |
| `drone_main_controller_node` | `inner_node` | 主控，协调观测、云台、任务状态和目标点 |
| `motion_controller_node` | `inner_node` | 生成 PX4 Offboard 目标点 |
| `mavlink_bridge_node` | `bridge_node` | ROS 侧 MAVLink 输入/输出桥接与链路状态网关 |
| `px4_pose_bridge_node` | `bridge_node` | 将 `/odom` 或 `/pose` 统一为 `/px4/vehicle_pose` |
| `offboard_setpoint_px4_bridge_node` | `bridge_node` | 将 Offboard 目标转成 PX4 原生 `px4_msgs` |
| `gimbal_joint_state_bridge_node` | `bridge_node` | 将云台状态转成 `/joint_states` |

这些节点的职责边界是固定的：

- 感知链路只负责把 TC/GC 数据整理成目标观测
- 云台链路只负责角度控制、限位和模式仲裁
- 主控链路只负责任务、距离阈值和生命周期调度
- 运动链路只负责把目标点变成 PX4 可执行的 Offboard 目标
- 桥接节点只负责和外部系统交换消息，不参与业务决策

---

## 3. 启动文件

### 3.1 `c3_drone_core.launch.py`

启动核心算法链路：
- `gimbal_controller_node`
- `target_processor_node`
- `mavlink_bridge_node`
- `drone_main_controller_node`
- `motion_controller_node`
- `px4_pose_bridge_node`
- `offboard_setpoint_px4_bridge_node`

适合只看业务逻辑、不拉起 Gazebo 的场景。

### 3.2 `c3_gazebo_sim.launch.py`

启动 Gazebo 联合仿真：
- Gazebo 环境
- 机体模型
- `sensor_mock_node`
- 全套业务节点
- RViz（可选）

### 3.3 `gimbal_sim.launch.py`

云台专项调试：
- `use_gui:=true` 时启用手动关节控制
- `use_controller:=true` 时启用 `gimbal_controller_node + gimbal_joint_state_bridge_node`

---

## 4. 数据流

```text
TC / GC / 任务命令
  ↓
`target_processor_node`
  ↓ `TargetObservation` / `GimbalVisualCommand`
`gimbal_controller_node`
  ↓ `/gimbal/state`
`drone_main_controller_node`
  ├─→ `/gimbal/motion_command`
  ├─→ `/mission/goal`
  └─→ `/main_controller/status`
`motion_controller_node`
  ↓ `/px4/offboard_goal`
`offboard_setpoint_px4_bridge_node`
  ↓
PX4 / Offboard
```

母船侧数据经 `mavlink_bridge_node` 进入：
- `/mission/cmd`
- `/ship/pose_world`
- `/ship/target_point`
- `/mavlink/target_obs`
- `/mavlink/mission_cmd_ack`

### 4.1 主流程说明

1. 母船通过 MAVLink 下发 `MissionCommand`、母船位姿和粗目标点。
2. `mavlink_bridge_node` 将外部数据桥接成 ROS topic。
3. `motion_controller_node` 根据 `MissionCommand` 和主控发布的 `/mission/goal` 生成 `/px4/offboard_goal`。
4. `drone_main_controller_node` 订阅目标观测、云台状态、运动模式和母船目标点：
   - 当任务为 `CMD_START` 时，发布任务目标点
   - 当任务为 `CMD_BACK` 时，切到回船目标点
   - 当目标距离小于 `enable_distance` 时，激活 TC/GC 生命周期，并进入视觉云台控制
   - 当目标距离大于阈值时，不启用视觉链路
5. `target_processor_node` 融合 TC/GC 结果，输出目标观测和视觉云台命令。
6. `gimbal_controller_node` 对视觉命令和运动前馈命令做仲裁，最终发布 `/gimbal/state`。
7. `drone_main_controller_node` 再根据 `/gimbal/state` 和目标距离决定是否发布 `/gimbal/motion_command`。
8. `motion_controller_node` 接收 `/mission/goal` 后，持续生成 PX4 Offboard 目标。

### 4.2 关键时序

- 先有任务，再有目标点
- 先有目标观测，再有视觉跟踪
- 先到距离阈值，再激活 TC/GC
- 先由主控做策略判断，再由云台和运动模块执行

---

## 5. TC / GC 的 URDF 与 TF 绑定

### 5.1 绑定原则

- `TC` 与 `GC` 作为云台上的两套相机载荷，统一挂在 `gimbal_pitch_link` 下。
- 物理挂载关系由 `c3_drone_with_gimbal.urdf.xacro` 定义，`robot_state_publisher` 负责发布 TF。
- 相机包不应重复发布 `tc_camera_link`、`gated_camera_link` 这类固定变换，避免 TF 冲突。
- 这里的 LINK 关系只做坐标绑定，不承载生命周期逻辑。

### 5.2 固定帧名

- 云台基准帧：`gimbal_pitch_link`
- TC 载荷帧：`tc_camera_link`
- TC 光学帧：`tc_camera_optical_frame`
- GC 载荷帧：`gated_camera_link`
- GC 光学帧：`gated_camera_optical_frame`

### 5.3 可调安装参数

当前 URDF 已将相机安装位姿抽成 xacro 参数，便于后续标定或换型：

- `tc_camera_xyz`
- `tc_camera_rpy`
- `gated_camera_xyz`
- `gated_camera_rpy`

### 5.4 对接约束

- 相机驱动/算法包只消费上述帧名，不自建新的挂载层级。
- 图像和检测结果继续走各自 topic，TF 只负责坐标绑定。
- 若后续需要做生命周期启停，主控只按距离阈值触发，不改 TF 树。
- TC/GC 自身不需要额外 srv；生命周期由主控统一调用标准 `ChangeState`。

### 5.5 距离门控策略

- 仅考虑非 `BACK` 态时，主控使用 `arrive_radius < enable_distance` 的策略。
- `target_range > enable_distance` 时：
  - 不控制云台进入视觉模式
  - TC/GC 保持 `Inactive`
- `target_range <= enable_distance` 时：
  - 主控调用 TC/GC 的生命周期服务，激活相机节点
  - 再进入云台视觉控制链路
- 这里的 `arrive_radius` 保留给运动模块到点判定，`enable_distance` 保留给感知与云台激活判定。
- `arrive_radius` 关注“是否到点”
- `enable_distance` 关注“是否值得打开视觉链路”

### 5.6 生命周期服务规范

队友相机包后续建议实现标准生命周期服务：

- 服务名：`/tc_camera_node/change_state`
- 服务名：`/gated_camera_node/change_state`
- 服务类型：`lifecycle_msgs/srv/ChangeState`

请求字段：

```text
lifecycle_msgs/msg/Transition transition
uint8 transition.id
string transition.label
```

推荐使用的 `transition.id`：

- `lifecycle_msgs/msg/Transition::TRANSITION_ACTIVATE`
- `lifecycle_msgs/msg/Transition::TRANSITION_DEACTIVATE`

响应字段：

```text
bool success
```

默认约定：

- 节点启动后处于 `Inactive`
- 主控在距离大于阈值时不调用激活
- 主控在距离小于等于阈值时调用 `TRANSITION_ACTIVATE`
- 主控在离开阈值或任务结束时调用 `TRANSITION_DEACTIVATE`

### 5.7 母船控制无人机生命周期服务

主控节点对外提供统一服务，供母船侧控制无人机任务生命周期：

- 服务名：`/main_controller/set_lifecycle`
- 服务类型：`c3_drone_driver/srv/SetDroneLifecycle`

请求字段：

```text
uint8 command
CMD_ACTIVATE=1
CMD_DEACTIVATE=2
```

响应字段：

```text
bool success
string message
uint8 mission_mode
```

语义约定：

- `CMD_ACTIVATE`：开启 TC/GC 生命周期
- `CMD_DEACTIVATE`：关闭 TC/GC 生命周期，并强制切回云台追踪模式
- 这个 srv 只负责 TC/GC 生命周期，不再混入任务切换语义

---

## 6. 外部接口概览

### 5.1 TC

- 发布：`/tc/detection`
- 类型：`c3_drone_driver/msg/TcDetection`
- 主要字段：`bbox.data`、`cloud`、`header.stamp`
- 约定帧：`tc_camera_optical_frame`

### 5.2 GC

- 发布：`/gc/detection`
- 类型：`c3_drone_driver/msg/TcDetection`
- 主要字段：`bbox.data`、`cloud`、`header.stamp`
- 约定帧：`gated_camera_optical_frame`

### 5.3 母船

- 输入：`/mavlink/heartbeat_rx`、`/mavlink/mission_cmd_rx`、`/mavlink/ship_pose_world_rx`、`/mavlink/ship_target_point_rx`
- 输出：`/mavlink/heartbeat_tx`、`/mavlink/mission_cmd_ack`、`/mavlink/target_obs`、`/mission/state`

---

## 7. 备注

- 当前仓库里的 `UWB`、`AIS` 仍属于预留能力，未进入主业务链路。
- `doc/` 更适合作为设计说明；具体接口以 `msg/`、`srv/`、`launch/` 和源码为准。
