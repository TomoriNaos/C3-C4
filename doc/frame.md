# 无人机飞控与视觉跟踪框架

> 版本：v1.2.0  
> 更新日期：2026-05-19

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
| `offboard_setpoint_bridge_node` | `bridge_node` | 将 `/px4/offboard_goal` 透传为统一的 setpoint |
| `offboard_setpoint_px4_bridge_node` | `bridge_node` | 将 Offboard 目标转成 PX4 原生 `px4_msgs` |
| `gimbal_joint_state_bridge_node` | `bridge_node` | 将云台状态转成 `/joint_states` |

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
- `offboard_setpoint_bridge_node`

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
`offboard_setpoint_bridge_node` 或 `offboard_setpoint_px4_bridge_node`
  ↓
PX4 / Offboard
```

母船侧数据经 `mavlink_bridge_node` 进入：
- `/mission/cmd`
- `/ship/pose_world`
- `/ship/target_point`
- `/mavlink/target_obs`
- `/mavlink/mission_cmd_ack`

---

## 5. 外部接口概览

### 5.1 TC

- 发布：`/tc/detection`
- 类型：`c3_drone_driver/msg/TcDetection`
- 主要字段：`bbox.data`、`cloud`、`header.stamp`

### 5.2 GC

- 发布：`/gc/points`
- 类型：`sensor_msgs/msg/PointCloud2`
- 主要字段：`header.stamp`、点云坐标系

### 5.3 母船

- 输入：`/mavlink/heartbeat_rx`、`/mavlink/mission_cmd_rx`、`/mavlink/ship_pose_world_rx`、`/mavlink/ship_target_point_rx`
- 输出：`/mavlink/heartbeat_tx`、`/mavlink/mission_cmd_ack`、`/mavlink/target_obs`、`/mission/state`

---

## 6. 备注

- 当前仓库里的 `UWB`、`AIS` 仍属于预留能力，未进入主业务链路。
- `doc/` 更适合作为设计说明；具体接口以 `msg/`、`srv/`、`launch/` 和源码为准。
