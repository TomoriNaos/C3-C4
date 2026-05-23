# MAVLink / 远距离通信桥接设计

> 版本：v1.2.0  
> 更新日期：2026-05-19

## 1. 当前实现范围

当前仓库中的 `mavlink_bridge_node` 是 **ROS 侧桥接与状态网关**，负责把“母船侧输入”映射到无人机内部 ROS 话题，并把观测结果和状态回传到母船侧接口。

> 说明：当前代码还没有实现二进制 MAVLink 帧编解码；真实串口/UDP 解包、打包可以在后续接入同名 topic 之后再补。

---

## 2. 节点职责

### `mavlink_bridge_node`

职责：
- 接收母船侧任务命令、心跳、目标区信息
- 透传为无人机内部 topic
- 发布命令 ACK 和链路状态
- 将目标观测转发给母船侧接口

它不负责：
- 主任务状态机
- 云台仲裁
- PX4 Offboard 控制

这些逻辑分别由 `drone_main_controller_node`、`gimbal_controller_node` 和 `motion_controller_node` / Offboard 桥接节点完成。

---

## 3. 当前话题定义

### 3.1 母船 -> 无人机

| 话题 | 类型 | 说明 |
|---|---|---|
| `/mavlink/heartbeat_rx` | `std_msgs/msg/Bool` | 母船侧心跳输入 |
| `/mavlink/mission_cmd_rx` | `c3_drone_driver/msg/MissionCommand` | 任务命令输入 |
| `/mavlink/ship_pose_world_rx` | `geometry_msgs/msg/PoseStamped` | 母船在世界系位置输入 |
| `/mavlink/ship_target_point_rx` | `geometry_msgs/msg/PoseStamped` | 船体系下的目标相对点输入 |

### 3.2 无人机 -> 母船

| 话题 | 类型 | 说明 |
|---|---|---|
| `/mavlink/heartbeat_tx` | `std_msgs/msg/Bool` | 无人机侧心跳输出 |
| `/mavlink/mission_cmd_ack` | `c3_drone_driver/msg/CommandAck` | 命令确认 |
| `/mavlink/target_obs` | `c3_drone_driver/msg/TargetObservation` | 目标观测回传 |
| `/mission/state` | `c3_drone_driver/msg/DroneStatus` | 任务状态回传 |
| `/mission/cmd` | `c3_drone_driver/msg/MissionCommand` | 转发到主控 |
| `/ship/pose_world` | `geometry_msgs/msg/PoseStamped` | 船位置回传给主控 |
| `/ship/target_point` | `geometry_msgs/msg/PoseStamped` | 船目标点回传给主控 |

---

## 4. 当前处理逻辑

### 4.1 心跳

- 每次收到 `/mavlink/heartbeat_rx`，更新时间戳
- `status_hz` 周期输出 `/mission/state`
- 若距离最近心跳超过 `link_degraded_s` 或 `link_lost_s`，链路状态切换为 `DEGRADED` / `LOST`

### 4.2 任务命令

- 收到 `/mavlink/mission_cmd_rx` 后，直接发布到 `/mission/cmd`
- 同时回一条 `CommandAck(result=ACCEPTED)` 到 `/mavlink/mission_cmd_ack`

### 4.3 目标观测

- 收到 `/target/observation_body` 后转发到 `/mavlink/target_obs`
- 使用 `obs_id` 做简单去重
- 若观测时间超出 `target_obs_valid_timeout_s`，则丢弃

### 4.4 船信息

- `/mavlink/ship_pose_world_rx` -> `/ship/pose_world`
- `/mavlink/ship_target_point_rx` -> `/ship/target_point`

---

## 5. 与 PX4 的分工

- `mavlink_bridge_node` 只处理 ROS 侧业务话题
- 真正发给 PX4 FCU 的 Offboard 消息由：
  - `offboard_setpoint_px4_bridge_node`
  负责
- `px4_msgs` 缺失时，后者不会编译生成

---
