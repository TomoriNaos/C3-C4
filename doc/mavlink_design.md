# C3 Drone MAVLink 方案设计（基于 frame.md / resources）

## 1. 目标与边界

本文档仅覆盖 `母船主控 <-> 无人机` 的 MAVLink 双向通信方案，服务于以下主链路：

1. 母船下发目标大致方位与任务控制命令；
2. 无人机回传视觉识别与测距观测结果；
3. 保障链路可恢复、状态可观测、术语一致。

不在本文展开：
- UWB 多机协同协议；
- AIS 航迹融合策略；
- 云台机械建模细节。

---

## 2. 术语对齐（必须统一）

为避免 `frame.md` 中“跟踪/检测/搜索”混用，统一如下：

- `母船主控（Mother Controller）`：母船上的任务控制与融合计算单元。
- `无人机任务机（Drone Companion）`：运行 ROS2 节点的机载计算机（非 PX4 飞控本体）。
- `飞控（PX4 FCU）`：PX4 Autopilot，负责姿态与位置控制。
- `TC`：Traditional Camera（传统可见光相机）。
- `GC`：Gated Camera（门控相机）。
- `目标观测（Target Observation）`：无人机侧融合输出，包含目标类型、相对位姿、置信度与来源。
- `任务模式（Mission Mode）`：母船与无人机任务级状态机。
- `云台模式（Gimbal Mode）`：云台控制仲裁模式（`TRACKING` / `DETECTING`）。

命名约束：
- “检测”仅指视觉识别行为（Detection）；
- “跟踪”仅指持续角度闭环行为（Tracking）；
- “任务命令”与“云台命令”严格分层，禁止同名复用。

---

## 3. 系统角色与 MAVLink 端点

### 3.1 端点定义

- 母船主控：`MAV_TYPE_ONBOARD_CONTROLLER`
- 无人机任务机：`MAV_TYPE_ONBOARD_CONTROLLER`
- PX4 FCU：`MAV_TYPE_QUADROTOR`（由 PX4 自身维护）

建议系统 ID：
- `sysid=1`：母船主控
- `sysid=21`：无人机任务机
- `sysid=22`：无人机 PX4 FCU

建议组件 ID：
- `compid=191`：任务通信组件（自定义，母船/无人机各自一份）
- `compid=1`：自动驾驶组件（PX4）

> 说明：母船与无人机任务机之间走远距离链路；无人机任务机与 PX4 可通过 MAVLink Router / MicroRTPS 网桥内部互联。

### 3.2 通道分层

- `Link-A（远距离）`：母船主控 <-> 无人机任务机（本文重点）。
- `Link-B（机内）`：无人机任务机 <-> PX4 FCU（复用标准 Offboard/Telemetry）。

---

## 4. 消息集设计

原则：
- 控制命令尽量复用标准 MAVLink；
- 业务观测数据使用 C3 私有消息（自定义 dialect）；
- 高实时流小包高频，状态/确认低频可靠。


### 4.1 下行（母船 -> 无人机）

#### A. 任务命令（复用 `COMMAND_LONG`）

- `command = MAV_CMD_USER_1`（建议映射：`C3_MISSION_CMD`）
- 参数定义：
  - `param1`: `mission_cmd`
    - `0` = `NOOP`
    - `1` = `START`
    - `2` = `BACK`
    - `3` = `CLOSE`
    - `4` = `HOLD`
  - `param2`: `target_id`（可选，无目标填 `-1`）
  - `param3`: `timeout_s`（命令超时）
  - `param4~7`: 预留

#### B. 粗引导目标点（自定义 `C3_TARGET_HINT`）

字段：
- `uint32_t hint_id`
- `uint64_t t_usec`
- `uint8_t frame`：`0=NED_REL_MOTHERSHIP`, `1=BODY_MOTHERSHIP`
- `float x_m` `float y_m` `float z_m`：相对母船坐标（米）
- `float vx_mps` `float vy_mps` `float vz_mps`：可选速度先验
- `float radius_m`：目标不确定性半径（搜索半径）
- `uint8_t target_type_hint`：目标类型先验（可未知=0）

频率建议：`1~2 Hz`（事件触发 + 周期保活）。


### 4.2 上行（无人机 -> 母船）

#### A. 目标观测回传（自定义 `C3_TARGET_OBS`）

字段：
- `uint32_t obs_id`
- `uint32_t track_id`
- `uint64_t t_usec`
- `uint8_t frame`：`0=BODY_DRONE`, `1=NED_DRONE`
- `float x_m` `float y_m` `float z_m`
- `float range_m`
- `float yaw_rad` `float pitch_rad`
- `uint8_t target_type`
- `float confidence`（`0~1`）
- `uint8_t source`：`1=TC_ONLY`, `2=GC_ONLY`, `3=FUSED`
- `uint8_t status`：`0=VALID`, `1=LOW_CONF`, `2=LOST`

频率建议：
- 有效跟踪期 `5~20 Hz`；
- 无目标期仅状态心跳（见 B）。

#### B. 无人机任务状态（自定义 `C3_DRONE_STATUS`）

字段：
- `uint64_t t_usec`
- `uint8_t mission_mode`：`IDLE/TRANSIT/SEARCH/TRACK/RETURN/ABORT`
- `uint8_t gimbal_mode`：`TRACKING/DETECTING`
- `uint8_t link_state`：`OK/DEGRADED/LOST`
- `float battery_remain`（0~1）
- `uint8_t nav_health`（EKF2健康度简化枚举）
- `uint8_t vision_health`（视觉链路健康度）

频率建议：`1 Hz`

#### C. 命令确认（复用 `COMMAND_ACK`）

- 对所有 `C3_MISSION_CMD` 必回 ACK；
- `result` 使用标准枚举，必要时在 `progress` 填阶段码。

---

## 5. ID 与可靠性策略

### 5.1 去重与乱序

- `hint_id`、`obs_id`、`track_id` 单调递增；
- 接收端维护最近窗口（建议 256 条）做去重；
- 超窗旧包丢弃，不触发状态回退。

### 5.2 丢包与重发

- 命令类（`COMMAND_LONG`）采用“ACK 驱动重发”：
  - 超时 `300 ms` 未 ACK 重发；
  - 最大重发 `5` 次；
  - 失败后进入 `DEGRADED` 并上报。
- 观测类（`C3_TARGET_OBS`）不重发，仅依赖连续流。

### 5.3 心跳与失联

- 双方都发送 `HEARTBEAT`（`1 Hz`）；
- 连续 `3 s` 未收到对端心跳 => `link_state=DEGRADED`；
- 连续 `8 s` 未收到 => `link_state=LOST`，无人机执行 `BACK/HOLD`（按配置）。

---

## 6. 任务状态机（与 frame.md 对齐）

状态定义：
- `IDLE`：待命
- `TRANSIT`：飞往母船下发粗目标区
- `SEARCH`：在目标区搜索（云台可进入 `DETECTING`）
- `TRACK`：已锁定目标并持续回传
- `RETURN`：返航
- `ABORT`：异常中止

关键转移：
1. `IDLE -> TRANSIT`：收到 `START` 且有有效 `C3_TARGET_HINT`。
2. `TRANSIT -> SEARCH`：到达 hint 半径阈值。
3. `SEARCH -> TRACK`：连续 `N` 帧 `C3_TARGET_OBS.status=VALID`。
4. `TRACK -> SEARCH`：连续 `T_lost` 丢失目标。
5. 任意状态 -> `RETURN`：收到 `BACK` 或低电量。
6. 任意状态 -> `ABORT`：收到 `CLOSE` 或飞控严重故障。

---

## 7. 与 ROS2 节点映射（落地建议）

建议在 `c3_drone_driver` 内拆分 3 个 C++ 节点：

1. `mavlink_bridge_node`
- 职责：MAVLink 收发、ACK、重发、心跳、去重。
- 订阅：`/target/observation_body`、`/mission/state`。
- 发布：`/mission/cmd`、`/mission/target_hint`。

2. `mission_manager_node`
- 职责：任务状态机、命令执行策略（START/BACK/CLOSE）。
- 订阅：`/mission/cmd`、`/mission/target_hint`、`/px4/status`。
- 发布：`/mission/state`、`/offboard/goal`。

3. `target_report_node`
- 职责：将 TC/GC 融合结果规范化为 `C3_TARGET_OBS` 输入。
- 订阅：`/target/observation_body`。
- 发布：`/mavlink/target_obs`（供 bridge node 打包）。

---

## 8. 自定义 MAVLink dialect 建议

文件：`c3.xml`（放在协议仓库或 `resources/mavlink/`）

建议消息号段：
- `C3_TARGET_HINT`: `msgid 42010`
- `C3_TARGET_OBS`: `msgid 42011`
- `C3_DRONE_STATUS`: `msgid 42012`

原则：
- 避免与 common/ardupilotmega 冲突；
- 一次定版后仅追加字段，不改已有字段语义；
- 预留版本字段（可在消息尾追加 `uint8_t proto_ver`）。

---

## 9. 参数基线（首版建议）

- `heartbeat_hz = 1.0`
- `cmd_ack_timeout_ms = 300`
- `cmd_max_retry = 5`
- `obs_pub_hz_track = 10`
- `obs_conf_min = 0.55`
- `target_lost_timeout_s = 2.0`
- `link_degraded_s = 3.0`
- `link_lost_s = 8.0`
- `hint_arrive_radius_m = 15.0`

---

## 10. 验收检查单（针对比赛提交）

1. 可演示 `START/BACK/CLOSE` 全链路闭环（含 ACK 与超时重发）。
2. 可在丢包场景下保持状态一致（不乱跳、不重复执行命令）。
3. 可稳定上报 `C3_TARGET_OBS`（含 source/confidence/status）。
4. 可触发失联策略并自动进入 `RETURN/HOLD`。
5. 日志可追溯：命令 ID、观测 ID、状态转移时间戳完整。

---

## 11. 与 AGENTS 原则对齐说明

- 仅设计飞控相关主链路（未扩展 UWB/AIS 细节）。
- 术语与职责分层明确，避免模块耦合与冗余命名。
- 保持可实现性优先：先标准消息复用，再最小自定义扩展。
- 所有关键行为（重发、失联、状态机）给出可配置参数，便于仿真与实船统一调参。

