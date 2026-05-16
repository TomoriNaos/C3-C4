# MAVLink 方案设计

## 1. 任务目标

基于 `母船主控 <-> 无人机` 的 MAVLink2.0 双向通信方案，服务于以下主链路：

1. 母船下发目标大致方位与任务控制命令；
2. 无人机回传视觉识别与测距观测结果；
3. 保障链路可恢复、状态可观测。

## 2. 系统角色与 MAVLink 端点

### 2.1 端点定义

- 母船主控：`MAV_TYPE_ONBOARD_CONTROLLER`
- 无人机任务机：`MAV_TYPE_ONBOARD_CONTROLLER`
- PX4 FCU：`MAV_TYPE_QUADROTOR`（由 PX4 自身维护）

系统 ID：
- `sysid=1`：母船主控
- `sysid=21`：无人机任务机 (依次增加，sysid=31、41)
- `sysid=22`：无人机 PX4 FCU (依次增加，sysid=32、42)

建议组件 ID：
- `compid=191`：任务通信组件（自定义，母船/无人机各自一份）
- `compid=1`：自动驾驶组件（PX4）

> 说明：母船与无人机任务机之间走远距离链路；无人机任务机与 PX4 可通过 MAVLink Router / MicroRTPS 网桥内部互联。

### 2.2 通道分层

- `Link-A（远距离）`：母船主控 <-> 无人机任务机
- `Link-B（机内）`：无人机任务机 <-> PX4 FCU（复用标准 Offboard/Telemetry）。

---

## 3. 消息集设计

### 3.1 下行（母船 -> 无人机）

#### A. 任务命令（复用MAVLink定义的 `COMMAND_LONG`）

- `command = MAV_CMD_USER_1`（映射为：`C3_MISSION_CMD`）
- 参数定义：
- `param1`: `mission_cmd`
    - `0` = `NOOP`
    - `1` = `START`
    - `2` = `BACK`
    - `3` = `CLOSE`
    - `4` = `HOLD`
  - `5` = `TRACKING`：关闭视觉节点，云台使用运动前馈/跟踪态
  - `6` = `DETECTING`：开启视觉节点，允许进入视觉检测/跟踪态
  - `param2`: `target_id`（可选，无目标填 `-1`）
  - `param3`: `timeout_s`（命令超时）
  - `param4~7`: 预留

  - `mission_cmd`：任务控制字。
    - `START/BACK/CLOSE/HOLD` 管任务流程
    - `TRACKING/DETECTING` 管视觉工作开关（全局使能/禁用）。
  - `target_id`：期望重点关注的目标编号；无特定目标时填 `-1`。
  - `timeout_s`：命令生效超时，超时后可回退到默认策略或请求重发。

#### B. 粗引导目标点（自定义 `C3_TARGET_HINT`）

字段：
- `uint32_t hint_id`
- `uint64_t t_usec`
- `float x_m` `float y_m` `float z_m`
- `float vx_mps` `float vy_mps` `float vz_mps`
- `float radius_m`
- `uint8_t target_type_hint`

频率：`1~2 Hz`。

字段解释：
- `hint_id`：粗引导消息编号。
- `t_usec`：该 hint 的生成时间戳，单位微秒；用于时序对齐
- `x_m/y_m/z_m`：目标粗位置，单位米。
- `vx_mps/vy_mps/vz_mps`：目标速度先验，单位米每秒；未知时填 `0`。
- `radius_m`：搜索半径或不确定性范围，供无人机到区后展开搜索。
- `target_type_hint`：目标类型先验，例如商船、渔船、浮标、漂浮障碍物等。


### 3.2 上行（无人机 -> 母船）

#### A. 目标观测回传（自定义 `C3_TARGET_OBS`）

字段：
- `uint32_t obs_id`
- `uint32_t track_id`
- `uint64_t t_usec`
- `float x_m` `float y_m` `float z_m`
- `float range_m`
- `float yaw_rad` `float pitch_rad`
- `uint8_t target_type`
- `float confidence`（`0~1`）
- `uint8_t status`：`0=VALID`, `1=LOW_CONF`, `2=LOST`

频率：事件触发为主，上限约 `5~20 Hz`；

字段解释：
- `obs_id`：单条观测消息编号，用于去重。
- `track_id`：连续跟踪轨迹编号；同一目标跨帧保持不变，丢失重建后可重新分配。
- `t_usec`：该观测对应的传感器融合时间戳，单位微秒；用于和云台姿态、飞控状态做时序对齐。
- `x_m/y_m/z_m`：目标相对无人机的位置，单位米。
- `range_m`：目标距离；通常等于位置向量模长。
- `yaw_rad/pitch_rad`：目标相对无人机机体或云台光轴的方位角/俯仰角，单位弧度。
- `target_type`：目标类别编号。
- `confidence`：当前观测可信度，范围 `0~1`。
- `status`：观测状态。`VALID` 表示可直接使用，`LOW_CONF` 表示可参考但不建议锁定，`LOST` 表示目标已丢失。

发送策略：
- 每当视觉/融合节点产出一条“新的观测结果”时，都立即发送一条 `C3_TARGET_OBS`。
- `VALID` 和 `LOW_CONF` 建议按新鲜结果逐条上报，但做发送频率上限控制，避免链路被高频重复结果占满。
- 当目标从“已观测”转为“丢失”时，立即发送一条 `status=LOST`。
- 处于持续丢失阶段时，不继续高频发送 `LOST`，如降到 `1 Hz` 左右。

#### B. 无人机任务状态（自定义 `C3_DRONE_STATUS`）

字段：
- `uint64_t t_usec`
- `uint8_t mission_mode`：`IDLE/TRANSIT/SEARCH/TRACK/RETURN/ABORT`
- `uint8_t gimbal_mode`：`TRACKING/DETECTING`
- `uint8_t link_state`：`OK/DEGRADED/LOST`
- `float battery_remain`（0~1）

频率：`1~2 Hz`

字段解释：
- `t_usec`：状态打包时间戳，单位微秒；用于说明这份状态对应的采样时刻。
- `mission_mode`：任务主状态机状态。
  - `gimbal_mode`：视觉/云台工作子状态。`TRACKING` 表示视觉节点关闭，云台按前馈保持；`DETECTING` 表示视觉节点开启，允许视觉请求接管。
- `link_state`：链路健康度，取值为 `OK/DEGRADED/LOST`。
- `battery_remain`：剩余电量比例。

#### C. 命令确认（复用 `COMMAND_ACK`）

- 对所有 `C3_MISSION_CMD` 必回 ACK；
- `result` 使用标准枚举，必要时在 `progress` 填阶段码。

---

## 4. ID 与可靠性策略

### 4.1 去重与乱序

- `hint_id`、`obs_id`、`track_id` 单调递增；
- 接收端维护最近窗口（ 256 条）做去重；
- 超窗包直接丢弃，不回退。

### 4.2 丢包与重发

- 命令类（`COMMAND_LONG`）采用“ACK 驱动重发”：
  - 超时 `300 ms` 未 ACK 重发；
  - 最大重发 `5` 次；
  - 失败后进入 `DEGRADED` 并上报。
- 观测类（`C3_TARGET_OBS`）不重发，仅依赖连续流。

### 4.3 心跳与失联

- 双方都发送 `HEARTBEAT`（`1 Hz`）；
- 连续 `3 s` 未收到对端心跳 => `link_state=DEGRADED`；
- 连续 `8 s` 未收到 => `link_state=LOST`，无人机执行 `BACK/HOLD`（按默认配置）。

---

## 5. 任务状态机

说明：
- 本项目需要区分两个层次的状态：
- `mission_mode`：任务主状态，描述飞行任务处于哪一步。
- `gimbal_mode`：视觉工作子状态，描述视觉节点是否开启、云台是否允许视觉接管。
- `TRACK` 和 `TRACKING` 不是一回事：前者是任务主状态，后者是视觉/云台子状态。

状态定义：
- `IDLE`：待命
- `TRANSIT`：飞往母船下发粗目标区
- `SEARCH`：在目标区搜索（通常要求 `gimbal_mode=DETECTING`）
- `TRACK`：已锁定目标并持续回传
- `RETURN`：返航
- `ABORT`：异常中止

视觉/云台工作子状态定义：
- `TRACKING`：视觉节点关闭，云台执行运动前馈或保持当前姿态。
- `DETECTING`：视觉节点开启，云台允许视觉命令接管，进行检测与跟踪。

关键转移：
1. `IDLE -> TRANSIT`：收到 `START` 且有有效 `C3_TARGET_HINT`。
2. `TRANSIT -> SEARCH`：到达 hint 半径阈值；主控可下发 `DETECTING`，开启视觉节点。
3. `SEARCH -> TRACK`：连续 `N` 帧 `C3_TARGET_OBS.status=VALID`。
4. `TRACK -> SEARCH`：连续 `T_lost` 丢失目标，但保持 `DETECTING` 继续搜。
5. 任意状态 -> `RETURN`：收到 `BACK` 或低电量；主控切回 `TRACKING`，关闭视觉节点。
6. 任意状态 -> `ABORT`：收到 `CLOSE` 或飞控严重故障；视觉节点可直接关闭。
7. 主控下发 `TRACKING`：全局禁用视觉，云台回到前馈/保持。
8. 主控下发 `DETECTING`：全局使能视觉，云台允许视觉请求接管。
9. 在 `DETECTING` 下，视觉节点可以通过服务请求云台局部接管；超时则回退前馈，持续超时再回 `TRACKING`。

---

## 6. 与 ROS2 节点映射

拆分为 3 个 C++ 节点：

1. `mavlink_bridge_node`
- 职责：MAVLink 收发、ACK、重发、心跳、去重。
- 订阅：`/mavlink/target_obs`、`/mission/state`。
- 发布：`/mission/cmd`、`/mission/target_hint`。

2. `mission_manager_node`
- 职责：任务状态机、命令执行策略（START/BACK/CLOSE）。
- 订阅：`/mission/cmd`、`/mission/target_hint`、`/px4/status`。
- 发布：`/mission/state`、`/offboard/goal`。

3. `target_processor_node`
- 职责：将融合后的观测规范化为 `C3_TARGET_OBS` 输入。
- 订阅：`/target/observation_body`（融合观测）。
- 发布：`/mavlink/target_obs`（供 bridge node 打包）。
- 说明：当前代码侧对应节点名为 `target_processor_node`，其内部调用的融合逻辑见 `doc/target_fusion.md`。

---

## 7. 自定义 MAVLink dialect 与 PX4 集成取舍

文件：`c3.xml`（放在协议仓库或 `resources/mavlink/`）

建议消息号段：
- `C3_TARGET_HINT`: `msgid 42010`
- `C3_TARGET_OBS`: `msgid 42011`
- `C3_DRONE_STATUS`: `msgid 42012`

原则：
- 避免与 common/ardupilotmega 冲突；
- 一次定版后仅追加字段，不改已有字段语义；
- 预留版本字段（可在消息尾追加 `uint8_t proto_ver`）。

设计取舍：
- 当前方案优先将 `C3_TARGET_HINT`、`C3_TARGET_OBS`、`C3_DRONE_STATUS` 放在“母船主控 <-> 无人机任务机”的链路上处理，而不是先落入 PX4 FCU 内部。
- 原因是这些消息主要服务于任务机上的 ROS2 感知、云台和任务管理节点，数据生产者/消费者都在伴飞计算机，不在飞控主循环内。
- 若直接采用 PX4 内部“自定义 uORB -> MAVLink stream”的官方路径，需要同时改 PX4、消息定义、MAVLink 打包逻辑，以及可能用到的地面站/SDK，维护成本更高。
- 只有当 FCU 本身必须直接消费或发布这些业务语义时，例如飞控侧需要基于目标观测做失效保护、记录 uORB 日志、或由 PX4 原生流控统一输出，才更推荐按 PX4 官方方式增加自定义 uORB 消息并映射为 MAVLink。

做方言的必要：
- 在 MAVLink 链路上传输 `C3_TARGET_HINT/C3_TARGET_OBS/C3_DRONE_STATUS` 这类标准消息里没有的结构化业务数据。
- 没有方言，就无法让两端以“有类型、可解析、可扩展”的方式识别这些自定义消息。

---

## 8. 参数

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

```
version -v1.0.2
```
