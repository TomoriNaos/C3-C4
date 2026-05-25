# 声呐设计框架

> 版本：0.1.0
> 日期：2026.05.25

# 1.项目背景

围绕 C3 比赛中“恶劣环境海上目标感知”任务，`c3_sonar_driver` 负责母船侧声呐链路：
1. 提供声呐安装位姿（URDF/xacro + TF）
2. 提供主控生命周期管理（srv + lifecycle client）
3. 提供对外通信发送节点（当前仅发送，不接收母船指令）
4. 为后续 SVD-HB GCC 与 Mackenzie 声速估计预留接口

# 2.节点分层

- `sonar_main_controller_node`（inner）
- `communicate_node`（bridge）
- `robot_state_publisher`（launch 中启动）

数据主链路：
1. 内部感知/处理节点产出 `/sonar/contact`
2. `sonar_main_controller_node` 产出 `/main_controller/status`
3. `communicate_node` 将 contact/status 发送到 `/sonar_link/*`

# 3.srv 与生命周期（主控）

沿用现有 `srv/SetDroneLifecycle.srv` 作为最小闭环：

- 服务名：`/main_controller/set_lifecycle`
- 服务类型：`c3_sonar_driver/srv/SetDroneLifecycle`
- 命令：
  - `CMD_ACTIVATE=1`
  - `CMD_DEACTIVATE=2`

`sonar_main_controller_node` 行为：
1. 对外提供 `/main_controller/set_lifecycle`
2. 对内调用 `lifecycle_msgs/srv/ChangeState`（默认目标 `/sonar_processor_node/change_state`）
3. 发布 `/main_controller/status`（`SonarStatus`）

# 4.xacro 与 TF 设计

文件：`urdf/c3_ship_with_sonar.urdf.xacro`

帧关系：
- `ship_base_link -> sonar_link`（固定安装）
- `sonar_link -> sonar_optical_frame`（光学帧约定）

可调参数：
- `sonar_mount_xyz`
- `sonar_mount_rpy`

发布方式：
- `launch/sonar_core.launch.py` 中使用 `xacro + robot_state_publisher` 统一发布 TF

# 5.communicate_node 发送内容

当前要求：只发送，不接收母船信息。

基础发送：
- `/sonar_link/heartbeat` (`std_msgs/Bool`)
- `/sonar_link/contact` (`SonarContact`)
- `/sonar_link/state` (`SonarStatus`)
- `/sonar_link/command_ack` (`SonarCommandAck`，为后续扩展保留)

除“点云+时间戳”外建议必须发送：
1. `contact_id`：去重与关联跟踪
2. `object_type`：目标类型（船/浮标/漂浮障碍物/小渔船）
3. `position` + `velocity`：相对位姿和速度
4. `range_m` + `bearing_rad`：便于主控快速航迹决策
5. `confidence`：用于融合层置信加权
6. `status`：链路状态、任务模式、声呐激活状态
7. `estimated_sound_speed_mps`：Mackenzie 输出
8. `estimated_latency_ms`：SVD-HB GCC 时延估计输出

# 6.内部话题结构

输入（对内）：
- `/sonar/contact` (`SonarContact`)：处理节点输出目标
- `/main_controller/status` (`SonarStatus`)：主控状态

发送（对外）：
- `/sonar_link/heartbeat`
- `/sonar_link/contact`
- `/sonar_link/state`
- `/sonar_link/command_ack`

生命周期控制：
- `/main_controller/set_lifecycle` (`SetDroneLifecycle`)
- `/sonar_processor_node/change_state` (`lifecycle_msgs/ChangeState`)

# 7.推荐项目结构

```text
c3_sonar_driver/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── communicate.yaml
│   └── main_controller.yaml
├── doc/
│   └── frame.md
├── launch/
│   └── sonar_core.launch.py
├── msg/
│   ├── SonarCommandAck.msg
│   ├── SonarContact.msg
│   ├── SonarMissionCommand.msg
│   └── SonarStatus.msg
├── srv/
│   └── SetDroneLifecycle.srv
├── src/
│   ├── bridge_node/
│   │   └── communicate_node.cpp
│   └── inner_node/
│       └── sonar_main_controller_node.cpp
└── urdf/
    └── c3_ship_with_sonar.urdf.xacro
```

# 8.后续对接点

1. 在 `sonar_processor_node` 中落地 Mackenzie 声速估计，写入 `SonarStatus.estimated_sound_speed_mps`
2. 在时延估计模块落地 SVD-HB GCC，写入 `SonarStatus.estimated_latency_ms`
3. 后续如果启用双向链路，再为 `communicate_node` 增加 `*_rx` 输入并复用 ACK 机制
