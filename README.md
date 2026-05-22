# C3 Drone Driver

> 版本：v1.0.0  
> 更新日期：2026-05-19

无人机主链路：
- 接收母船下发的任务命令与粗目标点（经 MAVLink 桥接）
- 融合 TC/GC (传统相机/门控相机)感知结果，输出目标观测与云台视觉跟踪命令
- 完成云台 yaw/pitch 仲裁控制（视觉优先于运动前馈）
- 由主控与运动模块生成 Offboard 目标点，驱动 PX4 飞行（联调模式）

详细设计见 `doc/`：
- `doc/frame.md`
- `doc/mavlink_design.md`
- `doc/pose_estimator.md`
- `doc/target_fusion.md`

---

## 1. 项目结构

```text
.
├── c3_drone_driver/                # ROS2 功能包
│   ├── src/
│   │   ├── inner_node/             # 主控/运动/云台/融合节点
│   │   └── bridge_node/            # PX4/MAVLink/仿真桥接节点
│   ├── msg/                        # 自定义消息
│   ├── srv/                        # 自定义服务
│   ├── config/                     # 参数配置
│   ├── launch/                     # 一体化启动文件
│   └── urdf/                       # 云台/机体模型
├── doc/                            # 设计文档
└── tools/                          # 环境与检查脚本
```

---

## 2. PX4 相关环境配置

> 可以手动执行下列步骤，或直接用脚本：`tools/setup_px4_stack.sh`

### 2.1 PX4 Autopilot

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

### 2.2 `px4_msgs`（ROS2 消息）

建议将 `px4_msgs` 放到你电脑中一个单独 ROS2 工作区（例如 `~/px4_ros2_ws/src`），并与 PX4 固件版本匹配。

```bash
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ..
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs
source install/setup.bash
```

### 2.3 Micro-XRCE-DDS-Agent

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

运行 Agent（PX4 ROS2 通信必需）：

```bash
MicroXRCEAgent udp4 -p 8888
```

---

## 3. 编译本项目

在本仓库根目录：

```bash
source /opt/ros/humble/setup.bash
# 如使用 PX4 原生消息桥，还需 source 你的 px4_msgs 工作区
# source ~/px4_ros2_ws/install/setup.bash

colcon build --packages-select c3_drone_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 4. 运行指令

## 4.1 Gazebo 仿真一体化

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch c3_drone_driver c3_gazebo_sim.launch.py
```

说明：
- 会启动 Gazebo + 机体模型 + `sensor_mock_node`
- 同时启动主链路节点：`target_processor`、`gimbal_controller`、`drone_main_controller`、`motion_controller`、`mavlink_bridge` 及桥接节点

## 4.2 不启动 Gazebo，仅启动核心节点

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch c3_drone_driver c3_drone_core.launch.py
```

## 4.3 云台单独联调

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch c3_drone_driver gimbal_sim.launch.py use_gui:=true use_controller:=true
```

## 4.4 与 PX4 SITL 联调

终端 A（启动 PX4 SITL，目录在 `PX4-Autopilot`）：
```bash
make px4_sitl gz_x500
```

终端 B（启动 XRCE Agent）：
```bash
MicroXRCEAgent udp4 -p 8888
```

终端 C（启动本项目核心）：
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch c3_drone_driver c3_drone_core.launch.py
```

终端 D（如果已编译出 `offboard_setpoint_px4_bridge_node`）：
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run c3_drone_driver offboard_setpoint_px4_bridge_node
```

---

## 5. Launch 说明

- `c3_drone_driver/launch/c3_gazebo_sim.launch.py`
  - Gazebo 联合仿真入口；包含模型生成、`sensor_mock_node`、全套控制/桥接节点。
- `c3_drone_driver/launch/c3_drone_core.launch.py`
  - 算法核心入口；不拉起 Gazebo，仅启动业务主链路节点。
- `c3_drone_driver/launch/gimbal_sim.launch.py`
  - 云台专项调试；支持 GUI 手动关节控制和控制器驱动模式。

---

## 6. 仿真说明

### 6.1 当前仓库内置仿真链路

`sensor_mock_node` 会发布：
- `/tc/detection`（含 bbox + TC 点云）
- `/gc/detection`（含 bbox + GC 点云）
- `/mission/cmd`（默认 `CMD_DETECTING`）

用于驱动 `target_processor_node -> gimbal_controller_node -> drone_main_controller_node -> motion_controller_node` 的完整闭环。

### 6.2 与 PX4 的关系

- 当前 `c3_drone_core.launch.py` 默认使用 `offboard_setpoint_bridge_node`（`PoseStamped -> PoseStamped`）
- 若要真正给 PX4 `fmu/in/*` 发送控制，需运行 `offboard_setpoint_px4_bridge_node`（依赖 `px4_msgs`）

---

## 7. 外部接口（TC / GC / 母船）

## 7.1 TC（Traditional Camera）接口

TC 对本项目发布：
- Topic: `/tc/detection`
- Type: `c3_drone_driver/msg/TcDetection`
- 关键字段：
  - `header.stamp`
  - `bbox.data = [x, y, w, h, confidence, target_id]`
  - `cloud`（`sensor_msgs/PointCloud2`，TC 光学坐标系）

本项目对 TC 不做强制订阅要求（TC 可订阅以下辅助信息）：
- `/gimbal/state` (`c3_drone_driver/msg/GimbalState`)

## 7.2 GC（Gated Camera）接口

GC 对本项目发布：
- Topic: `/gc/detection`
- Type: `c3_drone_driver/msg/TcDetection`
- 关键字段：
  - `header.stamp`
  - `bbox.data = [x, y, w, h, confidence, target_id]`
  - `cloud`（`sensor_msgs/PointCloud2`，GC 光学坐标系）

## 7.3 母船主控（MAVLink 上下行桥）接口

> 当前仓库内 `mavlink_bridge_node` 是 ROS 占位。真实串口/UDP 打包解包可接入这些 topic。

母船 -> 无人机（桥接输入）：
- `/mavlink/mission_cmd_rx` (`c3_drone_driver/msg/MissionCommand`)
- `/mavlink/ship_pose_world_rx` (`geometry_msgs/msg/PoseStamped`)
- `/mavlink/ship_target_point_rx` (`geometry_msgs/msg/PoseStamped`, `frame_id="ship"`)
- `/mavlink/heartbeat_rx` (`std_msgs/msg/Bool`)

无人机 -> 母船（桥接输出）：
- `/mavlink/target_obs` (`c3_drone_driver/msg/TargetObservation`)
- `/mavlink/mission_cmd_ack` (`c3_drone_driver/msg/CommandAck`)
- `/mavlink/heartbeat_tx` (`std_msgs/msg/Bool`)
- `/mission/state` (`c3_drone_driver/msg/DroneStatus`)

---

## 8. 节点功能简述

- `target_processor_node`
  - 融合 TC/GC 数据，发布 `/target/observation_body` 与 `/gimbal/visual_command`
- `gimbal_controller_node`
  - 执行云台 yaw/pitch 限幅和控制权仲裁（视觉优先）
- `drone_main_controller_node`
  - 汇总任务与感知状态，发布 `/mission/goal`、`/gimbal/motion_command`、`/main_controller/status`
- `motion_controller_node`
  - 基于任务目标和当前位姿生成 `/px4/offboard_goal`
- `mavlink_bridge_node`
  - 任务命令/心跳/目标观测的 ROS 侧桥接与链路状态输出
- `px4_pose_bridge_node`
  - 将 `/odom` 或 `/pose` 归一为 `/px4/vehicle_pose`
- `offboard_setpoint_bridge_node`
  - 将 `/px4/offboard_goal` 转发为 `/px4/setpoint_pose`
- `offboard_setpoint_px4_bridge_node`
  - 将 Offboard 目标转换为 PX4 `fmu/in/*` 原生消息（需 `px4_msgs`）

---

## 9. tools 说明

- `tools/setup_px4_stack.sh`
  - 一键准备 PX4 / px4_msgs / Micro-XRCE-DDS-Agent（可选）
- `tools/verify_stack.sh`
  - 检查编译结果、launch 语法、关键节点与 topic 是否齐全

使用示例：

```bash
# 仿真模式检查
bash tools/verify_stack.sh sim

# PX4联调模式检查（会额外检查 /fmu/in/* 话题）
bash tools/verify_stack.sh px4
```

---
