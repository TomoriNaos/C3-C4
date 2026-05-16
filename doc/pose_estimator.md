# Pose Estimator

## 1. 目标

`PoseEstimator` 负责在主控内部完成无人机位姿、云台姿态和目标观测之间的坐标转换。

## 2. 输入输出

### 输入

- PX4 / EKF2 位姿
- 云台 `yaw / pitch`
- `TargetObservation`
- 观测时间戳

### 输出

- `gimbal` 坐标系目标位姿
- `body` 坐标系目标位姿
- `NED` 坐标系目标位姿
- 目标方位角 / 俯仰角 / 距离

## 3. 类职责

### `PoseEstimator`

职责：
- 缓存当前飞控位姿
- 缓存当前云台姿态
- 计算 `Rz(yaw) * Ry(pitch)`
- 执行 `gimbal -> body -> NED` 坐标变换
- 提供目标方位和距离计算

### 核心接口

```text
updateVehiclePose()
updateGimbalState()
buildRBodyGimbal(yaw, pitch)
transformObservation()
bodyToNed()
computeBearing()
```

## 4. 坐标约定

```text
Camera Frame -> Gimbal Frame -> Body Frame -> NED Frame
```

实现约定：

- `T_gimbal_camera` 来自标定
- `R_body_gimbal = Rz(yaw) * Ry(pitch)`
- `R_ned_body` 来自 PX4/EKF2

## 5. 与主控的关系

- 主控调用本类做坐标变换
- `drone_main_controller_node` 作为主控节点，负责：
  - 订阅 PX4/EKF2 位姿（`/px4/vehicle_pose`）并调用 `updateVehiclePose()`
  - 订阅云台状态（`/gimbal/state`）并调用 `updateGimbalState()`
  - 订阅融合观测（`/target/observation_body`）并调用 `transformObservation()`
  - 发布转换后的 NED 观测（`/target/observation_ned`）
  - 发布云台前馈命令（`/gimbal/motion_command`）
