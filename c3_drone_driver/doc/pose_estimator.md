# Pose Estimator

> 版本：v1.2.0  
> 更新日期：2026-05-19

## 1. 目标

`PoseEstimator` 负责在主控内部完成：
- 无人机位姿缓存
- 云台姿态缓存
- 目标观测在云台 / 机体 / NED 坐标系之间的转换
- 目标方位角、俯仰角和距离估计

---

## 2. 类接口

### `PoseEstimator::Config`

当前仅包含机体到云台的平移外参：
- `body_to_gimbal_x`
- `body_to_gimbal_y`
- `body_to_gimbal_z`

对应配置文件：`config/pose_estimator_default.yaml`

### `PoseEstimator::Result`

输出结果包含：
- `position_gimbal`
- `position_body`
- `position_ned`
- `range`
- `yaw_body`
- `pitch_body`

### 核心方法

- `updateVehiclePose()`
- `updateGimbalState()`
- `transformObservation()`
- `bodyPointToGimbalYawPitch()`
- `shipRelativePointToNed()`

---

## 3. 坐标约定

```text
Gimbal Frame -> Body Frame -> NED Frame
```

当前实现里：
- `R_body_gimbal = Rz(yaw) * Ry(pitch)`
- `R_ned_body` 由 PX4 / EKF2 的四元数位姿换算
- `T_body_gimbal` 来自配置文件中的平移外参

---

## 4. 观测转换逻辑

### `transformObservation()`

输入：`TargetObservation`

当前实现规则：
- 若 `obs.frame == FRAME_BODY_DRONE`，则认为 `obs.position` 已经在机体系
- 否则，认为 `obs.position` 在云台系，先做云台到机体变换，再进入 NED

计算链路：
1. `p_body = p_in` 或 `R_body_gimbal * p_in + T_body_gimbal`
2. `p_ned = t_ned_body + R_ned_body * p_body`
3. `bearing(p_body)` 计算机体系下的方位与俯仰

---

## 5. 云台反解

### `bodyPointToGimbalYawPitch()`

作用：把机体系目标点反解成云台应转到的 yaw / pitch。

步骤：
1. 用当前云台姿态构造 `R_body_gimbal`
2. 取逆得到 `R_gimbal_body`
3. 计算 `p_gimbal = R_gimbal_body * (p_body - T_body_gimbal)`
4. 对 `p_gimbal` 做 `bearing()`

---

## 6. 船坐标到 NED

### `shipRelativePointToNed()`

当前实现只使用船体航向角（yaw），不展开完整 6DoF 船姿。

计算方式：
- 从船位姿四元数提取 yaw
- 在平面内旋转目标相对点
- 叠加船在 NED 中的位置

这适合当前“粗目标点 -> 无人机任务点”的简化场景。

---

## 7. 与主控的关系

`drone_main_controller_node` 负责：
- 订阅 `/px4/vehicle_pose`
- 订阅 `/gimbal/state`
- 订阅 `/target/observation_body`
- 调用 `PoseEstimator` 完成转换
- 反向生成 `/gimbal/motion_command`

---

## 8. 备注

- 当前实现没有做时间同步补偿
- 没有做完整相机内外参标定链路
- 没有直接处理 NED 输入观测，只处理当前消息定义中的 `frame` 语义
