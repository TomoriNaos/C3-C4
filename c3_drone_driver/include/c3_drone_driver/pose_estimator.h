#pragma once

#include <array>
#include <optional>

#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/target_observation.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"

namespace c3_drone_driver
{

class PoseEstimator
{
public:
  struct Config
  {
    double body_to_gimbal_x{0.0};
    double body_to_gimbal_y{0.0};
    double body_to_gimbal_z{0.0};
  };

  struct Result
  {
    std::array<double, 3> position_gimbal{};
    std::array<double, 3> position_body{};
    std::array<double, 3> position_ned{};
    double range{0.0};
    double yaw_body{0.0};
    double pitch_body{0.0};
  };

  explicit PoseEstimator(const Config &config);

  void updateVehiclePose(const geometry_msgs::msg::PoseStamped &pose_msg);
  void updateGimbalState(const msg::GimbalState &gimbal_msg);
  std::optional<Result> transformObservation(const msg::TargetObservation &obs) const;
  std::optional<std::pair<double, double>> bodyPointToGimbalYawPitch(
    const std::array<double, 3> &point_body) const;

private:
  using Mat3 = std::array<std::array<double, 3>, 3>;
  using Vec3 = std::array<double, 3>;

  static Mat3 matMul(const Mat3 &a, const Mat3 &b);
  static Vec3 matVecMul(const Mat3 &a, const Vec3 &v);
  static Mat3 transpose(const Mat3 &a);
  static Mat3 rZ(double yaw);
  static Mat3 rY(double pitch);
  static Mat3 quatToRotNedBody(
    double x, double y, double z, double w);

  static double norm(const Vec3 &v);
  static std::pair<double, double> bearing(const Vec3 &v);

  Mat3 buildRBodyGimbal(double yaw, double pitch) const;

  Config config_{};
  bool has_vehicle_pose_{false};
  bool has_gimbal_state_{false};
  Mat3 r_ned_body_{};
  Vec3 t_ned_body_{};
  double gimbal_yaw_{0.0};
  double gimbal_pitch_{0.0};
  rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gimbal_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace c3_drone_driver
