#include "c3_drone_driver/pose_estimator.h"

#include <cmath>

namespace c3_drone_driver
{

PoseEstimator::PoseEstimator(const Config &config)
: config_(config)
{
  r_ned_body_ = {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  t_ned_body_ = {0.0, 0.0, 0.0};
}

void PoseEstimator::updateVehiclePose(const geometry_msgs::msg::PoseStamped &pose_msg)
{
  const auto &p = pose_msg.pose.position;
  const auto &q = pose_msg.pose.orientation;
  t_ned_body_ = {p.x, p.y, p.z};
  r_ned_body_ = quatToRotNedBody(q.x, q.y, q.z, q.w);
  last_pose_stamp_ = rclcpp::Time(pose_msg.header.stamp);
  has_vehicle_pose_ = true;
}

void PoseEstimator::updateGimbalState(const msg::GimbalState &gimbal_msg)
{
  gimbal_yaw_ = static_cast<double>(gimbal_msg.yaw);
  gimbal_pitch_ = static_cast<double>(gimbal_msg.pitch);
  last_gimbal_stamp_ = rclcpp::Time(gimbal_msg.header.stamp);
  has_gimbal_state_ = true;
}

std::optional<PoseEstimator::Result> PoseEstimator::transformObservation(
  const msg::TargetObservation &obs) const
{
  if (!has_vehicle_pose_ || !has_gimbal_state_) {
    return std::nullopt;
  }

  const Vec3 p_in = {obs.position.x, obs.position.y, obs.position.z};
  const Mat3 r_body_gimbal = buildRBodyGimbal(gimbal_yaw_, gimbal_pitch_);
  const Vec3 t_body_gimbal = {
    config_.body_to_gimbal_x, config_.body_to_gimbal_y, config_.body_to_gimbal_z};

  Vec3 p_body = p_in;
  if (obs.frame == msg::TargetObservation::FRAME_BODY_DRONE) {
    p_body = p_in;
  } else {
    const Vec3 rotated = matVecMul(r_body_gimbal, p_in);
    p_body = {
      rotated[0] + t_body_gimbal[0],
      rotated[1] + t_body_gimbal[1],
      rotated[2] + t_body_gimbal[2]
    };
  }

  const Vec3 p_ned = {
    t_ned_body_[0] + matVecMul(r_ned_body_, p_body)[0],
    t_ned_body_[1] + matVecMul(r_ned_body_, p_body)[1],
    t_ned_body_[2] + matVecMul(r_ned_body_, p_body)[2]
  };

  const auto [yaw_body, pitch_body] = bearing(p_body);
  Result out;
  out.position_gimbal = p_in;
  out.position_body = p_body;
  out.position_ned = p_ned;
  out.range = norm(p_body);
  out.yaw_body = yaw_body;
  out.pitch_body = pitch_body;
  return out;
}

std::optional<std::pair<double, double>> PoseEstimator::bodyPointToGimbalYawPitch(
  const std::array<double, 3> &point_body) const
{
  if (!has_gimbal_state_) {
    return std::nullopt;
  }
  const Mat3 r_body_gimbal = buildRBodyGimbal(gimbal_yaw_, gimbal_pitch_);
  const Mat3 r_gimbal_body = transpose(r_body_gimbal);
  const Vec3 t_body_gimbal = {
    config_.body_to_gimbal_x, config_.body_to_gimbal_y, config_.body_to_gimbal_z};
  const Vec3 rel = {
    point_body[0] - t_body_gimbal[0],
    point_body[1] - t_body_gimbal[1],
    point_body[2] - t_body_gimbal[2]
  };
  const Vec3 p_gimbal = matVecMul(r_gimbal_body, rel);
  return bearing(p_gimbal);
}

PoseEstimator::Mat3 PoseEstimator::buildRBodyGimbal(double yaw, double pitch) const
{
  return matMul(rZ(yaw), rY(pitch));
}

PoseEstimator::Mat3 PoseEstimator::matMul(const Mat3 &a, const Mat3 &b)
{
  Mat3 c{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      c[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) {
        c[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  return c;
}

PoseEstimator::Vec3 PoseEstimator::matVecMul(const Mat3 &a, const Vec3 &v)
{
  return {
    a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
    a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
    a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2]
  };
}

PoseEstimator::Mat3 PoseEstimator::transpose(const Mat3 &a)
{
  return {{
    {{a[0][0], a[1][0], a[2][0]}},
    {{a[0][1], a[1][1], a[2][1]}},
    {{a[0][2], a[1][2], a[2][2]}}
  }};
}

PoseEstimator::Mat3 PoseEstimator::rZ(double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

PoseEstimator::Mat3 PoseEstimator::rY(double pitch)
{
  const double c = std::cos(pitch);
  const double s = std::sin(pitch);
  return {{{c, 0.0, s}, {0.0, 1.0, 0.0}, {-s, 0.0, c}}};
}

PoseEstimator::Mat3 PoseEstimator::quatToRotNedBody(double x, double y, double z, double w)
{
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n < 1e-9) {
    return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  }
  x /= n;
  y /= n;
  z /= n;
  w /= n;
  return {{
    {{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)}},
    {{2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)}},
    {{2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)}}
  }};
}

double PoseEstimator::norm(const Vec3 &v)
{
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

std::pair<double, double> PoseEstimator::bearing(const Vec3 &v)
{
  const double yaw = std::atan2(v[1], v[0]);
  const double horizontal = std::sqrt(v[0] * v[0] + v[1] * v[1]);
  const double pitch = std::atan2(v[2], horizontal);
  return {yaw, pitch};
}

}  // namespace c3_drone_driver
