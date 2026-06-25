#pragma once

#include <cmath>
#include <utility>

#include "geometry_msgs/msg/quaternion.hpp"

namespace usv_perception
{

inline geometry_msgs::msg::Quaternion quaternion_from_euler(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

inline double wave_height(double x, double y, double t, double amplitude = 0.10)
{
  const double primary = amplitude * std::sin(0.24 * x + 0.10 * y - 0.85 * t);
  const double cross = 0.45 * amplitude * std::sin(-0.12 * x + 0.20 * y - 0.55 * t + 1.3);
  const double ripple = 0.22 * amplitude * std::sin(0.55 * x - 1.45 * t);
  return primary + cross + ripple;
}

inline std::pair<double, double> wave_slope(double x, double y, double t, double amplitude = 0.10)
{
  const double dzdx =
    amplitude * 0.24 * std::cos(0.24 * x + 0.10 * y - 0.85 * t) -
    0.45 * amplitude * 0.12 * std::cos(-0.12 * x + 0.20 * y - 0.55 * t + 1.3) +
    0.22 * amplitude * 0.55 * std::cos(0.55 * x - 1.45 * t);
  const double dzdy =
    amplitude * 0.10 * std::cos(0.24 * x + 0.10 * y - 0.85 * t) +
    0.45 * amplitude * 0.20 * std::cos(-0.12 * x + 0.20 * y - 0.55 * t + 1.3);
  return {dzdx, dzdy};
}

}  // namespace usv_perception
