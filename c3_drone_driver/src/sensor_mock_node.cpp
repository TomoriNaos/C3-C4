#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "c3_drone_driver/msg/mission_command.hpp"
#include "c3_drone_driver/msg/tc_detection.hpp"

namespace c3_drone_driver
{

class SensorMockNode : public rclcpp::Node
{
public:
  SensorMockNode()
  : Node("sensor_mock_node")
  {
    pub_hz_ = declare_parameter<double>("pub_hz", 10.0);
    cloud_width_ = declare_parameter<int>("cloud_width", 320);
    cloud_height_ = declare_parameter<int>("cloud_height", 180);
    target_x_m_ = declare_parameter<double>("target_x_m", 18.0);
    target_y_m_ = declare_parameter<double>("target_y_m", 2.0);
    target_z_m_ = declare_parameter<double>("target_z_m", 0.5);
    target_type_ = declare_parameter<int>("target_type", 2);
    target_id_ = declare_parameter<int>("target_id", 101);
    bbox_w_px_ = declare_parameter<double>("bbox_w_px", 120.0);
    bbox_h_px_ = declare_parameter<double>("bbox_h_px", 100.0);
    yaw_amp_rad_ = declare_parameter<double>("yaw_amp_rad", 0.18);
    yaw_freq_hz_ = declare_parameter<double>("yaw_freq_hz", 0.08);
    noise_xy_m_ = declare_parameter<double>("noise_xy_m", 0.04);
    noise_z_m_ = declare_parameter<double>("noise_z_m", 0.08);

    tc_detection_pub_ = create_publisher<msg::TcDetection>("/tc/detection", rclcpp::SensorDataQoS());
    gc_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/gc/points", rclcpp::SensorDataQoS());
    mission_cmd_pub_ = create_publisher<msg::MissionCommand>("/mission/cmd", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, pub_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SensorMockNode::onTick, this));

    start_time_ = now();
    RCLCPP_INFO(get_logger(), "sensor_mock_node started");
  }

private:
  void onTick()
  {
    const auto stamp = now();
    const double t = (stamp - start_time_).seconds();

    publishMission(stamp);
    const auto target = targetPositionAt(t);
    publishTcDetection(stamp, target);
    publishCloud(stamp, target, "gated_camera_optical_frame", gc_pc_pub_, 1.0);
  }

  geometry_msgs::msg::Point targetPositionAt(double t) const
  {
    geometry_msgs::msg::Point p;
    p.x = target_x_m_;
    p.y = target_y_m_ + 1.2 * std::sin(2.0 * M_PI * yaw_freq_hz_ * t);
    p.z = target_z_m_ + 0.5 * std::cos(2.0 * M_PI * yaw_freq_hz_ * t);
    return p;
  }

  void publishMission(const rclcpp::Time &stamp)
  {
    msg::MissionCommand cmd;
    cmd.header.stamp = stamp;
    cmd.command = msg::MissionCommand::CMD_DETECTING;
    cmd.target_id = target_id_;
    cmd.timeout_s = 0.0F;
    mission_cmd_pub_->publish(cmd);
  }

  void publishTcDetection(const rclcpp::Time &stamp, const geometry_msgs::msg::Point &target)
  {
    const double yaw = std::atan2(target.y, std::max(1e-4, target.x));
    const double pitch = std::atan2(target.z, std::sqrt(target.x * target.x + target.y * target.y));
    const double cx = static_cast<double>(cloud_width_) * 0.5 + (yaw / yaw_amp_rad_) * 120.0;
    const double cy = static_cast<double>(cloud_height_) * 0.5 - (pitch / yaw_amp_rad_) * 80.0;

    msg::TcDetection detection;
    detection.header.stamp = stamp;
    detection.header.frame_id = "tc_camera_optical_frame";
    detection.bbox.data = {
      static_cast<float>(std::clamp(cx - bbox_w_px_ * 0.5, 0.0, static_cast<double>(cloud_width_ - 1))),
      static_cast<float>(std::clamp(cy - bbox_h_px_ * 0.5, 0.0, static_cast<double>(cloud_height_ - 1))),
      static_cast<float>(bbox_w_px_),
      static_cast<float>(bbox_h_px_),
      0.92F,
      static_cast<float>(target_id_),
      static_cast<float>(target_type_)
    };
    detection.cloud = buildCloud(stamp, target, "tc_camera_optical_frame", 0.9);
    tc_detection_pub_->publish(detection);
  }

  sensor_msgs::msg::PointCloud2 buildCloud(
    const rclcpp::Time &stamp,
    const geometry_msgs::msg::Point &target,
    const std::string &frame_id,
    double confidence_scale)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id;
    cloud.width = static_cast<uint32_t>(cloud_width_);
    cloud.height = static_cast<uint32_t>(cloud_height_);
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(static_cast<std::size_t>(cloud_width_ * cloud_height_));

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    const int cx = cloud_width_ / 2;
    const int cy = cloud_height_ / 2;
    for (int v = 0; v < cloud_height_; ++v)
    {
      for (int u = 0; u < cloud_width_; ++u, ++iter_x, ++iter_y, ++iter_z)
      {
        const int du = u - cx;
        const int dv = v - cy;
        const double r2 = static_cast<double>(du * du + dv * dv);
        const bool inside = r2 < 2200.0;
        const double bg_depth = 60.0 + 0.005 * r2;
        const double noise = (std::sin((u + v) * 0.07) + std::cos((u - v) * 0.05)) * 0.5;

        if (inside)
        {
          *iter_x = static_cast<float>(target.x + noise_xy_m_ * noise);
          *iter_y = static_cast<float>(target.y + noise_xy_m_ * std::sin(u * 0.03));
          *iter_z = static_cast<float>(target.z + noise_z_m_ * std::cos(v * 0.02) / confidence_scale);
        }
        else
        {
          *iter_x = static_cast<float>(bg_depth);
          *iter_y = static_cast<float>(0.02 * du);
          *iter_z = static_cast<float>(0.02 * dv);
        }
      }
    }
    return cloud;
  }

  void publishCloud(
    const rclcpp::Time &stamp,
    const geometry_msgs::msg::Point &target,
    const std::string &frame_id,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
    double confidence_scale)
  {
    pub->publish(buildCloud(stamp, target, frame_id, confidence_scale));
  }

  rclcpp::Publisher<msg::TcDetection>::SharedPtr tc_detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gc_pc_pub_;
  rclcpp::Publisher<msg::MissionCommand>::SharedPtr mission_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  double pub_hz_{10.0};
  int cloud_width_{320};
  int cloud_height_{180};
  double target_x_m_{18.0};
  double target_y_m_{2.0};
  double target_z_m_{0.5};
  int target_type_{2};
  int target_id_{101};
  double bbox_w_px_{120.0};
  double bbox_h_px_{100.0};
  double yaw_amp_rad_{0.18};
  double yaw_freq_hz_{0.08};
  double noise_xy_m_{0.04};
  double noise_z_m_{0.08};
};

}  // namespace c3_drone_driver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<c3_drone_driver::SensorMockNode>());
  rclcpp::shutdown();
  return 0;
}
