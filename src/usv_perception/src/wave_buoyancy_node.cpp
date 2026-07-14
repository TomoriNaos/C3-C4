#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "builtin_interfaces/msg/duration.hpp"
#include "gazebo_msgs/srv/apply_link_wrench.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "usv_perception/common.hpp"

using namespace std::chrono_literals;

namespace usv_perception
{

class WaveBuoyancyNode : public rclcpp::Node
{
public:
  WaveBuoyancyNode()
  : Node("wave_buoyancy_node"), start_time_(std::chrono::steady_clock::now())
  {
    link_name_ = declare_parameter<std::string>("link_name", "wamv::base_footprint");
    water_level_ = declare_parameter<double>("water_level", 0.32);
    wave_amplitude_ = declare_parameter<double>("wave_amplitude", 0.10);
    const double update_rate = declare_parameter<double>("update_rate", 5.0);
    vertical_force_scale_ = declare_parameter<double>("vertical_force_scale", 45.0);
    torque_scale_ = declare_parameter<double>("torque_scale", 12.0);

    client_ = create_client<gazebo_msgs::srv::ApplyLinkWrench>("/apply_link_wrench");
    status_pub_ = create_publisher<std_msgs::msg::String>("wave_buoyancy/status", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(update_rate, 0.1)));
    timer_ = create_wall_timer(period, std::bind(&WaveBuoyancyNode::on_timer, this));
  }

private:
  double elapsed_seconds() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  }

  void on_timer()
  {
    if (!client_->service_is_ready()) {
      if (!warned_waiting_) {
        RCLCPP_INFO(get_logger(), "Waiting for /apply_link_wrench to apply wave buoyancy");
        warned_waiting_ = true;
      }
      return;
    }

    const double t = elapsed_seconds();
    const double z = water_level_ + wave_height(0.0, 0.0, t, wave_amplitude_);
    const auto [dzdx, dzdy] = wave_slope(0.0, 0.0, t, wave_amplitude_);
    const double pitch = std::atan2(dzdx, 1.0);
    const double roll = -std::atan2(dzdy, 1.0);

    auto req = std::make_shared<gazebo_msgs::srv::ApplyLinkWrench::Request>();
    req->link_name = link_name_;
    req->reference_frame = link_name_;
    req->reference_point = geometry_msgs::msg::Point();
    req->wrench = geometry_msgs::msg::Wrench();
    req->wrench.force.z = vertical_force_scale_ * std::sin(0.9 * t);
    req->wrench.torque.x = torque_scale_ * roll;
    req->wrench.torque.y = torque_scale_ * pitch;
    req->start_time = get_clock()->now();
    req->duration = builtin_interfaces::msg::Duration();
    req->duration.sec = 2;
    client_->async_send_request(req);

    std_msgs::msg::String status;
    std::ostringstream text;
    text.setf(std::ios::fixed, std::ios::floatfield);
    text.precision(3);
    text << link_name_ << ": wave_z=" << z << ", roll_cmd=" << roll << ", pitch_cmd=" << pitch;
    status.data = text.str();
    status_pub_->publish(status);
  }

  std::string link_name_;
  double water_level_{0.32};
  double wave_amplitude_{0.10};
  double vertical_force_scale_{45.0};
  double torque_scale_{12.0};
  bool warned_waiting_{false};
  std::chrono::steady_clock::time_point start_time_;
  rclcpp::Client<gazebo_msgs::srv::ApplyLinkWrench>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::WaveBuoyancyNode>());
  rclcpp::shutdown();
  return 0;
}
