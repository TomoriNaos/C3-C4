#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "usv_perception/common.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace usv_perception
{

class DynamicTargetController : public rclcpp::Node
{
public:
  DynamicTargetController()
  : Node("dynamic_target_controller"), start_time_(std::chrono::steady_clock::now())
  {
    const double update_rate = declare_parameter<double>("update_rate", 15.0);
    wave_amplitude_ = declare_parameter<double>("wave_amplitude", 0.10);
    motion_time_scale_ = declare_parameter<double>("motion_time_scale", 3.0);
    vessel_name_ = declare_parameter<std::string>("vessel_name", "moving_vessel");
    fishing_boat_name_ = declare_parameter<std::string>("fishing_boat_name", "small_fishing_boat");
    fishnet_buoy_name_ = declare_parameter<std::string>("fishnet_buoy_name", "fishnet_buoy");
    obstacle_name_ = declare_parameter<std::string>("obstacle_name", "floating_obstacle");
    debris_name_ = declare_parameter<std::string>("debris_name", "drift_debris");

    client_ = create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("simulated_targets", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(update_rate, 0.1)));
    timer_ = create_wall_timer(period, std::bind(&DynamicTargetController::on_timer, this));
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
        RCLCPP_INFO(get_logger(), "Waiting for /set_entity_state to move simulated targets");
        warned_waiting_ = true;
      }
      return;
    }

    const double t = elapsed_seconds() * motion_time_scale_;
    std::vector<gazebo_msgs::msg::EntityState> targets;
    targets.reserve(5);
    targets.push_back(vessel_state(t));
    targets.push_back(fishing_boat_state(t));
    targets.push_back(fishnet_buoy_state(t));
    targets.push_back(obstacle_state(t));
    targets.push_back(debris_state(t));

    for (const auto & target : targets) {
      set_target_state(target);
    }
    publish_markers(targets);
  }

  gazebo_msgs::msg::EntityState vessel_state(double t) const
  {
    const double x = 12.5 + 4.8 * std::sin(0.24 * t);
    const double y = 5.8 * std::sin(0.62 * t);
    const double vx = 4.8 * 0.24 * std::cos(0.24 * t);
    const double vy = 5.8 * 0.62 * std::cos(0.62 * t);
    const double yaw = std::abs(vx) + std::abs(vy) > 1e-3 ? std::atan2(vy, vx) : 0.0;
    const double z = 0.42 + wave_height(x, y, t, wave_amplitude_);
    return make_state(vessel_name_, x, y, z, yaw, vx, vy);
  }

  gazebo_msgs::msg::EntityState fishing_boat_state(double t) const
  {
    const double x = 10.5 + 3.4 * std::sin(0.34 * t + 0.7);
    const double y = -3.8 + 3.4 * std::cos(0.58 * t);
    const double vx = 3.4 * 0.34 * std::cos(0.34 * t + 0.7);
    const double vy = -3.4 * 0.58 * std::sin(0.58 * t);
    const double yaw = std::abs(vx) + std::abs(vy) > 1e-3 ? std::atan2(vy, vx) : 0.0;
    const double z = 0.34 + wave_height(x, y, t, wave_amplitude_);
    return make_state(fishing_boat_name_, x, y, z, yaw, vx, vy);
  }

  gazebo_msgs::msg::EntityState fishnet_buoy_state(double t) const
  {
    const double x = 7.6 + 1.8 * std::sin(0.48 * t + 2.0);
    const double y = 3.4 * std::sin(0.86 * t);
    const double vx = 1.8 * 0.48 * std::cos(0.48 * t + 2.0);
    const double vy = 3.4 * 0.86 * std::cos(0.86 * t);
    const double yaw = std::abs(vx) + std::abs(vy) > 1e-3 ? std::atan2(vy, vx) : 0.0;
    const double z = 0.30 + wave_height(x, y, t, wave_amplitude_);
    return make_state(fishnet_buoy_name_, x, y, z, yaw, vx, vy);
  }

  gazebo_msgs::msg::EntityState obstacle_state(double t) const
  {
    const double x = 6.8 + 2.0 * std::sin(0.52 * t + 1.4);
    const double y = -2.6 + 2.2 * std::cos(0.76 * t);
    const double vx = 2.0 * 0.52 * std::cos(0.52 * t + 1.4);
    const double vy = -2.2 * 0.76 * std::sin(0.76 * t);
    const double yaw = std::abs(vx) + std::abs(vy) > 1e-3 ? std::atan2(vy, vx) : 0.0;
    const double z = 0.24 + wave_height(x, y, t, wave_amplitude_);
    return make_state(obstacle_name_, x, y, z, yaw, vx, vy);
  }

  gazebo_msgs::msg::EntityState debris_state(double t) const
  {
    const double x = 5.8 + 1.6 * std::sin(0.58 * t + 2.7);
    const double y = 2.8 + 1.6 * std::cos(0.82 * t + 0.4);
    const double vx = 1.6 * 0.58 * std::cos(0.58 * t + 2.7);
    const double vy = -1.6 * 0.82 * std::sin(0.82 * t + 0.4);
    const double yaw = std::abs(vx) + std::abs(vy) > 1e-3 ? std::atan2(vy, vx) : 0.0;
    const double z = 0.16 + wave_height(x, y, t, wave_amplitude_);
    return make_state(debris_name_, x, y, z, yaw, vx, vy);
  }

  static gazebo_msgs::msg::EntityState make_state(
    const std::string & name, double x, double y, double z, double yaw, double vx, double vy)
  {
    gazebo_msgs::msg::EntityState state;
    state.name = name;
    state.reference_frame = "world";
    state.pose.position.x = x;
    state.pose.position.y = y;
    state.pose.position.z = z;
    state.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw);
    state.twist.linear.x = vx;
    state.twist.linear.y = vy;
    state.twist.angular.z = 0.0;
    return state;
  }

  void set_target_state(const gazebo_msgs::msg::EntityState & state)
  {
    auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    req->state = state;
    client_->async_send_request(req);
  }

  void publish_markers(const std::vector<gazebo_msgs::msg::EntityState> & states)
  {
    visualization_msgs::msg::MarkerArray markers;
    const builtin_interfaces::msg::Time now = get_clock()->now();
    for (std::size_t index = 0; index < states.size(); ++index) {
      const auto & state = states[index];
      const bool is_vessel = state.name == vessel_name_ || state.name == fishing_boat_name_;
      const bool is_buoy = state.name == fishnet_buoy_name_;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = now;
      marker.ns = "simulated_targets";
      marker.id = static_cast<int>(index);
      marker.type = is_buoy ? visualization_msgs::msg::Marker::CYLINDER :
        visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = state.pose;
      marker.scale.x = state.name == vessel_name_ ? 2.2 : (is_vessel ? 1.35 : 0.55);
      marker.scale.y = state.name == vessel_name_ ? 0.8 : (is_vessel ? 0.50 : 0.55);
      marker.scale.z = is_vessel ? 0.55 : 0.42;
      marker.color.r = is_vessel ? 1.0 : 0.9;
      marker.color.g = state.name == vessel_name_ ? 0.85 : (is_vessel ? 0.35 : 0.2);
      marker.color.b = is_vessel ? 0.1 : 0.05;
      marker.color.a = 0.9;
      marker.lifetime.sec = 1;
      markers.markers.push_back(marker);

      visualization_msgs::msg::Marker text;
      text.header.frame_id = "world";
      text.header.stamp = now;
      text.ns = "simulated_target_labels";
      text.id = 100 + static_cast<int>(index);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = state.pose.position.x;
      text.pose.position.y = state.pose.position.y;
      text.pose.position.z = state.pose.position.z + 1.0;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.45;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      text.text = state.name;
      text.lifetime.sec = 1;
      markers.markers.push_back(text);
    }
    marker_pub_->publish(markers);
  }

  double wave_amplitude_{0.10};
  double motion_time_scale_{3.0};
  std::string vessel_name_;
  std::string fishing_boat_name_;
  std::string fishnet_buoy_name_;
  std::string obstacle_name_;
  std::string debris_name_;
  bool warned_waiting_{false};
  std::chrono::steady_clock::time_point start_time_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::DynamicTargetController>());
  rclcpp::shutdown();
  return 0;
}
