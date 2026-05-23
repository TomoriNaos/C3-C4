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
    survey_boat_name_ = declare_parameter<std::string>("survey_boat_name", "survey_boat");
    service_boat_name_ = declare_parameter<std::string>("service_boat_name", "service_boat");
    tracked_target_name_ = declare_parameter<std::string>("tracked_target_name", vessel_name_);

    client_ = create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("simulated_targets", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(update_rate, 0.1)));
    timer_ = create_wall_timer(period, std::bind(&DynamicTargetController::on_timer, this));
  }

private:
  struct Waypoint
  {
    double x;
    double y;
  };

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
    targets.reserve(7);
    targets.push_back(path_state(
      vessel_name_,
      {{18.0, -18.0}, {42.0, -8.0}, {66.0, 11.0}, {82.0, 28.0}, {58.0, 47.0},
        {30.0, 31.0}, {15.0, 8.0}},
      0.42, 0.30, t, 0.0));
    targets.push_back(path_state(
      fishing_boat_name_,
      {{-18.0, 34.0}, {8.0, 42.0}, {36.0, 37.0}, {62.0, 21.0}, {78.0, -4.0}},
      0.34, 0.17, t, 35.0));
    targets.push_back(path_state(
      fishnet_buoy_name_,
      {{24.0, -32.0}, {28.0, -18.0}, {34.0, -4.0}, {42.0, 10.0}, {48.0, 26.0}},
      0.30, 0.045, t, 90.0));
    targets.push_back(path_state(
      obstacle_name_,
      {{52.0, -46.0}, {57.0, -28.0}, {62.0, -10.0}, {70.0, 8.0}, {76.0, 24.0}},
      0.24, 0.060, t, 150.0));
    targets.push_back(path_state(
      debris_name_,
      {{8.0, 48.0}, {20.0, 52.0}, {37.0, 48.0}, {53.0, 42.0}, {71.0, 34.0}},
      0.16, 0.035, t, 210.0));
    targets.push_back(path_state(
      survey_boat_name_,
      {{-35.0, -24.0}, {-10.0, -28.0}, {18.0, -25.0}, {45.0, -18.0}, {72.0, -6.0}},
      0.36, 0.20, t, 20.0));
    targets.push_back(path_state(
      service_boat_name_,
      {{95.0, 36.0}, {74.0, 22.0}, {53.0, 12.0}, {31.0, 5.0}, {12.0, -2.0}},
      0.38, 0.15, t, 75.0));

    for (const auto & target : targets) {
      set_target_state(target);
    }
    publish_markers(targets);
  }

  gazebo_msgs::msg::EntityState path_state(
    const std::string & name,
    const std::vector<Waypoint> & path,
    double base_z,
    double speed,
    double t,
    double phase_seconds) const
  {
    if (path.size() < 2) {
      return make_state(name, 0.0, 0.0, base_z, 0.0, 0.0, 0.0);
    }

    double total_length = 0.0;
    std::vector<double> segment_lengths;
    segment_lengths.reserve(path.size() - 1);
    for (std::size_t index = 1; index < path.size(); ++index) {
      const double length = std::hypot(path[index].x - path[index - 1].x, path[index].y - path[index - 1].y);
      segment_lengths.push_back(length);
      total_length += length;
    }

    const double cycle_length = std::max(0.1, 2.0 * total_length);
    double distance = std::fmod(std::max(0.0, t + phase_seconds) * speed, cycle_length);
    bool reverse = false;
    if (distance > total_length) {
      distance = cycle_length - distance;
      reverse = true;
    }

    std::size_t segment_index = 0;
    while (segment_index < segment_lengths.size() && distance > segment_lengths[segment_index]) {
      distance -= segment_lengths[segment_index];
      ++segment_index;
    }
    segment_index = std::min(segment_index, segment_lengths.size() - 1);

    const auto & a = path[segment_index];
    const auto & b = path[segment_index + 1];
    const double length = std::max(0.1, segment_lengths[segment_index]);
    double ratio = std::clamp(distance / length, 0.0, 1.0);
    if (reverse) {
      ratio = 1.0 - ratio;
    }

    const double x = a.x + (b.x - a.x) * ratio;
    const double y = a.y + (b.y - a.y) * ratio;
    const double direction_sign = reverse ? -1.0 : 1.0;
    const double heading_x = direction_sign * (b.x - a.x);
    const double heading_y = direction_sign * (b.y - a.y);
    const double yaw = std::atan2(heading_y, heading_x);
    const double vx = speed * std::cos(yaw);
    const double vy = speed * std::sin(yaw);
    const double z = base_z + wave_height(x, y, t, wave_amplitude_);
    return make_state(name, x, y, z, yaw, vx, vy);
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
      const bool is_primary_target = state.name == tracked_target_name_;
      const bool is_vessel =
        state.name == vessel_name_ || state.name == fishing_boat_name_ ||
        state.name == survey_boat_name_ || state.name == service_boat_name_;
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
      marker.color.r = is_primary_target ? 0.05 : (is_vessel ? 1.0 : 0.9);
      marker.color.g = is_primary_target ? 1.0 : (state.name == vessel_name_ ? 0.85 : (is_vessel ? 0.35 : 0.2));
      marker.color.b = is_primary_target ? 0.25 : (is_vessel ? 0.1 : 0.05);
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
      text.text = is_primary_target ? "TRACK TARGET: " + state.name : state.name;
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
  std::string survey_boat_name_;
  std::string service_boat_name_;
  std::string tracked_target_name_;
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
