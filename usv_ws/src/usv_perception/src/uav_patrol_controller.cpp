#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "usv_perception/common.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace usv_perception
{

class UavPatrolController : public rclcpp::Node
{
public:
  UavPatrolController()
  : Node("uav_patrol_controller"), start_time_(std::chrono::steady_clock::now())
  {
    model_name_ = declare_parameter<std::string>("model_name", "scout_uav");
    const double update_rate = declare_parameter<double>("update_rate", 8.0);
    center_x_ = declare_parameter<double>("center_x", 16.0);
    center_y_ = declare_parameter<double>("center_y", 0.0);
    altitude_ = declare_parameter<double>("altitude", 26.0);
    radius_x_ = declare_parameter<double>("radius_x", 22.0);
    radius_y_ = declare_parameter<double>("radius_y", 16.0);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.045);
    patrol_speed_ = declare_parameter<double>("patrol_speed", 0.45);
    camera_pitch_ = declare_parameter<double>("camera_pitch", 0.30);
    target_tracking_enabled_ = declare_parameter<bool>("target_tracking_enabled", true);
    target_model_name_ = declare_parameter<std::string>("target_model_name", "moving_vessel");
    usv_model_name_ = declare_parameter<std::string>("usv_model_name", "wamv");
    model_states_topic_ = declare_parameter<std::string>("model_states_topic", "/model_states");
    remote_target_topic_ = declare_parameter<std::string>("remote_target_topic", "/uav/remote_target_status");
    remote_target_confidence_ = declare_parameter<double>("remote_target_confidence", 0.94);
    remote_target_class_id_ = declare_parameter<double>("remote_target_class_id", 5.0);
    target_follow_backoff_ = declare_parameter<double>("target_follow_backoff", 18.0);
    target_lateral_sweep_ = declare_parameter<double>("target_lateral_sweep", 13.0);
    target_timeout_ = declare_parameter<double>("target_timeout", 2.0);
    external_goal_topic_ = declare_parameter<std::string>("external_goal_topic", "/c3/drone/goal");
    external_goal_timeout_ = declare_parameter<double>("external_goal_timeout", 4.0);
    external_goal_speed_ = declare_parameter<double>("external_goal_speed", 1.6);

    client_ = create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("uav/status_marker", 10);
    remote_target_pub_ = create_publisher<std_msgs::msg::String>(remote_target_topic_, 10);
    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic_, 10, std::bind(&UavPatrolController::on_model_states, this, std::placeholders::_1));
    external_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      external_goal_topic_, 10, std::bind(&UavPatrolController::on_external_goal, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(update_rate, 0.1)));
    timer_ = create_wall_timer(period, std::bind(&UavPatrolController::on_timer, this));
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
    const double t = elapsed_seconds();
    const auto state = make_state(t);
    publish_tf(state);
    publish_marker(state);
    publish_remote_target();

    if (!client_->service_is_ready()) {
      if (!warned_waiting_) {
        RCLCPP_INFO(get_logger(), "Waiting for /set_entity_state to move scout UAV");
        warned_waiting_ = true;
      }
      return;
    }

    auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    req->state = state;
    client_->async_send_request(req);
  }

  gazebo_msgs::msg::EntityState make_state(double t) const
  {
    if (has_recent_external_goal()) {
      return make_external_goal_state();
    }
    if (target_tracking_enabled_ && has_recent_target()) {
      return make_target_scout_state(t);
    }
    return make_patrol_state(t);
  }

  gazebo_msgs::msg::EntityState make_external_goal_state() const
  {
    double current_x = has_uav_state_ ? uav_pose_.position.x : external_goal_.pose.position.x;
    double current_y = has_uav_state_ ? uav_pose_.position.y : external_goal_.pose.position.y;
    double current_z = has_uav_state_ ? uav_pose_.position.z : altitude_;
    const double goal_x = external_goal_.pose.position.x;
    const double goal_y = external_goal_.pose.position.y;
    const double goal_z = external_goal_.pose.position.z > 1.0 ? external_goal_.pose.position.z : altitude_;
    const double dx = goal_x - current_x;
    const double dy = goal_y - current_y;
    const double dz = goal_z - current_z;
    const double distance = std::hypot(dx, dy);
    const double step = std::min(distance, external_goal_speed_ / 8.0);
    const double ratio = distance > 1e-3 ? step / distance : 0.0;

    gazebo_msgs::msg::EntityState state;
    state.name = model_name_;
    state.reference_frame = "world";
    state.pose.position.x = current_x + dx * ratio;
    state.pose.position.y = current_y + dy * ratio;
    state.pose.position.z = current_z + std::clamp(dz, -0.25, 0.25);
    const double yaw = distance > 1e-3 ? std::atan2(dy, dx) : yaw_from_quaternion(uav_pose_.orientation);
    state.pose.orientation = quaternion_from_euler(0.0, camera_pitch_, yaw);
    state.twist.linear.x = external_goal_speed_ * std::cos(yaw);
    state.twist.linear.y = external_goal_speed_ * std::sin(yaw);
    state.twist.linear.z = 0.0;
    return state;
  }

  gazebo_msgs::msg::EntityState make_patrol_state(double t) const
  {
    const std::vector<Waypoint> path{
      {center_x_ + 78.0, center_y_ - 52.0},
      {center_x_ + 46.0, center_y_ - 30.0},
      {center_x_ + 18.0, center_y_ - 2.0},
      {center_x_ + 42.0, center_y_ + 38.0},
      {center_x_ + 86.0, center_y_ + 56.0},
      {center_x_ + 124.0, center_y_ + 18.0},
      {center_x_ + 96.0, center_y_ - 24.0}
    };
    const auto sample = sample_path(path, patrol_speed_, t);
    const double x = sample.x;
    const double y = sample.y;
    const double z = altitude_ + 0.9 * std::sin(0.035 * t);
    const double vx = sample.vx;
    const double vy = sample.vy;
    const double yaw = sample.yaw;

    gazebo_msgs::msg::EntityState state;
    state.name = model_name_;
    state.reference_frame = "world";
    state.pose.position.x = x;
    state.pose.position.y = y;
    state.pose.position.z = z;
    state.pose.orientation = quaternion_from_euler(0.0, camera_pitch_, yaw);
    state.twist.linear.x = vx;
    state.twist.linear.y = vy;
    state.twist.linear.z = 0.0;
    return state;
  }

  gazebo_msgs::msg::EntityState make_target_scout_state(double t) const
  {
    const double target_x = target_pose_.position.x;
    const double target_y = target_pose_.position.y;
    const double usv_x = has_usv_state_ ? usv_pose_.position.x : 0.0;
    const double usv_y = has_usv_state_ ? usv_pose_.position.y : 0.0;
    double dir_x = target_x - usv_x;
    double dir_y = target_y - usv_y;
    double length = std::hypot(dir_x, dir_y);
    if (length < 1.0) {
      dir_x = std::cos(yaw_from_quaternion(target_pose_.orientation));
      dir_y = std::sin(yaw_from_quaternion(target_pose_.orientation));
      length = 1.0;
    }
    dir_x /= length;
    dir_y /= length;
    const double side_x = -dir_y;
    const double side_y = dir_x;
    const double sweep = target_lateral_sweep_ * std::sin(0.22 * t);

    const double x = target_x - dir_x * target_follow_backoff_ + side_x * sweep;
    const double y = target_y - dir_y * target_follow_backoff_ + side_y * sweep;
    const double z = altitude_ + 1.2 * std::sin(0.05 * t);
    const double yaw = std::atan2(target_y - y, target_x - x);

    gazebo_msgs::msg::EntityState state;
    state.name = model_name_;
    state.reference_frame = "world";
    state.pose.position.x = x;
    state.pose.position.y = y;
    state.pose.position.z = z;
    state.pose.orientation = quaternion_from_euler(0.0, camera_pitch_, yaw);
    state.twist.linear.x = target_twist_.linear.x;
    state.twist.linear.y = target_twist_.linear.y;
    state.twist.linear.z = 0.0;
    return state;
  }

  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    const auto uav_index = find_model(*msg, model_name_);
    if (uav_index >= 0) {
      uav_pose_ = msg->pose[static_cast<std::size_t>(uav_index)];
      has_uav_state_ = true;
    }

    const auto target_index = find_model(*msg, target_model_name_);
    if (target_index >= 0) {
      target_pose_ = msg->pose[static_cast<std::size_t>(target_index)];
      target_twist_ = msg->twist[static_cast<std::size_t>(target_index)];
      has_target_state_ = true;
      last_target_time_ = std::chrono::steady_clock::now();
    }

    const auto usv_index = find_model(*msg, usv_model_name_);
    if (usv_index >= 0) {
      usv_pose_ = msg->pose[static_cast<std::size_t>(usv_index)];
      usv_twist_ = msg->twist[static_cast<std::size_t>(usv_index)];
      has_usv_state_ = true;
    }
  }

  void on_external_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    external_goal_ = *msg;
    has_external_goal_ = true;
    last_external_goal_time_ = std::chrono::steady_clock::now();
  }

  static int find_model(const gazebo_msgs::msg::ModelStates & msg, const std::string & name)
  {
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      if (msg.name[i] == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  bool has_recent_target() const
  {
    return has_target_state_ &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() - last_target_time_).count() <=
           target_timeout_;
  }

  bool has_recent_external_goal() const
  {
    return has_external_goal_ &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() - last_external_goal_time_).count() <=
           external_goal_timeout_;
  }

  struct PathSample
  {
    double x;
    double y;
    double yaw;
    double vx;
    double vy;
  };

  static PathSample sample_path(const std::vector<Waypoint> & path, double speed, double t)
  {
    if (path.size() < 2) {
      return {0.0, 0.0, 0.0, 0.0, 0.0};
    }

    std::vector<double> segment_lengths;
    segment_lengths.reserve(path.size() - 1);
    double total_length = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index) {
      const double length = std::hypot(path[index].x - path[index - 1].x, path[index].y - path[index - 1].y);
      segment_lengths.push_back(length);
      total_length += length;
    }

    const double cycle_length = std::max(0.1, 2.0 * total_length);
    double distance = std::fmod(std::max(0.0, t) * speed, cycle_length);
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

    const double direction_sign = reverse ? -1.0 : 1.0;
    const double yaw = std::atan2(direction_sign * (b.y - a.y), direction_sign * (b.x - a.x));
    return {
      a.x + (b.x - a.x) * ratio,
      a.y + (b.y - a.y) * ratio,
      yaw,
      speed * std::cos(yaw),
      speed * std::sin(yaw)
    };
  }

  void publish_tf(const gazebo_msgs::msg::EntityState & state)
  {
    const builtin_interfaces::msg::Time now = get_clock()->now();

    geometry_msgs::msg::TransformStamped uav_tf;
    uav_tf.header.stamp = now;
    uav_tf.header.frame_id = "world";
    uav_tf.child_frame_id = "scout_uav/base_link";
    uav_tf.transform.translation.x = state.pose.position.x;
    uav_tf.transform.translation.y = state.pose.position.y;
    uav_tf.transform.translation.z = state.pose.position.z;
    uav_tf.transform.rotation = state.pose.orientation;

    geometry_msgs::msg::TransformStamped camera_tf;
    camera_tf.header.stamp = now;
    camera_tf.header.frame_id = "scout_uav/base_link";
    camera_tf.child_frame_id = "uav_gated_camera_link";
    camera_tf.transform.translation.x = 0.45;
    camera_tf.transform.translation.y = 0.0;
    camera_tf.transform.translation.z = -0.10;
    camera_tf.transform.rotation = quaternion_from_euler(0.0, 0.22, 0.0);

    std::vector<geometry_msgs::msg::TransformStamped> transforms{uav_tf, camera_tf};
    tf_broadcaster_->sendTransform(transforms);
  }

  void publish_marker(const gazebo_msgs::msg::EntityState & state)
  {
    visualization_msgs::msg::MarkerArray markers;
    const builtin_interfaces::msg::Time now = get_clock()->now();

    visualization_msgs::msg::Marker uav;
    uav.header.frame_id = "world";
    uav.header.stamp = now;
    uav.ns = "scout_uav";
    uav.id = 1;
    uav.type = visualization_msgs::msg::Marker::CUBE;
    uav.action = visualization_msgs::msg::Marker::ADD;
    uav.pose = state.pose;
    uav.scale.x = 1.2;
    uav.scale.y = 0.45;
    uav.scale.z = 0.18;
    uav.color.r = 0.05;
    uav.color.g = 0.28;
    uav.color.b = 0.90;
    uav.color.a = 0.92;
    uav.lifetime.sec = 1;
    markers.markers.push_back(uav);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = "world";
    text.header.stamp = now;
    text.ns = "scout_uav";
    text.id = 2;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = state.pose.position.x;
    text.pose.position.y = state.pose.position.y;
    text.pose.position.z = state.pose.position.z + 1.6;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.8;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    text.text = "Scout UAV";
    text.lifetime.sec = 1;
    markers.markers.push_back(text);

    if (has_recent_target()) {
      visualization_msgs::msg::Marker target;
      target.header.frame_id = "world";
      target.header.stamp = now;
      target.ns = "uav_remote_target";
      target.id = 3;
      target.type = visualization_msgs::msg::Marker::SPHERE;
      target.action = visualization_msgs::msg::Marker::ADD;
      target.pose.position.x = target_pose_.position.x;
      target.pose.position.y = target_pose_.position.y;
      target.pose.position.z = target_pose_.position.z + 2.2;
      target.pose.orientation.w = 1.0;
      target.scale.x = 1.8;
      target.scale.y = 1.8;
      target.scale.z = 1.8;
      target.color.r = 0.15;
      target.color.g = 0.95;
      target.color.b = 1.0;
      target.color.a = 0.88;
      target.lifetime.sec = 1;
      markers.markers.push_back(target);

      visualization_msgs::msg::Marker label;
      label.header = target.header;
      label.ns = "uav_remote_target";
      label.id = 4;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = target_pose_.position.x;
      label.pose.position.y = target_pose_.position.y;
      label.pose.position.z = target_pose_.position.z + 4.0;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.85;
      label.color.r = 0.75;
      label.color.g = 1.0;
      label.color.b = 1.0;
      label.color.a = 1.0;
      label.text = "UAV SCOUT: " + target_model_name_;
      label.lifetime.sec = 1;
      markers.markers.push_back(label);
    }

    if (has_recent_external_goal()) {
      visualization_msgs::msg::Marker goal;
      goal.header.frame_id = "world";
      goal.header.stamp = now;
      goal.ns = "c3_uav_goal";
      goal.id = 5;
      goal.type = visualization_msgs::msg::Marker::SPHERE;
      goal.action = visualization_msgs::msg::Marker::ADD;
      goal.pose = external_goal_.pose;
      goal.pose.orientation.w = 1.0;
      goal.scale.x = 2.2;
      goal.scale.y = 2.2;
      goal.scale.z = 2.2;
      goal.color.r = 1.0;
      goal.color.g = 0.35;
      goal.color.b = 0.05;
      goal.color.a = 0.80;
      goal.lifetime.sec = 1;
      markers.markers.push_back(goal);
    }

    marker_pub_->publish(markers);
  }

  void publish_remote_target()
  {
    if (!has_recent_target() || !has_usv_state_) {
      return;
    }

    const double dx = target_pose_.position.x - usv_pose_.position.x;
    const double dy = target_pose_.position.y - usv_pose_.position.y;
    const double yaw = yaw_from_quaternion(usv_pose_.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double rel_x = cos_yaw * dx + sin_yaw * dy;
    const double rel_y = -sin_yaw * dx + cos_yaw * dy;
    const double dvx = target_twist_.linear.x - usv_twist_.linear.x;
    const double dvy = target_twist_.linear.y - usv_twist_.linear.y;
    const double rel_vx = cos_yaw * dvx + sin_yaw * dvy;
    const double rel_vy = -sin_yaw * dvx + cos_yaw * dvy;
    const double speed = std::hypot(rel_vx, rel_vy);

    std::ostringstream status;
    status.setf(std::ios::fixed, std::ios::floatfield);
    status << std::setprecision(2)
           << "[{\"id\":9001"
           << ",\"x\":" << rel_x
           << ",\"y\":" << rel_y
           << ",\"vx\":" << rel_vx
           << ",\"vy\":" << rel_vy
           << ",\"speed\":" << speed
           << ",\"confidence\":" << remote_target_confidence_
           << ",\"class_id\":" << remote_target_class_id_
           << ",\"last_source\":\"uav_remote_scout\""
           << ",\"hits\":20"
           << ",\"sources\":[\"uav_remote_scout\"]}]";

    std_msgs::msg::String msg;
    msg.data = status.str();
    remote_target_pub_->publish(msg);
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  std::string model_name_{"scout_uav"};
  std::string target_model_name_{"moving_vessel"};
  std::string usv_model_name_{"wamv"};
  std::string model_states_topic_{"/model_states"};
  std::string remote_target_topic_{"/uav/remote_target_status"};
  std::string external_goal_topic_{"/c3/drone/goal"};
  double center_x_{16.0};
  double center_y_{0.0};
  double altitude_{26.0};
  double radius_x_{22.0};
  double radius_y_{16.0};
  double angular_speed_{0.045};
  double patrol_speed_{0.45};
  double camera_pitch_{0.30};
  double remote_target_confidence_{0.94};
  double remote_target_class_id_{5.0};
  double target_follow_backoff_{18.0};
  double target_lateral_sweep_{13.0};
  double target_timeout_{2.0};
  double external_goal_timeout_{4.0};
  double external_goal_speed_{1.6};
  bool warned_waiting_{false};
  bool target_tracking_enabled_{true};
  bool has_target_state_{false};
  bool has_usv_state_{false};
  bool has_uav_state_{false};
  bool has_external_goal_{false};
  geometry_msgs::msg::Pose target_pose_;
  geometry_msgs::msg::Twist target_twist_;
  geometry_msgs::msg::Pose usv_pose_;
  geometry_msgs::msg::Twist usv_twist_;
  geometry_msgs::msg::Pose uav_pose_;
  geometry_msgs::msg::PoseStamped external_goal_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_target_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_external_goal_time_{std::chrono::steady_clock::now()};
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr external_goal_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr remote_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::UavPatrolController>());
  rclcpp::shutdown();
  return 0;
}
