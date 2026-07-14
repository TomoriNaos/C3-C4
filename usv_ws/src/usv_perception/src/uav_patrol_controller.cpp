#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
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
    update_dt_s_ = 1.0 / std::max(update_rate, 0.1);
    center_x_ = declare_parameter<double>("center_x", 16.0);
    center_y_ = declare_parameter<double>("center_y", 0.0);
    altitude_ = declare_parameter<double>("altitude", 26.0);
    radius_x_ = declare_parameter<double>("radius_x", 22.0);
    radius_y_ = declare_parameter<double>("radius_y", 16.0);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.045);
    patrol_speed_ = declare_parameter<double>("patrol_speed", 0.45);
    camera_pitch_ = declare_parameter<double>("camera_pitch", 0.30);
    usv_model_name_ = declare_parameter<std::string>("usv_model_name", "wamv");
    model_states_topic_ = declare_parameter<std::string>("model_states_topic", "/model_states");
    external_goal_topic_ = declare_parameter<std::string>("external_goal_topic", "/c3/drone/goal");
    external_goal_timeout_ = declare_parameter<double>("external_goal_timeout", 4.0);
    external_goal_speed_ = declare_parameter<double>("external_goal_speed", 1.6);
    goal_arrive_radius_ = declare_parameter<double>("goal_arrive_radius", 2.0);
    control_backend_ = declare_parameter<std::string>("control_backend", "px4");
    px4_goal_topic_ = declare_parameter<std::string>("px4_goal_topic", "/uav/offboard_goal");
    px4_pose_topic_ = declare_parameter<std::string>("px4_pose_topic", "/px4/vehicle_pose");
    gazebo_sim_goal_topic_ = declare_parameter<std::string>("gazebo_sim_goal_topic", "/uav/sim/command");
    gazebo_sim_pose_topic_ = declare_parameter<std::string>("gazebo_sim_pose_topic", "/uav/sim/estimated_pose");
    flight_status_topic_ = declare_parameter<std::string>("flight_status_topic", "/uav/flight_status");

    client_ = create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("uav/status_marker", 10);
    px4_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(px4_goal_topic_, 10);
    gazebo_sim_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(gazebo_sim_goal_topic_, 10);
    flight_status_pub_ = create_publisher<std_msgs::msg::String>(flight_status_topic_, 10);
    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic_, 10, std::bind(&UavPatrolController::on_model_states, this, std::placeholders::_1));
    external_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      external_goal_topic_, 10, std::bind(&UavPatrolController::on_external_goal, this, std::placeholders::_1));
    px4_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      px4_pose_topic_, 10, std::bind(&UavPatrolController::on_px4_pose, this, std::placeholders::_1));
    gazebo_sim_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      gazebo_sim_pose_topic_, 10, std::bind(&UavPatrolController::on_gazebo_sim_pose, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(update_rate, 0.1)));
    timer_ = create_wall_timer(period, std::bind(&UavPatrolController::on_timer, this));
  }

private:
  enum class MissionMode : uint8_t
  {
    PATROL = 0,
    TRANSIT = 1,
    HOLD = 2,
    BACK = 3
  };

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
    update_mission_mode();
    const auto state = make_state(t);
    auto display_state = state;
    if (control_backend_ == "gazebo_simulated" && has_uav_truth_state_) {
      display_state.pose = uav_truth_pose_;
    }
    publish_tf(display_state);
    publish_marker(display_state);
    publish_flight_status(state);

    if (control_backend_ == "px4") {
      publish_px4_goal(state);
      return;
    }

    if (control_backend_ == "gazebo_simulated") {
      publish_gazebo_sim_goal(state);
      return;
    }

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
    if (mission_mode_ == MissionMode::TRANSIT || mission_mode_ == MissionMode::BACK) {
      return make_external_goal_state();
    }
    return make_patrol_state(t);
  }

  gazebo_msgs::msg::EntityState make_external_goal_state() const
  {
    const auto goal = resolve_goal_world();
    double current_x = has_uav_state_ ? uav_pose_.position.x : goal[0];
    double current_y = has_uav_state_ ? uav_pose_.position.y : goal[1];
    double current_z = has_uav_state_ ? uav_pose_.position.z : altitude_;
    const double goal_x = goal[0];
    const double goal_y = goal[1];
    const double goal_z = goal[2];
    const double dx = goal_x - current_x;
    const double dy = goal_y - current_y;
    const double dz = goal_z - current_z;
    const double distance = std::hypot(dx, dy);
    const double step = std::min(distance, external_goal_speed_ * update_dt_s_);
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
    const double phase = angular_speed_ * t;
    const double local_x = center_x_ + radius_x_ * std::cos(phase);
    const double local_y = center_y_ + radius_y_ * std::sin(phase);
    const double local_vx = -radius_x_ * angular_speed_ * std::sin(phase);
    const double local_vy = radius_y_ * angular_speed_ * std::cos(phase);
    const auto world = ship_relative_to_world(local_x, local_y, altitude_);
    const auto tangent_world = ship_relative_vector_to_world(local_vx, local_vy);
    const double x = world[0];
    const double y = world[1];
    const double z = altitude_ + 0.9 * std::sin(0.035 * t);
    const double vx = tangent_world[0];
    const double vy = tangent_world[1];
    const double yaw = std::atan2(vy, vx);

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

  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    const auto uav_index = find_model(*msg, model_name_);
    if (uav_index >= 0) {
      uav_truth_pose_ = msg->pose[static_cast<std::size_t>(uav_index)];
      has_uav_truth_state_ = true;
      if (control_backend_ != "gazebo_simulated" || !has_uav_state_) {
        uav_pose_ = uav_truth_pose_;
        has_uav_state_ = true;
      }
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

  void on_px4_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    uav_pose_ = msg->pose;
    has_uav_state_ = true;
  }

  void on_gazebo_sim_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg || control_backend_ != "gazebo_simulated") {
      return;
    }
    uav_pose_ = msg->pose;
    has_uav_state_ = true;
  }

  void publish_px4_goal(const gazebo_msgs::msg::EntityState & state)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = get_clock()->now();
    // The bridge owns the ROS ENU to PX4 NED conversion.
    goal.header.frame_id = "world";
    goal.pose = state.pose;
    px4_goal_pub_->publish(goal);
  }

  void publish_gazebo_sim_goal(const gazebo_msgs::msg::EntityState & state)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = get_clock()->now();
    goal.header.frame_id = "world";
    goal.pose = state.pose;
    gazebo_sim_goal_pub_->publish(goal);
  }

  void publish_flight_status(const gazebo_msgs::msg::EntityState & command) const
  {
    const char * mode = "巡航";
    if (mission_mode_ == MissionMode::TRANSIT) {
      mode = "前往粗定位点";
    } else if (mission_mode_ == MissionMode::HOLD) {
      mode = "目标区域盘旋";
    }
    std::ostringstream out;
    out.setf(std::ios::fixed, std::ios::floatfield);
    out << std::setprecision(2)
        << "{\"模式\":\"" << mode << "\""
        << ",\"任务有效\":" << (has_recent_external_goal() ? "true" : "false")
        << ",\"指令_x\":" << command.pose.position.x
        << ",\"指令_y\":" << command.pose.position.y
        << ",\"指令高度\":" << command.pose.position.z;
    if (has_recent_external_goal()) {
      out << ",\"粗定位_x\":" << external_goal_.pose.position.x
          << ",\"粗定位_y\":" << external_goal_.pose.position.y;
    }
    out << "}";
    std_msgs::msg::String msg;
    msg.data = out.str();
    flight_status_pub_->publish(msg);
  }

  void update_mission_mode()
  {
    if (has_recent_external_goal()) {
      mission_mode_ = MissionMode::TRANSIT;
      if (has_uav_state_) {
        const auto goal = resolve_goal_world();
        const double distance = std::hypot(goal[0] - uav_pose_.position.x, goal[1] - uav_pose_.position.y);
        if (distance <= goal_arrive_radius_) {
          mission_mode_ = MissionMode::HOLD;
        }
      }
      return;
    }
    mission_mode_ = MissionMode::PATROL;
  }

  std::array<double, 3> resolve_goal_world() const
  {
    if (external_goal_.header.frame_id == "ship" || external_goal_.header.frame_id == "base_link") {
      return ship_relative_to_world(
        external_goal_.pose.position.x,
        external_goal_.pose.position.y,
        external_goal_.pose.position.z > 1.0 ? external_goal_.pose.position.z : altitude_);
    }
    return {
      external_goal_.pose.position.x,
      external_goal_.pose.position.y,
      external_goal_.pose.position.z > 1.0 ? external_goal_.pose.position.z : altitude_};
  }

  std::array<double, 3> ship_relative_to_world(double x, double y, double z) const
  {
    if (!has_usv_state_) {
      return {x, y, z};
    }
    const double yaw = yaw_from_quaternion(usv_pose_.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {
      usv_pose_.position.x + c * x - s * y,
      usv_pose_.position.y + s * x + c * y,
      z > 1.0 ? z : altitude_};
  }

  std::array<double, 2> ship_relative_vector_to_world(double x, double y) const
  {
    if (!has_usv_state_) {
      return {x, y};
    }
    const double yaw = yaw_from_quaternion(usv_pose_.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {c * x - s * y, s * x + c * y};
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

    if (has_recent_external_goal()) {
      visualization_msgs::msg::Marker goal;
      goal.header.frame_id = "world";
      goal.header.stamp = now;
      goal.ns = "c3_uav_goal";
      goal.id = 5;
      goal.type = visualization_msgs::msg::Marker::CUBE;
      goal.action = visualization_msgs::msg::Marker::ADD;
      goal.pose = external_goal_.pose;
      goal.pose.orientation.w = 1.0;
      goal.scale.x = 1.0;
      goal.scale.y = 1.0;
      goal.scale.z = 0.25;
      goal.color.r = 1.0;
      goal.color.g = 0.35;
      goal.color.b = 0.05;
      goal.color.a = 0.80;
      goal.lifetime.sec = 1;
      markers.markers.push_back(goal);
    }

    marker_pub_->publish(markers);
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  std::string model_name_{"scout_uav"};
  std::string usv_model_name_{"wamv"};
  std::string model_states_topic_{"/model_states"};
  std::string external_goal_topic_{"/c3/drone/goal"};
  std::string control_backend_{"px4"};
  std::string px4_goal_topic_{"/uav/offboard_goal"};
  std::string px4_pose_topic_{"/px4/vehicle_pose"};
  std::string gazebo_sim_goal_topic_{"/uav/sim/command"};
  std::string gazebo_sim_pose_topic_{"/uav/sim/estimated_pose"};
  std::string flight_status_topic_{"/uav/flight_status"};
  double center_x_{16.0};
  double center_y_{0.0};
  double altitude_{26.0};
  double radius_x_{22.0};
  double radius_y_{16.0};
  double angular_speed_{0.045};
  double patrol_speed_{0.45};
  double camera_pitch_{0.30};
  double external_goal_timeout_{4.0};
  double external_goal_speed_{1.6};
  double goal_arrive_radius_{2.0};
  double update_dt_s_{0.125};
  bool warned_waiting_{false};
  bool has_usv_state_{false};
  bool has_uav_state_{false};
  bool has_uav_truth_state_{false};
  bool has_external_goal_{false};
  MissionMode mission_mode_{MissionMode::PATROL};
  geometry_msgs::msg::Pose usv_pose_;
  geometry_msgs::msg::Twist usv_twist_;
  geometry_msgs::msg::Pose uav_pose_;
  geometry_msgs::msg::Pose uav_truth_pose_;
  geometry_msgs::msg::PoseStamped external_goal_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_external_goal_time_{std::chrono::steady_clock::now()};
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr external_goal_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr px4_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gazebo_sim_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flight_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr px4_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gazebo_sim_pose_sub_;
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
