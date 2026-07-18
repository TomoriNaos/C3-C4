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
#include "std_msgs/msg/bool.hpp"
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
    altitude_ = declare_parameter<double>("altitude", 5.0);
    radius_x_ = declare_parameter<double>("radius_x", 22.0);
    radius_y_ = declare_parameter<double>("radius_y", 16.0);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.045);
    patrol_speed_ = declare_parameter<double>("patrol_speed", 0.45);
    camera_pitch_ = declare_parameter<double>("camera_pitch", 0.30);
    patrol_camera_pitch_ = declare_parameter<double>("patrol_camera_pitch", camera_pitch_);
    camera_mount_pitch_ = declare_parameter<double>("camera_mount_pitch", 0.55);
    camera_pitch_min_ = declare_parameter<double>("camera_pitch_min", -0.20);
    camera_pitch_max_ = declare_parameter<double>("camera_pitch_max", 0.45);
    usv_model_name_ = declare_parameter<std::string>("usv_model_name", "wamv");
    model_states_topic_ = declare_parameter<std::string>("model_states_topic", "/model_states");
    external_goal_topic_ = declare_parameter<std::string>("external_goal_topic", "/c3/drone/goal");
    assist_request_topic_ = declare_parameter<std::string>("assist_request_topic", "/uav/assist_request");
    c3_detected_topic_ = declare_parameter<std::string>("c3_detected_topic", "/c3/detected_objects");
    track_status_topic_ = declare_parameter<std::string>("track_status_topic", "/tracked_objects_text");
    external_goal_timeout_ = declare_parameter<double>("external_goal_timeout", 4.0);
    external_goal_speed_ = declare_parameter<double>("external_goal_speed", 1.6);
    goal_arrive_radius_ = declare_parameter<double>("goal_arrive_radius", 2.0);
    observation_standoff_m_ = declare_parameter<double>("observation_standoff_m", 4.0);
    observation_altitude_ = declare_parameter<double>("observation_altitude", 5.0);
    observation_arrive_radius_ = declare_parameter<double>("observation_arrive_radius", 1.2);
    observation_active_topic_ =
      declare_parameter<std::string>("observation_active_topic", "/uav/observation/active");
    minimum_operating_range_m_ = declare_parameter<double>("minimum_operating_range_m", 30.0);
    return_requested_topic_ =
      declare_parameter<std::string>("return_requested_topic", "/uav/return_requested");
    return_altitude_ = declare_parameter<double>("return_altitude", 5.0);
    control_backend_ = declare_parameter<std::string>("control_backend", "px4");
    px4_goal_topic_ = declare_parameter<std::string>("px4_goal_topic", "/uav/offboard_goal");
    px4_pose_topic_ = declare_parameter<std::string>("px4_pose_topic", "/px4/vehicle_pose");
    gazebo_sim_goal_topic_ = declare_parameter<std::string>("gazebo_sim_goal_topic", "/uav/sim/command");
    gazebo_sim_pose_topic_ = declare_parameter<std::string>("gazebo_sim_pose_topic", "/uav/sim/estimated_pose");
    flight_status_topic_ = declare_parameter<std::string>("flight_status_topic", "/uav/flight_status");
    assist_goal_timeout_ = declare_parameter<double>("assist_goal_timeout", 3.0);
    uav_track_timeout_ = declare_parameter<double>("uav_track_timeout", 3.0);
    tracker_target_max_misses_ = declare_parameter<int>("tracker_target_max_misses", 80);

    client_ = create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("uav/status_marker", 10);
    px4_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(px4_goal_topic_, 10);
    gazebo_sim_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(gazebo_sim_goal_topic_, 10);
    flight_status_pub_ = create_publisher<std_msgs::msg::String>(flight_status_topic_, 10);
    observation_active_pub_ = create_publisher<std_msgs::msg::Bool>(observation_active_topic_, 10);
    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic_, 10, std::bind(&UavPatrolController::on_model_states, this, std::placeholders::_1));
    external_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      external_goal_topic_, 10, std::bind(&UavPatrolController::on_external_goal, this, std::placeholders::_1));
    assist_request_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      assist_request_topic_, 10, std::bind(&UavPatrolController::on_assist_request, this, std::placeholders::_1));
    c3_detected_sub_ = create_subscription<std_msgs::msg::String>(
      c3_detected_topic_, 10, std::bind(&UavPatrolController::on_c3_detected, this, std::placeholders::_1));
    track_status_sub_ = create_subscription<std_msgs::msg::String>(
      track_status_topic_, 10, std::bind(&UavPatrolController::on_track_status, this, std::placeholders::_1));
    return_requested_sub_ = create_subscription<std_msgs::msg::Bool>(
      return_requested_topic_, 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        return_requested_ = msg && msg->data;
      });
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
    OBSERVE = 2,
    ASSIST = 3,
    TRACK = 4,
    RETURN = 5
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
    std_msgs::msg::Bool observation_active;
    observation_active.data = mission_mode_ == MissionMode::OBSERVE ||
      mission_mode_ == MissionMode::ASSIST || mission_mode_ == MissionMode::TRACK;
    observation_active_pub_->publish(observation_active);

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
    if (mission_mode_ == MissionMode::RETURN) {
      return make_return_state();
    }
    if (mission_mode_ == MissionMode::TRANSIT || mission_mode_ == MissionMode::OBSERVE ||
      mission_mode_ == MissionMode::ASSIST || mission_mode_ == MissionMode::TRACK)
    {
      return make_observation_state();
    }
    return make_patrol_state(t);
  }

  gazebo_msgs::msg::EntityState make_observation_state() const
  {
    const auto target = resolve_goal_world();
    const auto observation = observation_position_world(target);
    const double yaw = std::atan2(target[1] - observation[1], target[0] - observation[0]);

    gazebo_msgs::msg::EntityState state;
    state.name = model_name_;
    state.reference_frame = "world";
    // The flight-dynamics layer owns speed limiting; it needs the true observation point.
    state.pose.position.x = observation[0];
    state.pose.position.y = observation[1];
    state.pose.position.z = observation[2];
    state.pose.orientation = quaternion_from_euler(0.0, observation_camera_pitch(observation, target), yaw);
    state.twist.linear.x = external_goal_speed_ * std::cos(yaw);
    state.twist.linear.y = external_goal_speed_ * std::sin(yaw);
    state.twist.linear.z = 0.0;
    return state;
  }

  std::array<double, 3> observation_position_world(const std::array<double, 3> & target) const
  {
    // Observe from the seaward side of the target.  This preserves the 30 m
    // exclusion zone and gives the down-looking camera a repeatable geometry.
    double dx = -1.0;
    double dy = 0.0;
    if (has_usv_state_) {
      dx = target[0] - usv_pose_.position.x;
      dy = target[1] - usv_pose_.position.y;
    } else if (has_uav_state_) {
      dx = uav_pose_.position.x - target[0];
      dy = uav_pose_.position.y - target[1];
    }
    double distance = std::hypot(dx, dy);
    if (distance < 1e-3) {
      dx = -1.0;
      dy = 0.0;
      distance = 1.0;
    }
    const double standoff = std::max(1.0, observation_standoff_m_);
    std::array<double, 3> observation{
      target[0] + standoff * dx / distance, target[1] + standoff * dy / distance,
      std::max(1.0, observation_altitude_)};
    return keep_outside_operating_radius(observation, target);
  }

  double observation_camera_pitch(
    const std::array<double, 3> & observation, const std::array<double, 3> & target) const
  {
    const double horizontal = std::max(0.5, std::hypot(
      target[0] - observation[0], target[1] - observation[1]));
    // The heatmap target is on the water surface even though its command pose
    // carries the cruise altitude for the flight controller.
    const double desired_optical_pitch = std::atan2(std::max(0.5, observation[2]), horizontal);
    return std::clamp(
      desired_optical_pitch - camera_mount_pitch_, camera_pitch_min_, camera_pitch_max_);
  }

  std::array<double, 3> make_return_position_world() const
  {
    if (!has_usv_state_) {
      return {0.0, 0.0, std::max(1.0, return_altitude_)};
    }
    return {usv_pose_.position.x, usv_pose_.position.y, std::max(1.0, return_altitude_)};
  }

  gazebo_msgs::msg::EntityState make_return_state() const
  {
    const auto home = make_return_position_world();
    gazebo_msgs::msg::EntityState state;
    state.name = model_name_;
    state.reference_frame = "world";
    state.pose.position.x = home[0];
    state.pose.position.y = home[1];
    state.pose.position.z = home[2];
    const double yaw = has_usv_state_ ? yaw_from_quaternion(usv_pose_.orientation) : 0.0;
    state.pose.orientation = quaternion_from_euler(0.0, patrol_camera_pitch_, yaw);
    return state;
  }

  std::array<double, 3> keep_outside_operating_radius(
    const std::array<double, 3> & position, const std::array<double, 3> & target) const
  {
    if (!has_usv_state_ || minimum_operating_range_m_ <= 0.0) {
      return position;
    }
    double dx = position[0] - usv_pose_.position.x;
    double dy = position[1] - usv_pose_.position.y;
    double range = std::hypot(dx, dy);
    if (range >= minimum_operating_range_m_) {
      return position;
    }
    dx = target[0] - usv_pose_.position.x;
    dy = target[1] - usv_pose_.position.y;
    range = std::hypot(dx, dy);
    if (range < 1e-3) {
      dx = 1.0;
      dy = 0.0;
      range = 1.0;
    }
    return {usv_pose_.position.x + minimum_operating_range_m_ * dx / range,
      usv_pose_.position.y + minimum_operating_range_m_ * dy / range, position[2]};
  }

  gazebo_msgs::msg::EntityState make_patrol_state(double t) const
  {
    const double phase = angular_speed_ * t;
    double local_x = center_x_ + radius_x_ * std::cos(phase);
    double local_y = center_y_ + radius_y_ * std::sin(phase);
    const double patrol_range = std::hypot(local_x, local_y);
    if (minimum_operating_range_m_ > 0.0 && patrol_range < minimum_operating_range_m_) {
      const double scale = minimum_operating_range_m_ / std::max(patrol_range, 1e-3);
      local_x *= scale;
      local_y *= scale;
    }
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
    state.pose.orientation = quaternion_from_euler(0.0, patrol_camera_pitch_, yaw);
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

  void on_assist_request(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    assist_goal_ = *msg;
    if (assist_goal_.header.frame_id.empty() || assist_goal_.header.frame_id == "base_link" ||
      assist_goal_.header.frame_id == "ship")
    {
      const auto world = ship_relative_to_world(
        assist_goal_.pose.position.x, assist_goal_.pose.position.y, assist_goal_.pose.position.z);
      assist_goal_.header.frame_id = "world";
      assist_goal_.pose.position.x = world[0];
      assist_goal_.pose.position.y = world[1];
      assist_goal_.pose.position.z = world[2];
    }
    has_assist_goal_ = true;
    last_assist_goal_time_ = std::chrono::steady_clock::now();
  }

  void on_c3_detected(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    std::size_t pos = 0;
    double best_confidence = -1.0;
    geometry_msgs::msg::PoseStamped best_goal;
    while (true) {
      const auto start = msg->data.find("{\"object_id\":", pos);
      if (start == std::string::npos) {
        break;
      }
      const auto end = msg->data.find('}', start);
      if (end == std::string::npos) {
        break;
      }
      const std::string block = msg->data.substr(start, end - start + 1);
      double x = 0.0;
      double y = 0.0;
      double confidence = 0.0;
      double class_id = -1.0;
      std::string source;
      if (extract_double(block, "\"x\":", x) && extract_double(block, "\"y\":", y) &&
        extract_double(block, "\"confidence\":", confidence) &&
        extract_double(block, "\"class_id\":", class_id) &&
        extract_string(block, "\"last_source\":\"", source) &&
        source.find("uav_") != std::string::npos && is_target_class(class_id) &&
        confidence > best_confidence)
      {
        const auto world = ship_relative_to_world(x, y, 0.0);
        best_goal.header.stamp = get_clock()->now();
        best_goal.header.frame_id = "world";
        best_goal.pose.position.x = world[0];
        best_goal.pose.position.y = world[1];
        best_goal.pose.position.z = 0.0;
        best_goal.pose.orientation.w = 1.0;
        best_confidence = confidence;
      }
      pos = end + 1;
    }
    if (best_confidence >= 0.0) {
      tracked_target_goal_ = best_goal;
      tracked_target_confidence_ = best_confidence;
      has_tracked_target_ = true;
      last_tracked_target_time_ = std::chrono::steady_clock::now();
    }
  }

  void on_track_status(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    std::size_t pos = 0;
    double best_score = -1.0;
    double best_confidence = 0.0;
    geometry_msgs::msg::PoseStamped best_goal;
    while (true) {
      const auto start = msg->data.find('{', pos);
      if (start == std::string::npos) {
        break;
      }
      const auto end = msg->data.find('}', start);
      if (end == std::string::npos) {
        break;
      }
      const std::string block = msg->data.substr(start, end - start + 1);
      double x = 0.0;
      double y = 0.0;
      double confidence = 0.0;
      double class_id = -1.0;
      double misses = 0.0;
      double hits = 0.0;
      std::string frame_id;
      std::string source;
      if (extract_double(block, "\"x\":", x) && extract_double(block, "\"y\":", y) &&
        extract_double(block, "\"confidence\":", confidence) &&
        extract_double(block, "\"class_id\":", class_id) &&
        is_target_class(class_id))
      {
        extract_double(block, "\"misses\":", misses);
        extract_double(block, "\"hits\":", hits);
        extract_string(block, "\"frame_id\":\"", frame_id);
        extract_string(block, "\"last_source\":\"", source);
        if (misses > tracker_target_max_misses_ || hits < 1.0) {
          pos = end + 1;
          continue;
        }
        const bool from_uav = source.find("uav_") != std::string::npos ||
          source.find("confirmation_uav") != std::string::npos;
        const double score = confidence + (from_uav ? 0.35 : 0.0) - 0.03 * misses;
        if (score > best_score) {
          const auto world = frame_id == "world" ? std::array<double, 3>{x, y, 0.0} :
            ship_relative_to_world(x, y, 0.0);
          best_goal.header.stamp = get_clock()->now();
          best_goal.header.frame_id = "world";
          best_goal.pose.position.x = world[0];
          best_goal.pose.position.y = world[1];
          best_goal.pose.position.z = 0.0;
          best_goal.pose.orientation.w = 1.0;
          best_score = score;
          best_confidence = confidence;
        }
      }
      pos = end + 1;
    }
    if (best_score >= 0.0) {
      tracked_target_goal_ = best_goal;
      tracked_target_confidence_ = best_confidence;
      has_tracked_target_ = true;
      last_tracked_target_time_ = std::chrono::steady_clock::now();
    }
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
    } else if (mission_mode_ == MissionMode::OBSERVE) {
      mode = "外海热力图观察识别";
    } else if (mission_mode_ == MissionMode::ASSIST) {
      mode = "主船请求协助观察";
    } else if (mission_mode_ == MissionMode::TRACK) {
      mode = "飞控持续跟踪目标";
    } else if (mission_mode_ == MissionMode::RETURN) {
      mode = "低电量返航";
    }
    std::ostringstream out;
    out.setf(std::ios::fixed, std::ios::floatfield);
    out << std::setprecision(2)
        << "{\"模式\":\"" << mode << "\""
        << ",\"任务有效\":" << (has_active_goal() ? "true" : "false")
        << ",\"指令_x\":" << command.pose.position.x
        << ",\"指令_y\":" << command.pose.position.y
        << ",\"指令高度\":" << command.pose.position.z
        << ",\"返航请求\":" << (return_requested_ ? "true" : "false");
    if (has_recent_external_goal()) {
      out << ",\"粗定位_x\":" << external_goal_.pose.position.x
          << ",\"粗定位_y\":" << external_goal_.pose.position.y
          << ",\"观察门控\":" << (mission_mode_ == MissionMode::OBSERVE ? "true" : "false");
    }
    if (has_recent_tracked_target()) {
      out << ",\"飞控目标置信度\":" << tracked_target_confidence_
          << ",\"目标持续跟踪\":true";
    }
    out << "}";
    std_msgs::msg::String msg;
    msg.data = out.str();
    flight_status_pub_->publish(msg);
  }

  void update_mission_mode()
  {
    if (return_requested_) {
      mission_mode_ = MissionMode::RETURN;
      return;
    }
    if (has_recent_tracked_target()) {
      mission_mode_ = MissionMode::TRACK;
      return;
    }
    if (has_recent_assist_goal()) {
      mission_mode_ = MissionMode::ASSIST;
      return;
    }
    if (has_active_goal()) {
      mission_mode_ = MissionMode::TRANSIT;
      if (has_uav_state_) {
        const auto goal = resolve_goal_world();
        const auto observation = observation_position_world(goal);
        const double distance = std::hypot(
          observation[0] - uav_pose_.position.x, observation[1] - uav_pose_.position.y);
        if (distance <= std::max(0.2, observation_arrive_radius_)) {
          mission_mode_ = MissionMode::OBSERVE;
        }
      }
      return;
    }
    mission_mode_ = MissionMode::PATROL;
  }

  std::array<double, 3> resolve_goal_world() const
  {
    if (has_recent_tracked_target()) {
      return {
        tracked_target_goal_.pose.position.x, tracked_target_goal_.pose.position.y,
        tracked_target_goal_.pose.position.z};
    }
    if (has_recent_assist_goal()) {
      return {assist_goal_.pose.position.x, assist_goal_.pose.position.y, assist_goal_.pose.position.z};
    }
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

  bool has_recent_assist_goal() const
  {
    return has_assist_goal_ &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() - last_assist_goal_time_).count() <=
             assist_goal_timeout_;
  }

  bool has_recent_tracked_target() const
  {
    return has_tracked_target_ &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() - last_tracked_target_time_).count() <=
             uav_track_timeout_;
  }

  bool has_active_goal() const
  {
    return has_recent_external_goal() || has_recent_assist_goal() || has_recent_tracked_target();
  }

  static bool is_target_class(double class_id)
  {
    return std::abs(class_id - 0.0) < 0.5;
  }

  static bool extract_double(const std::string & text, const std::string & key, double & value)
  {
    const auto key_pos = text.find(key);
    if (key_pos == std::string::npos) {
      return false;
    }
    const auto value_start = key_pos + key.size();
    const auto value_end = text.find_first_of(",}", value_start);
    try {
      value = std::stod(text.substr(value_start, value_end - value_start));
      return true;
    } catch (...) {
      return false;
    }
  }

  static bool extract_string(const std::string & text, const std::string & key, std::string & value)
  {
    const auto key_pos = text.find(key);
    if (key_pos == std::string::npos) {
      return false;
    }
    const auto value_start = key_pos + key.size();
    const auto value_end = text.find('"', value_start);
    if (value_end == std::string::npos) {
      return false;
    }
    value = text.substr(value_start, value_end - value_start);
    return true;
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

    const std::array<std::pair<const char *, double>, 4> camera_views{{
      {"uav_gated_camera_link", 0.0},
      {"uav_gated_camera_right_link", -1.57079632679},
      {"uav_gated_camera_back_link", 3.14159265359},
      {"uav_gated_camera_left_link", 1.57079632679},
    }};
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(1 + camera_views.size());
    transforms.push_back(uav_tf);
    for (const auto & [frame, yaw] : camera_views) {
      geometry_msgs::msg::TransformStamped camera_tf;
      camera_tf.header.stamp = now;
      camera_tf.header.frame_id = "scout_uav/base_link";
      camera_tf.child_frame_id = frame;
      camera_tf.transform.translation.x = 0.78;
      camera_tf.transform.translation.y = 0.0;
      camera_tf.transform.translation.z = -0.12;
      camera_tf.transform.rotation = quaternion_from_euler(0.0, camera_mount_pitch_, yaw);
      transforms.push_back(camera_tf);
    }
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
      const auto target = resolve_goal_world();
      goal.pose.position.x = target[0];
      goal.pose.position.y = target[1];
      goal.pose.position.z = 0.0;
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
  std::string assist_request_topic_{"/uav/assist_request"};
  std::string c3_detected_topic_{"/c3/detected_objects"};
  std::string track_status_topic_{"/tracked_objects_text"};
  std::string control_backend_{"px4"};
  std::string px4_goal_topic_{"/uav/offboard_goal"};
  std::string px4_pose_topic_{"/px4/vehicle_pose"};
  std::string gazebo_sim_goal_topic_{"/uav/sim/command"};
  std::string gazebo_sim_pose_topic_{"/uav/sim/estimated_pose"};
  std::string flight_status_topic_{"/uav/flight_status"};
  std::string observation_active_topic_{"/uav/observation/active"};
  std::string return_requested_topic_{"/uav/return_requested"};
  double center_x_{16.0};
  double center_y_{0.0};
  double altitude_{26.0};
  double radius_x_{22.0};
  double radius_y_{16.0};
  double angular_speed_{0.045};
  double patrol_speed_{0.45};
  double camera_pitch_{0.30};
  double patrol_camera_pitch_{0.30};
  double camera_mount_pitch_{0.55};
  double camera_pitch_min_{-0.20};
  double camera_pitch_max_{0.45};
  double external_goal_timeout_{4.0};
  double assist_goal_timeout_{3.0};
  double uav_track_timeout_{3.0};
  double external_goal_speed_{1.6};
  double goal_arrive_radius_{2.0};
  double observation_standoff_m_{4.0};
  double observation_altitude_{8.0};
  double observation_arrive_radius_{1.2};
  double minimum_operating_range_m_{30.0};
  double return_altitude_{10.0};
  double update_dt_s_{0.125};
  int tracker_target_max_misses_{80};
  bool warned_waiting_{false};
  bool has_usv_state_{false};
  bool has_uav_state_{false};
  bool has_uav_truth_state_{false};
  bool has_external_goal_{false};
  bool has_assist_goal_{false};
  bool has_tracked_target_{false};
  bool return_requested_{false};
  MissionMode mission_mode_{MissionMode::PATROL};
  geometry_msgs::msg::Pose usv_pose_;
  geometry_msgs::msg::Twist usv_twist_;
  geometry_msgs::msg::Pose uav_pose_;
  geometry_msgs::msg::Pose uav_truth_pose_;
  geometry_msgs::msg::PoseStamped external_goal_;
  geometry_msgs::msg::PoseStamped assist_goal_;
  geometry_msgs::msg::PoseStamped tracked_target_goal_;
  double tracked_target_confidence_{0.0};
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_external_goal_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_assist_goal_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_tracked_target_time_{std::chrono::steady_clock::now()};
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr external_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr assist_request_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr c3_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr track_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr return_requested_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr px4_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gazebo_sim_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flight_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr observation_active_pub_;
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
