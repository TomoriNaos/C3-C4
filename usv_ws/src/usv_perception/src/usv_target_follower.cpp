#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "usv_perception/common.hpp"

namespace usv_perception
{

struct TrackObservation
{
  int id{-1};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double speed{0.0};
  double confidence{0.0};
  double class_id{-1.0};
  int hits{0};
  std::string last_source{"unknown"};
  std::chrono::steady_clock::time_point stamp{std::chrono::steady_clock::now()};
};

struct AvoidanceCommand
{
  double bearing_offset{0.0};
  double speed_scale{1.0};
  double closest_hazard{std::numeric_limits<double>::infinity()};
  int hazard_count{0};
};

class UsvTargetFollower : public rclcpp::Node
{
public:
  UsvTargetFollower()
  : Node("usv_target_follower"),
    start_time_(std::chrono::steady_clock::now()),
    last_step_(start_time_)
  {
    model_name_ = declare_parameter<std::string>("model_name", "wamv");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/tracked_object_poses");
    track_status_topic_ = declare_parameter<std::string>("track_status_topic", "/tracked_objects_text");
    remote_target_topic_ = declare_parameter<std::string>("remote_target_topic", "/uav/remote_target_status");
    model_states_topic_ = declare_parameter<std::string>("model_states_topic", "/model_states");
    enabled_ = declare_parameter<bool>("enabled", true);
    update_rate_ = declare_parameter<double>("update_rate", 15.0);
    max_speed_ = declare_parameter<double>("max_speed", 1.50);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.70);
    desired_standoff_ = declare_parameter<double>("desired_standoff", 10.0);
    target_timeout_ = declare_parameter<double>("target_timeout", 5.0);
    track_status_timeout_ = declare_parameter<double>("track_status_timeout", 2.0);
    remote_target_timeout_ = declare_parameter<double>("remote_target_timeout", 3.5);
    max_follow_range_ = declare_parameter<double>("max_follow_range", 120.0);
    yaw_gain_ = declare_parameter<double>("yaw_gain", 1.00);
    speed_gain_ = declare_parameter<double>("speed_gain", 0.18);
    waterline_z_ = declare_parameter<double>("waterline_z", 0.32);
    startup_delay_ = declare_parameter<double>("startup_delay", 3.0);
    target_lead_time_ = declare_parameter<double>("target_lead_time", 1.8);
    max_lead_distance_ = declare_parameter<double>("max_lead_distance", 6.0);
    min_chase_speed_ = declare_parameter<double>("min_chase_speed", 0.45);
    far_range_speed_boost_ = declare_parameter<double>("far_range_speed_boost", 0.35);
    stale_target_speed_scale_ = declare_parameter<double>("stale_target_speed_scale", 0.75);
    target_lock_timeout_ = declare_parameter<double>("target_lock_timeout", 4.5);
    target_reacquire_gate_ = declare_parameter<double>("target_reacquire_gate", 9.0);
    turn_slowdown_gain_ = declare_parameter<double>("turn_slowdown_gain", 0.35);

    follow_class_id_ = declare_parameter<double>("follow_class_id", 5.0);
    follow_class_ids_ = declare_parameter<std::vector<double>>(
      "follow_class_ids", std::vector<double>{follow_class_id_});
    prefer_follow_class_ = declare_parameter<bool>("prefer_follow_class", true);
    min_follow_confidence_ = declare_parameter<double>("min_follow_confidence", 0.18);
    min_track_hits_ = declare_parameter<int>("min_track_hits", 2);

    obstacle_lookahead_ = declare_parameter<double>("obstacle_lookahead", 34.0);
    obstacle_lateral_window_ = declare_parameter<double>("obstacle_lateral_window", 13.0);
    obstacle_clearance_ = declare_parameter<double>("obstacle_clearance", 5.5);
    hard_stop_distance_ = declare_parameter<double>("hard_stop_distance", 8.0);
    hard_stop_lateral_ = declare_parameter<double>("hard_stop_lateral", 2.4);
    avoidance_gain_ = declare_parameter<double>("avoidance_gain", 0.85);
    obstacle_slowdown_gain_ = declare_parameter<double>("obstacle_slowdown_gain", 0.38);
    min_avoidance_speed_scale_ = declare_parameter<double>("min_avoidance_speed_scale", 0.50);
    centered_obstacle_bias_ = declare_parameter<double>("centered_obstacle_bias", 1.0);
    use_model_state_obstacles_ = declare_parameter<bool>("use_model_state_obstacles", true);
    model_obstacle_timeout_ = declare_parameter<double>("model_obstacle_timeout", 1.0);
    obstacle_model_names_ = declare_parameter<std::vector<std::string>>(
      "obstacle_model_names",
      std::vector<std::string>{
        "navigation_marker_port",
        "navigation_marker_starboard",
        "fishnet_buoy",
        "floating_obstacle",
        "drift_debris",
        "floating_container",
        "channel_buoy_north",
        "channel_buoy_south",
        "net_line_a"});

    client_ = create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      pose_topic_, 10, std::bind(&UsvTargetFollower::on_tracks, this, std::placeholders::_1));
    status_sub_ = create_subscription<std_msgs::msg::String>(
      track_status_topic_, 10, std::bind(&UsvTargetFollower::on_track_status, this, std::placeholders::_1));
    remote_target_sub_ = create_subscription<std_msgs::msg::String>(
      remote_target_topic_, 10,
      std::bind(&UsvTargetFollower::on_remote_target_status, this, std::placeholders::_1));
    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic_, 10, std::bind(&UsvTargetFollower::on_model_states, this, std::placeholders::_1));
    status_pub_ = create_publisher<std_msgs::msg::String>("usv_follow_status", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(update_rate_, 0.1)));
    timer_ = create_wall_timer(period, std::bind(&UsvTargetFollower::on_timer, this));
    RCLCPP_INFO(
      get_logger(), "USV target follower subscribed to poses=%s status=%s remote=%s",
      pose_topic_.c_str(), track_status_topic_.c_str(), remote_target_topic_.c_str());
  }

private:
  void on_tracks(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    pose_tracks_.clear();
    pose_tracks_.reserve(msg->poses.size());
    int fallback_id = -1;
    const auto now = std::chrono::steady_clock::now();
    for (const auto & pose : msg->poses) {
      const double x = pose.position.x;
      const double y = pose.position.y;
      const double range = std::hypot(x, y);
      if (!std::isfinite(range) || range > max_follow_range_ || x < -2.0) {
        continue;
      }
      TrackObservation track;
      track.id = fallback_id--;
      track.x = x;
      track.y = y;
      track.confidence = 0.45;
      track.hits = 1;
      track.last_source = "pose_array";
      track.stamp = now;
      pose_tracks_.push_back(track);
    }
    last_pose_time_ = now;
  }

  void on_track_status(const std_msgs::msg::String::SharedPtr msg)
  {
    auto parsed = parse_track_status(msg->data);
    if (parsed.empty()) {
      return;
    }
    metadata_tracks_ = std::move(parsed);
    last_status_time_ = std::chrono::steady_clock::now();
    has_status_tracks_ = true;
  }

  void on_remote_target_status(const std_msgs::msg::String::SharedPtr msg)
  {
    auto parsed = parse_track_status(msg->data);
    if (parsed.empty()) {
      return;
    }
    remote_target_tracks_ = std::move(parsed);
    last_remote_target_time_ = std::chrono::steady_clock::now();
    has_remote_target_tracks_ = true;
  }

  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    if (!use_model_state_obstacles_) {
      return;
    }

    model_obstacle_tracks_.clear();
    const auto now = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < msg->name.size() && i < msg->pose.size(); ++i) {
      if (std::find(obstacle_model_names_.begin(), obstacle_model_names_.end(), msg->name[i]) ==
        obstacle_model_names_.end())
      {
        continue;
      }

      const double dx = msg->pose[i].position.x - x_;
      const double dy = msg->pose[i].position.y - y_;
      const double cos_yaw = std::cos(yaw_);
      const double sin_yaw = std::sin(yaw_);
      const double rel_x = cos_yaw * dx + sin_yaw * dy;
      const double rel_y = -sin_yaw * dx + cos_yaw * dy;
      const double range = std::hypot(rel_x, rel_y);
      if (!std::isfinite(range) || range > max_follow_range_ || rel_x < -4.0) {
        continue;
      }

      TrackObservation obstacle;
      obstacle.id = -10000 - static_cast<int>(i);
      obstacle.x = rel_x;
      obstacle.y = rel_y;
      obstacle.confidence = 0.85;
      obstacle.class_id = 3.0;
      obstacle.hits = 3;
      obstacle.last_source = "gazebo_model_state";
      obstacle.stamp = now;
      model_obstacle_tracks_.push_back(obstacle);
    }
    last_model_obstacle_time_ = now;
  }

  void on_timer()
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::clamp(std::chrono::duration<double>(now - last_step_).count(), 0.0, 0.25);
    last_step_ = now;

    const auto tracks = active_tracks(now);
    const auto target = select_target(tracks, now);
    bool has_fresh_target = false;
    if (target.has_value()) {
      target_ = target.value();
      locked_target_id_ = target_.id;
      last_target_time_ = now;
      has_target_ = true;
      has_fresh_target = true;
    } else if (has_recent_target(now)) {
      target_.x += target_.vx * dt;
      target_.y += target_.vy * dt;
    }

    double command_speed = 0.0;
    double target_bearing = 0.0;
    double commanded_bearing = 0.0;
    double aim_x = 0.0;
    double aim_y = 0.0;
    AvoidanceCommand avoidance;
    if (enabled_ && has_recent_target(now)) {
      const double actual_range = std::hypot(target_.x, target_.y);
      aim_x = target_.x + target_.vx * target_lead_time_;
      aim_y = target_.y + target_.vy * target_lead_time_;
      const double lead_distance = std::hypot(aim_x - target_.x, aim_y - target_.y);
      if (lead_distance > max_lead_distance_) {
        const double scale = max_lead_distance_ / std::max(lead_distance, 1e-3);
        aim_x = target_.x + (aim_x - target_.x) * scale;
        aim_y = target_.y + (aim_y - target_.y) * scale;
      }

      target_bearing = std::atan2(aim_y, aim_x);
      avoidance = compute_avoidance(tracks, target_.id);
      commanded_bearing = std::clamp(target_bearing + avoidance.bearing_offset, -1.35, 1.35);
      const double yaw_rate = std::clamp(yaw_gain_ * commanded_bearing, -max_yaw_rate_, max_yaw_rate_);
      double base_speed = std::clamp(speed_gain_ * (actual_range - desired_standoff_), 0.0, max_speed_);
      if (actual_range > desired_standoff_ + 3.0) {
        base_speed = std::max(base_speed, min_chase_speed_);
      }
      if (actual_range > desired_standoff_ + 12.0) {
        const double boost = far_range_speed_boost_ *
          std::clamp((actual_range - desired_standoff_ - 12.0) / 36.0, 0.0, 1.0);
        base_speed = std::min(max_speed_, base_speed + boost);
      }
      if (!has_fresh_target) {
        base_speed *= stale_target_speed_scale_;
      }
      const double turn_scale = std::clamp(1.0 - turn_slowdown_gain_ * std::abs(commanded_bearing), 0.45, 1.0);
      command_speed = base_speed * avoidance.speed_scale * turn_scale;

      yaw_ = normalize_angle(yaw_ + yaw_rate * dt);
      x_ += command_speed * std::cos(yaw_) * dt;
      y_ += command_speed * std::sin(yaw_) * dt;
    }

    publish_tf();
    publish_status(tracks, target_bearing, commanded_bearing, command_speed, aim_x, aim_y, avoidance);
    if (enabled_ && std::chrono::duration<double>(now - start_time_).count() >= startup_delay_) {
      move_model();
    }
  }

  std::vector<TrackObservation> active_tracks(const std::chrono::steady_clock::time_point & now) const
  {
    std::vector<TrackObservation> tracks;
    if (has_status_tracks_ &&
      std::chrono::duration<double>(now - last_status_time_).count() <= track_status_timeout_)
    {
      tracks = metadata_tracks_;
    } else if (std::chrono::duration<double>(now - last_pose_time_).count() <= track_status_timeout_) {
      tracks = pose_tracks_;
    }

    if (has_remote_target_tracks_ &&
      std::chrono::duration<double>(now - last_remote_target_time_).count() <= remote_target_timeout_)
    {
      tracks.insert(tracks.end(), remote_target_tracks_.begin(), remote_target_tracks_.end());
    }

    if (use_model_state_obstacles_ &&
      std::chrono::duration<double>(now - last_model_obstacle_time_).count() <= model_obstacle_timeout_)
    {
      tracks.insert(tracks.end(), model_obstacle_tracks_.begin(), model_obstacle_tracks_.end());
    }
    return tracks;
  }

  std::optional<TrackObservation> select_target(
    const std::vector<TrackObservation> & tracks,
    const std::chrono::steady_clock::time_point & now) const
  {
    if (has_target_ &&
      std::chrono::duration<double>(now - last_target_time_).count() <= target_lock_timeout_)
    {
      for (const auto & track : tracks) {
        if (track.id == locked_target_id_ && is_valid_follow_candidate(track)) {
          return track;
        }
      }

      for (const auto & track : tracks) {
        if (!is_follow_class(track) || !is_valid_follow_candidate(track)) {
          continue;
        }
        if (std::hypot(track.x - target_.x, track.y - target_.y) <= target_reacquire_gate_) {
          return track;
        }
      }
    }

    bool has_class_candidate = false;
    for (const auto & track : tracks) {
      if (is_follow_class(track) && is_valid_follow_candidate(track)) {
        has_class_candidate = true;
        break;
      }
    }

    double best_score = std::numeric_limits<double>::infinity();
    std::optional<TrackObservation> best;
    for (const auto & track : tracks) {
      if (!is_valid_follow_candidate(track)) {
        continue;
      }
      const bool class_match = is_follow_class(track);
      if (prefer_follow_class_ && has_class_candidate && !class_match) {
        continue;
      }
      const double range = std::hypot(track.x, track.y);
      double score = 0.030 * range + 0.18 * std::abs(track.y);
      score -= class_match ? 4.0 : 0.0;
      score -= std::min(track.speed, 1.5) * 0.55;
      score -= std::clamp(track.confidence, 0.0, 1.0) * 0.35;
      score -= std::min(track.hits, 8) * 0.04;
      if (track.last_source.find("gated") != std::string::npos) {
        score -= 0.20;
      }
      if (track.last_source == "uav_remote_scout") {
        score -= 3.0;
      }
      if (score < best_score) {
        best_score = score;
        best = track;
      }
    }
    return best;
  }

  bool is_valid_follow_candidate(const TrackObservation & track) const
  {
    const double range = std::hypot(track.x, track.y);
    if (!std::isfinite(range) || track.x < 1.0 || range > max_follow_range_) {
      return false;
    }
    if (track.confidence < min_follow_confidence_) {
      return false;
    }
    if (track.last_source == "gazebo_model_state") {
      return false;
    }
    if (track.hits > 0 && track.hits < min_track_hits_ && track.last_source != "pose_array") {
      return false;
    }
    return true;
  }

  bool is_follow_class(const TrackObservation & track) const
  {
    if (follow_class_ids_.empty()) {
      return follow_class_id_ >= 0.0 && std::abs(track.class_id - follow_class_id_) < 0.5;
    }
    for (const double class_id : follow_class_ids_) {
      if (class_id >= 0.0 && std::abs(track.class_id - class_id) < 0.5) {
        return true;
      }
    }
    return false;
  }

  AvoidanceCommand compute_avoidance(
    const std::vector<TrackObservation> & tracks,
    int target_id) const
  {
    AvoidanceCommand command;
    for (const auto & obstacle : tracks) {
      if (obstacle.id == target_id) {
        continue;
      }
      if (obstacle.x < 0.5 || obstacle.x > obstacle_lookahead_ ||
        std::abs(obstacle.y) > obstacle_lateral_window_)
      {
        continue;
      }

      const double lateral_margin = obstacle_clearance_ - std::abs(obstacle.y);
      if (lateral_margin <= 0.0) {
        continue;
      }

      const double range = std::hypot(obstacle.x, obstacle.y);
      command.closest_hazard = std::min(command.closest_hazard, range);
      ++command.hazard_count;

      const double closeness = std::clamp((obstacle_lookahead_ - obstacle.x) / obstacle_lookahead_, 0.0, 1.0);
      const double lateral_pressure = std::clamp(lateral_margin / std::max(obstacle_clearance_, 0.1), 0.0, 1.0);
      double side = obstacle.y > 0.25 ? -1.0 : 1.0;
      if (std::abs(obstacle.y) <= 0.25) {
        side = centered_obstacle_bias_ >= 0.0 ? 1.0 : -1.0;
      }

      command.bearing_offset += side * avoidance_gain_ * lateral_pressure * (0.35 + 0.65 * closeness);
      const double slowdown = 1.0 - obstacle_slowdown_gain_ * lateral_pressure * closeness;
      command.speed_scale = std::min(
        command.speed_scale,
        std::clamp(slowdown, min_avoidance_speed_scale_, 1.0));

      if (obstacle.x < hard_stop_distance_ && std::abs(obstacle.y) < hard_stop_lateral_) {
        command.speed_scale = 0.0;
        command.bearing_offset += side * avoidance_gain_ * 0.9;
      }
    }
    command.bearing_offset = std::clamp(command.bearing_offset, -0.95, 0.95);
    return command;
  }

  bool has_recent_target(const std::chrono::steady_clock::time_point & now) const
  {
    return has_target_ && std::chrono::duration<double>(now - last_target_time_).count() <= target_timeout_;
  }

  void publish_tf()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = get_clock()->now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = waterline_z_;
    transform.transform.rotation = quaternion_from_euler(0.0, 0.0, yaw_);
    tf_broadcaster_->sendTransform(transform);
  }

  void publish_status(
    const std::vector<TrackObservation> & tracks,
    double target_bearing,
    double commanded_bearing,
    double command_speed,
    double aim_x,
    double aim_y,
    const AvoidanceCommand & avoidance)
  {
    const auto now = std::chrono::steady_clock::now();
    const double target_age = has_target_ ? std::chrono::duration<double>(now - last_target_time_).count() : -1.0;
    std::ostringstream status;
    status.setf(std::ios::fixed, std::ios::floatfield);
    status.precision(2);
    status << "{\"has_target\":" << (has_target_ ? "true" : "false")
           << ",\"target_id\":" << (has_target_ ? target_.id : -1)
           << ",\"target_x\":" << (has_target_ ? target_.x : 0.0)
           << ",\"target_y\":" << (has_target_ ? target_.y : 0.0)
           << ",\"target_class_id\":" << (has_target_ ? target_.class_id : -1.0)
           << ",\"locked_target_id\":" << locked_target_id_
           << ",\"target_age\":" << target_age
           << ",\"aim_x\":" << aim_x
           << ",\"aim_y\":" << aim_y
           << ",\"target_bearing\":" << target_bearing
           << ",\"commanded_bearing\":" << commanded_bearing
           << ",\"avoidance_offset\":" << avoidance.bearing_offset
           << ",\"speed\":" << command_speed
           << ",\"speed_scale\":" << avoidance.speed_scale
           << ",\"hazard_count\":" << avoidance.hazard_count
           << ",\"closest_hazard\":";
    if (std::isfinite(avoidance.closest_hazard)) {
      status << avoidance.closest_hazard;
    } else {
      status << -1.0;
    }
    status << ",\"track_count\":" << tracks.size() << "}";

    std_msgs::msg::String msg;
    msg.data = status.str();
    status_pub_->publish(msg);
  }

  void move_model()
  {
    if (!client_->service_is_ready()) {
      if (!warned_waiting_) {
        RCLCPP_INFO(get_logger(), "Waiting for /set_entity_state to move USV");
        warned_waiting_ = true;
      }
      return;
    }

    gazebo_msgs::msg::EntityState state;
    state.name = model_name_;
    state.reference_frame = "world";
    state.pose.position.x = x_;
    state.pose.position.y = y_;
    state.pose.position.z = waterline_z_;
    state.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw_);

    auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    req->state = state;
    client_->async_send_request(req);
  }

  static std::vector<TrackObservation> parse_track_status(const std::string & text)
  {
    std::vector<TrackObservation> tracks;
    std::size_t pos = 0;
    const auto now = std::chrono::steady_clock::now();
    while (true) {
      const auto start = text.find('{', pos);
      if (start == std::string::npos) {
        break;
      }
      const auto end = text.find('}', start);
      if (end == std::string::npos) {
        break;
      }

      const std::string block = text.substr(start, end - start + 1);
      TrackObservation track;
      track.stamp = now;
      double id_value = -1.0;
      if (extract_double(block, "\"id\":", id_value)) {
        track.id = static_cast<int>(std::round(id_value));
      }
      extract_double(block, "\"x\":", track.x);
      extract_double(block, "\"y\":", track.y);
      extract_double(block, "\"vx\":", track.vx);
      extract_double(block, "\"vy\":", track.vy);
      extract_double(block, "\"speed\":", track.speed);
      extract_double(block, "\"confidence\":", track.confidence);
      extract_double(block, "\"class_id\":", track.class_id);
      double hits_value = 0.0;
      if (extract_double(block, "\"hits\":", hits_value)) {
        track.hits = static_cast<int>(std::round(hits_value));
      }
      extract_string(block, "\"last_source\":\"", track.last_source);
      if (track.speed <= 0.0) {
        track.speed = std::hypot(track.vx, track.vy);
      }
      const double range = std::hypot(track.x, track.y);
      if (std::isfinite(range)) {
        tracks.push_back(track);
      }
      pos = end + 1;
    }
    return tracks;
  }

  static bool extract_double(const std::string & text, const std::string & key, double & out)
  {
    const auto key_pos = text.find(key);
    if (key_pos == std::string::npos) {
      return false;
    }
    const auto value_start = key_pos + key.size();
    const auto value_end = text.find_first_of(",}", value_start);
    const std::string token = text.substr(value_start, value_end - value_start);
    try {
      out = std::stod(token);
      return true;
    } catch (...) {
      return false;
    }
  }

  static bool extract_string(const std::string & text, const std::string & key, std::string & out)
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
    out = text.substr(value_start, value_end - value_start);
    return true;
  }

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  std::string model_name_{"wamv"};
  std::string pose_topic_{"/tracked_object_poses"};
  std::string track_status_topic_{"/tracked_objects_text"};
  std::string remote_target_topic_{"/uav/remote_target_status"};
  std::string model_states_topic_{"/model_states"};
  bool enabled_{true};
  bool warned_waiting_{false};
  bool has_target_{false};
  bool has_status_tracks_{false};
  bool has_remote_target_tracks_{false};
  bool prefer_follow_class_{true};
  bool use_model_state_obstacles_{true};
  double update_rate_{15.0};
  double max_speed_{1.50};
  double max_yaw_rate_{0.70};
  double desired_standoff_{10.0};
  double target_timeout_{5.0};
  double track_status_timeout_{2.0};
  double remote_target_timeout_{3.5};
  double model_obstacle_timeout_{1.0};
  double max_follow_range_{120.0};
  double yaw_gain_{1.00};
  double speed_gain_{0.18};
  double waterline_z_{0.32};
  double startup_delay_{3.0};
  double target_lead_time_{1.8};
  double max_lead_distance_{6.0};
  double min_chase_speed_{0.45};
  double far_range_speed_boost_{0.35};
  double stale_target_speed_scale_{0.75};
  double target_lock_timeout_{4.5};
  double target_reacquire_gate_{9.0};
  double turn_slowdown_gain_{0.35};
  double follow_class_id_{5.0};
  std::vector<double> follow_class_ids_;
  double min_follow_confidence_{0.18};
  int min_track_hits_{2};
  double obstacle_lookahead_{34.0};
  double obstacle_lateral_window_{13.0};
  double obstacle_clearance_{5.5};
  double hard_stop_distance_{8.0};
  double hard_stop_lateral_{2.4};
  double avoidance_gain_{0.85};
  double obstacle_slowdown_gain_{0.38};
  double min_avoidance_speed_scale_{0.50};
  double centered_obstacle_bias_{1.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  int locked_target_id_{-1};
  TrackObservation target_;
  std::vector<TrackObservation> metadata_tracks_;
  std::vector<TrackObservation> remote_target_tracks_;
  std::vector<TrackObservation> pose_tracks_;
  std::vector<TrackObservation> model_obstacle_tracks_;
  std::vector<std::string> obstacle_model_names_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_step_;
  std::chrono::steady_clock::time_point last_target_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_status_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_remote_target_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_pose_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_model_obstacle_time_{std::chrono::steady_clock::now()};
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr remote_target_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::UsvTargetFollower>());
  rclcpp::shutdown();
  return 0;
}
