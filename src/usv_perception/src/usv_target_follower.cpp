#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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
  int misses{0};
  std::string last_source{"unknown"};
  std::string frame_id{"base_link"};
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
    c3_detected_topic_ = declare_parameter<std::string>("c3_detected_topic", "/c3/detected_objects");
    uav_assist_topic_ = declare_parameter<std::string>("uav_assist_topic", "/uav/assist_request");
    model_states_topic_ = declare_parameter<std::string>("model_states_topic", "/model_states");
    // Normal operation is sensor-only. The model-state target feed exists
    // exclusively for repeatable Gazebo demonstrations and is opt-in.
    demo_mode_ = declare_parameter<bool>("demo_mode", false);
    demo_target_model_name_ =
      declare_parameter<std::string>("demo_target_model_name", "moving_vessel");
    flight_controller_goal_topic_ =
      declare_parameter<std::string>("flight_controller_goal_topic", "/usv/offboard_goal");
    flight_controller_standoff_m_ = declare_parameter<double>("flight_controller_standoff_m", -1.0);
    demo_target_confidence_ = declare_parameter<double>("demo_target_confidence", 0.98);
    demo_target_smoothing_tau_s_ = declare_parameter<double>("demo_target_smoothing_tau_s", 1.2);
    enabled_ = declare_parameter<bool>("enabled", true);
    update_rate_ = declare_parameter<double>("update_rate", 15.0);
    max_speed_ = declare_parameter<double>("max_speed", 1.50);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.70);
    desired_standoff_ = declare_parameter<double>("desired_standoff", 10.0);
    target_timeout_ = declare_parameter<double>("target_timeout", 5.0);
    track_status_timeout_ = declare_parameter<double>("track_status_timeout", 2.0);
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
    require_follow_class_for_acquisition_ =
      declare_parameter<bool>("require_follow_class_for_acquisition", true);
    require_visual_source_for_acquisition_ =
      declare_parameter<bool>("require_visual_source_for_acquisition", true);
    require_uav_source_for_initial_acquisition_ =
      declare_parameter<bool>("require_uav_source_for_initial_acquisition", true);
    turn_slowdown_gain_ = declare_parameter<double>("turn_slowdown_gain", 0.35);
    target_bearing_filter_tau_s_ = declare_parameter<double>("target_bearing_filter_tau_s", 1.2);
    heading_deadband_rad_ = declare_parameter<double>("heading_deadband_rad", 0.06);

    follow_class_id_ = declare_parameter<double>("follow_class_id", 5.0);
    follow_class_ids_ = declare_parameter<std::vector<double>>(
      "follow_class_ids", std::vector<double>{follow_class_id_});
    prefer_follow_class_ = declare_parameter<bool>("prefer_follow_class", true);
    min_follow_confidence_ = declare_parameter<double>("min_follow_confidence", 0.18);
    min_track_hits_ = declare_parameter<int>("min_track_hits", 2);
    max_follow_track_misses_ = declare_parameter<int>("max_follow_track_misses", 8);
    require_camera_sector_confirmation_ =
      declare_parameter<bool>("require_camera_sector_confirmation", true);
    camera_sector_gate_range_m_ = declare_parameter<double>("camera_sector_gate_range_m", 80.0);
    camera_sector_timeout_s_ = declare_parameter<double>("camera_sector_timeout_s", 2.0);
    camera_sector_min_confidence_ =
      declare_parameter<double>("camera_sector_min_confidence", 0.20);
    camera_detection_topics_ = declare_parameter<std::vector<std::string>>(
      "camera_detection_topics", std::vector<std::string>{
        "/gated_camera/detection_points", "/gated_camera/right/detection_points",
        "/gated_camera/back/detection_points", "/gated_camera/left/detection_points"});

    obstacle_lookahead_ = declare_parameter<double>("obstacle_lookahead", 34.0);
    obstacle_lateral_window_ = declare_parameter<double>("obstacle_lateral_window", 13.0);
    obstacle_clearance_ = declare_parameter<double>("obstacle_clearance", 5.5);
    hard_stop_distance_ = declare_parameter<double>("hard_stop_distance", 8.0);
    hard_stop_lateral_ = declare_parameter<double>("hard_stop_lateral", 2.4);
    avoidance_gain_ = declare_parameter<double>("avoidance_gain", 0.85);
    obstacle_slowdown_gain_ = declare_parameter<double>("obstacle_slowdown_gain", 0.38);
    min_avoidance_speed_scale_ = declare_parameter<double>("min_avoidance_speed_scale", 0.50);
    centered_obstacle_bias_ = declare_parameter<double>("centered_obstacle_bias", 1.0);
    min_obstacle_confidence_ = declare_parameter<double>("min_obstacle_confidence", 0.35);
    min_obstacle_hits_ = declare_parameter<int>("min_obstacle_hits", 2);

    client_ = create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      pose_topic_, 10, std::bind(&UsvTargetFollower::on_tracks, this, std::placeholders::_1));
    status_sub_ = create_subscription<std_msgs::msg::String>(
      track_status_topic_, 10, std::bind(&UsvTargetFollower::on_track_status, this, std::placeholders::_1));
    c3_detected_sub_ = create_subscription<std_msgs::msg::String>(
      c3_detected_topic_, 10, std::bind(&UsvTargetFollower::on_c3_detected, this, std::placeholders::_1));
    if (demo_mode_) {
      model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
        model_states_topic_, 10, std::bind(&UsvTargetFollower::on_model_states, this, std::placeholders::_1));
    }
    for (const auto & topic : camera_detection_topics_) {
      if (topic.empty()) {
        continue;
      }
      camera_detection_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_camera_detections(msg);}));
    }
    status_pub_ = create_publisher<std_msgs::msg::String>("usv_follow_status", 10);
    uav_assist_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(uav_assist_topic_, 10);
    flight_controller_goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(flight_controller_goal_topic_, 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(update_rate_, 0.1)));
    timer_ = create_wall_timer(period, std::bind(&UsvTargetFollower::on_timer, this));
    RCLCPP_INFO(
      get_logger(), "USV target follower subscribed to poses=%s status=%s c3=%s",
      pose_topic_.c_str(), track_status_topic_.c_str(), c3_detected_topic_.c_str());
  }

private:
  void on_tracks(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    pose_tracks_.clear();
    pose_tracks_.reserve(msg->poses.size());
    int fallback_id = -1;
    const auto now = std::chrono::steady_clock::now();
    for (const auto & pose : msg->poses) {
      double x = pose.position.x;
      double y = pose.position.y;
      if (msg->header.frame_id == "world") {
        const double dx = x - x_;
        const double dy = y - y_;
        const double c = std::cos(yaw_);
        const double s = std::sin(yaw_);
        x = c * dx + s * dy;
        y = -s * dx + c * dy;
      }
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

  void on_c3_detected(const std_msgs::msg::String::SharedPtr msg)
  {
    auto parsed = parse_track_status(msg->data, "c3_detected_objects");
    if (parsed.empty()) {
      return;
    }
    c3_detected_tracks_ = std::move(parsed);
    last_c3_detected_time_ = std::chrono::steady_clock::now();
    has_c3_detected_tracks_ = true;
  }

  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    if (!msg || !demo_mode_) {
      return;
    }
    const auto target_it =
      std::find(msg->name.begin(), msg->name.end(), demo_target_model_name_);
    if (target_it == msg->name.end()) {
      return;
    }
    const auto target_index = static_cast<std::size_t>(std::distance(msg->name.begin(), target_it));
    if (target_index >= msg->pose.size()) {
      return;
    }

    update_demo_target(
      msg->pose[target_index].position.x,
      msg->pose[target_index].position.y,
      std::chrono::steady_clock::now());
  }

  void update_demo_target(
    double target_world_x,
    double target_world_y,
    const std::chrono::steady_clock::time_point & now)
  {
    if (!std::isfinite(target_world_x) || !std::isfinite(target_world_y)) {
      return;
    }

    double dt = 0.0;
    if (has_demo_raw_target_) {
      dt = std::clamp(std::chrono::duration<double>(now - last_demo_raw_time_).count(), 1e-3, 0.5);
    }

    if (!has_demo_raw_target_) {
      smoothed_demo_x_ = target_world_x;
      smoothed_demo_y_ = target_world_y;
      smoothed_demo_vx_ = 0.0;
      smoothed_demo_vy_ = 0.0;
      has_demo_raw_target_ = true;
    } else {
      const double tau = std::max(0.05, demo_target_smoothing_tau_s_);
      const double alpha = std::clamp(dt / (tau + dt), 0.0, 1.0);
      const double previous_x = smoothed_demo_x_;
      const double previous_y = smoothed_demo_y_;
      smoothed_demo_x_ += alpha * (target_world_x - smoothed_demo_x_);
      smoothed_demo_y_ += alpha * (target_world_y - smoothed_demo_y_);
      smoothed_demo_vx_ = (smoothed_demo_x_ - previous_x) / dt;
      smoothed_demo_vy_ = (smoothed_demo_y_ - previous_y) / dt;
    }
    last_demo_raw_time_ = now;

    const double dx = smoothed_demo_x_ - x_;
    const double dy = smoothed_demo_y_ - y_;
    const double range = std::hypot(dx, dy);
    if (!std::isfinite(range) || range > max_follow_range_) {
      return;
    }

    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    TrackObservation track;
    track.id = -9001;
    track.x = c * dx + s * dy;
    track.y = -s * dx + c * dy;
    track.vx = c * smoothed_demo_vx_ + s * smoothed_demo_vy_;
    track.vy = -s * smoothed_demo_vx_ + c * smoothed_demo_vy_;
    track.speed = std::hypot(track.vx, track.vy);
    track.confidence = std::clamp(demo_target_confidence_, 0.0, 1.0);
    track.class_id = follow_class_id_;
    track.hits = 999;
    track.last_source = "demo_target_stream";
    track.frame_id = "base_link";
    track.stamp = now;
    demo_track_ = track;
    has_demo_track_ = true;
    last_demo_time_ = now;
    publish_flight_controller_goal(smoothed_demo_x_, smoothed_demo_y_);
  }

  void publish_flight_controller_goal(double target_world_x, double target_world_y)
  {
    if (!flight_controller_goal_pub_) {
      return;
    }
    const double dx = target_world_x - x_;
    const double dy = target_world_y - y_;
    const double range = std::hypot(dx, dy);
    double goal_x = target_world_x;
    double goal_y = target_world_y;
    const double standoff = flight_controller_standoff_m_ >= 0.0 ?
      flight_controller_standoff_m_ : desired_standoff_;
    if (std::isfinite(range) && range > std::max(standoff, 0.0) + 0.2 && standoff > 0.0) {
      goal_x = target_world_x - dx / range * standoff;
      goal_y = target_world_y - dy / range * standoff;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = get_clock()->now();
    goal.header.frame_id = "world";
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    goal.pose.position.z = waterline_z_;
    goal.pose.orientation = quaternion_from_euler(0.0, 0.0, std::atan2(dy, dx));
    flight_controller_goal_pub_->publish(goal);
  }

  static int cloud_field_offset(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
  {
    for (const auto & field : msg.fields) {
      if (field.name == name) {
        return static_cast<int>(field.offset);
      }
    }
    return -1;
  }

  static float read_cloud_float(const std::uint8_t * data, int offset)
  {
    float value = 0.0F;
    std::memcpy(&value, data + offset, sizeof(float));
    return value;
  }

  static std::size_t camera_sector(double x, double y)
  {
    // front, right, back, left in base_link coordinates.
    if (std::abs(x) >= std::abs(y)) {
      return x >= 0.0 ? 0U : 2U;
    }
    return y < 0.0 ? 1U : 3U;
  }

  void on_camera_detections(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg || msg->point_step == 0U) {
      return;
    }
    const int x_offset = cloud_field_offset(*msg, "x");
    const int y_offset = cloud_field_offset(*msg, "y");
    const int intensity_offset = cloud_field_offset(*msg, "intensity");
    if (x_offset < 0 || y_offset < 0 || intensity_offset < 0) {
      return;
    }
    const auto stamp = std::chrono::steady_clock::now();
    const std::size_t count = msg->data.size() / msg->point_step;
    for (std::size_t i = 0; i < count; ++i) {
      const auto * point = msg->data.data() + i * msg->point_step;
      const float x = read_cloud_float(point, x_offset);
      const float y = read_cloud_float(point, y_offset);
      const float confidence = read_cloud_float(point, intensity_offset);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(confidence) ||
        confidence < camera_sector_min_confidence_)
      {
        continue;
      }
      camera_sector_last_detection_[camera_sector(x, y)] = stamp;
    }
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
    }
    publish_uav_assist_request();

    double command_speed = 0.0;
    double target_bearing = 0.0;
    double commanded_bearing = 0.0;
    double aim_x = 0.0;
    double aim_y = 0.0;
    AvoidanceCommand avoidance;
    if (enabled_ && has_recent_target(now)) {
      const double previous_yaw = yaw_;
      const double actual_range = std::hypot(target_.x, target_.y);
      aim_x = target_.x + target_.vx * target_lead_time_;
      aim_y = target_.y + target_.vy * target_lead_time_;
      const double lead_distance = std::hypot(aim_x - target_.x, aim_y - target_.y);
      if (lead_distance > max_lead_distance_) {
        const double scale = max_lead_distance_ / std::max(lead_distance, 1e-3);
        aim_x = target_.x + (aim_x - target_.x) * scale;
        aim_y = target_.y + (aim_y - target_.y) * scale;
      }

      const double raw_target_bearing = std::atan2(aim_y, aim_x);
      if (!has_filtered_target_bearing_) {
        filtered_target_bearing_ = raw_target_bearing;
        has_filtered_target_bearing_ = true;
      } else {
        const double tau = std::max(0.05, target_bearing_filter_tau_s_);
        const double alpha = std::clamp(dt / (tau + dt), 0.0, 1.0);
        filtered_target_bearing_ = normalize_angle(
          filtered_target_bearing_ + alpha * normalize_angle(raw_target_bearing - filtered_target_bearing_));
      }
      target_bearing = filtered_target_bearing_;
      avoidance = compute_avoidance(tracks, target_.id);
      commanded_bearing = std::clamp(target_bearing + avoidance.bearing_offset, -1.35, 1.35);
      if (std::abs(commanded_bearing) < heading_deadband_rad_) {
        commanded_bearing = 0.0;
      }
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
      if (!has_fresh_target) {
        predict_stale_target_in_updated_body_frame(previous_yaw, command_speed, dt);
      }
    } else {
      has_filtered_target_bearing_ = false;
    }

    publish_tf();
    publish_status(tracks, target_bearing, commanded_bearing, command_speed, aim_x, aim_y, avoidance);
    if (enabled_ && std::chrono::duration<double>(now - start_time_).count() >= startup_delay_) {
      move_model();
    }
  }

  std::vector<TrackObservation> active_tracks(const std::chrono::steady_clock::time_point & now) const
  {
    const bool demo_target_is_fresh = demo_mode_ && has_demo_track_ &&
      std::chrono::duration<double>(now - last_demo_time_).count() <= track_status_timeout_;
    const bool c3_is_fresh = has_c3_detected_tracks_ &&
      std::chrono::duration<double>(now - last_c3_detected_time_).count() <= track_status_timeout_;
    const bool tracker_is_fresh = has_status_tracks_ &&
      std::chrono::duration<double>(now - last_status_time_).count() <= track_status_timeout_;
    const bool poses_are_fresh =
      std::chrono::duration<double>(now - last_pose_time_).count() <= track_status_timeout_;

    const auto has_semantic_follow_target = [this](const std::vector<TrackObservation> & tracks) {
        return std::any_of(tracks.begin(), tracks.end(), [this](const TrackObservation & track) {
          return is_valid_follow_candidate(track) && is_follow_class(track) &&
                 (has_target_ || !require_visual_source_for_acquisition_ ||
                 is_visual_confirmation_source(track));
        });
      };
    const auto has_valid_target = [this](const std::vector<TrackObservation> & tracks) {
        return std::any_of(tracks.begin(), tracks.end(), [this](const TrackObservation & track) {
          return is_valid_follow_candidate(track) &&
                 (has_target_ || !require_visual_source_for_acquisition_ ||
                 is_visual_confirmation_source(track));
        });
      };

    const auto metadata_body = tracks_to_body(metadata_tracks_);
    const auto c3_body = tracks_to_body(c3_detected_tracks_);
    const auto append_tracks = [](std::vector<TrackObservation> & out, const std::vector<TrackObservation> & in) {
        out.insert(out.end(), in.begin(), in.end());
      };

    if (demo_target_is_fresh) {
      std::vector<TrackObservation> fused_tracks{demo_track_};
      if (tracker_is_fresh) {
        append_tracks(fused_tracks, metadata_body);
      }
      if (c3_is_fresh) {
        append_tracks(fused_tracks, c3_body);
      }
      if (poses_are_fresh) {
        append_tracks(fused_tracks, pose_tracks_);
      }
      return fused_tracks;
    }

    // The tracker owns semantic association from camera detections. C3 provides
    // a fused spatial candidate and should still be available when tracker
    // metadata drops out.
    if (tracker_is_fresh && has_semantic_follow_target(metadata_body)) {
      return metadata_body;
    }
    if (tracker_is_fresh && has_valid_target(metadata_body)) {
      return metadata_body;
    }
    if (c3_is_fresh && has_semantic_follow_target(c3_body)) {
      return c3_body;
    }
    if (c3_is_fresh && has_valid_target(c3_body)) {
      return c3_body;
    }
    if (poses_are_fresh) {
      return pose_tracks_;
    }
    return {};
  }

  std::vector<TrackObservation> tracks_to_body(const std::vector<TrackObservation> & tracks) const
  {
    std::vector<TrackObservation> result;
    result.reserve(tracks.size());
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    for (auto track : tracks) {
      if (track.frame_id == "world") {
        const double dx = track.x - x_;
        const double dy = track.y - y_;
        const double vx = track.vx;
        const double vy = track.vy;
        track.x = c * dx + s * dy;
        track.y = -s * dx + c * dy;
        track.vx = c * vx + s * vy;
        track.vy = -s * vx + c * vy;
        track.speed = std::hypot(track.vx, track.vy);
        track.frame_id = "base_link";
      }
      result.push_back(track);
    }
    return result;
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

    const auto valid_for_acquisition = [this](const TrackObservation & track) {
        return is_valid_follow_candidate(track) &&
               (!require_visual_source_for_acquisition_ || is_visual_confirmation_source(track)) &&
               (!require_uav_source_for_initial_acquisition_ || has_target_ || is_uav_source(track));
      };
    bool has_class_candidate = false;
    for (const auto & track : tracks) {
      if (is_follow_class(track) && valid_for_acquisition(track)) {
        has_class_candidate = true;
        break;
      }
    }
    if (require_follow_class_for_acquisition_ && !has_class_candidate) {
      return std::nullopt;
    }

    double best_score = std::numeric_limits<double>::infinity();
    std::optional<TrackObservation> best;
    for (const auto & track : tracks) {
      if (!valid_for_acquisition(track)) {
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
    if (track.hits > 0 && track.hits < min_track_hits_ && track.last_source != "pose_array") {
      return false;
    }
    if (track.misses > max_follow_track_misses_) {
      return false;
    }
    if (!has_recent_camera_detection_for(track)) {
      return false;
    }
    return true;
  }

  bool has_recent_camera_detection_for(const TrackObservation & track) const
  {
    const double range = std::hypot(track.x, track.y);
    if (track.last_source.find("c3_detected") != std::string::npos ||
      track.last_source.find("demo_target_stream") != std::string::npos ||
      track.last_source.find("uav_") != std::string::npos ||
      (has_target_ && track.last_source == "ais"))
    {
      return true;
    }
    if (!require_camera_sector_confirmation_ || range > camera_sector_gate_range_m_) {
      return true;
    }
    const auto & last_seen = camera_sector_last_detection_[camera_sector(track.x, track.y)];
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_seen).count() <=
      camera_sector_timeout_s_;
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

  static bool is_visual_confirmation_source(const TrackObservation & track)
  {
    return track.last_source.find("camera") != std::string::npos ||
           track.last_source.find("demo_target_stream") != std::string::npos ||
           track.last_source.find("c3_detected") != std::string::npos ||
           track.last_source.find("c3_confirmation") != std::string::npos;
  }

  static bool is_uav_source(const TrackObservation & track)
  {
    return track.last_source.find("uav_") != std::string::npos ||
           track.last_source.find("demo_target_stream") != std::string::npos ||
           track.last_source.find("confirmation_uav") != std::string::npos;
  }

  void publish_uav_assist_request()
  {
    if (!has_target_ || target_.last_source.find("uav_") != std::string::npos ||
      target_.last_source.find("confirmation_uav") != std::string::npos)
    {
      return;
    }
    geometry_msgs::msg::PoseStamped request;
    request.header.stamp = get_clock()->now();
    request.header.frame_id = "world";
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    request.pose.position.x = x_ + c * target_.x - s * target_.y;
    request.pose.position.y = y_ + s * target_.x + c * target_.y;
    request.pose.position.z = 0.0;
    request.pose.orientation.w = 1.0;
    uav_assist_pub_->publish(request);
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
      if (obstacle.confidence < min_obstacle_confidence_ ||
        (obstacle.hits > 0 && obstacle.hits < min_obstacle_hits_))
      {
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
    return has_target_ && std::chrono::duration<double>(now - last_target_time_).count() <= target_timeout_ &&
      has_recent_camera_detection_for(target_);
  }

  void predict_stale_target_in_updated_body_frame(
    double previous_yaw, double usv_speed, double dt)
  {
    // Tracker points are in base_link. Predict in world coordinates while the
    // hull moves, then transform back instead of drifting in the old body frame.
    const double cos_old = std::cos(previous_yaw);
    const double sin_old = std::sin(previous_yaw);
    const double target_world_x = cos_old * target_.x - sin_old * target_.y;
    const double target_world_y = sin_old * target_.x + cos_old * target_.y;
    const double target_world_vx = cos_old * target_.vx - sin_old * target_.vy;
    const double target_world_vy = sin_old * target_.vx + cos_old * target_.vy;
    const double relative_world_x = target_world_x + target_world_vx * dt - usv_speed * std::cos(yaw_) * dt;
    const double relative_world_y = target_world_y + target_world_vy * dt - usv_speed * std::sin(yaw_) * dt;
    const double cos_new = std::cos(yaw_);
    const double sin_new = std::sin(yaw_);
    target_.x = cos_new * relative_world_x + sin_new * relative_world_y;
    target_.y = -sin_new * relative_world_x + cos_new * relative_world_y;
    target_.vx = cos_new * target_world_vx + sin_new * target_world_vy - usv_speed;
    target_.vy = -sin_new * target_world_vx + cos_new * target_world_vy;
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

  static std::vector<TrackObservation> parse_track_status(
    const std::string & text,
    const std::string & default_source = "unknown")
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
      track.last_source.clear();
      track.stamp = now;
      double id_value = -1.0;
      if (extract_double(block, "\"id\":", id_value) ||
        extract_double(block, "\"object_id\":", id_value))
      {
        track.id = static_cast<int>(std::round(id_value));
      }
      if (!extract_double(block, "\"x\":", track.x) ||
        !extract_double(block, "\"y\":", track.y))
      {
        pos = end + 1;
        continue;
      }
      extract_double(block, "\"vx\":", track.vx);
      extract_double(block, "\"vy\":", track.vy);
      extract_double(block, "\"speed\":", track.speed);
      extract_double(block, "\"confidence\":", track.confidence);
      extract_double(block, "\"class_id\":", track.class_id);
      double hits_value = 0.0;
      if (extract_double(block, "\"hits\":", hits_value) ||
        extract_double(block, "\"updates\":", hits_value))
      {
        track.hits = static_cast<int>(std::round(hits_value));
      }
      double misses_value = 0.0;
      if (extract_double(block, "\"misses\":", misses_value)) {
        track.misses = static_cast<int>(std::round(misses_value));
      }
      extract_string(block, "\"last_source\":\"", track.last_source);
      extract_string(block, "\"frame_id\":\"", track.frame_id);
      if (track.last_source.empty()) {
        track.last_source = default_source;
      }
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
  std::string c3_detected_topic_{"/c3/detected_objects"};
  std::string uav_assist_topic_{"/uav/assist_request"};
  std::string model_states_topic_{"/model_states"};
  std::string demo_target_model_name_{"moving_vessel"};
  std::string flight_controller_goal_topic_{"/usv/offboard_goal"};
  bool enabled_{true};
  bool warned_waiting_{false};
  bool has_target_{false};
  bool has_status_tracks_{false};
  bool has_c3_detected_tracks_{false};
  bool demo_mode_{false};
  bool has_demo_track_{false};
  bool has_demo_raw_target_{false};
  bool prefer_follow_class_{true};
  double update_rate_{15.0};
  double max_speed_{1.50};
  double max_yaw_rate_{0.70};
  double desired_standoff_{10.0};
  double target_timeout_{5.0};
  double track_status_timeout_{2.0};
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
  bool require_follow_class_for_acquisition_{true};
  bool require_visual_source_for_acquisition_{true};
  bool require_uav_source_for_initial_acquisition_{true};
  double turn_slowdown_gain_{0.35};
  double target_bearing_filter_tau_s_{1.2};
  double heading_deadband_rad_{0.06};
  double flight_controller_standoff_m_{-1.0};
  double demo_target_confidence_{0.98};
  double demo_target_smoothing_tau_s_{1.2};
  double smoothed_demo_x_{0.0};
  double smoothed_demo_y_{0.0};
  double smoothed_demo_vx_{0.0};
  double smoothed_demo_vy_{0.0};
  double follow_class_id_{5.0};
  std::vector<double> follow_class_ids_;
  double min_follow_confidence_{0.18};
  int min_track_hits_{2};
  int max_follow_track_misses_{8};
  bool require_camera_sector_confirmation_{true};
  double camera_sector_gate_range_m_{80.0};
  double camera_sector_timeout_s_{2.0};
  double camera_sector_min_confidence_{0.20};
  std::vector<std::string> camera_detection_topics_;
  double obstacle_lookahead_{34.0};
  double obstacle_lateral_window_{13.0};
  double obstacle_clearance_{5.5};
  double hard_stop_distance_{8.0};
  double hard_stop_lateral_{2.4};
  double avoidance_gain_{0.85};
  double obstacle_slowdown_gain_{0.38};
  double min_avoidance_speed_scale_{0.50};
  double centered_obstacle_bias_{1.0};
  double min_obstacle_confidence_{0.35};
  int min_obstacle_hits_{2};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double filtered_target_bearing_{0.0};
  bool has_filtered_target_bearing_{false};
  int locked_target_id_{-1};
  TrackObservation target_;
  TrackObservation demo_track_;
  std::vector<TrackObservation> metadata_tracks_;
  std::vector<TrackObservation> c3_detected_tracks_;
  std::vector<TrackObservation> pose_tracks_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_step_;
  std::chrono::steady_clock::time_point last_target_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_status_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_c3_detected_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_pose_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_demo_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_demo_raw_time_{std::chrono::steady_clock::now()};
  std::array<std::chrono::steady_clock::time_point, 4> camera_sector_last_detection_{};
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr c3_detected_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> camera_detection_subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr uav_assist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr flight_controller_goal_pub_;
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
