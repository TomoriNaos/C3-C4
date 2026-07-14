#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace usv_perception
{

struct EvalTrack
{
  int id{-1};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double confidence{0.0};
  double class_id{-1.0};
  int hits{0};
  std::string last_source{"unknown"};
};

struct GroundTruthObject
{
  std::string name;
  bool target{false};
  double rel_x{0.0};
  double rel_y{0.0};
  double rel_vx{0.0};
  double rel_vy{0.0};
  double range{0.0};
  double cpa{std::numeric_limits<double>::infinity()};
  double tcpa{0.0};
};

class TrackingEvaluator : public rclcpp::Node
{
public:
  TrackingEvaluator()
  : Node("tracking_evaluator")
  {
    model_states_topic_ = declare_parameter<std::string>("model_states_topic", "/model_states");
    track_status_topic_ = declare_parameter<std::string>("track_status_topic", "/tracked_objects_text");
    usv_model_name_ = declare_parameter<std::string>("usv_model_name", "wamv");
    target_model_names_ = declare_parameter<std::vector<std::string>>(
      "target_model_names",
      std::vector<std::string>{
        "moving_vessel",
        "small_fishing_boat",
        "survey_boat",
        "service_boat"});
    obstacle_model_names_ = declare_parameter<std::vector<std::string>>(
      "obstacle_model_names",
      std::vector<std::string>{
        "fishnet_buoy",
        "floating_obstacle",
        "drift_debris",
        "floating_container",
        "channel_buoy_north",
        "channel_buoy_south",
        "navigation_marker_port",
        "navigation_marker_starboard",
        "net_line_a"});
    match_gate_ = declare_parameter<double>("match_gate", 7.0);
    eval_range_ = declare_parameter<double>("eval_range", 140.0);
    track_timeout_ = declare_parameter<double>("track_timeout", 2.5);
    cpa_horizon_ = declare_parameter<double>("cpa_horizon", 120.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 2.0);

    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic_, 10, std::bind(&TrackingEvaluator::on_model_states, this, std::placeholders::_1));
    track_sub_ = create_subscription<std_msgs::msg::String>(
      track_status_topic_, 10, std::bind(&TrackingEvaluator::on_tracks, this, std::placeholders::_1));
    metrics_pub_ = create_publisher<std_msgs::msg::String>("tracking_metrics", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(publish_rate_, 0.1)));
    timer_ = create_wall_timer(period, std::bind(&TrackingEvaluator::on_timer, this));
  }

private:
  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    last_model_states_ = *msg;
    has_model_states_ = true;
  }

  void on_tracks(const std_msgs::msg::String::SharedPtr msg)
  {
    tracks_ = parse_track_status(msg->data);
    last_track_time_ = std::chrono::steady_clock::now();
    has_tracks_ = true;
  }

  void on_timer()
  {
    if (!has_model_states_) {
      return;
    }

    const auto usv_index = find_model(last_model_states_, usv_model_name_);
    if (usv_index < 0) {
      return;
    }

    const bool tracks_fresh = has_tracks_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_track_time_).count() <= track_timeout_;
    const std::vector<EvalTrack> active_tracks = tracks_fresh ? tracks_ : std::vector<EvalTrack>{};
    const auto objects = ground_truth_objects(static_cast<std::size_t>(usv_index));

    std::set<int> used_tracks;
    int frame_tp = 0;
    int frame_fn = 0;
    int frame_fp = 0;
    int frame_id_switches = 0;

    for (const auto & object : objects) {
      if (!object.target) {
        continue;
      }
      int best_index = -1;
      double best_distance = match_gate_;
      for (std::size_t track_index = 0; track_index < active_tracks.size(); ++track_index) {
        if (used_tracks.count(static_cast<int>(track_index)) != 0) {
          continue;
        }
        const auto & track = active_tracks[track_index];
        const double distance = std::hypot(track.x - object.rel_x, track.y - object.rel_y);
        if (distance < best_distance) {
          best_distance = distance;
          best_index = static_cast<int>(track_index);
        }
      }

      if (best_index >= 0) {
        ++frame_tp;
        used_tracks.insert(best_index);
        const int track_id = active_tracks[static_cast<std::size_t>(best_index)].id;
        auto it = last_target_track_ids_.find(object.name);
        if (it != last_target_track_ids_.end() && it->second != track_id) {
          ++frame_id_switches;
        }
        last_target_track_ids_[object.name] = track_id;
      } else {
        ++frame_fn;
      }
    }

    for (std::size_t track_index = 0; track_index < active_tracks.size(); ++track_index) {
      if (used_tracks.count(static_cast<int>(track_index)) != 0) {
        continue;
      }
      bool close_to_any_ground_truth = false;
      for (const auto & object : objects) {
        if (std::hypot(active_tracks[track_index].x - object.rel_x, active_tracks[track_index].y - object.rel_y) <
          match_gate_)
        {
          close_to_any_ground_truth = true;
          break;
        }
      }
      if (!close_to_any_ground_truth) {
        ++frame_fp;
      }
    }

    ++frames_;
    total_tp_ += frame_tp;
    total_fn_ += frame_fn;
    total_fp_ += frame_fp;
    total_id_switches_ += frame_id_switches;

    const auto risk = closest_risk(objects);
    publish_metrics(
      active_tracks, objects, frame_tp, frame_fn, frame_fp, frame_id_switches,
      risk.closest_name, risk.min_distance, risk.min_cpa, risk.min_tcpa);
  }

  std::vector<GroundTruthObject> ground_truth_objects(std::size_t usv_index) const
  {
    std::vector<GroundTruthObject> objects;
    const auto & usv_pose = last_model_states_.pose[usv_index];
    const auto & usv_twist = last_model_states_.twist[usv_index];
    const double usv_yaw = yaw_from_quaternion(usv_pose);
    const double cos_yaw = std::cos(usv_yaw);
    const double sin_yaw = std::sin(usv_yaw);

    for (std::size_t i = 0; i < last_model_states_.name.size() && i < last_model_states_.pose.size() &&
      i < last_model_states_.twist.size(); ++i)
    {
      const std::string & name = last_model_states_.name[i];
      const bool is_target = contains(target_model_names_, name);
      const bool is_obstacle = contains(obstacle_model_names_, name);
      if (!is_target && !is_obstacle) {
        continue;
      }

      const double dx = last_model_states_.pose[i].position.x - usv_pose.position.x;
      const double dy = last_model_states_.pose[i].position.y - usv_pose.position.y;
      const double rel_x = cos_yaw * dx + sin_yaw * dy;
      const double rel_y = -sin_yaw * dx + cos_yaw * dy;
      const double range = std::hypot(rel_x, rel_y);
      if (!std::isfinite(range) || range > eval_range_) {
        continue;
      }

      const double dvx = last_model_states_.twist[i].linear.x - usv_twist.linear.x;
      const double dvy = last_model_states_.twist[i].linear.y - usv_twist.linear.y;
      const double rel_vx = cos_yaw * dvx + sin_yaw * dvy;
      const double rel_vy = -sin_yaw * dvx + cos_yaw * dvy;

      GroundTruthObject object;
      object.name = name;
      object.target = is_target;
      object.rel_x = rel_x;
      object.rel_y = rel_y;
      object.rel_vx = rel_vx;
      object.rel_vy = rel_vy;
      object.range = range;
      const double v2 = rel_vx * rel_vx + rel_vy * rel_vy;
      object.tcpa = v2 > 1e-4 ? std::clamp(-(rel_x * rel_vx + rel_y * rel_vy) / v2, 0.0, cpa_horizon_) : 0.0;
      object.cpa = std::hypot(rel_x + rel_vx * object.tcpa, rel_y + rel_vy * object.tcpa);
      objects.push_back(object);
    }
    return objects;
  }

  struct RiskSummary
  {
    std::string closest_name{"none"};
    double min_distance{-1.0};
    double min_cpa{-1.0};
    double min_tcpa{-1.0};
  };

  static RiskSummary closest_risk(const std::vector<GroundTruthObject> & objects)
  {
    RiskSummary summary;
    double best_distance = std::numeric_limits<double>::infinity();
    double best_cpa = std::numeric_limits<double>::infinity();
    double best_tcpa = -1.0;
    std::string best_name = "none";
    for (const auto & object : objects) {
      if (object.range < best_distance) {
        best_distance = object.range;
        best_name = object.name;
      }
      if (object.cpa < best_cpa) {
        best_cpa = object.cpa;
        best_tcpa = object.tcpa;
      }
    }
    if (std::isfinite(best_distance)) {
      summary.closest_name = best_name;
      summary.min_distance = best_distance;
    }
    if (std::isfinite(best_cpa)) {
      summary.min_cpa = best_cpa;
      summary.min_tcpa = best_tcpa;
    }
    return summary;
  }

  void publish_metrics(
    const std::vector<EvalTrack> & active_tracks,
    const std::vector<GroundTruthObject> & objects,
    int frame_tp,
    int frame_fn,
    int frame_fp,
    int frame_id_switches,
    const std::string & closest_name,
    double min_distance,
    double min_cpa,
    double min_tcpa)
  {
    const int frame_truth_targets = static_cast<int>(std::count_if(
      objects.begin(), objects.end(), [](const GroundTruthObject & object) {return object.target;}));
    const int denominator_detection = std::max(1, frame_tp + frame_fn);
    const int denominator_false_positive = std::max(1, frame_tp + frame_fp);
    const double frame_miss_rate = static_cast<double>(frame_fn) / denominator_detection;
    const double frame_false_positive_rate = static_cast<double>(frame_fp) / denominator_false_positive;
    const double total_miss_rate = static_cast<double>(total_fn_) / std::max(1, total_tp_ + total_fn_);
    const double total_false_positive_rate = static_cast<double>(total_fp_) / std::max(1, total_tp_ + total_fp_);

    std::ostringstream status;
    status.setf(std::ios::fixed, std::ios::floatfield);
    status << std::setprecision(3)
           << "{\"frames\":" << frames_
           << ",\"tracks\":" << active_tracks.size()
           << ",\"ground_truth_objects\":" << objects.size()
           << ",\"ground_truth_targets\":" << frame_truth_targets
           << ",\"tp\":" << frame_tp
           << ",\"fp\":" << frame_fp
           << ",\"fn\":" << frame_fn
           << ",\"false_positive_rate\":" << frame_false_positive_rate
           << ",\"miss_rate\":" << frame_miss_rate
           << ",\"total_tp\":" << total_tp_
           << ",\"total_fp\":" << total_fp_
           << ",\"total_fn\":" << total_fn_
           << ",\"total_false_positive_rate\":" << total_false_positive_rate
           << ",\"total_miss_rate\":" << total_miss_rate
           << ",\"id_switches\":" << total_id_switches_
           << ",\"frame_id_switches\":" << frame_id_switches
           << ",\"closest_model\":\"" << closest_name << "\""
           << ",\"min_distance\":" << min_distance
           << ",\"min_cpa\":" << min_cpa
           << ",\"min_tcpa\":" << min_tcpa
           << "}";

    std_msgs::msg::String msg;
    msg.data = status.str();
    metrics_pub_->publish(msg);
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

  static bool contains(const std::vector<std::string> & values, const std::string & value)
  {
    return std::find(values.begin(), values.end(), value) != values.end();
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static std::vector<EvalTrack> parse_track_status(const std::string & text)
  {
    std::vector<EvalTrack> tracks;
    std::size_t pos = 0;
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
      EvalTrack track;
      double id_value = -1.0;
      if (extract_double(block, "\"id\":", id_value)) {
        track.id = static_cast<int>(std::round(id_value));
      }
      extract_double(block, "\"x\":", track.x);
      extract_double(block, "\"y\":", track.y);
      extract_double(block, "\"vx\":", track.vx);
      extract_double(block, "\"vy\":", track.vy);
      extract_double(block, "\"confidence\":", track.confidence);
      extract_double(block, "\"class_id\":", track.class_id);
      double hits_value = 0.0;
      if (extract_double(block, "\"hits\":", hits_value)) {
        track.hits = static_cast<int>(std::round(hits_value));
      }
      extract_string(block, "\"last_source\":\"", track.last_source);
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

  std::string model_states_topic_{"/model_states"};
  std::string track_status_topic_{"/tracked_objects_text"};
  std::string usv_model_name_{"wamv"};
  std::vector<std::string> target_model_names_;
  std::vector<std::string> obstacle_model_names_;
  double match_gate_{7.0};
  double eval_range_{140.0};
  double track_timeout_{2.5};
  double cpa_horizon_{120.0};
  double publish_rate_{2.0};
  bool has_model_states_{false};
  bool has_tracks_{false};
  gazebo_msgs::msg::ModelStates last_model_states_;
  std::vector<EvalTrack> tracks_;
  std::map<std::string, int> last_target_track_ids_;
  int frames_{0};
  int total_tp_{0};
  int total_fp_{0};
  int total_fn_{0};
  int total_id_switches_{0};
  std::chrono::steady_clock::time_point last_track_time_{std::chrono::steady_clock::now()};
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr track_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::TrackingEvaluator>());
  rclcpp::shutdown();
  return 0;
}
