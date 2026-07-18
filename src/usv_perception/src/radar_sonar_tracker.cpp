#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "c3_sonar_driver/msg/sonar_detect.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "usv_perception/common.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace usv_perception
{

struct Detection
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double confidence{0.0};
  double class_id{-1.0};
  double aspect_ratio{1.0};
  double range_rate{0.0};
  double vx{0.0};
  double vy{0.0};
  std::int64_t ais_mmsi{-1};
  std::string source;
};

struct Track
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double covariance_m2{16.0};
  int hits{1};
  int misses{0};
  double confidence{0.0};
  double class_id{-1.0};
  double avg_intensity{0.0};
  double bbox_aspect_ratio{1.0};
  double range_rate{0.0};
  std::int64_t ais_mmsi{-1};
  std::string last_source;
  std::string state{"候选"};
  std::chrono::steady_clock::time_point last_update{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_predict{std::chrono::steady_clock::now()};
  std::set<std::string> sources;

  void predict(const std::chrono::steady_clock::time_point & now)
  {
    const double raw_dt = std::chrono::duration<double>(now - last_predict).count();
    const double dt = std::clamp(raw_dt, 0.0, 0.5);
    x += vx * dt;
    y += vy * dt;
    covariance_m2 = std::clamp(
      covariance_m2 + 1.8 * dt + 0.35 * static_cast<double>(misses), 1.0, 100.0);
    last_predict = now;
  }

  void update(const Detection & det, const std::chrono::steady_clock::time_point & now)
  {
    const double alpha = std::clamp(0.25 + 0.45 * det.confidence, 0.25, 0.70);
    const double beta = std::clamp(0.08 + 0.22 * det.confidence, 0.08, 0.30);
    const double raw_dt = std::chrono::duration<double>(now - last_update).count();
    const double dt = std::clamp(raw_dt, 0.03, 0.5);
    const double residual_x = det.x - x;
    const double residual_y = det.y - y;
    x += alpha * residual_x;
    y += alpha * residual_y;
    vx += beta * residual_x / dt;
    vy += beta * residual_y / dt;
    if (det.source == "ais") {
      vx = 0.65 * vx + 0.35 * det.vx;
      vy = 0.65 * vy + 0.35 * det.vy;
      if (det.ais_mmsi >= 0) {
        ais_mmsi = det.ais_mmsi;
      }
    }
    covariance_m2 = std::clamp((1.0 - alpha) * covariance_m2 + 1.0, 0.8, 80.0);
    ++hits;
    misses = 0;
    confidence = std::max(confidence * 0.92, det.confidence);
    const bool semantic_detection = det.class_id >= 0.0;
    const bool preserve_visual_source = det.source == "ais" &&
      (last_source.find("camera") != std::string::npos || last_source.find("yolo") != std::string::npos);
    if (semantic_detection) {
      class_id = det.class_id;
      if (!preserve_visual_source) {
        last_source = det.source;
      }
    } else if (class_id < 0.0) {
      last_source = det.source;
    }
    avg_intensity = avg_intensity <= 0.0 ? det.confidence : 0.75 * avg_intensity + 0.25 * det.confidence;
    bbox_aspect_ratio = 0.80 * bbox_aspect_ratio + 0.20 * std::clamp(det.aspect_ratio, 0.2, 5.0);
    range_rate = 0.80 * range_rate + 0.20 * det.range_rate;
    state = hits >= 2 ? "确认" : "候选";
    last_update = now;
    last_predict = now;
    sources.insert(det.source);
  }
};

class RadarSonarTracker : public rclcpp::Node
{
public:
  RadarSonarTracker()
  : Node("radar_sonar_tracker")
  {
    const std::string radar_topic = declare_parameter<std::string>("radar_topic", "/mmwave_radar/scan");
    sonar_scan_topics_ = declare_parameter<std::vector<std::string>>(
      "sonar_scan_topics", std::vector<std::string>{});
    sonar_scan_yaws_ = declare_parameter<std::vector<double>>("sonar_scan_yaws", std::vector<double>{0.0});
    const std::string legacy_sonar_topic = declare_parameter<std::string>("sonar_topic", "");
    sonar_detect_topic_ = declare_parameter<std::string>("sonar_detect_topic", "/sonar/detect");
    if (sonar_scan_topics_.empty() && !legacy_sonar_topic.empty()) {
      sonar_scan_topics_.push_back(legacy_sonar_topic);
    }
    while (sonar_scan_yaws_.size() < sonar_scan_topics_.size()) {
      sonar_scan_yaws_.push_back(0.0);
    }
    const std::string gated_points_topic =
      declare_parameter<std::string>("gated_points_topic", "/gated_camera/detection_points");
    const std::string uav_points_topic =
      declare_parameter<std::string>("uav_points_topic", "/uav/gated_camera/detection_points");
    const auto gated_points_topics = declare_parameter<std::vector<std::string>>(
      "gated_points_topics", std::vector<std::string>{});
    const auto uav_points_topics = declare_parameter<std::vector<std::string>>(
      "uav_points_topics", std::vector<std::string>{});
    const auto uav_depth_points_topics = declare_parameter<std::vector<std::string>>(
      "uav_depth_points_topics", std::vector<std::string>{});
    const std::string uav_observation_active_topic = declare_parameter<std::string>(
      "uav_observation_active_topic", "/uav/observation/active");
    require_uav_observation_active_ =
      declare_parameter<bool>("require_uav_observation_active", true);
    const std::string pseudocolor_gated_points_topic =
      declare_parameter<std::string>(
        "pseudocolor_gated_points_topic", "/gated_camera/pseudocolor/detection_points");
    const std::string stf_gated_points_topic =
      declare_parameter<std::string>("stf_gated_points_topic", "/gated_camera/stf_detection_points");
    const std::string bev_gated_points_topic =
      declare_parameter<std::string>("bev_gated_points_topic", "/gated_camera/bev_detection_points");
    const std::string ais_topic = declare_parameter<std::string>("ais_topic", "/ais/targets");
    const std::string c3_confirmed_topic =
      declare_parameter<std::string>("c3_confirmed_topic", "/c3/detected_objects");
    const std::string model_states_topic = declare_parameter<std::string>("model_states_topic", "/model_states");
    usv_model_name_ = declare_parameter<std::string>("usv_model_name", "wamv");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    radar_x_offset_ = declare_parameter<double>("radar_x_offset", -0.35);
    radar_y_offset_ = declare_parameter<double>("radar_y_offset", 0.0);
    sonar_x_offset_ = declare_parameter<double>("sonar_x_offset", 0.65);
    sonar_window_frames_ = declare_parameter<int>("sonar_fusion_window_frames", 3);
    sonar_min_points_for_valid_detect_ = declare_parameter<int>("sonar_min_points_for_valid_detect", 20);
    sonar_default_confidence_ = declare_parameter<double>("sonar_default_confidence", 0.65);
    sonar_detect_timeout_ = declare_parameter<double>("sonar_detect_timeout_s", 2.0);
    cluster_gap_ = declare_parameter<double>("cluster_gap", 0.75);
    min_cluster_points_ = declare_parameter<int>("min_cluster_points", 2);
    max_tracking_range_ = declare_parameter<double>("max_tracking_range", 80.0);
    association_gate_ = declare_parameter<double>("association_gate", 2.8);
    track_timeout_ = declare_parameter<double>("track_timeout", 2.0);
    camera_fusion_timeout_ = declare_parameter<double>("camera_fusion_timeout", 0.8);
    uav_fusion_timeout_ = declare_parameter<double>("uav_fusion_timeout", 1.2);
    min_camera_confidence_ = declare_parameter<double>("min_camera_confidence", 0.30);
    min_uav_camera_confidence_ = declare_parameter<double>("min_uav_camera_confidence", 0.15);
    camera_detection_cluster_radius_ =
      declare_parameter<double>("camera_detection_cluster_radius", 4.0);
    ais_confidence_ = declare_parameter<double>("ais_confidence", 0.88);
    min_ais_confidence_ = declare_parameter<double>("min_ais_confidence", 0.45);
    ais_can_create_tracks_ = declare_parameter<bool>("ais_can_create_tracks", false);
    ais_can_create_target_tracks_ =
      declare_parameter<bool>("ais_can_create_target_tracks", true);
    ais_target_class_ids_ = declare_parameter<std::vector<double>>(
      "ais_target_class_ids", std::vector<double>{0.0});
    ais_maintenance_gate_m_ = declare_parameter<double>("ais_maintenance_gate_m", 25.0);
    max_tracks_per_class_ = declare_parameter<int>("max_tracks_per_class", 2);
    max_track_misses_ = declare_parameter<int>("max_track_misses", 400);
    class_replacement_margin_ = declare_parameter<double>("class_replacement_margin", 0.05);
    mahalanobis_gate_ = declare_parameter<double>("mahalanobis_gate", 3.0);
    velocity_gate_mps_ = declare_parameter<double>("velocity_gate_mps", 1.0);
    feature_similarity_gate_ = declare_parameter<double>("feature_similarity_gate", 0.70);
    tracker_frame_id_ = declare_parameter<std::string>("tracker_frame_id", "world");

    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic, 10,
      std::bind(&RadarSonarTracker::on_model_states, this, std::placeholders::_1));

    radar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      radar_topic, rclcpp::SensorDataQoS(),
      std::bind(&RadarSonarTracker::on_radar, this, std::placeholders::_1));
    sonar_sector_windows_.resize(sonar_scan_topics_.size());
    for (std::size_t i = 0; i < sonar_scan_topics_.size(); ++i) {
      const double mount_yaw = sonar_scan_yaws_[i];
      sonar_subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
        sonar_scan_topics_[i], rclcpp::SensorDataQoS(),
        [this, mount_yaw, i](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          on_sonar_scan(msg, mount_yaw, i);
        }));
    }
    if (!sonar_detect_topic_.empty()) {
      sonar_detect_sub_ = create_subscription<c3_sonar_driver::msg::SonarDetect>(
        sonar_detect_topic_, 10,
        std::bind(&RadarSonarTracker::on_sonar_detect, this, std::placeholders::_1));
    }
    const auto subscribe_gated = [this](const std::string & topic) {
        if (!topic.empty()) {
          gated_points_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
              on_detection_points(msg, "gated_camera", 0.82);
            }));
        }
      };
    const auto subscribe_uav = [this](const std::string & topic) {
        if (!topic.empty()) {
          uav_points_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
              if (!require_uav_observation_active_ || uav_observation_active_) {
                on_detection_points(msg, "uav_gated_camera", 0.72);
              }
            }));
        }
      };
    if (gated_points_topics.empty()) {
      subscribe_gated(gated_points_topic);
    } else {
      for (const auto & topic : gated_points_topics) {
        subscribe_gated(topic);
      }
    }
    if (uav_points_topics.empty()) {
      subscribe_uav(uav_points_topic);
    } else {
      for (const auto & topic : uav_points_topics) {
        subscribe_uav(topic);
      }
    }
    for (const auto & topic : uav_depth_points_topics) {
      if (!topic.empty()) {
        uav_points_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
          topic, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            if (!require_uav_observation_active_ || uav_observation_active_) {
              on_detection_points(msg, "uav_depth_camera", 0.90);
            }
          }));
      }
    }
    uav_observation_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      uav_observation_active_topic, 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        uav_observation_active_ = msg && msg->data;
      });
    if (!pseudocolor_gated_points_topic.empty()) {
      pseudocolor_gated_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pseudocolor_gated_points_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          on_detection_points(msg, "pseudocolor_gated_yolo", 0.78);
        });
    }
    if (!stf_gated_points_topic.empty()) {
      stf_gated_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        stf_gated_points_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          on_detection_points(msg, "stf_gated_slices", 0.68);
        });
    }
    if (!bev_gated_points_topic.empty()) {
      bev_gated_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        bev_gated_points_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          on_detection_points(msg, "gated_bev", 0.63);
        });
    }
    ais_sub_ = create_subscription<std_msgs::msg::String>(
      ais_topic, 10, std::bind(&RadarSonarTracker::on_ais_targets, this, std::placeholders::_1));
    c3_confirmed_sub_ = create_subscription<std_msgs::msg::String>(
      c3_confirmed_topic, 10,
      std::bind(&RadarSonarTracker::on_c3_confirmed, this, std::placeholders::_1));

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("tracked_objects", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("tracked_object_poses", 10);
    text_pub_ = create_publisher<std_msgs::msg::String>("tracked_objects_text", 10);
    RCLCPP_INFO(
      get_logger(), "Tracking radar=%s sonar_detect=%s sonar_sectors=%zu gated=%s uav=%s pseudocolor=%s stf_gated=%s bev_gated=%s ais=%s",
      radar_topic.c_str(), sonar_detect_topic_.empty() ? "off" : sonar_detect_topic_.c_str(), sonar_scan_topics_.size(),
      gated_points_subs_.size() == 1 ? gated_points_topic.c_str() : "multi", uav_points_subs_.size() == 1 ? uav_points_topic.c_str() : "multi",
      pseudocolor_gated_points_topic.empty() ? "off" : pseudocolor_gated_points_topic.c_str(),
      stf_gated_points_topic.empty() ? "off" : stf_gated_points_topic.c_str(),
      bev_gated_points_topic.empty() ? "off" : bev_gated_points_topic.c_str(), ais_topic.c_str());
  }

private:
  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    for (std::size_t i = 0; i < msg->name.size() && i < msg->pose.size(); ++i) {
      if (msg->name[i] != usv_model_name_) {
        continue;
      }
      usv_x_ = msg->pose[i].position.x;
      usv_y_ = msg->pose[i].position.y;
      usv_yaw_ = yaw_from_quaternion(msg->pose[i]);
      has_usv_pose_ = true;
      return;
    }
  }

  void on_sonar_detect(const c3_sonar_driver::msg::SonarDetect::SharedPtr msg)
  {
    (void)msg;
    return;
    if (!msg || msg->detect_id == last_sonar_detect_id_ || !std::isfinite(msg->confidence) || msg->confidence <= 0.0F) {
      return;
    }
    Detection detection;
    detection.x = msg->position.x;
    detection.y = msg->position.y;
    detection.z = msg->position.z;
    if (std::hypot(detection.x, detection.y) < 1e-4 && std::isfinite(msg->range_m)) {
      detection.x = sonar_x_offset_ + msg->range_m * std::cos(msg->bearing_rad);
      detection.y = msg->range_m * std::sin(msg->bearing_rad);
    }
    detection.confidence = std::clamp(static_cast<double>(msg->confidence), 0.0, 0.98);
    detection.source = "c3_sonar";
    last_sonar_detect_id_ = msg->detect_id;
    const auto now = std::chrono::steady_clock::now();
    update_tracks({detection}, now);
    publish_tracks();
  }

  void on_sonar_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg,
    double mount_yaw,
    std::size_t sector_index)
  {
    (void)msg;
    (void)mount_yaw;
    (void)sector_index;
    return;
    if (!msg) {
      return;
    }
    std::vector<std::pair<double, double>> frame_points;
    frame_points.reserve(msg->ranges.size());
    for (std::size_t index = 0; index < msg->ranges.size(); ++index) {
      const double range_value = msg->ranges[index];
      if (!std::isfinite(range_value) || range_value < msg->range_min || range_value > msg->range_max) {
        continue;
      }
      const double angle = msg->angle_min + static_cast<double>(index) * msg->angle_increment + mount_yaw;
      frame_points.emplace_back(
        sonar_x_offset_ + range_value * std::cos(angle),
        range_value * std::sin(angle));
    }

    if (frame_points.empty()) {
      return;
    }

    if (sector_index >= sonar_sector_windows_.size()) {
      sonar_sector_windows_.resize(sector_index + 1);
    }
    auto & sector_window = sonar_sector_windows_[sector_index];
    sector_window.push_back(frame_points);
    const std::size_t max_window = static_cast<std::size_t>(std::max(1, sonar_window_frames_));
    while (sector_window.size() > max_window) {
      sector_window.pop_front();
    }

    std::vector<std::pair<double, double>> fused_points;
    for (const auto & sector_history : sonar_sector_windows_) {
      for (const auto & history_frame : sector_history) {
        fused_points.insert(fused_points.end(), history_frame.begin(), history_frame.end());
      }
    }
    if (static_cast<int>(fused_points.size()) < sonar_min_points_for_valid_detect_) {
      return;
    }

    auto detections = cluster_points(
      fused_points, "sonar", std::clamp(sonar_default_confidence_, 0.0, 0.98));
    if (!detections.empty()) {
      last_sonar_detections_ = detections;
      last_sonar_time_ = std::chrono::steady_clock::now();
      has_sonar_detection_ = true;
    }
  }

  void on_radar(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    (void)msg;
    return;
    auto detections = cluster_radar_scan(*msg);
    const auto now = std::chrono::steady_clock::now();
    if (has_sonar_detection_ && std::chrono::duration<double>(now - last_sonar_time_).count() < sonar_detect_timeout_) {
      detections.insert(detections.end(), last_sonar_detections_.begin(), last_sonar_detections_.end());
    }
    update_tracks(detections, now);
    publish_tracks();
  }

  std::vector<Detection> cluster_points(
    const std::vector<std::pair<double, double>> & points,
    const std::string & source,
    double base_confidence) const
  {
    std::vector<std::vector<std::pair<double, double>>> clusters;
    std::vector<std::pair<double, double>> current;
    std::pair<double, double> previous;
    bool has_previous = false;
    for (const auto & point : points) {
      if (has_previous) {
        const double gap = std::hypot(point.first - previous.first, point.second - previous.second);
        if (gap > cluster_gap_) {
          finish_cluster(clusters, current);
          current.clear();
        }
      }
      current.push_back(point);
      previous = point;
      has_previous = true;
    }
    finish_cluster(clusters, current);

    std::vector<Detection> detections;
    for (const auto & cluster : clusters) {
      if (static_cast<int>(cluster.size()) < min_cluster_points_) {
        continue;
      }
      double sum_x = 0.0;
      double sum_y = 0.0;
      double min_x = std::numeric_limits<double>::infinity();
      double min_y = std::numeric_limits<double>::infinity();
      double max_x = -std::numeric_limits<double>::infinity();
      double max_y = -std::numeric_limits<double>::infinity();
      for (const auto & point : cluster) {
        sum_x += point.first;
        sum_y += point.second;
        min_x = std::min(min_x, point.first);
        min_y = std::min(min_y, point.second);
        max_x = std::max(max_x, point.first);
        max_y = std::max(max_y, point.second);
      }
      const double count = static_cast<double>(cluster.size());
      const double spread = std::hypot(max_x - min_x, max_y - min_y);
      const double confidence = std::clamp(base_confidence + 0.02 * spread + 0.01 * count, 0.35, 0.95);
      Detection detection;
      detection.x = sum_x / count;
      detection.y = sum_y / count;
      detection.confidence = confidence;
      detection.source = source;
      detections.push_back(detection);
    }
    return detections;
  }

  void on_detection_points(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const std::string & source,
    double source_weight)
  {
    if (!is_yolo_source(source)) {
      return;
    }
    auto detections = parse_detection_cloud(*msg, source, source_weight);
    if (detections.empty()) {
      return;
    }
    for (auto & detection : detections) {
      detection = detection_to_world(detection);
    }
    const auto now = std::chrono::steady_clock::now();
    update_tracks(detections, now);
    publish_tracks();
  }

  void on_ais_targets(const std_msgs::msg::String::SharedPtr msg)
  {
    auto detections = parse_ais_targets(msg->data);
    if (detections.empty()) {
      return;
    }
    for (auto & detection : detections) {
      detection = detection_to_world(detection);
    }
    const auto now = std::chrono::steady_clock::now();
    update_tracks(detections, now);
    publish_tracks();
  }

  void on_c3_confirmed(const std_msgs::msg::String::SharedPtr msg)
  {
    (void)msg;
    return;
    if (!msg) {
      return;
    }
    const auto detections = parse_c3_confirmed_targets(msg->data);
    if (detections.empty()) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    update_tracks(detections, now);
    publish_tracks();
  }

  std::vector<Detection> cluster_radar_scan(const sensor_msgs::msg::LaserScan & msg) const
  {
    std::vector<std::vector<std::pair<double, double>>> clusters;
    std::vector<std::pair<double, double>> current;
    std::pair<double, double> previous;
    bool has_previous = false;

    for (std::size_t index = 0; index < msg.ranges.size(); ++index) {
      const double range_value = msg.ranges[index];
      const double angle = msg.angle_min + static_cast<double>(index) * msg.angle_increment;
      if (!std::isfinite(range_value) || range_value < msg.range_min || range_value > msg.range_max ||
        range_value > max_tracking_range_)
      {
        finish_cluster(clusters, current);
        current.clear();
        has_previous = false;
        continue;
      }

      const double x = radar_x_offset_ + range_value * std::cos(angle);
      const double y = radar_y_offset_ + range_value * std::sin(angle);
      if (x < -1.0) {
        continue;
      }

      const std::pair<double, double> point{x, y};
      if (has_previous) {
        const double gap = std::hypot(point.first - previous.first, point.second - previous.second);
        if (gap > cluster_gap_) {
          finish_cluster(clusters, current);
          current.clear();
        }
      }
      current.push_back(point);
      previous = point;
      has_previous = true;
    }
    finish_cluster(clusters, current);

    std::vector<Detection> detections;
    for (const auto & cluster : clusters) {
      if (static_cast<int>(cluster.size()) < min_cluster_points_) {
        continue;
      }
      double sum_x = 0.0;
      double sum_y = 0.0;
      double min_x = std::numeric_limits<double>::infinity();
      double min_y = std::numeric_limits<double>::infinity();
      double max_x = -std::numeric_limits<double>::infinity();
      double max_y = -std::numeric_limits<double>::infinity();
      for (const auto & point : cluster) {
        sum_x += point.first;
        sum_y += point.second;
        min_x = std::min(min_x, point.first);
        min_y = std::min(min_y, point.second);
        max_x = std::max(max_x, point.first);
        max_y = std::max(max_y, point.second);
      }
      const double count = static_cast<double>(cluster.size());
      const double spread = std::hypot(max_x - min_x, max_y - min_y);
      const double confidence = std::clamp(0.45 + 0.08 * count + 0.08 * spread, 0.45, 0.95);
      Detection detection;
      detection.x = sum_x / count;
      detection.y = sum_y / count;
      detection.confidence = confidence;
      detection.source = "mmwave_radar";
      detections.push_back(detection);
    }
    return detections;
  }

  std::vector<Detection> parse_detection_cloud(
    const sensor_msgs::msg::PointCloud2 & msg,
    const std::string & source,
    double source_weight) const
  {
    const auto offset_x = field_offset(msg, "x");
    const auto offset_y = field_offset(msg, "y");
    const auto offset_z = field_offset(msg, "z");
    const auto offset_intensity = field_offset(msg, "intensity");
    const auto offset_class_id = field_offset(msg, "class_id");
    if (offset_x < 0 || offset_y < 0 || msg.point_step < 8) {
      return {};
    }

    std::vector<Detection> detections;
    const std::size_t point_count = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
    detections.reserve(point_count);
    for (std::size_t index = 0; index < point_count; ++index) {
      const auto * base = msg.data.data() + index * msg.point_step;
      const double x = read_float(base, offset_x);
      const double y = read_float(base, offset_y);
      const double z = offset_z >= 0 ? read_float(base, offset_z) : 0.0;
      if (!std::isfinite(x) || !std::isfinite(y) || x < -2.0 ||
        std::hypot(x, y) > max_tracking_range_)
      {
        continue;
      }

      const double raw_confidence = offset_intensity >= 0 ? read_float(base, offset_intensity) : 0.55;
      const bool from_uav = source.rfind("uav_", 0) == 0;
      const double minimum_confidence = from_uav ? min_uav_camera_confidence_ : min_camera_confidence_;
      if (!std::isfinite(raw_confidence) || raw_confidence < minimum_confidence) {
        continue;
      }
      const double confidence = std::clamp(raw_confidence * source_weight, 0.0, 0.98);
      const double class_id = offset_class_id >= 0 ? read_float(base, offset_class_id) : -1.0;
      auto match = std::find_if(detections.begin(), detections.end(), [&](const Detection & detection) {
          return std::abs(detection.class_id - class_id) < 0.5 &&
                 std::hypot(detection.x - x, detection.y - y) <= camera_detection_cluster_radius_;
        });
      if (match == detections.end()) {
        Detection detection;
        detection.x = x;
        detection.y = y;
        detection.z = z;
        detection.confidence = confidence;
        detection.class_id = class_id;
        detection.source = source;
        detections.push_back(detection);
      } else {
        const double old_weight = std::max(0.05, match->confidence);
        const double new_weight = std::max(0.05, confidence);
        match->x = (old_weight * match->x + new_weight * x) / (old_weight + new_weight);
        match->y = (old_weight * match->y + new_weight * y) / (old_weight + new_weight);
        match->z = (old_weight * match->z + new_weight * z) / (old_weight + new_weight);
        match->confidence = std::max(match->confidence, confidence);
      }
    }
    return detections;
  }

  static bool is_yolo_source(const std::string & source)
  {
    return source.find("camera") != std::string::npos ||
           source.find("yolo") != std::string::npos;
  }

  Detection detection_to_world(const Detection & detection) const
  {
    if (!has_usv_pose_ || tracker_frame_id_ != "world") {
      return detection;
    }
    Detection out = detection;
    const double c = std::cos(usv_yaw_);
    const double s = std::sin(usv_yaw_);
    out.x = usv_x_ + c * detection.x - s * detection.y;
    out.y = usv_y_ + s * detection.x + c * detection.y;
    out.vx = c * detection.vx - s * detection.vy;
    out.vy = s * detection.vx + c * detection.vy;
    return out;
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static int field_offset(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
  {
    for (const auto & field : msg.fields) {
      if (field.name == name) {
        return static_cast<int>(field.offset);
      }
    }
    return -1;
  }

  static float read_float(const unsigned char * data, int offset)
  {
    float value = 0.0F;
    std::memcpy(&value, data + offset, sizeof(float));
    return value;
  }

  std::vector<Detection> parse_ais_targets(const std::string & text) const
  {
    std::vector<Detection> detections;
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
      Detection det;
      if (!extract_double(block, "\"x\":", det.x) || !extract_double(block, "\"y\":", det.y)) {
        pos = end + 1;
        continue;
      }
      extract_double(block, "\"class_id\":", det.class_id);
      double mmsi = -1.0;
      if (extract_double(block, "\"mmsi\":", mmsi) && std::isfinite(mmsi)) {
        det.ais_mmsi = static_cast<std::int64_t>(std::llround(mmsi));
      }
      extract_double(block, "\"vx\":", det.vx);
      extract_double(block, "\"vy\":", det.vy);
      det.confidence = ais_confidence_;
      extract_double(block, "\"confidence\":", det.confidence);
      det.confidence = std::clamp(det.confidence, min_ais_confidence_, 0.98);
      det.source = "ais";
      if (std::isfinite(det.x) && std::isfinite(det.y) && det.x > -5.0 &&
        std::hypot(det.x, det.y) <= max_tracking_range_)
      {
        detections.push_back(det);
      }
      pos = end + 1;
    }
    return detections;
  }

  std::vector<Detection> parse_c3_confirmed_targets(const std::string & text) const
  {
    std::vector<Detection> detections;
    std::size_t pos = 0;
    while (true) {
      const auto start = text.find("{\"object_id\":", pos);
      if (start == std::string::npos) {
        break;
      }
      const auto end = text.find('}', start);
      if (end == std::string::npos) {
        break;
      }
      const std::string block = text.substr(start, end - start + 1);
      Detection det;
      if (!extract_double(block, "\"x\":", det.x) || !extract_double(block, "\"y\":", det.y) ||
        !extract_double(block, "\"class_id\":", det.class_id) ||
        !extract_double(block, "\"confidence\":", det.confidence))
      {
        pos = end + 1;
        continue;
      }
      extract_double(block, "\"z\":", det.z);
      std::string source;
      extract_string(block, "\"last_source\":\"", source);
      det.source = source.find("uav_") != std::string::npos ?
        "c3_confirmation_uav" : "c3_confirmation_main";
      det.confidence = std::clamp(det.confidence, 0.0, 0.98);
      if (std::isfinite(det.x) && std::isfinite(det.y) && det.class_id >= 0.0 &&
        std::hypot(det.x, det.y) <= max_tracking_range_)
      {
        detections.push_back(det);
      }
      pos = end + 1;
    }
    return detections;
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

  void finish_cluster(
    std::vector<std::vector<std::pair<double, double>>> & clusters,
    const std::vector<std::pair<double, double>> & cluster) const
  {
    if (static_cast<int>(cluster.size()) >= min_cluster_points_) {
      clusters.push_back(cluster);
    }
  }

  void update_tracks(const std::vector<Detection> & detections, const std::chrono::steady_clock::time_point & now)
  {
    for (auto & track : tracks_) {
      track.predict(now);
      ++track.misses;
    }
    prune_stale_tracks(now);

    std::vector<Detection> unmatched = detections;
    for (auto & track : tracks_) {
      int best_index = -1;
      double best_score = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < unmatched.size(); ++index) {
        const auto & det = unmatched[index];
        if (!can_associate(track, det)) {
          continue;
        }
        const double score = association_score(track, det);
        if (score < best_score) {
          best_score = score;
          best_index = static_cast<int>(index);
        }
      }
      if (best_index >= 0) {
        track.update(unmatched[static_cast<std::size_t>(best_index)], now);
        unmatched.erase(unmatched.begin() + best_index);
      }
    }

    for (const auto & det : unmatched) {
      const bool ais_seed = det.source == "ais" && can_ais_create_track(det);
      if ((!is_yolo_source(det.source) && !ais_seed) || det.class_id < 0.0) {
        continue;
      }
      if (active_track_count_for_class(det.class_id) >= max_tracks_per_class_ &&
        !replace_weaker_track_for_class(det, now))
      {
        continue;
      }
      Track track;
      track.id = next_track_id_++;
      track.x = det.x;
      track.y = det.y;
      track.vx = det.vx;
      track.vy = det.vy;
      track.confidence = det.confidence;
      track.class_id = det.class_id;
      track.avg_intensity = det.confidence;
      track.bbox_aspect_ratio = std::clamp(det.aspect_ratio, 0.2, 5.0);
      track.range_rate = det.range_rate;
      track.ais_mmsi = det.ais_mmsi;
      track.last_source = det.source;
      track.last_update = now;
      track.last_predict = now;
      track.sources.insert(det.source);
      tracks_.push_back(track);
    }

    prune_stale_tracks(now);
  }

  void prune_stale_tracks(const std::chrono::steady_clock::time_point & now)
  {
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [&](const Track & track) {
          return std::chrono::duration<double>(now - track.last_update).count() >= track_timeout_ ||
                 track.misses >= max_track_misses_;
        }),
      tracks_.end());
  }

  bool replace_weaker_track_for_class(
    const Detection & det,
    const std::chrono::steady_clock::time_point & now)
  {
    int weakest_index = -1;
    double weakest_confidence = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      const auto & track = tracks_[i];
      if (track.class_id < 0.0 || std::abs(track.class_id - det.class_id) >= 0.5) {
        continue;
      }
      if (track.confidence < weakest_confidence) {
        weakest_confidence = track.confidence;
        weakest_index = static_cast<int>(i);
      }
    }
    if (weakest_index < 0 || det.confidence <= weakest_confidence + class_replacement_margin_) {
      return false;
    }
    auto & track = tracks_[static_cast<std::size_t>(weakest_index)];
    track.x = det.x;
    track.y = det.y;
    track.vx = det.vx;
    track.vy = det.vy;
    track.covariance_m2 = 16.0;
    track.hits = 1;
    track.misses = 0;
    track.confidence = det.confidence;
    track.class_id = det.class_id;
    track.avg_intensity = det.confidence;
    track.bbox_aspect_ratio = std::clamp(det.aspect_ratio, 0.2, 5.0);
    track.range_rate = det.range_rate;
    track.ais_mmsi = det.ais_mmsi;
    track.last_source = det.source;
    track.state = "候选";
    track.last_update = now;
    track.last_predict = now;
    track.sources.clear();
    track.sources.insert(det.source);
    return true;
  }

  int active_track_count_for_class(double class_id) const
  {
    return static_cast<int>(std::count_if(
      tracks_.begin(), tracks_.end(), [class_id](const Track & track) {
        return track.class_id >= 0.0 && std::abs(track.class_id - class_id) < 0.5;
      }));
  }

  bool can_ais_create_track(const Detection & det) const
  {
    if (ais_can_create_tracks_) {
      return det.confidence >= min_ais_confidence_;
    }
    if (!ais_can_create_target_tracks_ || det.confidence < min_ais_confidence_) {
      return false;
    }
    return std::any_of(ais_target_class_ids_.begin(), ais_target_class_ids_.end(),
      [&det](double class_id) {return std::abs(det.class_id - class_id) < 0.5;});
  }

  double mahalanobis_distance(const Track & track, const Detection & det) const
  {
    const double variance = std::max(
      0.5, track.covariance_m2 + 2.0 * (1.0 - std::clamp(det.confidence, 0.0, 0.98)));
    const double dx = det.x - track.x;
    const double dy = det.y - track.y;
    return std::sqrt((dx * dx + dy * dy) / variance);
  }

  double feature_similarity(const Track & track, const Detection & det) const
  {
    double score = 0.0;
    double weight = 0.0;
    if (track.class_id >= 0.0 && det.class_id >= 0.0) {
      score += std::abs(track.class_id - det.class_id) < 0.5 ? 0.45 : 0.0;
      weight += 0.45;
    }
    score += (1.0 - std::clamp(std::abs(track.avg_intensity - det.confidence), 0.0, 1.0)) * 0.25;
    weight += 0.25;
    const double aspect_delta = std::abs(track.bbox_aspect_ratio - std::clamp(det.aspect_ratio, 0.2, 5.0));
    score += (1.0 - std::clamp(aspect_delta / 3.0, 0.0, 1.0)) * 0.15;
    weight += 0.15;
    score += (1.0 - std::clamp(std::abs(track.range_rate - det.range_rate) / 2.0, 0.0, 1.0)) * 0.15;
    weight += 0.15;
    return weight > 0.0 ? score / weight : 0.0;
  }

  bool can_associate(const Track & track, const Detection & det) const
  {
    if (det.source == "ais" && det.ais_mmsi >= 0 && track.ais_mmsi >= 0) {
      if (det.ais_mmsi != track.ais_mmsi) {
        return false;
      }
      return std::hypot(track.x - det.x, track.y - det.y) <= ais_maintenance_gate_m_;
    }
    if (mahalanobis_distance(track, det) < mahalanobis_gate_) {
      return true;
    }
    const double distance = std::hypot(track.x - det.x, track.y - det.y);
    const double observed_vx = 2.0 * (det.x - track.x);
    const double observed_vy = 2.0 * (det.y - track.y);
    const double velocity_error = std::hypot(observed_vx - track.vx, observed_vy - track.vy);
    if (distance < association_gate_ * 1.5 && velocity_error < velocity_gate_mps_) {
      return true;
    }
    return feature_similarity(track, det) >= feature_similarity_gate_ && distance < association_gate_ * 3.0;
  }

  double association_score(const Track & track, const Detection & det) const
  {
    return mahalanobis_distance(track, det) - 0.8 * feature_similarity(track, det);
  }

  void publish_tracks()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = tracker_frame_id_;
    pose_array.header.stamp = get_clock()->now();

    std::ostringstream status;
    status << "[";
    for (std::size_t index = 0; index < tracks_.size(); ++index) {
      const auto & track = tracks_[index];
      const bool confirmed = track.hits >= 2;
      const double color_scale = std::min(1.0, 0.35 + 0.12 * static_cast<double>(track.hits));
      const double speed = std::hypot(track.vx, track.vy);
      const double yaw = speed > 0.05 ? std::atan2(track.vy, track.vx) : 0.0;

      visualization_msgs::msg::Marker sphere;
      sphere.header = pose_array.header;
      sphere.ns = "tracks";
      sphere.id = track.id;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = track.x;
      sphere.pose.position.y = track.y;
      sphere.pose.position.z = 0.8;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.8;
      sphere.scale.y = 0.8;
      sphere.scale.z = 0.8;
      sphere.color.r = confirmed ? 0.1 : 1.0;
      sphere.color.g = color_scale;
      sphere.color.b = confirmed ? 1.0 : 0.1;
      sphere.color.a = 0.92;
      sphere.lifetime.sec = 1;
      marker_array.markers.push_back(sphere);

      visualization_msgs::msg::Marker arrow;
      arrow.header = pose_array.header;
      arrow.ns = "track_velocity";
      arrow.id = 1000 + track.id;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose.position.x = track.x;
      arrow.pose.position.y = track.y;
      arrow.pose.position.z = 1.1;
      arrow.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw);
      arrow.scale.x = std::clamp(speed, 0.6, 3.0);
      arrow.scale.y = 0.12;
      arrow.scale.z = 0.12;
      arrow.color.r = 0.0;
      arrow.color.g = 1.0;
      arrow.color.b = 0.4;
      arrow.color.a = 0.85;
      arrow.lifetime.sec = 1;
      marker_array.markers.push_back(arrow);

      visualization_msgs::msg::Marker text;
      text.header = pose_array.header;
      text.ns = "track_labels";
      text.id = 2000 + track.id;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = track.x;
      text.pose.position.y = track.y;
      text.pose.position.z = 1.8;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.45;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      std::ostringstream label;
      label.setf(std::ios::fixed, std::ios::floatfield);
      label << "ID " << track.id << " " << std::setprecision(1) << speed << " m/s";
      text.text = label.str();
      text.lifetime.sec = 1;
      marker_array.markers.push_back(text);

      geometry_msgs::msg::Pose pose;
      pose.position.x = track.x;
      pose.position.y = track.y;
      pose.position.z = 0.8;
      pose.orientation = quaternion_from_euler(0.0, 0.0, yaw);
      pose_array.poses.push_back(pose);

      if (index > 0) {
        status << ",";
      }
      status.setf(std::ios::fixed, std::ios::floatfield);
      status << std::setprecision(2)
             << "{\"id\":" << track.id
             << ",\"frame_id\":\"" << tracker_frame_id_ << "\""
             << ",\"state\":\"" << track.state << "\""
             << ",\"x\":" << track.x
             << ",\"y\":" << track.y
             << ",\"vx\":" << track.vx
             << ",\"vy\":" << track.vy
             << ",\"speed\":" << speed
             << ",\"confidence\":" << track.confidence
             << ",\"class_id\":" << track.class_id
             << ",\"ais_mmsi\":" << track.ais_mmsi
             << ",\"covariance_m2\":" << track.covariance_m2
             << ",\"last_source\":\"" << track.last_source << "\""
             << ",\"hits\":" << track.hits
             << ",\"misses\":" << track.misses
             << ",\"sources\":[";
      int source_index = 0;
      for (const auto & source : track.sources) {
        if (source_index++ > 0) {
          status << ",";
        }
        status << "\"" << source << "\"";
      }
      status << "]}";
    }
    status << "]";

    marker_pub_->publish(marker_array);
    pose_pub_->publish(pose_array);
    std_msgs::msg::String message;
    message.data = status.str();
    text_pub_->publish(message);
  }

  std::string base_frame_{"base_link"};
  std::string tracker_frame_id_{"world"};
  std::string usv_model_name_{"wamv"};
  std::string sonar_detect_topic_{"/sonar/detect"};
  double radar_x_offset_{-0.35};
  double radar_y_offset_{0.0};
  double sonar_x_offset_{0.65};
  int sonar_window_frames_{3};
  int sonar_min_points_for_valid_detect_{20};
  double sonar_default_confidence_{0.65};
  double sonar_detect_timeout_{2.0};
  double cluster_gap_{0.75};
  int min_cluster_points_{2};
  double max_tracking_range_{80.0};
  double association_gate_{2.8};
  double track_timeout_{2.0};
  double camera_fusion_timeout_{0.8};
  double uav_fusion_timeout_{1.2};
  double min_camera_confidence_{0.30};
  double min_uav_camera_confidence_{0.15};
  double camera_detection_cluster_radius_{4.0};
  double ais_confidence_{0.88};
  double min_ais_confidence_{0.45};
  double ais_maintenance_gate_m_{25.0};
  int max_tracks_per_class_{2};
  int max_track_misses_{400};
  double class_replacement_margin_{0.05};
  double mahalanobis_gate_{3.0};
  double velocity_gate_mps_{1.0};
  double feature_similarity_gate_{0.70};
  bool ais_can_create_tracks_{false};
  bool ais_can_create_target_tracks_{true};
  std::vector<double> ais_target_class_ids_;
  bool require_uav_observation_active_{true};
  bool uav_observation_active_{false};
  bool has_usv_pose_{false};
  double usv_x_{0.0};
  double usv_y_{0.0};
  double usv_yaw_{0.0};
  int next_track_id_{1};
  bool has_sonar_detection_{false};
  uint32_t last_sonar_detect_id_{0};
  std::vector<Detection> last_sonar_detections_;
  std::chrono::steady_clock::time_point last_sonar_time_{std::chrono::steady_clock::now()};
  std::vector<Track> tracks_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr radar_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  std::vector<std::string> sonar_scan_topics_;
  std::vector<double> sonar_scan_yaws_;
  std::vector<std::deque<std::vector<std::pair<double, double>>>> sonar_sector_windows_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> sonar_subs_;
  rclcpp::Subscription<c3_sonar_driver::msg::SonarDetect>::SharedPtr sonar_detect_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> gated_points_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> uav_points_subs_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr uav_observation_active_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pseudocolor_gated_points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr stf_gated_points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr bev_gated_points_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ais_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr c3_confirmed_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::RadarSonarTracker>());
  rclcpp::shutdown();
  return 0;
}
