#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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
  std::string source;
};

struct Track
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  int hits{1};
  int misses{0};
  double confidence{0.0};
  double class_id{-1.0};
  std::string last_source;
  std::chrono::steady_clock::time_point last_update{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_predict{std::chrono::steady_clock::now()};
  std::set<std::string> sources;

  void predict(const std::chrono::steady_clock::time_point & now)
  {
    const double raw_dt = std::chrono::duration<double>(now - last_predict).count();
    const double dt = std::clamp(raw_dt, 0.0, 0.5);
    x += vx * dt;
    y += vy * dt;
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
    ++hits;
    misses = 0;
    confidence = std::max(confidence * 0.92, det.confidence);
    class_id = det.class_id >= 0.0 ? det.class_id : class_id;
    last_source = det.source;
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
    const std::string sonar_topic = declare_parameter<std::string>("sonar_topic", "/sonar/range");
    const std::string gated_points_topic =
      declare_parameter<std::string>("gated_points_topic", "/gated_camera/detection_points");
    const std::string uav_points_topic =
      declare_parameter<std::string>("uav_points_topic", "/uav/gated_camera/detection_points");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    radar_x_offset_ = declare_parameter<double>("radar_x_offset", -0.35);
    radar_y_offset_ = declare_parameter<double>("radar_y_offset", 0.0);
    sonar_x_offset_ = declare_parameter<double>("sonar_x_offset", 0.65);
    cluster_gap_ = declare_parameter<double>("cluster_gap", 0.75);
    min_cluster_points_ = declare_parameter<int>("min_cluster_points", 2);
    max_tracking_range_ = declare_parameter<double>("max_tracking_range", 80.0);
    association_gate_ = declare_parameter<double>("association_gate", 2.8);
    track_timeout_ = declare_parameter<double>("track_timeout", 2.0);
    camera_fusion_timeout_ = declare_parameter<double>("camera_fusion_timeout", 0.8);
    uav_fusion_timeout_ = declare_parameter<double>("uav_fusion_timeout", 1.2);
    min_camera_confidence_ = declare_parameter<double>("min_camera_confidence", 0.20);

    radar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      radar_topic, rclcpp::SensorDataQoS(),
      std::bind(&RadarSonarTracker::on_radar, this, std::placeholders::_1));
    sonar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      sonar_topic, rclcpp::SensorDataQoS(),
      std::bind(&RadarSonarTracker::on_sonar_scan, this, std::placeholders::_1));
    gated_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      gated_points_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        on_detection_points(msg, "gated_camera", 0.82);
      });
    uav_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      uav_points_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        on_detection_points(msg, "uav_gated_camera", 0.72);
      });

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("tracked_objects", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("tracked_object_poses", 10);
    text_pub_ = create_publisher<std_msgs::msg::String>("tracked_objects_text", 10);
    RCLCPP_INFO(
      get_logger(), "Tracking radar=%s sonar=%s gated=%s uav=%s",
      radar_topic.c_str(), sonar_topic.c_str(), gated_points_topic.c_str(), uav_points_topic.c_str());
  }

private:
  void on_sonar_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double sum_x = 0.0;
    double sum_y = 0.0;
    int count = 0;
    for (std::size_t index = 0; index < msg->ranges.size(); ++index) {
      const double range_value = msg->ranges[index];
      if (!std::isfinite(range_value) || range_value < msg->range_min || range_value > msg->range_max) {
        continue;
      }
      const double angle = msg->angle_min + static_cast<double>(index) * msg->angle_increment;
      sum_x += sonar_x_offset_ + range_value * std::cos(angle);
      sum_y += range_value * std::sin(angle);
      ++count;
    }

    if (count > 0) {
      last_sonar_detection_ = Detection{sum_x / count, sum_y / count, 0.0, 0.65, -1.0, "sonar"};
      last_sonar_time_ = std::chrono::steady_clock::now();
      has_sonar_detection_ = true;
    }
  }

  void on_radar(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto detections = cluster_radar_scan(*msg);
    const auto now = std::chrono::steady_clock::now();
    if (has_sonar_detection_ && std::chrono::duration<double>(now - last_sonar_time_).count() < 0.35) {
      detections.push_back(last_sonar_detection_);
    }
    update_tracks(detections, now);
    publish_tracks();
  }

  void on_detection_points(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const std::string & source,
    double source_weight)
  {
    const auto detections = parse_detection_cloud(*msg, source, source_weight);
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
      detections.push_back(Detection{sum_x / count, sum_y / count, 0.0, confidence, -1.0, "mmwave_radar"});
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
      const double confidence = std::clamp(raw_confidence * source_weight, min_camera_confidence_, 0.98);
      if (confidence < min_camera_confidence_) {
        continue;
      }
      const double class_id = offset_class_id >= 0 ? read_float(base, offset_class_id) : -1.0;
      detections.push_back(Detection{x, y, z, confidence, class_id, source});
    }
    return detections;
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

    std::vector<Detection> unmatched = detections;
    for (auto & track : tracks_) {
      int best_index = -1;
      double best_distance = association_gate_;
      for (std::size_t index = 0; index < unmatched.size(); ++index) {
        const auto & det = unmatched[index];
        const double distance = std::hypot(track.x - det.x, track.y - det.y);
        if (distance < best_distance) {
          best_distance = distance;
          best_index = static_cast<int>(index);
        }
      }
      if (best_index >= 0) {
        track.update(unmatched[static_cast<std::size_t>(best_index)], now);
        unmatched.erase(unmatched.begin() + best_index);
      }
    }

    for (const auto & det : unmatched) {
      if (det.source == "sonar") {
        continue;
      }
      Track track;
      track.id = next_track_id_++;
      track.x = det.x;
      track.y = det.y;
      track.confidence = det.confidence;
      track.class_id = det.class_id;
      track.last_source = det.source;
      track.last_update = now;
      track.last_predict = now;
      track.sources.insert(det.source);
      tracks_.push_back(track);
    }

    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [&](const Track & track) {
          return std::chrono::duration<double>(now - track.last_update).count() >= track_timeout_ ||
                 track.misses >= 20;
        }),
      tracks_.end());
  }

  void publish_tracks()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = base_frame_;
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
             << ",\"x\":" << track.x
             << ",\"y\":" << track.y
             << ",\"vx\":" << track.vx
             << ",\"vy\":" << track.vy
             << ",\"speed\":" << speed
             << ",\"confidence\":" << track.confidence
             << ",\"class_id\":" << track.class_id
             << ",\"last_source\":\"" << track.last_source << "\""
             << ",\"hits\":" << track.hits
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
  double radar_x_offset_{-0.35};
  double radar_y_offset_{0.0};
  double sonar_x_offset_{0.65};
  double cluster_gap_{0.75};
  int min_cluster_points_{2};
  double max_tracking_range_{80.0};
  double association_gate_{2.8};
  double track_timeout_{2.0};
  double camera_fusion_timeout_{0.8};
  double uav_fusion_timeout_{1.2};
  double min_camera_confidence_{0.20};
  int next_track_id_{1};
  bool has_sonar_detection_{false};
  Detection last_sonar_detection_;
  std::chrono::steady_clock::time_point last_sonar_time_{std::chrono::steady_clock::now()};
  std::vector<Track> tracks_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr radar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sonar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr gated_points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr uav_points_sub_;
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
