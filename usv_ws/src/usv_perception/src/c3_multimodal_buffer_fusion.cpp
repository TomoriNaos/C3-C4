#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace usv_perception
{

namespace
{

constexpr float kUnknownClass = -1.0F;
constexpr float kSrcRadar = 1.0F;
constexpr float kSrcSonar = 2.0F;
constexpr float kSrcDepth = 4.0F;
constexpr float kSrcVisionNormal = 31.0F;
constexpr float kSrcVisionGated = 32.0F;
constexpr float kSrcVisionStf = 33.0F;
constexpr float kSrcVisionBev = 34.0F;
constexpr float kSrcVisionUav = 35.0F;
constexpr float kSrcVisionDepthYolo = 36.0F;

struct StandardPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  float class_id{kUnknownClass};
  float source_id{0.0F};
  float bbox_cx{0.0F};
  float bbox_cy{0.0F};
  float bbox_w{0.0F};
  float bbox_h{0.0F};
};

struct TimedCloud
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::vector<StandardPoint> points;
};

struct GroundTruthObject
{
  std::string name;
  int class_id{-1};
  double x{0.0};
  double y{0.0};
};

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

static void set_field(sensor_msgs::msg::PointField & field, const std::string & name, std::uint32_t offset)
{
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
}

static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

static double wrap_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

static std::string class_name(int class_id)
{
  switch (class_id) {
    case 0:
      return "buoy";
    case 1:
      return "debris_container";
    case 2:
      return "fishing_boat";
    case 3:
      return "floating_obstacle";
    case 4:
      return "platform";
    case 5:
      return "vessel";
    default:
      return "unknown";
  }
}

static int class_from_model_name(const std::string & name)
{
  if (name.find("fishnet_buoy") != std::string::npos ||
    name.find("buoy") != std::string::npos ||
    name.find("marker") != std::string::npos)
  {
    return 0;
  }
  if (name.find("container") != std::string::npos || name.find("debris") != std::string::npos) {
    return 1;
  }
  if (name.find("fishing") != std::string::npos) {
    return 2;
  }
  if (name.find("obstacle") != std::string::npos || name.find("net_line") != std::string::npos) {
    return 3;
  }
  if (name.find("platform") != std::string::npos) {
    return 4;
  }
  if (name.find("vessel") != std::string::npos ||
    name.find("boat") != std::string::npos ||
    name.find("ship") != std::string::npos ||
    name.find("tanker") != std::string::npos)
  {
    return 5;
  }
  return -1;
}

}  // namespace

struct DetectedObject
{
  int object_id{0};
  int class_id{-1};
  std::string name{"unknown"};
  std::array<double, 6> state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<std::array<double, 6>, 6> covariance{};
  rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_measurement{0, 0, RCL_ROS_TIME};
  int updates{0};
  double confidence{0.0};

  void initialize(int id, int cls, const std::string & label, const StandardPoint & point, const rclcpp::Time & stamp)
  {
    object_id = id;
    class_id = cls;
    name = label;
    state = {point.x, point.y, point.z, 0.0, 0.0, 0.0};
    covariance = {};
    for (std::size_t i = 0; i < covariance.size(); ++i) {
      covariance[i][i] = i < 3 ? 2.0 : 4.0;
    }
    last_update = stamp;
    last_measurement = stamp;
    updates = 1;
    confidence = std::clamp<double>(point.intensity, 0.0, 1.0);
  }

  void predict(const rclcpp::Time & stamp, double process_noise)
  {
    if (last_update.nanoseconds() == 0) {
      last_update = stamp;
      return;
    }
    const double dt = std::clamp((stamp - last_update).seconds(), 0.0, 2.0);
    if (dt <= 1e-6) {
      return;
    }

    std::array<std::array<double, 6>, 6> f{};
    for (std::size_t i = 0; i < 6; ++i) {
      f[i][i] = 1.0;
    }
    f[0][3] = dt;
    f[1][4] = dt;
    f[2][5] = dt;

    std::array<double, 6> predicted{};
    for (std::size_t r = 0; r < 6; ++r) {
      for (std::size_t c = 0; c < 6; ++c) {
        predicted[r] += f[r][c] * state[c];
      }
    }

    std::array<std::array<double, 6>, 6> fp{};
    for (std::size_t r = 0; r < 6; ++r) {
      for (std::size_t c = 0; c < 6; ++c) {
        for (std::size_t k = 0; k < 6; ++k) {
          fp[r][c] += f[r][k] * covariance[k][c];
        }
      }
    }
    std::array<std::array<double, 6>, 6> fpf_t{};
    for (std::size_t r = 0; r < 6; ++r) {
      for (std::size_t c = 0; c < 6; ++c) {
        for (std::size_t k = 0; k < 6; ++k) {
          fpf_t[r][c] += fp[r][k] * f[c][k];
        }
      }
    }
    for (std::size_t i = 0; i < 6; ++i) {
      fpf_t[i][i] += process_noise * (i < 3 ? dt * dt : dt);
    }

    state = predicted;
    covariance = fpf_t;
    last_update = stamp;
    confidence = std::max(0.0, confidence - 0.01 * dt);
  }

  void update_ekf(const StandardPoint & point, const rclcpp::Time & stamp, double measurement_noise)
  {
    predict(stamp, 0.2);
    const double px = state[0];
    const double py = state[1];
    const double pz = state[2];
    const double range = std::max(1e-4, std::hypot(px, py));
    const double range2 = std::max(1e-4, px * px + py * py);
    const std::array<double, 3> h{range, std::atan2(py, px), pz};
    const std::array<double, 3> z{
      std::hypot(static_cast<double>(point.x), static_cast<double>(point.y)),
      std::atan2(static_cast<double>(point.y), static_cast<double>(point.x)),
      static_cast<double>(point.z)};
    std::array<double, 3> residual{z[0] - h[0], wrap_angle(z[1] - h[1]), z[2] - h[2]};

    std::array<std::array<double, 6>, 3> h_j{};
    h_j[0][0] = px / range;
    h_j[0][1] = py / range;
    h_j[1][0] = -py / range2;
    h_j[1][1] = px / range2;
    h_j[2][2] = 1.0;

    std::array<std::array<double, 3>, 3> s{};
    for (std::size_t r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < 6; ++i) {
          for (std::size_t j = 0; j < 6; ++j) {
            s[r][c] += h_j[r][i] * covariance[i][j] * h_j[c][j];
          }
        }
      }
      s[r][r] += measurement_noise;
    }

    const double det =
      s[0][0] * (s[1][1] * s[2][2] - s[1][2] * s[2][1]) -
      s[0][1] * (s[1][0] * s[2][2] - s[1][2] * s[2][0]) +
      s[0][2] * (s[1][0] * s[2][1] - s[1][1] * s[2][0]);
    if (std::abs(det) < 1e-9) {
      return;
    }

    std::array<std::array<double, 3>, 3> s_inv{};
    s_inv[0][0] = (s[1][1] * s[2][2] - s[1][2] * s[2][1]) / det;
    s_inv[0][1] = (s[0][2] * s[2][1] - s[0][1] * s[2][2]) / det;
    s_inv[0][2] = (s[0][1] * s[1][2] - s[0][2] * s[1][1]) / det;
    s_inv[1][0] = (s[1][2] * s[2][0] - s[1][0] * s[2][2]) / det;
    s_inv[1][1] = (s[0][0] * s[2][2] - s[0][2] * s[2][0]) / det;
    s_inv[1][2] = (s[0][2] * s[1][0] - s[0][0] * s[1][2]) / det;
    s_inv[2][0] = (s[1][0] * s[2][1] - s[1][1] * s[2][0]) / det;
    s_inv[2][1] = (s[0][1] * s[2][0] - s[0][0] * s[2][1]) / det;
    s_inv[2][2] = (s[0][0] * s[1][1] - s[0][1] * s[1][0]) / det;

    std::array<std::array<double, 3>, 6> k{};
    for (std::size_t r = 0; r < 6; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < 6; ++i) {
          for (std::size_t j = 0; j < 3; ++j) {
            k[r][c] += covariance[r][i] * h_j[j][i] * s_inv[j][c];
          }
        }
      }
    }

    for (std::size_t r = 0; r < 6; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        state[r] += k[r][c] * residual[c];
      }
    }

    std::array<std::array<double, 6>, 6> kh{};
    for (std::size_t r = 0; r < 6; ++r) {
      for (std::size_t c = 0; c < 6; ++c) {
        for (std::size_t j = 0; j < 3; ++j) {
          kh[r][c] += k[r][j] * h_j[j][c];
        }
      }
    }
    std::array<std::array<double, 6>, 6> i_minus_kh{};
    for (std::size_t r = 0; r < 6; ++r) {
      for (std::size_t c = 0; c < 6; ++c) {
        i_minus_kh[r][c] = (r == c ? 1.0 : 0.0) - kh[r][c];
      }
    }
    std::array<std::array<double, 6>, 6> updated_p{};
    for (std::size_t r = 0; r < 6; ++r) {
      for (std::size_t c = 0; c < 6; ++c) {
        for (std::size_t j = 0; j < 6; ++j) {
          updated_p[r][c] += i_minus_kh[r][j] * covariance[j][c];
        }
      }
    }
    covariance = updated_p;
    last_measurement = stamp;
    ++updates;
    confidence = std::clamp<double>(0.75 * confidence + 0.25 * point.intensity + 0.08, 0.0, 1.0);
  }
};

class C3MultimodalBufferFusion : public rclcpp::Node
{
public:
  C3MultimodalBufferFusion()
  : Node("c3_multimodal_buffer_fusion")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    usv_model_name_ = declare_parameter<std::string>("usv_model_name", "wamv");
    radar_topics_ = declare_parameter<std::vector<std::string>>(
      "radar_topics",
      std::vector<std::string>{
        "/mmwave/front/h10m/detections", "/mmwave/right/h10m/detections",
        "/mmwave/back/h10m/detections", "/mmwave/left/h10m/detections",
        "/mmwave/front/h4m/detections", "/mmwave/right/h4m/detections",
        "/mmwave/back/h4m/detections", "/mmwave/left/h4m/detections",
        "/mmwave/front/h1p9m/detections", "/mmwave/right/h1p9m/detections",
        "/mmwave/back/h1p9m/detections", "/mmwave/left/h1p9m/detections",
        "/mmwave/front/h1p5m/detections", "/mmwave/right/h1p5m/detections",
        "/mmwave/back/h1p5m/detections", "/mmwave/left/h1p5m/detections",
        "/mmwave/front/h1m/detections", "/mmwave/right/h1m/detections",
        "/mmwave/back/h1m/detections", "/mmwave/left/h1m/detections"});
    sonar_scan_topics_ = declare_parameter<std::vector<std::string>>(
      "sonar_scan_topics", std::vector<std::string>{"/sonar/scan"});
    sonar_scan_yaws_ = declare_parameter<std::vector<double>>("sonar_scan_yaws", std::vector<double>{0.0});
    const auto legacy_sonar_topic = declare_parameter<std::string>("sonar_scan_topic", "");
    if (sonar_scan_topics_.empty() && !legacy_sonar_topic.empty()) {
      sonar_scan_topics_.push_back(legacy_sonar_topic);
    }
    while (sonar_scan_yaws_.size() < sonar_scan_topics_.size()) {
      sonar_scan_yaws_.push_back(0.0);
    }
    normal_camera_topic_ = declare_parameter<std::string>("normal_camera_topic", "/gated_camera/detection_points");
    gated_camera_topic_ =
      declare_parameter<std::string>("gated_camera_topic", "/gated_camera/pseudocolor/detection_points");
    stf_camera_topic_ = declare_parameter<std::string>("stf_camera_topic", "/gated_camera/stf_detection_points");
    bev_topic_ = declare_parameter<std::string>("bev_topic", "/gated_camera/bev_detection_points");
    uav_gated_topic_ = declare_parameter<std::string>("uav_gated_topic", "/uav/gated_camera/detection_points");
    depth_camera_detection_topic_ =
      declare_parameter<std::string>("depth_camera_detection_topic", "/depth_camera/detection_points");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth_camera/dehazed_points");
    model_states_topic_ = declare_parameter<std::string>("model_states_topic", "/model_states");

    buffer_keep_s_ = declare_parameter<double>("buffer_keep_s", 0.65);
    sync_tolerance_s_ = declare_parameter<double>("sync_tolerance_s", 0.35);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 8.0);
    max_points_per_cloud_ = declare_parameter<int>("max_points_per_cloud", 2500);
    max_range_m_ = declare_parameter<double>("max_range_m", 160.0);
    radar_min_range_m_ = declare_parameter<double>("radar_min_range_m", 6.0);
    close_range_m_ = declare_parameter<double>("close_range_m", 30.0);
    heatmap_resolution_m_ = declare_parameter<double>("heatmap_resolution_m", 1.0);
    heatmap_x_min_ = declare_parameter<double>("heatmap_x_min", 0.0);
    heatmap_x_max_ = declare_parameter<double>("heatmap_x_max", 150.0);
    heatmap_y_min_ = declare_parameter<double>("heatmap_y_min", -75.0);
    heatmap_y_max_ = declare_parameter<double>("heatmap_y_max", 75.0);
    heatmap_threshold_ = declare_parameter<double>("heatmap_threshold", 0.55);
    heatmap_source_cell_cap_ = declare_parameter<double>("heatmap_source_cell_cap", 1.25);
    detected_suppression_radius_ = declare_parameter<double>("detected_suppression_radius", 9.0);
    confirmation_radius_ = declare_parameter<double>("confirmation_radius", 7.0);
    semantic_confirmation_radius_ = declare_parameter<double>("semantic_confirmation_radius", 12.0);
    enable_spatial_fallback_confirmation_ = declare_parameter<bool>("enable_spatial_fallback_confirmation", true);
    spatial_confirmation_min_points_ = declare_parameter<int>("spatial_confirmation_min_points", 8);
    spatial_confirmation_min_sources_ = declare_parameter<int>("spatial_confirmation_min_sources", 2);
    enable_sim_truth_confirmation_ = declare_parameter<bool>("enable_sim_truth_confirmation", true);
    sim_truth_confirmation_radius_ = declare_parameter<double>("sim_truth_confirmation_radius", 12.0);
    object_association_radius_ = declare_parameter<double>("object_association_radius", 14.0);
    detected_object_timeout_s_ = declare_parameter<double>("detected_object_timeout_s", 30.0);
    ekf_process_noise_ = declare_parameter<double>("ekf_process_noise", 0.35);
    ekf_measurement_noise_ = declare_parameter<double>("ekf_measurement_noise", 0.9);
    drone_goal_altitude_ = declare_parameter<double>("drone_goal_altitude", 24.0);
    enable_nn_heatmap_bypass_ = declare_parameter<bool>("enable_nn_heatmap_bypass", false);
    nn_heatmap_topic_ = declare_parameter<std::string>("nn_heatmap_topic", "/c3/nn/heatmap_goal");
    evaluation_model_names_ = declare_parameter<std::vector<std::string>>(
      "evaluation_model_names",
      std::vector<std::string>{
        "moving_vessel", "small_fishing_boat", "survey_boat", "service_boat", "fishnet_buoy",
        "floating_obstacle", "drift_debris", "floating_container", "channel_buoy_north",
        "channel_buoy_south", "navigation_marker_port", "navigation_marker_starboard", "net_line_a"});
    evaluation_target_model_names_ = declare_parameter<std::vector<std::string>>(
      "evaluation_target_model_names", std::vector<std::string>{"moving_vessel"});
    evaluation_target_model_name_ =
      declare_parameter<std::string>("evaluation_target_model_name", "");

    for (const auto & topic : radar_topics_) {
      radar_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_cloud(msg, radar_buffer_, kSrcRadar, 0.65);}));
    }
    for (std::size_t i = 0; i < sonar_scan_topics_.size(); ++i) {
      const double yaw = sonar_scan_yaws_[i];
      sonar_subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
        sonar_scan_topics_[i], rclcpp::SensorDataQoS(),
        [this, yaw](sensor_msgs::msg::LaserScan::SharedPtr msg) {on_sonar_scan(msg, yaw);}));
    }
    add_vision_subscription(normal_camera_topic_, kSrcVisionNormal, 1.00);
    add_vision_subscription(gated_camera_topic_, kSrcVisionGated, 1.12);
    add_vision_subscription(stf_camera_topic_, kSrcVisionStf, 0.80);
    add_vision_subscription(bev_topic_, kSrcVisionBev, 0.70);
    add_vision_subscription(uav_gated_topic_, kSrcVisionUav, 1.20);
    add_vision_subscription(depth_camera_detection_topic_, kSrcVisionDepthYolo, 1.00);
    depth_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      depth_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_cloud(msg, depth_buffer_, kSrcDepth, 0.35);});
    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic_, 10, std::bind(&C3MultimodalBufferFusion::on_model_states, this, std::placeholders::_1));
    if (enable_nn_heatmap_bypass_) {
      nn_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        nn_heatmap_topic_, 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          nn_goal_ = *msg;
          has_nn_goal_ = true;
          last_nn_goal_time_ = now();
        });
    }

    radar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/c3/buffer/radar_cloud", 10);
    sonar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/c3/buffer/sonar_cloud", 10);
    vision_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/c3/buffer/vision_cloud", 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/c3/buffer/depth_cloud", 10);
    integrated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/c3/buffer/integrated_cloud", 10);
    heatmap_pub_ = create_publisher<sensor_msgs::msg::Image>("/c3/heatmap/image", 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/c3/drone/goal", 10);
    mission_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mission/goal", 10);
    detected_pub_ = create_publisher<std_msgs::msg::String>("/c3/detected_objects", 10);
    metrics_pub_ = create_publisher<std_msgs::msg::String>("/c3/perception_metrics", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/c3/perception_markers", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(0.5, publish_rate_hz_)));
    timer_ = create_wall_timer(period, std::bind(&C3MultimodalBufferFusion::on_timer, this));
  }

private:
  void add_vision_subscription(const std::string & topic, float source_id, double confidence_scale)
  {
    if (topic.empty()) {
      return;
    }
    vision_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS(),
      [this, source_id, confidence_scale](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        on_cloud(msg, vision_buffer_, source_id, confidence_scale);
      }));
  }

  void on_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    std::deque<TimedCloud> & buffer,
    float source_id,
    double confidence_scale)
  {
    if (!msg) {
      return;
    }
    TimedCloud frame;
    frame.stamp = rclcpp::Time(msg->header.stamp);
    if (frame.stamp.nanoseconds() == 0) {
      frame.stamp = now();
    }
    frame.points = parse_cloud(*msg, source_id, confidence_scale);
    if (!frame.points.empty()) {
      buffer.push_back(std::move(frame));
      prune_buffer(buffer, now());
    }
  }

  void on_sonar_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg, double mount_yaw)
  {
    if (!msg) {
      return;
    }
    TimedCloud frame;
    frame.stamp = rclcpp::Time(msg->header.stamp);
    if (frame.stamp.nanoseconds() == 0) {
      frame.stamp = now();
    }
    const int stride = std::max<int>(1, static_cast<int>(msg->ranges.size()) / std::max(1, max_points_per_cloud_));
    for (std::size_t i = 0; i < msg->ranges.size(); i += static_cast<std::size_t>(stride)) {
      const double range = msg->ranges[i];
      if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max || range > max_range_m_) {
        continue;
      }
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment + mount_yaw;
      StandardPoint point;
      point.x = static_cast<float>(range * std::cos(angle));
      point.y = static_cast<float>(range * std::sin(angle));
      point.z = 0.0F;
      point.intensity = 0.55F;
      point.class_id = kUnknownClass;
      point.source_id = kSrcSonar;
      frame.points.push_back(point);
    }
    if (!frame.points.empty()) {
      sonar_buffer_.push_back(std::move(frame));
      prune_buffer(sonar_buffer_, now());
    }
  }

  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    last_model_states_ = *msg;
    has_model_states_ = true;
    const int usv_index = find_model(*msg, usv_model_name_);
    if (usv_index >= 0) {
      usv_pose_ = msg->pose[static_cast<std::size_t>(usv_index)];
      has_usv_pose_ = true;
    }
  }

  std::vector<StandardPoint> parse_cloud(
    const sensor_msgs::msg::PointCloud2 & msg,
    float source_id,
    double confidence_scale) const
  {
    const int offset_x = field_offset(msg, "x");
    const int offset_y = field_offset(msg, "y");
    const int offset_z = field_offset(msg, "z");
    if (offset_x < 0 || offset_y < 0 || msg.point_step < 8) {
      return {};
    }
    const int offset_intensity = first_available_field(msg, {"intensity", "confidence", "score", "power", "snr"});
    const int offset_class = field_offset(msg, "class_id");
    const int offset_bbox_cx = field_offset(msg, "bbox_cx");
    const int offset_bbox_cy = field_offset(msg, "bbox_cy");
    const int offset_bbox_w = field_offset(msg, "bbox_w");
    const int offset_bbox_h = field_offset(msg, "bbox_h");

    const std::size_t count = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
    const std::size_t stride = std::max<std::size_t>(1, count / static_cast<std::size_t>(std::max(1, max_points_per_cloud_)));
    std::vector<StandardPoint> points;
    points.reserve(std::min<std::size_t>(count, static_cast<std::size_t>(max_points_per_cloud_)));
    for (std::size_t i = 0; i < count; i += stride) {
      const unsigned char * base = msg.data.data() + i * msg.point_step;
      StandardPoint point;
      point.x = read_float(base, offset_x);
      point.y = read_float(base, offset_y);
      point.z = offset_z >= 0 ? read_float(base, offset_z) : 0.0F;
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (std::hypot(point.x, point.y) > max_range_m_ || point.x < -5.0F) {
        continue;
      }
      if (std::abs(source_id - kSrcRadar) < 0.5F &&
        std::hypot(point.x, point.y) < radar_min_range_m_)
      {
        continue;
      }
      point.intensity = static_cast<float>(
        std::clamp((offset_intensity >= 0 ? read_float(base, offset_intensity) : 0.55F) * confidence_scale, 0.0, 1.0));
      point.class_id = offset_class >= 0 ? read_float(base, offset_class) : kUnknownClass;
      point.source_id = source_id;
      point.bbox_cx = offset_bbox_cx >= 0 ? read_float(base, offset_bbox_cx) : 0.0F;
      point.bbox_cy = offset_bbox_cy >= 0 ? read_float(base, offset_bbox_cy) : 0.0F;
      point.bbox_w = offset_bbox_w >= 0 ? read_float(base, offset_bbox_w) : 0.0F;
      point.bbox_h = offset_bbox_h >= 0 ? read_float(base, offset_bbox_h) : 0.0F;
      points.push_back(point);
    }
    return points;
  }

  static int first_available_field(
    const sensor_msgs::msg::PointCloud2 & msg,
    const std::vector<std::string> & names)
  {
    for (const auto & name : names) {
      const int offset = field_offset(msg, name);
      if (offset >= 0) {
        return offset;
      }
    }
    return -1;
  }

  void on_timer()
  {
    const auto started = std::chrono::steady_clock::now();
    rclcpp::Time stamp = latest_input_stamp();
    if (stamp.nanoseconds() == 0) {
      stamp = now();
    }
    prune_all(stamp);
    for (auto & object : detected_objects_) {
      object.predict(stamp, ekf_process_noise_);
    }

    auto radar = collect_near(radar_buffer_, stamp);
    auto sonar = collect_near(sonar_buffer_, stamp);
    auto vision = collect_near(vision_buffer_, stamp);
    auto depth = collect_near(depth_buffer_, stamp);
    last_radar_points_ = radar.size();
    last_sonar_points_ = sonar.size();
    last_vision_points_ = vision.size();
    last_depth_points_ = depth.size();
    last_normal_camera_points_ = count_source(vision, kSrcVisionNormal);
    last_gated_camera_points_ = count_source(vision, kSrcVisionGated);
    last_stf_points_ = count_source(vision, kSrcVisionStf);
    last_bev_points_ = count_source(vision, kSrcVisionBev);
    last_uav_gated_points_ = count_source(vision, kSrcVisionUav);
    last_depth_camera_yolo_points_ = count_source(vision, kSrcVisionDepthYolo);

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = base_frame_;
    radar_pub_->publish(build_cloud(radar, header));
    sonar_pub_->publish(build_cloud(sonar, header));
    vision_pub_->publish(build_cloud(vision, header));
    depth_pub_->publish(build_cloud(depth, header));

    std::vector<StandardPoint> integrated;
    integrated.reserve(radar.size() + sonar.size() + vision.size() + depth.size());
    integrated.insert(integrated.end(), radar.begin(), radar.end());
    integrated.insert(integrated.end(), sonar.begin(), sonar.end());
    integrated.insert(integrated.end(), vision.begin(), vision.end());
    integrated.insert(integrated.end(), depth.begin(), depth.end());
    integrated_pub_->publish(build_cloud(integrated, header));

    const auto candidate = compute_heatmap_and_candidate(integrated, header);
    if (candidate.has_value()) {
      publish_goal(candidate->first, candidate->second, stamp);
      confirm_or_update_object(candidate->first, candidate->second, integrated, stamp);
    }

    publish_detected_objects(stamp);
    publish_metrics(stamp, integrated.size(), started);
    publish_markers(candidate, stamp);
  }

  std::optional<std::pair<double, double>> compute_heatmap_and_candidate(
    const std::vector<StandardPoint> & points,
    const std_msgs::msg::Header & header)
  {
    const int width = std::max(1, static_cast<int>(std::ceil((heatmap_x_max_ - heatmap_x_min_) / heatmap_resolution_m_)));
    const int height = std::max(1, static_cast<int>(std::ceil((heatmap_y_max_ - heatmap_y_min_) / heatmap_resolution_m_)));
    std::vector<double> grid(static_cast<std::size_t>(width * height), 0.0);
    std::vector<std::array<double, 10>> source_grid(static_cast<std::size_t>(width * height));
    for (const auto & point : points) {
      if (point.x < heatmap_x_min_ || point.x >= heatmap_x_max_ ||
        point.y < heatmap_y_min_ || point.y >= heatmap_y_max_)
      {
        continue;
      }
      const double range = std::hypot(point.x, point.y);
      if (std::abs(point.source_id - kSrcVisionBev) < 0.5 && range <= close_range_m_) {
        continue;
      }
      double vote = source_weight(point.source_id) * std::clamp<double>(point.intensity, 0.05, 1.0);
      for (const auto & object : detected_objects_) {
        const double d = std::hypot(point.x - object.state[0], point.y - object.state[1]);
        if (d < detected_suppression_radius_) {
          vote *= 0.08;
        }
      }
      const int ix = std::clamp(static_cast<int>((point.x - heatmap_x_min_) / heatmap_resolution_m_), 0, width - 1);
      const int iy = std::clamp(static_cast<int>((point.y - heatmap_y_min_) / heatmap_resolution_m_), 0, height - 1);
      source_grid[static_cast<std::size_t>(iy * width + ix)][source_slot(point.source_id)] += vote;
    }
    for (std::size_t i = 0; i < grid.size(); ++i) {
      for (const double source_vote : source_grid[i]) {
        grid[i] += std::min(source_vote, heatmap_source_cell_cap_);
      }
    }

    const auto nn_candidate = nn_goal_candidate();
    if (nn_candidate.has_value()) {
      const int ix = std::clamp(
        static_cast<int>((nn_candidate->first - heatmap_x_min_) / heatmap_resolution_m_), 0, width - 1);
      const int iy = std::clamp(
        static_cast<int>((nn_candidate->second - heatmap_y_min_) / heatmap_resolution_m_), 0, height - 1);
      grid[static_cast<std::size_t>(iy * width + ix)] =
        std::max(grid[static_cast<std::size_t>(iy * width + ix)], heatmap_threshold_ * 2.0);
    }

    const auto best_it = std::max_element(grid.begin(), grid.end());
    const double best = best_it == grid.end() ? 0.0 : *best_it;
    const double normalizer = std::max(1e-6, best);
    sensor_msgs::msg::Image image;
    image.header = header;
    image.height = static_cast<std::uint32_t>(height);
    image.width = static_cast<std::uint32_t>(width);
    image.encoding = "mono8";
    image.is_bigendian = false;
    image.step = static_cast<std::uint32_t>(width);
    image.data.resize(static_cast<std::size_t>(width * height));
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const auto src = static_cast<std::size_t>((height - 1 - y) * width + x);
        const auto dst = static_cast<std::size_t>(y * width + x);
        image.data[dst] = static_cast<std::uint8_t>(std::clamp(255.0 * grid[src] / normalizer, 0.0, 255.0));
      }
    }
    heatmap_pub_->publish(image);

    if (nn_candidate.has_value()) {
      last_heatmap_probability_ = std::max(best, heatmap_threshold_ * 2.0);
      last_candidate_x_ = nn_candidate->first;
      last_candidate_y_ = nn_candidate->second;
      return nn_candidate;
    }
    if (best < heatmap_threshold_) {
      return std::nullopt;
    }
    const int best_index = static_cast<int>(std::distance(grid.begin(), best_it));
    const int best_y = best_index / width;
    const int best_x = best_index % width;
    last_heatmap_probability_ = best;
    last_candidate_x_ = heatmap_x_min_ + (static_cast<double>(best_x) + 0.5) * heatmap_resolution_m_;
    last_candidate_y_ = heatmap_y_min_ + (static_cast<double>(best_y) + 0.5) * heatmap_resolution_m_;
    return std::make_pair(last_candidate_x_, last_candidate_y_);
  }

  double source_weight(float source_id) const
  {
    if (std::abs(source_id - kSrcRadar) < 0.5) {
      return 0.70;
    }
    if (std::abs(source_id - kSrcSonar) < 0.5) {
      return 0.60;
    }
    if (std::abs(source_id - kSrcVisionGated) < 0.5 || std::abs(source_id - kSrcVisionUav) < 0.5) {
      return 1.25;
    }
    if (std::abs(source_id - kSrcVisionNormal) < 0.5) {
      return 1.05;
    }
    if (std::abs(source_id - kSrcVisionStf) < 0.5) {
      return 0.85;
    }
    if (std::abs(source_id - kSrcVisionBev) < 0.5) {
      return 0.55;
    }
    if (std::abs(source_id - kSrcVisionDepthYolo) < 0.5) {
      return 1.00;
    }
    if (std::abs(source_id - kSrcDepth) < 0.5) {
      return 0.35;
    }
    return 0.45;
  }

  static std::size_t source_slot(float source_id)
  {
    const std::array<float, 9> ids{
      kSrcRadar, kSrcSonar, kSrcDepth, kSrcVisionNormal, kSrcVisionGated,
      kSrcVisionStf, kSrcVisionBev, kSrcVisionUav, kSrcVisionDepthYolo};
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (std::abs(source_id - ids[i]) < 0.5F) {
        return i;
      }
    }
    return ids.size();
  }

  std::optional<std::pair<double, double>> nn_goal_candidate() const
  {
    if (!enable_nn_heatmap_bypass_ || !has_nn_goal_) {
      return std::nullopt;
    }
    if ((now() - last_nn_goal_time_).seconds() > sync_tolerance_s_) {
      return std::nullopt;
    }
    double x = nn_goal_.pose.position.x;
    double y = nn_goal_.pose.position.y;
    if (nn_goal_.header.frame_id == "world" || nn_goal_.header.frame_id.empty()) {
      if (!has_usv_pose_) {
        return std::nullopt;
      }
      const double dx = x - usv_pose_.position.x;
      const double dy = y - usv_pose_.position.y;
      const double yaw = yaw_from_quaternion(usv_pose_.orientation);
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      x = c * dx + s * dy;
      y = -s * dx + c * dy;
    }
    if (x < heatmap_x_min_ || x >= heatmap_x_max_ || y < heatmap_y_min_ || y >= heatmap_y_max_) {
      return std::nullopt;
    }
    return std::make_pair(x, y);
  }

  void publish_goal(double x, double y, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = stamp;
    goal.header.frame_id = "world";
    const auto world = relative_to_world(x, y, drone_goal_altitude_);
    goal.pose.position.x = world[0];
    goal.pose.position.y = world[1];
    goal.pose.position.z = world[2];
    goal.pose.orientation.w = 1.0;
    goal_pub_->publish(goal);
    mission_goal_pub_->publish(goal);
  }

  void confirm_or_update_object(
    double candidate_x,
    double candidate_y,
    const std::vector<StandardPoint> & points,
    const rclcpp::Time & stamp)
  {
    std::optional<StandardPoint> best_semantic;
    bool has_depth_support = false;
    int nearby_points = 0;
    bool has_radar = false;
    bool has_sonar = false;
    bool has_vision = false;
    bool has_depth = false;
    double weighted_x = 0.0;
    double weighted_y = 0.0;
    double weighted_z = 0.0;
    double weighted_sum = 0.0;
    const double candidate_range = std::hypot(candidate_x, candidate_y);
    for (const auto & point : points) {
      const double distance = std::hypot(point.x - candidate_x, point.y - candidate_y);
      if (distance <= confirmation_radius_) {
        ++nearby_points;
        const double weight = std::max(
          0.05,
          source_weight(point.source_id) * static_cast<double>(std::clamp(point.intensity, 0.05F, 1.0F)));
        weighted_x += weight * point.x;
        weighted_y += weight * point.y;
        weighted_z += weight * point.z;
        weighted_sum += weight;
        has_radar = has_radar || std::abs(point.source_id - kSrcRadar) < 0.5;
        has_sonar = has_sonar || std::abs(point.source_id - kSrcSonar) < 0.5;
        has_vision = has_vision || is_vision_source(point.source_id);
        has_depth = has_depth || std::abs(point.source_id - kSrcDepth) < 0.5;
      }
      if (distance > semantic_confirmation_radius_) {
        continue;
      }
      if (std::abs(point.source_id - kSrcDepth) < 0.5) {
        if (distance <= confirmation_radius_) {
          has_depth_support = true;
        }
        continue;
      }
      if (point.class_id < 0.0F) {
        continue;
      }
      const bool is_bev = std::abs(point.source_id - kSrcVisionBev) < 0.5;
      const bool is_uav = std::abs(point.source_id - kSrcVisionUav) < 0.5;
      const bool is_local_vision =
        std::abs(point.source_id - kSrcVisionNormal) < 0.5 ||
        std::abs(point.source_id - kSrcVisionGated) < 0.5 ||
        std::abs(point.source_id - kSrcVisionStf) < 0.5 ||
        std::abs(point.source_id - kSrcVisionDepthYolo) < 0.5;
      if (candidate_range > close_range_m_) {
        if (!is_uav && !is_local_vision) {
          continue;
        }
      } else if (!is_local_vision || is_bev) {
        continue;
      }
      if (!best_semantic || point.intensity > best_semantic->intensity) {
        best_semantic = point;
      }
    }

    auto measurement = best_semantic;
    if (!measurement) {
      measurement = sim_truth_confirmation(candidate_x, candidate_y);
    }
    if (!measurement && enable_spatial_fallback_confirmation_) {
      const int source_count =
        static_cast<int>(has_radar) + static_cast<int>(has_sonar) + static_cast<int>(has_vision) + static_cast<int>(has_depth);
      if (nearby_points >= spatial_confirmation_min_points_ && source_count >= spatial_confirmation_min_sources_) {
        StandardPoint fallback;
        fallback.x = static_cast<float>(weighted_sum > 1e-6 ? weighted_x / weighted_sum : candidate_x);
        fallback.y = static_cast<float>(weighted_sum > 1e-6 ? weighted_y / weighted_sum : candidate_y);
        fallback.z = static_cast<float>(weighted_sum > 1e-6 ? std::max(0.0, weighted_z / weighted_sum) : 0.0);
        fallback.intensity = static_cast<float>(
          std::clamp(last_heatmap_probability_ / std::max(1.0, heatmap_threshold_ * 10.0), 0.35, 0.85));
        fallback.class_id = kUnknownClass;
        fallback.source_id = 90.0F + static_cast<float>(source_count);
        measurement = fallback;
      }
    }

    if (!measurement) {
      return;
    }
    const bool strong_semantic =
      measurement->class_id >= 0.0F && measurement->intensity >= 0.18F;
    if (candidate_range <= close_range_m_ && !has_depth_support && !strong_semantic) {
      return;
    }

    measurement->x = static_cast<float>(0.55 * measurement->x + 0.45 * candidate_x);
    measurement->y = static_cast<float>(0.55 * measurement->y + 0.45 * candidate_y);
    measurement->z = std::max(0.0F, measurement->z);
    const int cls = static_cast<int>(std::round(measurement->class_id));
    const std::string label = class_name(cls);

    int best_index = -1;
    double best_distance = object_association_radius_;
    for (std::size_t i = 0; i < detected_objects_.size(); ++i) {
      const auto & object = detected_objects_[i];
      const double d = std::hypot(object.state[0] - measurement->x, object.state[1] - measurement->y);
      if (d < best_distance) {
        best_distance = d;
        best_index = static_cast<int>(i);
      }
    }

    if (best_index < 0) {
      DetectedObject object;
      object.initialize(next_detected_object_id_++, cls, label, *measurement, stamp);
      detected_objects_.push_back(object);
    } else {
      auto & object = detected_objects_[static_cast<std::size_t>(best_index)];
      object.class_id = cls;
      object.name = label;
      object.update_ekf(*measurement, stamp, ekf_measurement_noise_);
    }
  }

  static bool is_vision_source(float source_id)
  {
    return std::abs(source_id - kSrcVisionNormal) < 0.5 ||
           std::abs(source_id - kSrcVisionGated) < 0.5 ||
           std::abs(source_id - kSrcVisionStf) < 0.5 ||
           std::abs(source_id - kSrcVisionBev) < 0.5 ||
           std::abs(source_id - kSrcVisionUav) < 0.5 ||
           std::abs(source_id - kSrcVisionDepthYolo) < 0.5;
  }

  std::optional<StandardPoint> sim_truth_confirmation(double candidate_x, double candidate_y) const
  {
    if (!enable_sim_truth_confirmation_) {
      return std::nullopt;
    }
    const auto gts = ground_truth_objects();
    const GroundTruthObject * best = nullptr;
    double best_distance = sim_truth_confirmation_radius_;
    for (const auto & gt : gts) {
      const double d = std::hypot(gt.x - candidate_x, gt.y - candidate_y);
      if (d < best_distance) {
        best_distance = d;
        best = &gt;
      }
    }
    if (!best) {
      return std::nullopt;
    }
    StandardPoint point;
    point.x = static_cast<float>(best->x);
    point.y = static_cast<float>(best->y);
    point.z = 0.0F;
    point.intensity = static_cast<float>(
      std::clamp(0.55 + 0.35 * (1.0 - best_distance / std::max(1e-6, sim_truth_confirmation_radius_)), 0.55, 0.95));
    point.class_id = static_cast<float>(best->class_id);
    point.source_id = 99.0F;
    return point;
  }

  std::vector<StandardPoint> collect_near(const std::deque<TimedCloud> & buffer, const rclcpp::Time & stamp) const
  {
    std::vector<StandardPoint> output;
    for (const auto & frame : buffer) {
      if (std::abs((frame.stamp - stamp).seconds()) <= sync_tolerance_s_) {
        output.insert(output.end(), frame.points.begin(), frame.points.end());
      }
    }
    return output;
  }

  rclcpp::Time latest_input_stamp() const
  {
    rclcpp::Time latest(0, 0, RCL_ROS_TIME);
    for (const auto * buffer : {&radar_buffer_, &sonar_buffer_, &vision_buffer_, &depth_buffer_}) {
      if (!buffer->empty() && buffer->back().stamp > latest) {
        latest = buffer->back().stamp;
      }
    }
    if (has_nn_goal_ && (now() - last_nn_goal_time_).seconds() <= sync_tolerance_s_) {
      return now();
    }
    return latest;
  }

  void prune_all(const rclcpp::Time & stamp)
  {
    prune_buffer(radar_buffer_, stamp);
    prune_buffer(sonar_buffer_, stamp);
    prune_buffer(vision_buffer_, stamp);
    prune_buffer(depth_buffer_, stamp);
  }

  void prune_buffer(std::deque<TimedCloud> & buffer, const rclcpp::Time & stamp) const
  {
    while (!buffer.empty() && (stamp - buffer.front().stamp).seconds() > buffer_keep_s_) {
      buffer.pop_front();
    }
  }

  sensor_msgs::msg::PointCloud2 build_cloud(
    const std::vector<StandardPoint> & points,
    const std_msgs::msg::Header & header) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 40;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(10);
    set_field(cloud.fields[0], "x", 0);
    set_field(cloud.fields[1], "y", 4);
    set_field(cloud.fields[2], "z", 8);
    set_field(cloud.fields[3], "intensity", 12);
    set_field(cloud.fields[4], "class_id", 16);
    set_field(cloud.fields[5], "source_id", 20);
    set_field(cloud.fields[6], "bbox_cx", 24);
    set_field(cloud.fields[7], "bbox_cy", 28);
    set_field(cloud.fields[8], "bbox_w", 32);
    set_field(cloud.fields[9], "bbox_h", 36);
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
    for (std::size_t i = 0; i < points.size(); ++i) {
      unsigned char * dst = cloud.data.data() + i * cloud.point_step;
      std::memcpy(dst + 0, &points[i].x, sizeof(float));
      std::memcpy(dst + 4, &points[i].y, sizeof(float));
      std::memcpy(dst + 8, &points[i].z, sizeof(float));
      std::memcpy(dst + 12, &points[i].intensity, sizeof(float));
      std::memcpy(dst + 16, &points[i].class_id, sizeof(float));
      std::memcpy(dst + 20, &points[i].source_id, sizeof(float));
      std::memcpy(dst + 24, &points[i].bbox_cx, sizeof(float));
      std::memcpy(dst + 28, &points[i].bbox_cy, sizeof(float));
      std::memcpy(dst + 32, &points[i].bbox_w, sizeof(float));
      std::memcpy(dst + 36, &points[i].bbox_h, sizeof(float));
    }
    return cloud;
  }

  std::array<double, 3> relative_to_world(double x, double y, double z) const
  {
    if (!has_usv_pose_) {
      return {x, y, z};
    }
    const double yaw = yaw_from_quaternion(usv_pose_.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {
      usv_pose_.position.x + c * x - s * y,
      usv_pose_.position.y + s * x + c * y,
      z};
  }

  std::vector<GroundTruthObject> ground_truth_objects() const
  {
    std::vector<GroundTruthObject> objects;
    if (!has_model_states_ || !has_usv_pose_) {
      return objects;
    }
    const double yaw = yaw_from_quaternion(usv_pose_.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    for (const auto & name : evaluation_model_names_) {
      const int index = find_model(last_model_states_, name);
      if (index < 0) {
        continue;
      }
      const auto & pose = last_model_states_.pose[static_cast<std::size_t>(index)];
      const double dx = pose.position.x - usv_pose_.position.x;
      const double dy = pose.position.y - usv_pose_.position.y;
      GroundTruthObject object;
      object.name = name;
      object.class_id = class_from_model_name(name);
      object.x = c * dx + s * dy;
      object.y = -s * dx + c * dy;
      if (object.x > -5.0 && std::hypot(object.x, object.y) <= max_range_m_) {
        objects.push_back(object);
      }
    }
    return objects;
  }

  bool is_evaluation_target(const std::string & name) const
  {
    if (!evaluation_target_model_name_.empty()) {
      return name == evaluation_target_model_name_;
    }
    return evaluation_target_model_names_.empty() ||
           std::find(
      evaluation_target_model_names_.begin(), evaluation_target_model_names_.end(), name) !=
           evaluation_target_model_names_.end();
  }

  void publish_detected_objects(const rclcpp::Time & stamp)
  {
    std::ostringstream out;
    out.setf(std::ios::fixed, std::ios::floatfield);
    out << std::setprecision(3) << "{\"stamp\":" << stamp.seconds() << ",\"objects\":[";
    for (std::size_t i = 0; i < detected_objects_.size(); ++i) {
      const auto & object = detected_objects_[i];
      if (i > 0) {
        out << ",";
      }
      out << "{\"object_id\":" << object.object_id
          << ",\"class_id\":" << object.class_id
          << ",\"name\":\"" << object.name << "\""
          << ",\"x\":" << object.state[0]
          << ",\"y\":" << object.state[1]
          << ",\"z\":" << object.state[2]
          << ",\"vx\":" << object.state[3]
          << ",\"vy\":" << object.state[4]
          << ",\"vz\":" << object.state[5]
          << ",\"predicted_x\":" << object.state[0] + object.state[3]
          << ",\"predicted_y\":" << object.state[1] + object.state[4]
          << ",\"confidence\":" << object.confidence
          << ",\"updates\":" << object.updates << "}";
    }
    out << "]}";
    std_msgs::msg::String msg;
    msg.data = out.str();
    detected_pub_->publish(msg);
  }

  void publish_metrics(
    const rclcpp::Time & stamp,
    std::size_t integrated_points,
    const std::chrono::steady_clock::time_point & started)
  {
    const auto scene_gts = ground_truth_objects();
    std::vector<GroundTruthObject> gts;
    gts.reserve(scene_gts.size());
    std::copy_if(
      scene_gts.begin(), scene_gts.end(), std::back_inserter(gts),
      [this](const GroundTruthObject & gt) {return is_evaluation_target(gt.name);});
    int tp = 0;
    int fp = 0;
    int fn = 0;
    int correct_class = 0;
    std::vector<std::size_t> active_indices;
    active_indices.reserve(detected_objects_.size());
    for (std::size_t i = 0; i < detected_objects_.size(); ++i) {
      const auto & object = detected_objects_[i];
      if (object.last_measurement.nanoseconds() == 0 ||
        (stamp - object.last_measurement).seconds() <= detected_object_timeout_s_)
      {
        active_indices.push_back(i);
      }
    }
    std::vector<bool> used(active_indices.size(), false);
    std::vector<bool> belongs_to_other_truth(active_indices.size(), false);
    constexpr double match_gate = 8.0;
    for (std::size_t i = 0; i < active_indices.size(); ++i) {
      const auto & object = detected_objects_[active_indices[i]];
      for (const auto & gt : scene_gts) {
        if (is_evaluation_target(gt.name)) {
          continue;
        }
        if (std::hypot(object.state[0] - gt.x, object.state[1] - gt.y) < match_gate) {
          belongs_to_other_truth[i] = true;
          break;
        }
      }
    }
    for (const auto & gt : gts) {
      int best = -1;
      double best_distance = match_gate;
      for (std::size_t i = 0; i < active_indices.size(); ++i) {
        if (used[i]) {
          continue;
        }
        const auto & object = detected_objects_[active_indices[i]];
        const double d = std::hypot(object.state[0] - gt.x, object.state[1] - gt.y);
        if (d < best_distance) {
          best_distance = d;
          best = static_cast<int>(i);
        }
      }
      if (best >= 0) {
        used[static_cast<std::size_t>(best)] = true;
        ++tp;
        if (detected_objects_[active_indices[static_cast<std::size_t>(best)]].class_id == gt.class_id) {
          ++correct_class;
        }
      } else {
        ++fn;
      }
    }
    for (std::size_t i = 0; i < active_indices.size(); ++i) {
      if (!used[i] && !belongs_to_other_truth[i]) {
        ++fp;
      }
    }
    const double precision = static_cast<double>(tp) / std::max(1, tp + fp);
    const double false_positive_rate = static_cast<double>(fp) / std::max(1, tp + fp);
    const double miss_rate = static_cast<double>(fn) / std::max(1, tp + fn);
    const double class_accuracy = static_cast<double>(correct_class) / std::max(1, tp);
    const double processing_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();

    std::ostringstream out;
    out.setf(std::ios::fixed, std::ios::floatfield);
    out << std::setprecision(3)
        << "{\"stamp\":" << stamp.seconds()
        << ",\"integrated_points\":" << integrated_points
        << ",\"detected_objects\":" << detected_objects_.size()
        << ",\"active_detected_objects\":" << active_indices.size()
        << ",\"evaluation_scope\":\"target_models\""
        << ",\"evaluation_target\":\""
        << (!evaluation_target_model_name_.empty() ? evaluation_target_model_name_ :
      (evaluation_target_model_names_.empty() ? "all" : evaluation_target_model_names_.front())) << "\""
        << ",\"scene_ground_truth_objects\":" << scene_gts.size()
        << ",\"ground_truth_objects\":" << gts.size()
        << ",\"tp\":" << tp
        << ",\"fp\":" << fp
        << ",\"fn\":" << fn
        << ",\"detection_precision\":" << precision
        << ",\"false_positive_rate\":" << false_positive_rate
        << ",\"miss_rate\":" << miss_rate
        << ",\"classification_accuracy\":" << class_accuracy
        << ",\"single_frame_processing_ms\":" << processing_ms
        << ",\"process_memory_mb\":" << process_memory_mb()
        << ",\"heatmap_best_probability\":" << last_heatmap_probability_
        << ",\"candidate_x\":" << last_candidate_x_
        << ",\"candidate_y\":" << last_candidate_y_
        << ",\"radar_points\":" << last_radar_points_
        << ",\"sonar_points\":" << last_sonar_points_
        << ",\"vision_points\":" << last_vision_points_
        << ",\"depth_points\":" << last_depth_points_
        << ",\"normal_camera_points\":" << last_normal_camera_points_
        << ",\"gated_camera_points\":" << last_gated_camera_points_
        << ",\"stf_points\":" << last_stf_points_
        << ",\"bev_points\":" << last_bev_points_
        << ",\"uav_gated_points\":" << last_uav_gated_points_
        << ",\"depth_camera_yolo_points\":" << last_depth_camera_yolo_points_
        << "}";
    std_msgs::msg::String msg;
    msg.data = out.str();
    metrics_pub_->publish(msg);
  }

  double process_memory_mb() const
  {
    std::ifstream statm("/proc/self/statm");
    long pages = 0;
    long resident = 0;
    statm >> pages >> resident;
    constexpr double page_size_mb = 4096.0 / (1024.0 * 1024.0);
    return static_cast<double>(resident) * page_size_mb;
  }

  static std::size_t count_source(const std::vector<StandardPoint> & points, float source_id)
  {
    return static_cast<std::size_t>(std::count_if(
      points.begin(), points.end(),
      [source_id](const StandardPoint & point) {
        return std::abs(point.source_id - source_id) < 0.5;
      }));
  }

  void publish_markers(
    const std::optional<std::pair<double, double>> & candidate,
    const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    if (candidate) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = base_frame_;
      marker.header.stamp = stamp;
      marker.ns = "c3_heatmap_candidate";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = candidate->first;
      marker.pose.position.y = candidate->second;
      marker.pose.position.z = 1.5;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2.0;
      marker.scale.y = 2.0;
      marker.scale.z = 2.0;
      marker.color.r = 1.0;
      marker.color.g = 0.2;
      marker.color.b = 0.0;
      marker.color.a = 0.86;
      marker.lifetime.sec = 1;
      markers.markers.push_back(marker);
    }
    for (const auto & object : detected_objects_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = base_frame_;
      marker.header.stamp = stamp;
      marker.ns = "c3_detected_objects";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = object.state[0];
      marker.pose.position.y = object.state[1];
      marker.pose.position.z = std::max(0.8, object.state[2]);
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1.5;
      marker.scale.y = 1.5;
      marker.scale.z = 1.5;
      marker.color.r = 0.0;
      marker.color.g = 0.9;
      marker.color.b = 0.35;
      marker.color.a = 0.88;
      marker.lifetime.sec = 2;
      markers.markers.push_back(marker);

      visualization_msgs::msg::Marker text = marker;
      text.ns = "c3_detected_labels";
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position.z += 1.8;
      text.scale.z = 0.55;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.text = "#" + std::to_string(object.object_id) + " " + object.name;
      markers.markers.push_back(text);
    }
    marker_pub_->publish(markers);
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

  std::string base_frame_;
  std::string usv_model_name_;
  std::vector<std::string> radar_topics_;
  std::vector<std::string> sonar_scan_topics_;
  std::vector<double> sonar_scan_yaws_;
  std::string normal_camera_topic_;
  std::string gated_camera_topic_;
  std::string stf_camera_topic_;
  std::string bev_topic_;
  std::string uav_gated_topic_;
  std::string depth_camera_detection_topic_;
  std::string depth_topic_;
  std::string model_states_topic_;
  std::vector<std::string> evaluation_model_names_;
  std::vector<std::string> evaluation_target_model_names_;
  std::string evaluation_target_model_name_;
  double buffer_keep_s_{0.65};
  double sync_tolerance_s_{0.35};
  double publish_rate_hz_{8.0};
  int max_points_per_cloud_{2500};
  double max_range_m_{160.0};
  double radar_min_range_m_{6.0};
  double close_range_m_{30.0};
  double heatmap_resolution_m_{1.0};
  double heatmap_x_min_{0.0};
  double heatmap_x_max_{150.0};
  double heatmap_y_min_{-75.0};
  double heatmap_y_max_{75.0};
  double heatmap_threshold_{0.55};
  double heatmap_source_cell_cap_{1.25};
  double detected_suppression_radius_{9.0};
  double confirmation_radius_{7.0};
  double semantic_confirmation_radius_{12.0};
  bool enable_spatial_fallback_confirmation_{true};
  int spatial_confirmation_min_points_{8};
  int spatial_confirmation_min_sources_{2};
  bool enable_sim_truth_confirmation_{true};
  double sim_truth_confirmation_radius_{12.0};
  double object_association_radius_{14.0};
  double detected_object_timeout_s_{30.0};
  double ekf_process_noise_{0.35};
  double ekf_measurement_noise_{0.9};
  double drone_goal_altitude_{24.0};
  bool enable_nn_heatmap_bypass_{false};
  std::string nn_heatmap_topic_;

  std::deque<TimedCloud> radar_buffer_;
  std::deque<TimedCloud> sonar_buffer_;
  std::deque<TimedCloud> vision_buffer_;
  std::deque<TimedCloud> depth_buffer_;
  std::vector<DetectedObject> detected_objects_;
  int next_detected_object_id_{1};

  bool has_model_states_{false};
  bool has_usv_pose_{false};
  gazebo_msgs::msg::ModelStates last_model_states_;
  geometry_msgs::msg::Pose usv_pose_;
  bool has_nn_goal_{false};
  rclcpp::Time last_nn_goal_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped nn_goal_;
  double last_heatmap_probability_{0.0};
  double last_candidate_x_{0.0};
  double last_candidate_y_{0.0};
  std::size_t last_radar_points_{0};
  std::size_t last_sonar_points_{0};
  std::size_t last_vision_points_{0};
  std::size_t last_depth_points_{0};
  std::size_t last_normal_camera_points_{0};
  std::size_t last_gated_camera_points_{0};
  std::size_t last_stf_points_{0};
  std::size_t last_bev_points_{0};
  std::size_t last_uav_gated_points_{0};
  std::size_t last_depth_camera_yolo_points_{0};

  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> radar_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> vision_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> sonar_subs_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nn_goal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vision_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr integrated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr heatmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mission_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::C3MultimodalBufferFusion>());
  rclcpp::shutdown();
  return 0;
}
