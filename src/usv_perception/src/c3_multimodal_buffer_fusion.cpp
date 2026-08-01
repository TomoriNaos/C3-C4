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
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "c3_sonar_driver/msg/sonar_detect.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "onnxruntime_cxx_api.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/bool.hpp"
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
constexpr float kSrcVisionUavDepthYolo = 37.0F;

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
  float radial_velocity{0.0F};
  float power{0.0F};
  float snr{0.0F};
  float range{0.0F};
  float azimuth_rad{0.0F};
  float point_score{0.0F};
};

struct TimedCloud
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::vector<StandardPoint> points;
};

// Sensor-only evidence track used to suppress transient sea clutter before
// neural inference. It is intentionally independent of Gazebo model states.
struct NeuralClutterTrack
{
  float source_id{0.0F};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double radial_velocity{0.0};
  double variance{9.0};
  int hits{0};
  int missed{0};
  std::deque<double> nis_history;
  rclcpp::Time last_prediction{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_measurement{0, 0, RCL_ROS_TIME};
};

struct NeuralModalCluster
{
  float source_id{0.0F};
  double x{0.0};
  double y{0.0};
  double intensity{0.0};
  double radial_velocity{0.0};
  int count{0};
  double cfar_background{0.0};
  bool cfar_pass{true};
  int track_index{-1};
};

struct HeatmapCandidateSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  double x{0.0};
  double y{0.0};
  double score{0.0};
  int cluster_id{-1};
};

struct HeatmapPeak
{
  double x{0.0};
  double y{0.0};
  double score{0.0};
  int cluster_id{-1};
};

struct SpatialConfirmationCandidate
{
  double x{0.0};
  double y{0.0};
  int hits{0};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
};

// One visual object may be emitted as several support points. Rank objects,
// rather than points, so a large bounding box cannot win by point count alone.
struct VisualCandidate
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  int class_id{-1};
  double confidence{0.0};
  double gated_confidence{-1.0};
  double depth_confidence{-1.0};
  int point_count{0};
};

// Persistent world-frame record of a heatmap cluster.  The alpha-beta predictor
// avoids re-dispatching the UAV to a location it has already inspected.
struct InspectionCluster
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  bool dispatched{false};
  bool inspected{false};
  bool uav_recognized{false};
  bool confirmed_to_main{false};
  int recognized_class_id{-1};
  double recognized_confidence{0.0};
  std::deque<std::array<double, 2>> history;
  rclcpp::Time last_prediction{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_measurement{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_inspection{0, 0, RCL_ROS_TIME};
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
  float last_source_id{0.0F};

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
    last_source_id = point.source_id;
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
    last_source_id = point.source_id;
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
        "/mmwave/front/h4m/detections", "/mmwave/right/h4m/detections",
        "/mmwave/back/h4m/detections", "/mmwave/left/h4m/detections",
        "/mmwave/front/h1p9m/detections", "/mmwave/right/h1p9m/detections",
        "/mmwave/back/h1p9m/detections", "/mmwave/left/h1p9m/detections"});
    sonar_scan_topics_ = declare_parameter<std::vector<std::string>>(
      "sonar_scan_topics", std::vector<std::string>{});
    sonar_scan_yaws_ = declare_parameter<std::vector<double>>("sonar_scan_yaws", std::vector<double>{0.0});
    sonar_detect_topic_ = declare_parameter<std::string>("sonar_detect_topic", "/sonar/detect");
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
    normal_camera_topics_ =
      declare_parameter<std::vector<std::string>>("normal_camera_topics", std::vector<std::string>{});
    gated_camera_topics_ =
      declare_parameter<std::vector<std::string>>("gated_camera_topics", std::vector<std::string>{});
    if (normal_camera_topics_.empty() && !normal_camera_topic_.empty()) {
      normal_camera_topics_.push_back(normal_camera_topic_);
    }
    if (gated_camera_topics_.empty() && !gated_camera_topic_.empty()) {
      gated_camera_topics_.push_back(gated_camera_topic_);
    }
    stf_camera_topic_ = declare_parameter<std::string>("stf_camera_topic", "/gated_camera/stf_detection_points");
    bev_topic_ = declare_parameter<std::string>("bev_topic", "/gated_camera/bev_detection_points");
    uav_gated_topic_ = declare_parameter<std::string>("uav_gated_topic", "/uav/gated_camera/detection_points");
    uav_gated_topics_ =
      declare_parameter<std::vector<std::string>>("uav_gated_topics", std::vector<std::string>{});
    if (uav_gated_topics_.empty() && !uav_gated_topic_.empty()) {
      uav_gated_topics_.push_back(uav_gated_topic_);
    }
    uav_depth_topics_ =
      declare_parameter<std::vector<std::string>>("uav_depth_topics", std::vector<std::string>{});
    uav_observation_active_topic_ = declare_parameter<std::string>(
      "uav_observation_active_topic", "/uav/observation/active");
    require_uav_observation_active_ =
      declare_parameter<bool>("require_uav_observation_active", true);
    depth_camera_detection_topic_ =
      declare_parameter<std::string>("depth_camera_detection_topic", "/depth_camera/detection_points");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth_camera/dehazed_points");
    model_states_topic_ = declare_parameter<std::string>("model_states_topic", "/model_states");

    buffer_keep_s_ = declare_parameter<double>("buffer_keep_s", 0.65);
    sync_tolerance_s_ = declare_parameter<double>("sync_tolerance_s", 0.35);
    sonar_fusion_window_frames_ = declare_parameter<int>("sonar_fusion_window_frames", 3);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 8.0);
    diagnostic_image_rate_hz_ = declare_parameter<double>("diagnostic_image_rate_hz", 3.0);
    max_points_per_cloud_ = declare_parameter<int>("max_points_per_cloud", 2500);
    radar_display_max_points_ = declare_parameter<int>("radar_display_max_points", 100);
    radar_display_cluster_radius_m_ =
      declare_parameter<double>("radar_display_cluster_radius_m", 3.0);
    radar_display_min_cluster_points_ =
      declare_parameter<int>("radar_display_min_cluster_points", 2);
    radar_display_max_points_per_cluster_ =
      declare_parameter<int>("radar_display_max_points_per_cluster", 6);
    max_range_m_ = declare_parameter<double>("max_range_m", 160.0);
    radar_min_range_m_ = declare_parameter<double>("radar_min_range_m", 1.0);
    radar_near_heatmap_radius_m_ = declare_parameter<double>("radar_near_heatmap_radius_m", 35.0);
    radar_near_heatmap_min_weight_ = declare_parameter<double>("radar_near_heatmap_min_weight", 0.10);
    close_range_m_ = declare_parameter<double>("close_range_m", 30.0);
    heatmap_resolution_m_ = declare_parameter<double>("heatmap_resolution_m", 1.0);
    heatmap_x_min_ = declare_parameter<double>("heatmap_x_min", 0.0);
    heatmap_x_max_ = declare_parameter<double>("heatmap_x_max", 150.0);
    heatmap_y_min_ = declare_parameter<double>("heatmap_y_min", -75.0);
    heatmap_y_max_ = declare_parameter<double>("heatmap_y_max", 75.0);
    pointcloud_visual_x_min_ = declare_parameter<double>("pointcloud_visual_x_min", -150.0);
    pointcloud_visual_x_max_ = declare_parameter<double>("pointcloud_visual_x_max", 150.0);
    pointcloud_visual_y_min_ = declare_parameter<double>("pointcloud_visual_y_min", -150.0);
    pointcloud_visual_y_max_ = declare_parameter<double>("pointcloud_visual_y_max", 150.0);
    heatmap_threshold_ = declare_parameter<double>("heatmap_threshold", 0.55);
    heatmap_source_cell_cap_ = declare_parameter<double>("heatmap_source_cell_cap", 1.25);
    heatmap_cluster_sigma_m_ = declare_parameter<double>("heatmap_cluster_sigma_m", 1.0);
    heatmap_cluster_radius_m_ = declare_parameter<double>("heatmap_cluster_radius_m", 2.0);
    heatmap_peak_nms_radius_m_ = declare_parameter<double>("heatmap_peak_nms_radius_m", 5.0);
    heatmap_max_peaks_ = declare_parameter<int>("heatmap_max_peaks", 5);
    heatmap_mode_ = declare_parameter<std::string>("heatmap_mode", "vote");
    neural_heatmap_model_path_ = declare_parameter<std::string>("neural_heatmap_model_path", "");
    neural_heatmap_threshold_ = declare_parameter<double>("neural_heatmap_threshold", 0.50);
    neural_heatmap_nms_kernel_ = declare_parameter<int>("neural_heatmap_nms_kernel", 5);
    heatmap_temporal_smoothing_alpha_ =
      declare_parameter<double>("heatmap_temporal_smoothing_alpha", 0.70);
    neural_max_points_per_modality_ = declare_parameter<int>("neural_max_points_per_modality", 1200);
    neural_execution_provider_ = declare_parameter<std::string>("neural_execution_provider", "auto");
    neural_intra_op_threads_ = declare_parameter<int>("neural_intra_op_threads", 4);
    neural_cuda_device_id_ = declare_parameter<int>("neural_cuda_device_id", 0);
    neural_depth_optical_frame_ = declare_parameter<bool>("neural_depth_optical_frame", true);
    neural_depth_x_offset_m_ = declare_parameter<double>("neural_depth_x_offset_m", 1.65);
    neural_depth_y_offset_m_ = declare_parameter<double>("neural_depth_y_offset_m", 0.0);
    neural_depth_z_offset_m_ = declare_parameter<double>("neural_depth_z_offset_m", 0.98);
    sonar_buffer_keep_s_ = declare_parameter<double>("sonar_buffer_keep_s", 1.5);
    sonar_sync_tolerance_s_ = declare_parameter<double>("sonar_sync_tolerance_s", 1.0);
    neural_clutter_filter_enabled_ = declare_parameter<bool>("neural_clutter_filter_enabled", true);
    neural_clutter_cluster_radius_m_ = declare_parameter<double>("neural_clutter_cluster_radius_m", 2.5);
    neural_clutter_min_cluster_points_ = declare_parameter<int>("neural_clutter_min_cluster_points", 2);
    neural_clutter_mahalanobis_gate_ = declare_parameter<double>("neural_clutter_mahalanobis_gate", 9.21);
    neural_clutter_measurement_noise_m_ = declare_parameter<double>("neural_clutter_measurement_noise_m", 2.0);
    neural_clutter_velocity_noise_mps_ = declare_parameter<double>("neural_clutter_velocity_noise_mps", 1.2);
    neural_clutter_confirmation_interval_s_ = declare_parameter<double>("neural_clutter_confirmation_interval_s", 0.30);
    neural_clutter_min_track_hits_ = declare_parameter<int>("neural_clutter_min_track_hits", 3);
    neural_clutter_track_timeout_s_ = declare_parameter<double>("neural_clutter_track_timeout_s", 2.0);
    neural_clutter_residual_mean_max_ = declare_parameter<double>("neural_clutter_residual_mean_max", 6.0);
    neural_clutter_cfar_scale_ = declare_parameter<double>("neural_clutter_cfar_scale", 1.10);
    neural_radar_min_cluster_points_ = declare_parameter<int>("neural_radar_min_cluster_points", 3);
    neural_radar_mahalanobis_gate_ = declare_parameter<double>("neural_radar_mahalanobis_gate", 6.0);
    neural_radar_confirmation_interval_s_ =
      declare_parameter<double>("neural_radar_confirmation_interval_s", 0.45);
    neural_radar_min_track_hits_ = declare_parameter<int>("neural_radar_min_track_hits", 4);
    neural_radar_residual_mean_max_ =
      declare_parameter<double>("neural_radar_residual_mean_max", 3.5);
    neural_radar_cfar_scale_ = declare_parameter<double>("neural_radar_cfar_scale", 1.35);
    radar_point_filter_enabled_ = declare_parameter<bool>("radar_point_filter_enabled", true);
    radar_filter_zone_a_max_m_ = declare_parameter<double>("radar_filter_zone_a_max_m", 1.0);
    radar_filter_zone_b_max_m_ = declare_parameter<double>("radar_filter_zone_b_max_m", 100.0);
    radar_filter_zone_c_max_m_ = declare_parameter<double>("radar_filter_zone_c_max_m", 300.0);
    radar_filter_zone_b_min_snr_ = declare_parameter<double>("radar_filter_zone_b_min_snr", 0.4);
    radar_filter_zone_c_min_snr_ = declare_parameter<double>("radar_filter_zone_c_min_snr", 0.3);
    radar_filter_zone_d_min_snr_ = declare_parameter<double>("radar_filter_zone_d_min_snr", 0.3);
    radar_filter_zone_b_min_power_ = declare_parameter<double>("radar_filter_zone_b_min_power", 0.0005);
    radar_filter_zone_c_min_power_ = declare_parameter<double>("radar_filter_zone_c_min_power", 0.0003);
    radar_filter_zone_d_min_power_ = declare_parameter<double>("radar_filter_zone_d_min_power", 0.0002);
    radar_filter_min_point_score_ = declare_parameter<double>("radar_filter_min_point_score", 0.8);
    radar_filter_score_intensity_scale_ =
      declare_parameter<double>("radar_filter_score_intensity_scale", 8.0);
    radar_filter_history_frames_ = declare_parameter<int>("radar_filter_history_frames", 3);
    radar_filter_fail_open_ = declare_parameter<bool>("radar_filter_fail_open", true);
    radar_filter_local_cfar_enabled_ = declare_parameter<bool>("radar_filter_local_cfar_enabled", true);
    radar_filter_local_range_bin_m_ = declare_parameter<double>("radar_filter_local_range_bin_m", 5.0);
    radar_filter_local_azimuth_bin_deg_ =
      declare_parameter<double>("radar_filter_local_azimuth_bin_deg", 1.0);
    radar_filter_local_guard_cells_ = declare_parameter<int>("radar_filter_local_guard_cells", 1);
    radar_filter_local_train_cells_ = declare_parameter<int>("radar_filter_local_train_cells", 3);
    radar_filter_local_os_quantile_ = declare_parameter<double>("radar_filter_local_os_quantile", 0.75);
    radar_filter_local_cfar_scale_ = declare_parameter<double>("radar_filter_local_cfar_scale", 1.25);
    detected_suppression_radius_ = declare_parameter<double>("detected_suppression_radius", 9.0);
    confirmation_radius_ = declare_parameter<double>("confirmation_radius", 7.0);
    semantic_confirmation_radius_ = declare_parameter<double>("semantic_confirmation_radius", 12.0);
    enable_spatial_fallback_confirmation_ = declare_parameter<bool>("enable_spatial_fallback_confirmation", true);
    spatial_confirmation_min_points_ = declare_parameter<int>("spatial_confirmation_min_points", 8);
    spatial_confirmation_min_sources_ = declare_parameter<int>("spatial_confirmation_min_sources", 2);
    spatial_confirmation_min_frames_ = declare_parameter<int>("spatial_confirmation_min_frames", 3);
    spatial_confirmation_max_spread_m_ = declare_parameter<double>("spatial_confirmation_max_spread_m", 4.0);
    spatial_confirmation_association_m_ =
      declare_parameter<double>("spatial_confirmation_association_m", 3.0);
    spatial_confirmation_timeout_s_ =
      declare_parameter<double>("spatial_confirmation_timeout_s", 1.2);
    object_association_radius_ = declare_parameter<double>("object_association_radius", 14.0);
    object_semantic_maintenance_radius_ =
      declare_parameter<double>("object_semantic_maintenance_radius", 12.0);
    object_semantic_maintenance_min_confidence_ =
      declare_parameter<double>("object_semantic_maintenance_min_confidence", 0.22);
    detected_object_timeout_s_ = declare_parameter<double>("detected_object_timeout_s", 30.0);
    ekf_process_noise_ = declare_parameter<double>("ekf_process_noise", 0.35);
    ekf_measurement_noise_ = declare_parameter<double>("ekf_measurement_noise", 0.9);
    drone_goal_altitude_ = declare_parameter<double>("drone_goal_altitude", 10.0);
    drone_goal_grid_m_ = declare_parameter<double>("drone_goal_grid_m", 1.0);
    drone_dispatch_min_range_m_ = declare_parameter<double>("drone_dispatch_min_range_m", 30.0);
    uav_goal_stability_window_s_ = declare_parameter<double>("uav_goal_stability_window_s", 5.0);
    uav_goal_update_period_s_ = declare_parameter<double>("uav_goal_update_period_s", 5.0);
    uav_goal_min_stability_ratio_ = declare_parameter<double>("uav_goal_min_stability_ratio", 0.65);
    uav_goal_min_samples_ = declare_parameter<int>("uav_goal_min_samples", 12);
    inspection_cluster_association_m_ = declare_parameter<double>("inspection_cluster_association_m", 7.0);
    inspection_cluster_memory_s_ = declare_parameter<double>("inspection_cluster_memory_s", 600.0);
    uav_inspection_dwell_s_ = declare_parameter<double>("uav_inspection_dwell_s", 4.0);
    uav_confirmation_class_ids_ = declare_parameter<std::vector<double>>(
      "uav_confirmation_class_ids", std::vector<double>{0.0, 1.0, 3.0});
    uav_confirmation_min_confidence_ = declare_parameter<double>("uav_confirmation_min_confidence", 0.15);
    uav_direct_confirmation_min_confidence_ =
      declare_parameter<double>("uav_direct_confirmation_min_confidence", 0.35);
    main_direct_confirmation_min_confidence_ =
      declare_parameter<double>("main_direct_confirmation_min_confidence", 0.30);
    uav_gated_confidence_weight_ = declare_parameter<double>("uav_gated_confidence_weight", 1.20);
    uav_depth_confidence_weight_ = declare_parameter<double>("uav_depth_confidence_weight", 0.90);
    uav_dispatch_min_yolo_confidence_ =
      declare_parameter<double>("uav_dispatch_min_yolo_confidence", 0.15);
    uav_dispatch_association_m_ = declare_parameter<double>("uav_dispatch_association_m", 12.0);
    visual_candidate_cluster_radius_m_ =
      declare_parameter<double>("visual_candidate_cluster_radius_m", 4.0);
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
    if (heatmap_mode_ != "vote" && heatmap_mode_ != "neural") {
      RCLCPP_WARN(
        get_logger(), "Unknown heatmap_mode '%s'; using vote", heatmap_mode_.c_str());
      heatmap_mode_ = "vote";
    }
    if (heatmap_mode_ == "neural") {
      initialize_neural_heatmap_model();
    }
    sonar_frame_windows_.resize(sonar_scan_topics_.size());

    for (const auto & topic : radar_topics_) {
      radar_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_cloud(msg, radar_buffer_, kSrcRadar, 0.65);}));
    }
    for (std::size_t i = 0; i < sonar_scan_topics_.size(); ++i) {
      const double yaw = sonar_scan_yaws_[i];
      sonar_subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
        sonar_scan_topics_[i], rclcpp::SensorDataQoS(),
        [this, yaw, i](sensor_msgs::msg::LaserScan::SharedPtr msg) {on_sonar_scan(msg, yaw, i);}));
    }
    if (!sonar_detect_topic_.empty()) {
      sonar_detect_sub_ = create_subscription<c3_sonar_driver::msg::SonarDetect>(
        sonar_detect_topic_, 10,
        [this](c3_sonar_driver::msg::SonarDetect::SharedPtr msg) {on_sonar_detect(msg);});
    }
    add_vision_subscriptions(normal_camera_topics_, kSrcVisionNormal, 1.00);
    add_vision_subscriptions(gated_camera_topics_, kSrcVisionGated, 1.12);
    add_vision_subscription(stf_camera_topic_, kSrcVisionStf, 0.80);
    add_vision_subscription(bev_topic_, kSrcVisionBev, 0.70);
    add_vision_subscriptions(uav_gated_topics_, kSrcVisionUav, uav_gated_confidence_weight_);
    add_vision_subscriptions(uav_depth_topics_, kSrcVisionUavDepthYolo, uav_depth_confidence_weight_);
    add_vision_subscription(depth_camera_detection_topic_, kSrcVisionDepthYolo, 1.00);
    depth_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      depth_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_cloud(msg, depth_buffer_, kSrcDepth, 0.35);});
    uav_observation_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      uav_observation_active_topic_, 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        const bool active = msg && msg->data;
        if (active && !uav_observation_active_ && active_inspection_cluster_id_ >= 0) {
          active_observation_started_ = now();
        }
        if (!active) {
          active_observation_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        uav_observation_active_ = active;
      });
    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic_, 10, std::bind(&C3MultimodalBufferFusion::on_model_states, this, std::placeholders::_1));

    radar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/c3/buffer/radar_cloud", 10);
    sonar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/c3/buffer/sonar_cloud", 10);
    vision_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/c3/buffer/vision_cloud", 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/c3/buffer/depth_cloud", 10);
    integrated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/c3/buffer/integrated_cloud", 10);
    heatmap_pub_ = create_publisher<sensor_msgs::msg::Image>("/c3/heatmap/image", 10);
    cluster_overlay_pub_ = create_publisher<sensor_msgs::msg::Image>("/c3/heatmap/cluster_overlay", 10);
    inspection_clusters_pub_ = create_publisher<sensor_msgs::msg::Image>("/c3/inspection_clusters/image", 10);
    neural_weights_pub_ = create_publisher<sensor_msgs::msg::Image>("/c3/heatmap/modality_weights", 10);
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
        // Keep patrol-time UAV detections out of the heatmap, but retain them
        // as the high-confidence visual gate required before dispatching a UAV.
        on_cloud(msg, vision_buffer_, source_id, confidence_scale);
      }));
  }

  void add_vision_subscriptions(
    const std::vector<std::string> & topics, float source_id, double confidence_scale)
  {
    for (const auto & topic : topics) {
      add_vision_subscription(topic, source_id, confidence_scale);
    }
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
    if (std::abs(source_id - kSrcRadar) < 0.5F) {
      frame.points = filter_radar_detection_points(frame.points);
    }
    if (!frame.points.empty()) {
      buffer.push_back(std::move(frame));
      prune_buffer(buffer, now());
    }
  }

  void on_sonar_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg,
    double mount_yaw,
    std::size_t sector_index)
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
      if (sector_index >= sonar_frame_windows_.size()) {
        sonar_frame_windows_.resize(sector_index + 1);
      }
      auto & sector_window = sonar_frame_windows_[sector_index];
      sector_window.push_back(frame);
      const std::size_t window_size =
        static_cast<std::size_t>(std::max(1, sonar_fusion_window_frames_));
      while (sector_window.size() > window_size) {
        sector_window.pop_front();
      }

      TimedCloud fused_frame;
      fused_frame.stamp = frame.stamp;
      for (const auto & history_frame : sector_window) {
        fused_frame.points.insert(
          fused_frame.points.end(), history_frame.points.begin(), history_frame.points.end());
      }
      sonar_buffer_.push_back(std::move(fused_frame));
      prune_buffer(sonar_buffer_, now(), sonar_buffer_keep_s_);
    }
  }

  void on_sonar_detect(const c3_sonar_driver::msg::SonarDetect::SharedPtr msg)
  {
    if (!msg || !std::isfinite(msg->confidence) || msg->confidence <= 0.0F) {
      return;
    }
    TimedCloud frame;
    frame.stamp = rclcpp::Time(msg->header.stamp);
    if (frame.stamp.nanoseconds() == 0) {
      frame.stamp = now();
    }
    // The simulator already produces a finite-width, delayed and noisy acoustic
    // return in SonarDetect.cloud.  Retaining that beam footprint prevents a
    // whole sonar measurement from being collapsed to one artificial point.
    if (msg->cloud.width * msg->cloud.height > 0U) {
      frame.points = parse_cloud(msg->cloud, kSrcSonar, std::clamp<double>(msg->confidence, 0.05, 1.0));
    }
    if (frame.points.empty()) {
      StandardPoint point;
      point.x = std::isfinite(msg->position.x) ? static_cast<float>(msg->position.x) :
        static_cast<float>(msg->range_m * std::cos(msg->bearing_rad));
      point.y = std::isfinite(msg->position.y) ? static_cast<float>(msg->position.y) :
        static_cast<float>(msg->range_m * std::sin(msg->bearing_rad));
      point.z = std::isfinite(msg->position.z) ? static_cast<float>(msg->position.z) : 0.0F;
      point.intensity = std::clamp(msg->confidence, 0.0F, 1.0F);
      point.class_id = kUnknownClass;
      point.source_id = kSrcSonar;
      frame.points.push_back(point);
    }
    sonar_buffer_.push_back(std::move(frame));
    prune_buffer(sonar_buffer_, now(), sonar_buffer_keep_s_);
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
    const int offset_radial_velocity = field_offset(msg, "radial_velocity");
    const int offset_power = field_offset(msg, "power");
    const int offset_snr = field_offset(msg, "snr");
    const int offset_range = field_offset(msg, "range");
    const int offset_azimuth_deg = field_offset(msg, "azimuth_deg");

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
      if (std::hypot(point.x, point.y) > max_range_m_) {
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
      point.radial_velocity = offset_radial_velocity >= 0 ? read_float(base, offset_radial_velocity) : 0.0F;
      if (!std::isfinite(point.radial_velocity)) {
        point.radial_velocity = 0.0F;
      }
      point.power = offset_power >= 0 ? read_float(base, offset_power) : point.intensity;
      point.snr = offset_snr >= 0 ? read_float(base, offset_snr) : point.intensity;
      point.range = offset_range >= 0 ? read_float(base, offset_range) : std::hypot(point.x, point.y);
      const float azimuth_deg = offset_azimuth_deg >= 0 ?
        read_float(base, offset_azimuth_deg) :
        static_cast<float>(std::atan2(point.y, point.x) * 180.0 / M_PI);
      point.azimuth_rad = static_cast<float>(azimuth_deg * M_PI / 180.0);
      if (!std::isfinite(point.power)) {
        point.power = 0.0F;
      }
      if (!std::isfinite(point.snr)) {
        point.snr = 0.0F;
      }
      if (!std::isfinite(point.range) || point.range <= 0.0F) {
        point.range = std::hypot(point.x, point.y);
      }
      if (!std::isfinite(point.azimuth_rad)) {
        point.azimuth_rad = static_cast<float>(std::atan2(point.y, point.x));
      }
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

  void initialize_neural_heatmap_model()
  {
    if (neural_heatmap_model_path_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "heatmap_mode=neural requires neural_heatmap_model_path; neural heatmap is disabled");
      return;
    }
    try {
      if (neural_execution_provider_ != "auto" && neural_execution_provider_ != "cuda" &&
        neural_execution_provider_ != "cpu")
      {
        RCLCPP_WARN(
          get_logger(), "Unknown neural_execution_provider '%s'; using auto",
          neural_execution_provider_.c_str());
        neural_execution_provider_ = "auto";
      }
      if (neural_execution_provider_ == "auto" || neural_execution_provider_ == "cuda") {
        try {
          neural_session_ = create_neural_session(true);
          neural_active_execution_provider_ = "cuda";
        } catch (const std::exception & exc) {
          RCLCPP_WARN(
            get_logger(), "CUDA Execution Provider is unavailable (%s); falling back to CPU", exc.what());
        }
      }
      if (!neural_session_) {
        neural_session_ = create_neural_session(false);
        neural_active_execution_provider_ = "cpu";
      }
      if (neural_session_->GetInputCount() != neural_input_names_.size() ||
        neural_session_->GetOutputCount() != neural_output_names_.size())
      {
        throw std::runtime_error("unexpected position-confidence ONNX input/output count");
      }
      const auto confidence_shape = neural_session_->GetOutputTypeInfo(0).
        GetTensorTypeAndShapeInfo().GetShape();
      const int expected_width = static_cast<int>(std::ceil(
        (heatmap_x_max_ - heatmap_x_min_) / heatmap_resolution_m_));
      const int expected_height = static_cast<int>(std::ceil(
        (heatmap_y_max_ - heatmap_y_min_) / heatmap_resolution_m_));
      if (confidence_shape.size() != 4 || confidence_shape[0] != 1 || confidence_shape[1] != 1 ||
        confidence_shape[2] != expected_height || confidence_shape[3] != expected_width)
      {
        throw std::runtime_error("ONNX confidence_map grid does not match configured heatmap grid");
      }
      neural_heatmap_ready_ = true;
      RCLCPP_INFO(
        get_logger(), "Neural heatmap enabled: %s (%dx%d grid, provider=%s, intra_op_threads=%d)",
        neural_heatmap_model_path_.c_str(), expected_width, expected_height,
        neural_active_execution_provider_.c_str(), std::max(1, neural_intra_op_threads_));
    } catch (const std::exception & exc) {
      neural_session_.reset();
      neural_heatmap_ready_ = false;
      RCLCPP_ERROR(
        get_logger(), "Could not load neural heatmap model '%s': %s",
        neural_heatmap_model_path_.c_str(), exc.what());
    }
  }

  std::unique_ptr<Ort::Session> create_neural_session(bool use_cuda)
  {
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(std::max(1, neural_intra_op_threads_));
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    if (use_cuda) {
      OrtCUDAProviderOptions cuda_options{};
      cuda_options.device_id = neural_cuda_device_id_;
      options.AppendExecutionProvider_CUDA(cuda_options);
    }
    return std::make_unique<Ort::Session>(neural_ort_env_, neural_heatmap_model_path_.c_str(), options);
  }

  std::vector<StandardPoint> collect_source(
    const std::vector<StandardPoint> & points, float source_id) const
  {
    std::vector<StandardPoint> selected;
    selected.reserve(points.size());
    for (const auto & point : points) {
      if (std::abs(point.source_id - source_id) < 0.5F) {
        selected.push_back(point);
      }
    }
    return selected;
  }

  std::vector<float> neural_points(
    const std::vector<StandardPoint> & points, bool optical_depth_frame, bool with_velocity) const
  {
    const std::size_t max_points = static_cast<std::size_t>(
      std::max(1, neural_max_points_per_modality_));
    const std::size_t stride = std::max<std::size_t>(1, (points.size() + max_points - 1) / max_points);
    std::vector<float> result;
    result.reserve(std::min(points.size(), max_points) * 3U);
    for (std::size_t i = 0; i < points.size(); i += stride) {
      float x = points[i].x;
      float y = points[i].y;
      float z = points[i].z;
      if (optical_depth_frame && neural_depth_optical_frame_) {
        // The training recorder applies this exact optical-camera to base_link conversion.
        x = static_cast<float>(neural_depth_x_offset_m_) + points[i].z;
        y = static_cast<float>(neural_depth_y_offset_m_) - points[i].x;
        z = static_cast<float>(neural_depth_z_offset_m_) - points[i].y;
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        x < heatmap_x_min_ || x >= heatmap_x_max_ || y < heatmap_y_min_ || y >= heatmap_y_max_)
      {
        continue;
      }
      result.push_back(x);
      result.push_back(y);
      result.push_back(with_velocity ? points[i].radial_velocity : z);
    }
    return result;
  }

  std::vector<StandardPoint> downsample_modal_points(const std::vector<StandardPoint> & points) const
  {
    const std::size_t maximum = static_cast<std::size_t>(std::max(1, neural_max_points_per_modality_));
    const std::size_t stride = std::max<std::size_t>(1, (points.size() + maximum - 1U) / maximum);
    std::vector<StandardPoint> output;
    output.reserve(std::min(points.size(), maximum));
    for (std::size_t i = 0; i < points.size(); i += stride) {
      output.push_back(points[i]);
    }
    return output;
  }

  int radar_range_zone(double range_m) const
  {
    if (range_m < radar_filter_zone_a_max_m_) {
      return 0;
    }
    if (range_m < radar_filter_zone_b_max_m_) {
      return 1;
    }
    if (range_m < radar_filter_zone_c_max_m_) {
      return 2;
    }
    return 3;
  }

  bool radar_point_hard_filter(const StandardPoint & point) const
  {
    if (!std::isfinite(point.range) || point.range <= 0.0F ||
      !std::isfinite(point.azimuth_rad) || !std::isfinite(point.radial_velocity))
    {
      return false;
    }

    const int zone = radar_range_zone(point.range);
    if (zone == 0) {
      return false;
    }
    if (zone == 1) {
      return point.snr >= radar_filter_zone_b_min_snr_ &&
             point.power >= radar_filter_zone_b_min_power_;
    }
    if (zone == 2) {
      return point.snr >= radar_filter_zone_c_min_snr_ &&
             point.power >= radar_filter_zone_c_min_power_;
    }
    return point.snr >= radar_filter_zone_d_min_snr_ &&
           point.power >= radar_filter_zone_d_min_power_;
  }

  std::pair<double, double> radar_power_score_thresholds(int zone) const
  {
    if (zone == 1) {
      return {radar_filter_zone_b_min_power_, 2.0 * radar_filter_zone_b_min_power_};
    }
    if (zone == 2) {
      return {radar_filter_zone_c_min_power_, 2.0 * radar_filter_zone_c_min_power_};
    }
    return {radar_filter_zone_d_min_power_, 2.0 * radar_filter_zone_d_min_power_};
  }

  double radar_detection_strength(const StandardPoint & point) const
  {
    return static_cast<double>(std::max(0.0F, point.power)) +
           0.05 * static_cast<double>(std::max(0.0F, point.snr));
  }

  double radar_point_score(const StandardPoint & point) const
  {
    const int zone = radar_range_zone(point.range);
    double score = 0.0;
    if (zone == 1) {
      if (point.snr >= 5.5F) {
        score += 4.0;
      } else if (point.snr >= 4.0F) {
        score += 3.0;
      } else if (point.snr >= 3.0F) {
        score += 1.5;
      }
    } else {
      if (point.snr >= 5.5F) {
        score += 4.0;
      } else if (point.snr >= 4.0F) {
        score += 3.0;
      } else if (point.snr >= 2.5F) {
        score += 1.5;
      }
    }

    const auto [low_power, mid_power] = radar_power_score_thresholds(zone);
    if (point.power >= mid_power) {
      score += 3.0;
    } else if (point.power >= low_power) {
      score += 1.5;
    }
    if (zone == 2) {
      score += 0.5;
    } else if (zone == 3) {
      score += 1.0;
    }
    if (std::abs(point.radial_velocity) >= 0.2F) {
      score += 0.5;
    }
    return score;
  }

  std::vector<StandardPoint> radar_local_adaptive_detection(
    const std::vector<StandardPoint> & points) const
  {
    if (!radar_filter_local_cfar_enabled_ || points.empty()) {
      return points;
    }

    std::vector<StandardPoint> kept;
    kept.reserve(points.size());
    const double az_bin_rad = std::max(1e-6, radar_filter_local_azimuth_bin_deg_ * M_PI / 180.0);
    const double range_bin = std::max(1e-6, radar_filter_local_range_bin_m_);
    const int guard_cells = std::max(0, radar_filter_local_guard_cells_);
    const int train_cells = std::max(1, radar_filter_local_train_cells_);

    for (const auto & point : points) {
      std::vector<double> reference_values;
      reference_values.reserve(points.size());
      const int point_range_bin = static_cast<int>(
        std::floor(static_cast<double>(point.range) / range_bin));
      const int point_azimuth_bin = static_cast<int>(
        std::floor(static_cast<double>(point.azimuth_rad) / az_bin_rad));

      for (const auto & other : points) {
        const int other_range_bin = static_cast<int>(
          std::floor(static_cast<double>(other.range) / range_bin));
        const int other_azimuth_bin = static_cast<int>(
          std::floor(static_cast<double>(other.azimuth_rad) / az_bin_rad));
        const int dr = std::abs(other_range_bin - point_range_bin);
        const int da = std::abs(other_azimuth_bin - point_azimuth_bin);
        const bool in_guard = dr <= guard_cells && da <= guard_cells;
        const bool in_train = dr <= guard_cells + train_cells && da <= guard_cells + train_cells;
        if (!in_train || in_guard) {
          continue;
        }
        reference_values.push_back(radar_detection_strength(other));
      }

      if (reference_values.size() < 6U) {
        kept.push_back(point);
        continue;
      }

      std::sort(reference_values.begin(), reference_values.end());
      const double quantile = std::clamp(radar_filter_local_os_quantile_, 0.0, 1.0);
      const auto os_index = std::min(
        reference_values.size() - 1U,
        static_cast<std::size_t>(
          std::floor(quantile * static_cast<double>(reference_values.size() - 1U))));
      const double threshold = radar_filter_local_cfar_scale_ * reference_values[os_index];
      if (radar_detection_strength(point) >= threshold) {
        kept.push_back(point);
      }
    }
    return kept;
  }

  std::vector<StandardPoint> filter_radar_detection_points(
    const std::vector<StandardPoint> & raw_points)
  {
    if (!radar_point_filter_enabled_ || raw_points.empty()) {
      return raw_points;
    }

    radar_filter_history_.push_back(raw_points);
    const std::size_t history_limit = static_cast<std::size_t>(
      std::clamp(radar_filter_history_frames_, 1, 4));
    while (radar_filter_history_.size() > history_limit) {
      radar_filter_history_.pop_front();
    }

    std::vector<StandardPoint> fused_raw;
    std::size_t total_size = 0;
    for (const auto & frame : radar_filter_history_) {
      total_size += frame.size();
    }
    fused_raw.reserve(total_size);
    for (const auto & frame : radar_filter_history_) {
      fused_raw.insert(fused_raw.end(), frame.begin(), frame.end());
    }

    std::vector<StandardPoint> scored;
    scored.reserve(fused_raw.size());
    for (auto point : fused_raw) {
      if (!radar_point_hard_filter(point)) {
        continue;
      }
      point.point_score = static_cast<float>(radar_point_score(point));
      if (point.point_score < radar_filter_min_point_score_) {
        continue;
      }
      const double score_intensity = static_cast<double>(point.point_score) /
        std::max(1e-3, radar_filter_score_intensity_scale_);
      point.intensity = static_cast<float>(
        std::clamp(std::max(static_cast<double>(point.intensity), score_intensity), 0.0, 1.0));
      scored.push_back(point);
    }
    if (scored.empty() && radar_filter_fail_open_) {
      return fused_raw.empty() ? raw_points : fused_raw;
    }
    auto filtered = radar_local_adaptive_detection(scored);
    if (filtered.empty() && radar_filter_fail_open_) {
      return scored.empty() ? fused_raw : scored;
    }
    return filtered;
  }

  std::vector<StandardPoint> filter_neural_modal_clutter(
    const std::vector<StandardPoint> & raw_points, float source_id, const rclcpp::Time & stamp)
  {
    const auto points = downsample_modal_points(raw_points);
    if (!neural_clutter_filter_enabled_ || points.empty()) {
      return points;
    }

    const bool is_radar = std::abs(source_id - kSrcRadar) < 0.5F;
    const double radius = std::max(0.5, neural_clutter_cluster_radius_m_);
    const int min_cluster_points = is_radar ? neural_radar_min_cluster_points_ :
      neural_clutter_min_cluster_points_;
    const double mahalanobis_gate = is_radar ? neural_radar_mahalanobis_gate_ :
      neural_clutter_mahalanobis_gate_;
    const double confirmation_interval = is_radar ? neural_radar_confirmation_interval_s_ :
      neural_clutter_confirmation_interval_s_;
    const int min_track_hits = is_radar ? neural_radar_min_track_hits_ : neural_clutter_min_track_hits_;
    const double residual_mean_max = is_radar ? neural_radar_residual_mean_max_ :
      neural_clutter_residual_mean_max_;
    const double cfar_scale = is_radar ? neural_radar_cfar_scale_ : neural_clutter_cfar_scale_;
    std::vector<NeuralModalCluster> clusters;
    for (const auto & point : points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        continue;
      }
      int best = -1;
      double best_distance = radius;
      for (std::size_t i = 0; i < clusters.size(); ++i) {
        const double distance = std::hypot(point.x - clusters[i].x, point.y - clusters[i].y);
        if (distance < best_distance) {
          best = static_cast<int>(i);
          best_distance = distance;
        }
      }
      if (best < 0) {
        NeuralModalCluster cluster;
        cluster.source_id = source_id;
        cluster.x = point.x;
        cluster.y = point.y;
        cluster.intensity = std::max(0.0F, point.intensity);
        cluster.radial_velocity = point.radial_velocity;
        cluster.count = 1;
        clusters.push_back(cluster);
        continue;
      }
      auto & cluster = clusters[static_cast<std::size_t>(best)];
      const double weight = static_cast<double>(cluster.count);
      cluster.x = (cluster.x * weight + point.x) / (weight + 1.0);
      cluster.y = (cluster.y * weight + point.y) / (weight + 1.0);
      cluster.intensity = (cluster.intensity * weight + std::max(0.0F, point.intensity)) / (weight + 1.0);
      cluster.radial_velocity = (cluster.radial_velocity * weight + point.radial_velocity) / (weight + 1.0);
      ++cluster.count;
    }

    // Ordered-statistic local background estimate. A new cluster must stand out
    // locally or persist across several independent sensor updates.
    for (auto & cluster : clusters) {
      std::vector<double> background;
      const double range = std::hypot(cluster.x, cluster.y);
      const double bearing = std::atan2(cluster.y, cluster.x);
      for (const auto & other : clusters) {
        const double other_range = std::hypot(other.x, other.y);
        const double other_bearing = std::atan2(other.y, other.x);
        if (std::abs(other_range - range) > 10.0 || std::abs(wrap_angle(other_bearing - bearing)) > 0.30) {
          continue;
        }
        const double separation = std::hypot(other.x - cluster.x, other.y - cluster.y);
        if (separation > radius * 1.25) {
          background.push_back(other.intensity);
        }
      }
      if (background.size() >= 6U) {
        std::sort(background.begin(), background.end());
        cluster.cfar_background = background[(background.size() - 1U) * 3U / 4U];
        cluster.cfar_pass = cluster.intensity >= cfar_scale * cluster.cfar_background;
      }
    }

    std::vector<bool> updated(neural_clutter_tracks_.size(), false);
    for (auto & track : neural_clutter_tracks_) {
      const double dt = std::clamp((stamp - track.last_prediction).seconds(), 0.0, 2.0);
      track.x += track.vx * dt;
      track.y += track.vy * dt;
      track.variance += 0.8 * dt * dt;
      track.last_prediction = stamp;
    }

    for (auto & cluster : clusters) {
      int best = -1;
      double best_nis = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < neural_clutter_tracks_.size(); ++i) {
        auto & track = neural_clutter_tracks_[i];
        if (updated[i] || std::abs(track.source_id - source_id) > 0.5F) {
          continue;
        }
        const double dx = cluster.x - track.x;
        const double dy = cluster.y - track.y;
        const double variance = std::max(0.25, track.variance +
          neural_clutter_measurement_noise_m_ * neural_clutter_measurement_noise_m_);
        double nis = (dx * dx + dy * dy) / variance;
        if (std::abs(source_id - kSrcRadar) < 0.5F && track.hits >= 2 &&
          std::isfinite(cluster.radial_velocity))
        {
          const double predicted_velocity = (track.vx * cluster.x + track.vy * cluster.y) /
            std::max(1.0, std::hypot(cluster.x, cluster.y));
          const double velocity_error = cluster.radial_velocity - predicted_velocity;
          nis += velocity_error * velocity_error /
            std::max(0.05, neural_clutter_velocity_noise_mps_ * neural_clutter_velocity_noise_mps_);
        }
        if (nis <= mahalanobis_gate && nis < best_nis) {
          best = static_cast<int>(i);
          best_nis = nis;
        }
      }
      if (best < 0) {
        NeuralClutterTrack track;
        track.source_id = source_id;
        track.x = cluster.x;
        track.y = cluster.y;
        track.radial_velocity = cluster.radial_velocity;
        track.hits = 1;
        track.last_prediction = stamp;
        track.last_measurement = stamp;
        neural_clutter_tracks_.push_back(track);
        updated.push_back(true);
        cluster.track_index = static_cast<int>(neural_clutter_tracks_.size() - 1U);
        continue;
      }

      auto & track = neural_clutter_tracks_[static_cast<std::size_t>(best)];
      const double elapsed = (stamp - track.last_measurement).seconds();
      if (elapsed >= confirmation_interval) {
        const double observation_vx = (cluster.x - track.x) / std::max(elapsed, 0.05);
        const double observation_vy = (cluster.y - track.y) / std::max(elapsed, 0.05);
        track.vx = 0.70 * track.vx + 0.30 * observation_vx;
        track.vy = 0.70 * track.vy + 0.30 * observation_vy;
        ++track.hits;
        track.last_measurement = stamp;
        track.nis_history.push_back(best_nis);
        while (track.nis_history.size() > 6U) {
          track.nis_history.pop_front();
        }
      }
      track.x = 0.65 * track.x + 0.35 * cluster.x;
      track.y = 0.65 * track.y + 0.35 * cluster.y;
      track.radial_velocity = 0.70 * track.radial_velocity + 0.30 * cluster.radial_velocity;
      track.variance = std::max(0.5, 0.65 * track.variance);
      track.missed = 0;
      updated[static_cast<std::size_t>(best)] = true;
      cluster.track_index = best;
    }

    for (std::size_t i = 0; i < neural_clutter_tracks_.size(); ++i) {
      if (!updated[i]) {
        ++neural_clutter_tracks_[i].missed;
      }
    }
    std::vector<NeuralModalCluster> accepted;
    for (const auto & cluster : clusters) {
      if (cluster.track_index < 0 || static_cast<std::size_t>(cluster.track_index) >= neural_clutter_tracks_.size()) {
        continue;
      }
      const auto & track = neural_clutter_tracks_[static_cast<std::size_t>(cluster.track_index)];
      const double mean_nis = track.nis_history.empty() ? 0.0 :
        std::accumulate(track.nis_history.begin(), track.nis_history.end(), 0.0) /
        static_cast<double>(track.nis_history.size());
      const bool spatial_support = cluster.count >= min_cluster_points;
      const bool stable = track.hits >= min_track_hits && mean_nis <= residual_mean_max;
      if (spatial_support && stable && (cluster.cfar_pass || track.hits > min_track_hits)) {
        accepted.push_back(cluster);
      }
    }

    std::vector<StandardPoint> filtered;
    for (const auto & point : points) {
      for (const auto & cluster : accepted) {
        if (std::hypot(point.x - cluster.x, point.y - cluster.y) <= radius * 1.35) {
          filtered.push_back(point);
          break;
        }
      }
    }
    neural_clutter_tracks_.erase(
      std::remove_if(neural_clutter_tracks_.begin(), neural_clutter_tracks_.end(), [this, &stamp](const auto & track) {
        return (stamp - track.last_measurement).seconds() > neural_clutter_track_timeout_s_ || track.missed > 8;
      }), neural_clutter_tracks_.end());
    return filtered;
  }

  void publish_empty_heatmap(const std_msgs::msg::Header & header) const
  {
    const int width = std::max(1, static_cast<int>(std::ceil(
      (heatmap_x_max_ - heatmap_x_min_) / heatmap_resolution_m_)));
    const int height = std::max(1, static_cast<int>(std::ceil(
      (heatmap_y_max_ - heatmap_y_min_) / heatmap_resolution_m_)));
    sensor_msgs::msg::Image image;
    image.header = header;
    image.height = static_cast<std::uint32_t>(height);
    image.width = static_cast<std::uint32_t>(width);
    image.encoding = "mono8";
    image.step = static_cast<std::uint32_t>(width);
    image.data.assign(static_cast<std::size_t>(width * height), 0U);
    heatmap_pub_->publish(image);
  }

  void smooth_heatmap_scores(std::vector<double> & scores)
  {
    const double alpha = std::clamp(heatmap_temporal_smoothing_alpha_, 0.0, 1.0);
    if (alpha >= 0.999 || previous_neural_scores_.size() != scores.size()) {
      previous_neural_scores_ = scores;
      return;
    }
    for (std::size_t i = 0; i < scores.size(); ++i) {
      scores[i] = alpha * scores[i] + (1.0 - alpha) * previous_neural_scores_[i];
    }
    previous_neural_scores_ = scores;
  }

  std::optional<std::pair<double, double>> compute_neural_heatmap_and_candidate(
    const std::vector<StandardPoint> & radar,
    const std::vector<StandardPoint> & sonar,
    const std::vector<StandardPoint> & vision,
    const std::vector<StandardPoint> & depth,
    const std_msgs::msg::Header & header)
  {
    last_heatmap_probability_ = 0.0;
    if (!neural_heatmap_ready_ || !neural_session_) {
      publish_empty_heatmap(header);
      return std::nullopt;
    }

    try {
      const rclcpp::Time filter_stamp = rclcpp::Time(header.stamp).nanoseconds() == 0 ? now() : rclcpp::Time(header.stamp);
      const auto filtered_radar = filter_neural_modal_clutter(radar, kSrcRadar, filter_stamp);
      const auto filtered_sonar = filter_neural_modal_clutter(sonar, kSrcSonar, filter_stamp);
      last_neural_radar_input_points_ = filtered_radar.size();
      last_neural_sonar_input_points_ = filtered_sonar.size();
      auto dehaze_points = neural_points(depth, true, false);
      auto gated_points = neural_points(collect_source(vision, kSrcVisionGated), false, false);
      auto sonar_points = neural_points(filtered_sonar, false, false);
      auto radar_points = neural_points(filtered_radar, false, false);
      auto radar_velocity_points = neural_points(filtered_radar, false, true);
      std::array<std::vector<int64_t>, 5> batches{
        std::vector<int64_t>(dehaze_points.size() / 3U, 0),
        std::vector<int64_t>(gated_points.size() / 3U, 0),
        std::vector<int64_t>(sonar_points.size() / 3U, 0),
        std::vector<int64_t>(radar_points.size() / 3U, 0),
        std::vector<int64_t>(radar_velocity_points.size() / 3U, 0)};
      float empty_float = 0.0F;
      int64_t empty_int64 = 0;
      const auto make_points_tensor = [this, &empty_float](std::vector<float> & data) {
        const std::array<int64_t, 2> shape{static_cast<int64_t>(data.size() / 3U), 3};
        return Ort::Value::CreateTensor<float>(
          neural_memory_info_, data.empty() ? &empty_float : data.data(), data.size(), shape.data(), shape.size());
      };
      const auto make_batch_tensor = [this, &empty_int64](std::vector<int64_t> & data) {
        const std::array<int64_t, 1> shape{static_cast<int64_t>(data.size())};
        return Ort::Value::CreateTensor<int64_t>(
          neural_memory_info_, data.empty() ? &empty_int64 : data.data(), data.size(), shape.data(), shape.size());
      };
      std::array<Ort::Value, 10> inputs{
        make_points_tensor(dehaze_points), make_batch_tensor(batches[0]),
        make_points_tensor(gated_points), make_batch_tensor(batches[1]),
        make_points_tensor(sonar_points), make_batch_tensor(batches[2]),
        make_points_tensor(radar_points), make_batch_tensor(batches[3]),
        make_points_tensor(radar_velocity_points), make_batch_tensor(batches[4])};
      auto outputs = neural_session_->Run(
        Ort::RunOptions{nullptr}, neural_input_names_.data(), inputs.data(), inputs.size(),
        neural_output_names_.data(), neural_output_names_.size());
      if (outputs.size() != neural_output_names_.size() || !outputs[0].IsTensor()) {
        throw std::runtime_error("position-confidence ONNX returned invalid outputs");
      }

      const auto confidence_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
      const int height = static_cast<int>(confidence_shape[2]);
      const int width = static_cast<int>(confidence_shape[3]);
      const float * confidence = outputs[0].GetTensorData<float>();
      const float * offsets = outputs[1].GetTensorData<float>();
      const float * weights = outputs[3].GetTensorData<float>();
      if (confidence == nullptr || offsets == nullptr || weights == nullptr) {
        throw std::runtime_error("position-confidence ONNX returned empty tensors");
      }

      sensor_msgs::msg::Image heatmap;
      heatmap.header = header;
      heatmap.height = static_cast<std::uint32_t>(height);
      heatmap.width = static_cast<std::uint32_t>(width);
      heatmap.encoding = "mono8";
      heatmap.step = static_cast<std::uint32_t>(width);
      heatmap.data.resize(static_cast<std::size_t>(height * width));
      sensor_msgs::msg::Image modality_weights;
      modality_weights.header = header;
      modality_weights.height = static_cast<std::uint32_t>(height);
      modality_weights.width = static_cast<std::uint32_t>(width);
      modality_weights.encoding = "32FC4";
      modality_weights.step = static_cast<std::uint32_t>(width * 4 * sizeof(float));
      modality_weights.data.resize(static_cast<std::size_t>(height * width * 4 * sizeof(float)));

      std::vector<double> scores(static_cast<std::size_t>(height * width), 0.0);
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const std::size_t source = static_cast<std::size_t>(y * width + x);
          scores[source] = std::clamp(confidence[source], 0.0F, 1.0F);
        }
      }
      smooth_heatmap_scores(scores);
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const std::size_t source = static_cast<std::size_t>(y * width + x);
          const std::size_t destination = static_cast<std::size_t>((height - 1 - y) * width + x);
          const double score = scores[source];
          heatmap.data[destination] = static_cast<std::uint8_t>(std::lround(score * 255.0));
          for (int modality = 0; modality < 4; ++modality) {
            const float weight = weights[static_cast<std::size_t>(modality * height * width) + source];
            const std::size_t weight_offset =
              (destination * 4U + static_cast<std::size_t>(modality)) * sizeof(float);
            std::memcpy(modality_weights.data.data() + weight_offset, &weight, sizeof(float));
          }
        }
      }
      heatmap_pub_->publish(heatmap);
      neural_weights_pub_->publish(modality_weights);
      const auto peak_indices = select_peak_indices(scores, width, height, neural_heatmap_threshold_);
      if (peak_indices.empty()) {
        last_heatmap_peaks_.clear();
        return std::nullopt;
      }
      std::vector<HeatmapPeak> peaks;
      peaks.reserve(peak_indices.size());
      for (const int peak_index : peak_indices) {
        const int peak_x = peak_index % width;
        const int peak_y = peak_index / width;
        const std::size_t index = static_cast<std::size_t>(peak_index);
        peaks.push_back(HeatmapPeak{
          heatmap_x_min_ + (static_cast<double>(peak_x) + 0.5) * heatmap_resolution_m_ + offsets[index],
          heatmap_y_min_ + (static_cast<double>(peak_y) + 0.5) * heatmap_resolution_m_ +
            offsets[static_cast<std::size_t>(height * width) + index],
          scores[index], -1});
      }
      return finalize_heatmap_peaks(std::move(peaks), header);
    } catch (const std::exception & exc) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Neural heatmap inference failed: %s", exc.what());
      publish_empty_heatmap(header);
      return std::nullopt;
    }
  }

  std::vector<int> select_peak_indices(
    const std::vector<double> & scores, int width, int height, double threshold) const
  {
    std::vector<int> order;
    order.reserve(scores.size());
    for (std::size_t i = 0; i < scores.size(); ++i) {
      if (scores[i] >= threshold) {
        order.push_back(static_cast<int>(i));
      }
    }
    std::sort(order.begin(), order.end(), [&scores](int left, int right) {
      return scores[static_cast<std::size_t>(left)] > scores[static_cast<std::size_t>(right)];
    });

    std::vector<int> selected;
    const double min_distance = std::max(heatmap_resolution_m_, heatmap_peak_nms_radius_m_);
    for (const int index : order) {
      const int x = index % width;
      const int y = index / width;
      bool separated = true;
      for (const int previous : selected) {
        const int previous_x = previous % width;
        const int previous_y = previous / width;
        if (std::hypot(
            static_cast<double>(x - previous_x) * heatmap_resolution_m_,
            static_cast<double>(y - previous_y) * heatmap_resolution_m_) < min_distance)
        {
          separated = false;
          break;
        }
      }
      if (separated) {
        selected.push_back(index);
        if (selected.size() >= static_cast<std::size_t>(std::max(1, heatmap_max_peaks_))) {
          break;
        }
      }
    }
    return selected;
  }

  InspectionCluster * find_inspection_cluster(int id)
  {
    const auto it = std::find_if(
      inspection_clusters_.begin(), inspection_clusters_.end(), [id](const InspectionCluster & cluster) {
        return cluster.id == id;
      });
    return it == inspection_clusters_.end() ? nullptr : &(*it);
  }

  static void append_cluster_history(InspectionCluster & cluster)
  {
    constexpr std::size_t kMaxHistory = 160;
    if (cluster.history.empty() ||
      std::hypot(cluster.history.back()[0] - cluster.x, cluster.history.back()[1] - cluster.y) >= 0.25)
    {
      cluster.history.push_back({cluster.x, cluster.y});
    }
    while (cluster.history.size() > kMaxHistory) {
      cluster.history.pop_front();
    }
  }

  void predict_inspection_clusters(const rclcpp::Time & stamp)
  {
    for (auto & cluster : inspection_clusters_) {
      if (cluster.last_prediction.nanoseconds() == 0) {
        cluster.last_prediction = stamp;
        continue;
      }
      const double dt = std::clamp((stamp - cluster.last_prediction).seconds(), 0.0, 2.0);
      cluster.x += cluster.vx * dt;
      cluster.y += cluster.vy * dt;
      cluster.last_prediction = stamp;
      append_cluster_history(cluster);
    }
    inspection_clusters_.erase(
      std::remove_if(
        inspection_clusters_.begin(), inspection_clusters_.end(), [this, &stamp](const InspectionCluster & cluster) {
          return cluster.last_measurement.nanoseconds() != 0 &&
                 (stamp - cluster.last_measurement).seconds() > inspection_cluster_memory_s_;
        }),
      inspection_clusters_.end());
  }

  void update_inspection_clusters(std::vector<HeatmapPeak> & peaks, const rclcpp::Time & stamp)
  {
    predict_inspection_clusters(stamp);
    const double association = std::max(0.5, inspection_cluster_association_m_);
    for (auto & peak : peaks) {
      const auto world = relative_to_world(peak.x, peak.y, 0.0);
      InspectionCluster * best = nullptr;
      double best_distance = association;
      for (auto & cluster : inspection_clusters_) {
        const double distance = std::hypot(world[0] - cluster.x, world[1] - cluster.y);
        if (distance < best_distance) {
          best_distance = distance;
          best = &cluster;
        }
      }
      if (best == nullptr) {
        InspectionCluster cluster;
        cluster.id = next_inspection_cluster_id_++;
        cluster.x = world[0];
        cluster.y = world[1];
        cluster.last_prediction = stamp;
        cluster.last_measurement = stamp;
        append_cluster_history(cluster);
        inspection_clusters_.push_back(cluster);
        peak.cluster_id = cluster.id;
        continue;
      }

      // Alpha-beta filtering is stable for moving surface targets without requiring
      // an absolute-heading-stable measurement frame.
      const double update_dt = std::max(0.05, (stamp - best->last_measurement).seconds());
      const double error_x = world[0] - best->x;
      const double error_y = world[1] - best->y;
      constexpr double kAlpha = 0.48;
      constexpr double kBeta = 0.12;
      best->x += kAlpha * error_x;
      best->y += kAlpha * error_y;
      best->vx += kBeta * error_x / update_dt;
      best->vy += kBeta * error_y / update_dt;
      best->last_measurement = stamp;
      append_cluster_history(*best);
      peak.cluster_id = best->id;
    }
  }

  std::optional<std::pair<double, double>> finalize_heatmap_peaks(
    std::vector<HeatmapPeak> peaks, const std_msgs::msg::Header & header)
  {
    update_inspection_clusters(peaks, rclcpp::Time(header.stamp));
    last_heatmap_peaks_ = std::move(peaks);
    last_candidate_cluster_id_ = -1;

    // Finish the current inspection before selecting another peak. This prevents
    // normal heatmap fluctuations from redirecting the UAV mid-mission.
    if (active_inspection_cluster_id_ >= 0) {
      if (const InspectionCluster * active = find_inspection_cluster(active_inspection_cluster_id_);
        active != nullptr && !active->inspected)
      {
        const auto relative = world_to_relative(active->x, active->y);
        last_heatmap_probability_ = 0.0;
        for (const auto & peak : last_heatmap_peaks_) {
          if (peak.cluster_id == active->id) {
            last_heatmap_probability_ = peak.score;
            break;
          }
        }
        last_candidate_x_ = relative[0];
        last_candidate_y_ = relative[1];
        last_candidate_cluster_id_ = active->id;
        return std::make_pair(relative[0], relative[1]);
      }
      active_inspection_cluster_id_ = -1;
    }

    for (const auto & peak : last_heatmap_peaks_) {
      const InspectionCluster * cluster = find_inspection_cluster(peak.cluster_id);
      if (cluster != nullptr && (cluster->inspected || cluster->dispatched)) {
        continue;
      }
      last_heatmap_probability_ = peak.score;
      last_candidate_x_ = peak.x;
      last_candidate_y_ = peak.y;
      last_candidate_cluster_id_ = peak.cluster_id;
      return std::make_pair(peak.x, peak.y);
    }
    last_heatmap_probability_ = 0.0;
    return std::nullopt;
  }

  void mark_cluster_dispatched(const std::pair<double, double> & world_goal)
  {
    InspectionCluster * cluster = find_inspection_cluster(last_candidate_cluster_id_);
    if (cluster == nullptr) {
      return;
    }
    cluster->dispatched = true;
    active_inspection_cluster_id_ = cluster->id;
    (void)world_goal;
  }

  void mark_cluster_inspected(int id, const rclcpp::Time & stamp)
  {
    InspectionCluster * cluster = find_inspection_cluster(id);
    if (cluster == nullptr) {
      return;
    }
    cluster->inspected = true;
    cluster->last_inspection = stamp;
    active_inspection_cluster_id_ = -1;
    active_observation_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    uav_goal_history_.clear();
    RCLCPP_INFO(get_logger(), "UAV inspected heatmap cluster %d at world (%.1f, %.1f)",
      cluster->id, cluster->x, cluster->y);
  }

  void update_uav_recognition_records(
    const std::vector<VisualCandidate> & candidates, const rclcpp::Time & stamp)
  {
    if (!uav_observation_active_) {
      return;
    }
    for (const auto & candidate : candidates) {
      if (candidate.confidence < uav_confirmation_min_confidence_ ||
        !is_uav_confirmation_class(static_cast<float>(candidate.class_id)))
      {
        continue;
      }
      const auto world = relative_to_world(candidate.x, candidate.y, candidate.z);
      InspectionCluster * best = nullptr;
      double best_distance = std::max(0.5, uav_dispatch_association_m_);
      for (auto & cluster : inspection_clusters_) {
        const double distance = std::hypot(world[0] - cluster.x, world[1] - cluster.y);
        if (distance < best_distance) {
          best_distance = distance;
          best = &cluster;
        }
      }
      if (best != nullptr) {
        best->uav_recognized = true;
        best->recognized_class_id = candidate.class_id;
        best->recognized_confidence = std::max(best->recognized_confidence, candidate.confidence);
        best->x = 0.65 * best->x + 0.35 * world[0];
        best->y = 0.65 * best->y + 0.35 * world[1];
        best->last_measurement = stamp;
      }
    }
  }

  bool is_uav_confirmation_class(float class_id) const
  {
    if (class_id < 0.0F) {
      return false;
    }
    return std::any_of(
      uav_confirmation_class_ids_.begin(), uav_confirmation_class_ids_.end(),
      [class_id](double expected) {return std::abs(class_id - expected) < 0.5F;});
  }

  double unscaled_visual_confidence(const StandardPoint & point) const
  {
    double scale = 1.0;
    if (std::abs(point.source_id - kSrcVisionUav) < 0.5F) {
      scale = uav_gated_confidence_weight_;
    } else if (std::abs(point.source_id - kSrcVisionUavDepthYolo) < 0.5F) {
      scale = uav_depth_confidence_weight_;
    } else if (std::abs(point.source_id - kSrcVisionGated) < 0.5F) {
      scale = 1.12;
    }
    return std::clamp(static_cast<double>(point.intensity) / std::max(0.05, scale), 0.0, 1.0);
  }

  std::vector<VisualCandidate> rank_visual_candidates(
    const std::vector<StandardPoint> & vision, bool uav_sources) const
  {
    std::vector<VisualCandidate> candidates;
    const double radius = std::max(0.5, visual_candidate_cluster_radius_m_);
    for (const auto & point : vision) {
      if ((is_uav_vision_source(point.source_id) != uav_sources) || point.class_id < 0.0F) {
        continue;
      }
      if (!uav_sources &&
        std::abs(point.source_id - kSrcVisionNormal) >= 0.5F &&
        std::abs(point.source_id - kSrcVisionGated) >= 0.5F &&
        std::abs(point.source_id - kSrcVisionDepthYolo) >= 0.5F)
      {
        continue;
      }

      const int class_id = static_cast<int>(std::lround(point.class_id));
      const double confidence = unscaled_visual_confidence(point);
      auto match = std::find_if(candidates.begin(), candidates.end(), [&](const VisualCandidate & candidate) {
          return candidate.class_id == class_id &&
                 std::hypot(candidate.x - point.x, candidate.y - point.y) <= radius;
        });
      if (match == candidates.end()) {
        VisualCandidate candidate;
        candidate.x = point.x;
        candidate.y = point.y;
        candidate.z = point.z;
        candidate.class_id = class_id;
        candidate.point_count = 1;
        if (std::abs(point.source_id - kSrcVisionUav) < 0.5F) {
          candidate.gated_confidence = confidence;
        } else if (std::abs(point.source_id - kSrcVisionUavDepthYolo) < 0.5F) {
          candidate.depth_confidence = confidence;
        } else {
          candidate.confidence = confidence;
        }
        candidates.push_back(candidate);
        continue;
      }

      const double count = static_cast<double>(match->point_count);
      match->x = (match->x * count + point.x) / (count + 1.0);
      match->y = (match->y * count + point.y) / (count + 1.0);
      match->z = (match->z * count + point.z) / (count + 1.0);
      ++match->point_count;
      if (std::abs(point.source_id - kSrcVisionUav) < 0.5F) {
        match->gated_confidence = std::max(match->gated_confidence, confidence);
      } else if (std::abs(point.source_id - kSrcVisionUavDepthYolo) < 0.5F) {
        match->depth_confidence = std::max(match->depth_confidence, confidence);
      } else {
        match->confidence = std::max(match->confidence, confidence);
      }
    }

    for (auto & candidate : candidates) {
      if (uav_sources) {
        double weighted_confidence = 0.0;
        double weight_sum = 0.0;
        if (candidate.gated_confidence >= 0.0) {
          weighted_confidence += uav_gated_confidence_weight_ * candidate.gated_confidence;
          weight_sum += uav_gated_confidence_weight_;
        }
        if (candidate.depth_confidence >= 0.0) {
          weighted_confidence += uav_depth_confidence_weight_ * candidate.depth_confidence;
          weight_sum += uav_depth_confidence_weight_;
        }
        candidate.confidence = weight_sum > 0.0 ? weighted_confidence / weight_sum : 0.0;
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const VisualCandidate & left, const VisualCandidate & right) {
        return left.confidence > right.confidence;
      });
    return candidates;
  }

  std::optional<std::pair<double, double>> select_ranked_uav_candidate(
    const std::optional<std::pair<double, double>> & active_or_heatmap_candidate,
    const std::vector<VisualCandidate> & candidates,
    const rclcpp::Time & stamp)
  {
    if (active_inspection_cluster_id_ >= 0) {
      return active_or_heatmap_candidate;
    }

    const double association = std::max(0.5, uav_dispatch_association_m_);
    for (const auto & candidate : candidates) {
      if (candidate.confidence < uav_dispatch_min_yolo_confidence_ ||
        std::hypot(candidate.x, candidate.y) < drone_dispatch_min_range_m_)
      {
        continue;
      }
      const auto world = relative_to_world(candidate.x, candidate.y, candidate.z);
      InspectionCluster * best = nullptr;
      double best_distance = association;
      for (auto & cluster : inspection_clusters_) {
        const double distance = std::hypot(world[0] - cluster.x, world[1] - cluster.y);
        if (distance < best_distance) {
          best_distance = distance;
          best = &cluster;
        }
      }
      if (best != nullptr && (best->inspected || best->dispatched)) {
        continue;
      }
      if (best == nullptr) {
        InspectionCluster cluster;
        cluster.id = next_inspection_cluster_id_++;
        cluster.x = world[0];
        cluster.y = world[1];
        cluster.last_prediction = stamp;
        cluster.last_measurement = stamp;
        append_cluster_history(cluster);
        inspection_clusters_.push_back(cluster);
        best = &inspection_clusters_.back();
      }

      last_candidate_cluster_id_ = best->id;
      last_candidate_x_ = candidate.x;
      last_candidate_y_ = candidate.y;
      last_heatmap_probability_ = candidate.confidence;
      return std::make_pair(candidate.x, candidate.y);
    }
    last_candidate_cluster_id_ = -1;
    return std::nullopt;
  }

  bool confirm_visual_candidate(
    const VisualCandidate & candidate, bool from_uav, const rclcpp::Time & stamp)
  {
    StandardPoint evidence;
    evidence.x = static_cast<float>(candidate.x);
    evidence.y = static_cast<float>(candidate.y);
    evidence.z = static_cast<float>(candidate.z);
    evidence.class_id = static_cast<float>(candidate.class_id);
    evidence.source_id = from_uav ? kSrcVisionUav : kSrcVisionNormal;
    const double scale = from_uav ? uav_gated_confidence_weight_ : 1.0;
    evidence.intensity = static_cast<float>(std::clamp(candidate.confidence * scale, 0.0, 1.0));
    return confirm_or_update_object(
      candidate.x, candidate.y, std::vector<StandardPoint>{evidence}, stamp,
      from_uav ? uav_confirmation_min_confidence_ : main_direct_confirmation_min_confidence_,
      candidate.class_id);
  }

  void confirm_direct_visual_targets(
    const std::vector<VisualCandidate> & candidates, bool from_uav, const rclcpp::Time & stamp)
  {
    const double threshold = from_uav ?
      uav_direct_confirmation_min_confidence_ : main_direct_confirmation_min_confidence_;
    for (const auto & candidate : candidates) {
      if (candidate.confidence < threshold ||
        !is_uav_confirmation_class(static_cast<float>(candidate.class_id)))
      {
        continue;
      }
      if (confirm_visual_candidate(candidate, from_uav, stamp) && from_uav) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "UAV directly confirmed class %d at %.1f m with confidence %.2f",
          candidate.class_id, std::hypot(candidate.x, candidate.y), candidate.confidence);
      }
    }
  }

  void confirm_uav_recognized_clusters(const rclcpp::Time & stamp)
  {
    for (auto & cluster : inspection_clusters_) {
      if (!cluster.uav_recognized || cluster.confirmed_to_main) {
        continue;
      }
      const auto relative = world_to_relative(cluster.x, cluster.y);
      VisualCandidate candidate;
      candidate.x = relative[0];
      candidate.y = relative[1];
      candidate.class_id = cluster.recognized_class_id;
      candidate.confidence = cluster.recognized_confidence;
      if (confirm_visual_candidate(candidate, true, stamp)) {
        cluster.confirmed_to_main = true;
        RCLCPP_INFO(get_logger(), "UAV confirmed cluster %d; enabling USV tracking", cluster.id);
      }
    }
  }

  void complete_active_uav_inspection(const rclcpp::Time & stamp)
  {
    if (!uav_observation_active_ || active_inspection_cluster_id_ < 0 ||
      active_observation_started_.nanoseconds() == 0)
    {
      return;
    }
    if ((stamp - active_observation_started_).seconds() >= std::max(0.5, uav_inspection_dwell_s_)) {
      mark_cluster_inspected(active_inspection_cluster_id_, stamp);
    }
  }

  bool spatial_candidate_is_temporally_confirmed(double x, double y, const rclcpp::Time & stamp)
  {
    const double association = std::max(0.1, spatial_confirmation_association_m_);
    SpatialConfirmationCandidate * best = nullptr;
    double best_distance = association;
    for (auto & candidate : spatial_confirmation_candidates_) {
      const double distance = std::hypot(candidate.x - x, candidate.y - y);
      if (distance < best_distance) {
        best_distance = distance;
        best = &candidate;
      }
    }

    if (best == nullptr) {
      spatial_confirmation_candidates_.push_back(SpatialConfirmationCandidate{x, y, 1, stamp});
      prune_spatial_confirmation_candidates(stamp);
      return spatial_confirmation_min_frames_ <= 1;
    }

    best->x = 0.65 * best->x + 0.35 * x;
    best->y = 0.65 * best->y + 0.35 * y;
    best->last_seen = stamp;
    ++best->hits;
    prune_spatial_confirmation_candidates(stamp);
    return best->hits >= std::max(1, spatial_confirmation_min_frames_);
  }

  void prune_spatial_confirmation_candidates(const rclcpp::Time & stamp)
  {
    spatial_confirmation_candidates_.erase(
      std::remove_if(
        spatial_confirmation_candidates_.begin(), spatial_confirmation_candidates_.end(),
        [this, &stamp](const SpatialConfirmationCandidate & candidate) {
          return (stamp - candidate.last_seen).seconds() > spatial_confirmation_timeout_s_;
        }),
      spatial_confirmation_candidates_.end());
  }

  static void set_rgb_pixel(
    sensor_msgs::msg::Image & image, int x, int y, std::uint8_t red,
    std::uint8_t green, std::uint8_t blue)
  {
    if (x < 0 || y < 0 || x >= static_cast<int>(image.width) || y >= static_cast<int>(image.height)) {
      return;
    }
    const std::size_t offset = (static_cast<std::size_t>(y) * image.width + static_cast<std::size_t>(x)) * 3U;
    image.data[offset] = red;
    image.data[offset + 1] = green;
    image.data[offset + 2] = blue;
  }

  static void draw_rgb_line(
    sensor_msgs::msg::Image & image, int x0, int y0, int x1, int y1,
    std::uint8_t red, std::uint8_t green, std::uint8_t blue)
  {
    const int steps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
    for (int i = 0; i <= std::max(1, steps); ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(std::max(1, steps));
      set_rgb_pixel(image, static_cast<int>(std::lround(x0 + ratio * (x1 - x0))),
        static_cast<int>(std::lround(y0 + ratio * (y1 - y0))), red, green, blue);
    }
  }

  static void draw_rgb_rectangle(
    sensor_msgs::msg::Image & image, int min_x, int min_y, int max_x, int max_y,
    std::uint8_t red, std::uint8_t green, std::uint8_t blue)
  {
    draw_rgb_line(image, min_x, min_y, max_x, min_y, red, green, blue);
    draw_rgb_line(image, max_x, min_y, max_x, max_y, red, green, blue);
    draw_rgb_line(image, max_x, max_y, min_x, max_y, red, green, blue);
    draw_rgb_line(image, min_x, max_y, min_x, min_y, red, green, blue);
  }

  std::array<double, 2> world_to_relative(double world_x, double world_y) const
  {
    if (!has_usv_pose_) {
      return {world_x, world_y};
    }
    const double yaw = yaw_from_quaternion(usv_pose_.orientation);
    const double dx = world_x - usv_pose_.position.x;
    const double dy = world_y - usv_pose_.position.y;
    return {std::cos(yaw) * dx + std::sin(yaw) * dy, -std::sin(yaw) * dx + std::cos(yaw) * dy};
  }

  sensor_msgs::msg::Image make_fused_pointcloud_image(
    const std::vector<StandardPoint> & points, const std_msgs::msg::Header & header) const
  {
    constexpr int kWidth = 600;
    constexpr int kHeight = 420;
    sensor_msgs::msg::Image image;
    image.header = header;
    image.height = kHeight;
    image.width = kWidth;
    image.encoding = "rgb8";
    image.step = kWidth * 3;
    image.data.assign(static_cast<std::size_t>(kWidth * kHeight * 3), 10U);
    const auto project = [this](double x, double y) {
        const double px = (x - pointcloud_visual_x_min_) /
          std::max(1e-6, pointcloud_visual_x_max_ - pointcloud_visual_x_min_) * (kWidth - 1);
        const double py = (pointcloud_visual_y_max_ - y) /
          std::max(1e-6, pointcloud_visual_y_max_ - pointcloud_visual_y_min_) * (kHeight - 1);
        return std::array<int, 2>{static_cast<int>(std::lround(px)), static_cast<int>(std::lround(py))};
      };
    for (int line = 0; line <= 5; ++line) {
      const int x = line * (kWidth - 1) / 5;
      const int y = line * (kHeight - 1) / 5;
      draw_rgb_line(image, x, 0, x, kHeight - 1, 28U, 44U, 54U);
      draw_rgb_line(image, 0, y, kWidth - 1, y, 28U, 44U, 54U);
    }
    for (const auto & point : points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < pointcloud_visual_x_min_ ||
        point.x >= pointcloud_visual_x_max_ || point.y < pointcloud_visual_y_min_ ||
        point.y >= pointcloud_visual_y_max_)
      {
        continue;
      }
      std::array<std::uint8_t, 3> color{210U, 220U, 230U};
      if (std::abs(point.source_id - kSrcRadar) < 0.5F) {
        color = {45U, 215U, 255U};
      } else if (std::abs(point.source_id - kSrcSonar) < 0.5F) {
        color = {190U, 80U, 255U};
      } else if (std::abs(point.source_id - kSrcDepth) < 0.5F) {
        color = {255U, 165U, 55U};
      } else if (std::abs(point.source_id - kSrcVisionUav) < 0.5F) {
        color = {255U, 65U, 205U};
      } else if (is_vision_source(point.source_id)) {
        color = {65U, 235U, 110U};
      }
      const auto pixel = project(point.x, point.y);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          set_rgb_pixel(image, pixel[0] + dx, pixel[1] + dy, color[0], color[1], color[2]);
        }
      }
    }
    return image;
  }

  void publish_cluster_overlay(
    const std::vector<StandardPoint> & points, const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::Image image = make_fused_pointcloud_image(points, header);
    const auto project = [this, &image](double x, double y) {
        const double px = (x - pointcloud_visual_x_min_) /
          std::max(1e-6, pointcloud_visual_x_max_ - pointcloud_visual_x_min_) *
          (static_cast<int>(image.width) - 1);
        const double py = (pointcloud_visual_y_max_ - y) /
          std::max(1e-6, pointcloud_visual_y_max_ - pointcloud_visual_y_min_) *
          (static_cast<int>(image.height) - 1);
        return std::array<int, 2>{static_cast<int>(std::lround(px)), static_cast<int>(std::lround(py))};
      };
    const int cluster_half_x = std::max(2, static_cast<int>(std::lround(
      2.5 / (pointcloud_visual_x_max_ - pointcloud_visual_x_min_) * image.width)));
    const int cluster_half_y = std::max(2, static_cast<int>(std::lround(
      2.5 / (pointcloud_visual_y_max_ - pointcloud_visual_y_min_) * image.height)));
    for (const auto & peak : last_heatmap_peaks_) {
      const auto pixel = project(peak.x, peak.y);
      draw_rgb_rectangle(image, pixel[0] - cluster_half_x, pixel[1] - cluster_half_y,
        pixel[0] + cluster_half_x, pixel[1] + cluster_half_y, 60U, 245U, 245U);
    }
    // Red boxes are simulation truth for visual comparison only; no control path reads them.
    for (const auto & truth : ground_truth_objects()) {
      const auto pixel = project(truth.x, truth.y);
      const int half_x = std::max(2, static_cast<int>(std::lround(3.0 /
        (pointcloud_visual_x_max_ - pointcloud_visual_x_min_) * image.width)));
      const int half_y = std::max(2, static_cast<int>(std::lround(1.5 /
        (pointcloud_visual_y_max_ - pointcloud_visual_y_min_) * image.height)));
      draw_rgb_rectangle(image, pixel[0] - half_x, pixel[1] - half_y,
        pixel[0] + half_x, pixel[1] + half_y, 255U, 35U, 35U);
    }
    cluster_overlay_pub_->publish(image);
  }

  void publish_inspection_cluster_image(
    const std::vector<StandardPoint> & points, const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::Image image = make_fused_pointcloud_image(points, header);
    const auto project = [this, &image](double x, double y) {
        const double px = (x - pointcloud_visual_x_min_) /
          std::max(1e-6, pointcloud_visual_x_max_ - pointcloud_visual_x_min_) *
          (static_cast<int>(image.width) - 1);
        const double py = (pointcloud_visual_y_max_ - y) /
          std::max(1e-6, pointcloud_visual_y_max_ - pointcloud_visual_y_min_) *
          (static_cast<int>(image.height) - 1);
        return std::array<int, 2>{static_cast<int>(std::lround(px)), static_cast<int>(std::lround(py))};
      };
    std::vector<std::array<double, 2>> recognized_centers;
    for (const auto & cluster : inspection_clusters_) {
      if (cluster.uav_recognized) {
        recognized_centers.push_back(world_to_relative(cluster.x, cluster.y));
      }
    }
    // Highlight only measurements associated with a UAV-recognized cluster.  This
    // makes the object currently eligible for tracking distinct from background points.
    for (const auto & point : points) {
      const bool belongs_to_recognized_cluster = std::any_of(
        recognized_centers.begin(), recognized_centers.end(), [&point](const std::array<double, 2> & center) {
          return std::hypot(point.x - center[0], point.y - center[1]) <= 7.0;
        });
      if (!belongs_to_recognized_cluster) {
        continue;
      }
      const auto pixel = project(point.x, point.y);
      for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
          set_rgb_pixel(image, pixel[0] + dx, pixel[1] + dy, 45U, 255U, 80U);
        }
      }
    }
    for (const auto & cluster : inspection_clusters_) {
      if (!cluster.uav_recognized) {
        continue;
      }
      for (std::size_t i = 1; i < cluster.history.size(); ++i) {
        const auto previous_relative = world_to_relative(cluster.history[i - 1][0], cluster.history[i - 1][1]);
        const auto current_relative = world_to_relative(cluster.history[i][0], cluster.history[i][1]);
        const auto previous = project(previous_relative[0], previous_relative[1]);
        const auto current = project(current_relative[0], current_relative[1]);
        draw_rgb_line(image, previous[0], previous[1], current[0], current[1], 80U, 170U, 255U);
      }
      const auto relative = world_to_relative(cluster.x, cluster.y);
      const auto predicted_relative = world_to_relative(cluster.x + 7.0 * cluster.vx, cluster.y + 7.0 * cluster.vy);
      const auto current = project(relative[0], relative[1]);
      const auto predicted = project(predicted_relative[0], predicted_relative[1]);
      draw_rgb_line(image, current[0], current[1], predicted[0], predicted[1], 245U, 210U, 60U);
      draw_rgb_rectangle(image, current[0] - 5, current[1] - 5, current[0] + 5, current[1] + 5,
        45U, 255U, 80U);
    }
    inspection_clusters_pub_->publish(image);
  }

  bool should_publish_diagnostic_images(const rclcpp::Time & stamp)
  {
    const double period = 1.0 / std::max(0.2, diagnostic_image_rate_hz_);
    if (last_diagnostic_image_publish_time_.nanoseconds() != 0 &&
      (stamp - last_diagnostic_image_publish_time_).seconds() < period)
    {
      return false;
    }
    last_diagnostic_image_publish_time_ = stamp;
    return true;
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
    const auto uav_preflight_vision = vision;
    const auto ranked_uav_candidates = rank_visual_candidates(uav_preflight_vision, true);
    if (require_uav_observation_active_ && !uav_observation_active_) {
      vision.erase(
        std::remove_if(vision.begin(), vision.end(), [](const StandardPoint & point) {
          return is_uav_vision_source(point.source_id);
        }),
        vision.end());
    }
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
    last_uav_depth_points_ = count_source(vision, kSrcVisionUavDepthYolo);
    last_depth_camera_yolo_points_ = count_source(vision, kSrcVisionDepthYolo);

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = base_frame_;
    const auto radar_display = cluster_radar_for_display(radar);
    radar_pub_->publish(build_cloud(radar_display, header));
    sonar_pub_->publish(build_cloud(sonar, header));
    vision_pub_->publish(build_cloud(vision, header));
    depth_pub_->publish(build_cloud(depth, header));

    std::vector<StandardPoint> integrated;
    integrated.reserve(radar.size() + sonar.size() + vision.size() + depth.size());
    integrated.insert(integrated.end(), radar.begin(), radar.end());
    integrated.insert(integrated.end(), sonar.begin(), sonar.end());
    integrated.insert(integrated.end(), vision.begin(), vision.end());
    integrated.insert(integrated.end(), depth.begin(), depth.end());

    std::vector<StandardPoint> integrated_display;
    integrated_display.reserve(radar_display.size() + sonar.size() + vision.size() + depth.size());
    integrated_display.insert(integrated_display.end(), radar_display.begin(), radar_display.end());
    integrated_display.insert(integrated_display.end(), sonar.begin(), sonar.end());
    integrated_display.insert(integrated_display.end(), vision.begin(), vision.end());
    integrated_display.insert(integrated_display.end(), depth.begin(), depth.end());
    integrated_pub_->publish(build_cloud(integrated_display, header));

    const auto candidate = compute_heatmap_and_candidate(integrated, radar, sonar, vision, depth, header);
    const auto ranked_main_candidates = rank_visual_candidates(vision, false);
    update_uav_recognition_records(ranked_uav_candidates, stamp);
    confirm_direct_visual_targets(ranked_uav_candidates, true, stamp);
    confirm_direct_visual_targets(ranked_main_candidates, false, stamp);
    confirm_uav_recognized_clusters(stamp);
    complete_active_uav_inspection(stamp);
    maintain_detected_objects_from_semantics(vision, stamp);
    if (should_publish_diagnostic_images(stamp)) {
      publish_cluster_overlay(integrated_display, header);
      publish_inspection_cluster_image(integrated_display, header);
    }
    const auto ranked_uav_goal =
      select_ranked_uav_candidate(candidate, ranked_uav_candidates, stamp);
    const auto stable_uav_goal = update_stable_uav_goal(ranked_uav_goal, stamp);
    if (stable_uav_goal.has_value()) {
      publish_goal(stable_uav_goal->first, stable_uav_goal->second, stamp);
      mark_cluster_dispatched(*stable_uav_goal);
      last_uav_goal_publish_time_ = stamp;
    }

    publish_detected_objects(stamp);
    publish_metrics(stamp, integrated.size(), started);
    publish_markers(candidate, stamp);
  }

  std::optional<std::pair<double, double>> update_stable_uav_goal(
    const std::optional<std::pair<double, double>> & candidate,
    const rclcpp::Time & stamp)
  {
    const double window = std::max(0.5, uav_goal_stability_window_s_);
    while (!uav_goal_history_.empty() && (stamp - uav_goal_history_.front().stamp).seconds() > window) {
      uav_goal_history_.pop_front();
    }
    if (!candidate) {
      return std::nullopt;
    }
    const auto candidate_world = relative_to_world(candidate->first, candidate->second, drone_goal_altitude_);
    const double candidate_range = has_usv_pose_ ?
      std::hypot(candidate_world[0] - usv_pose_.position.x, candidate_world[1] - usv_pose_.position.y) :
      std::hypot(candidate_world[0], candidate_world[1]);
    if (candidate_range < drone_dispatch_min_range_m_) {
      return std::nullopt;
    }
    if (!uav_goal_history_.empty() &&
      uav_goal_history_.back().cluster_id != last_candidate_cluster_id_)
    {
      uav_goal_history_.clear();
    }
    uav_goal_history_.push_back(HeatmapCandidateSample{
      stamp, candidate_world[0], candidate_world[1], last_heatmap_probability_, last_candidate_cluster_id_});
    if (uav_goal_history_.size() < static_cast<std::size_t>(std::max(1, uav_goal_min_samples_))) {
      return std::nullopt;
    }
    const double history_span = (uav_goal_history_.back().stamp - uav_goal_history_.front().stamp).seconds();
    if (history_span < 0.85 * window ||
      (last_uav_goal_publish_time_.nanoseconds() != 0 &&
      (stamp - last_uav_goal_publish_time_).seconds() < std::max(0.5, uav_goal_update_period_s_)))
    {
      return std::nullopt;
    }

    const auto select_densest_cell = [this](const std::vector<HeatmapCandidateSample> & samples, double cell_size) {
        struct CellVote {int x; int y; int count; double score;};
        std::vector<CellVote> cells;
        for (const auto & sample : samples) {
          const int x = static_cast<int>(std::floor(sample.x / cell_size));
          const int y = static_cast<int>(std::floor(sample.y / cell_size));
          auto cell = std::find_if(cells.begin(), cells.end(), [x, y](const CellVote & value) {
              return value.x == x && value.y == y;
            });
          if (cell == cells.end()) {
            cells.push_back(CellVote{x, y, 1, sample.score});
          } else {
            ++cell->count;
            cell->score += sample.score;
          }
        }
        return *std::max_element(cells.begin(), cells.end(), [](const CellVote & a, const CellVote & b) {
            return a.count == b.count ? a.score < b.score : a.count < b.count;
          });
      };
    const auto filter_cell = [this](const std::vector<HeatmapCandidateSample> & samples,
        double cell_size, int cell_x, int cell_y) {
        std::vector<HeatmapCandidateSample> filtered;
        for (const auto & sample : samples) {
          if (static_cast<int>(std::floor(sample.x / cell_size)) == cell_x &&
            static_cast<int>(std::floor(sample.y / cell_size)) == cell_y)
          {
            filtered.push_back(sample);
          }
        }
        return filtered;
      };

    const std::vector<HeatmapCandidateSample> samples(uav_goal_history_.begin(), uav_goal_history_.end());
    const auto large = select_densest_cell(samples, 5.0);
    const double stable_ratio = static_cast<double>(large.count) / static_cast<double>(samples.size());
    last_uav_goal_stability_ratio_ = stable_ratio;
    if (stable_ratio < std::clamp(uav_goal_min_stability_ratio_, 0.0, 1.0)) {
      return std::nullopt;
    }
    const auto large_samples = filter_cell(samples, 5.0, large.x, large.y);
    const auto medium = select_densest_cell(large_samples, 2.0);
    const auto medium_samples = filter_cell(large_samples, 2.0, medium.x, medium.y);
    const auto fine = select_densest_cell(medium_samples, std::max(0.1, drone_goal_grid_m_));
    const double final_grid = std::max(0.1, drone_goal_grid_m_);
    last_uav_goal_large_cell_x_ = large.x;
    last_uav_goal_large_cell_y_ = large.y;
    return std::make_pair(
      (static_cast<double>(fine.x) + 0.5) * final_grid,
      (static_cast<double>(fine.y) + 0.5) * final_grid);
  }

  std::optional<std::pair<double, double>> compute_heatmap_and_candidate(
    const std::vector<StandardPoint> & points,
    const std::vector<StandardPoint> & radar,
    const std::vector<StandardPoint> & sonar,
    const std::vector<StandardPoint> & vision,
    const std::vector<StandardPoint> & depth,
    const std_msgs::msg::Header & header)
  {
    if (heatmap_mode_ == "neural") {
      return compute_neural_heatmap_and_candidate(radar, sonar, vision, depth, header);
    }
    const int width = std::max(1, static_cast<int>(std::ceil((heatmap_x_max_ - heatmap_x_min_) / heatmap_resolution_m_)));
    const int height = std::max(1, static_cast<int>(std::ceil((heatmap_y_max_ - heatmap_y_min_) / heatmap_resolution_m_)));
    std::vector<double> grid(static_cast<std::size_t>(width * height), 0.0);
    std::vector<std::array<double, 11>> source_grid(static_cast<std::size_t>(width * height));
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
      if (std::abs(point.source_id - kSrcRadar) < 0.5 &&
        radar_near_heatmap_radius_m_ > radar_min_range_m_ && range < radar_near_heatmap_radius_m_)
      {
        const double ratio = std::clamp(
          (range - radar_min_range_m_) /
          std::max(0.1, radar_near_heatmap_radius_m_ - radar_min_range_m_), 0.0, 1.0);
        const double multiplier = radar_near_heatmap_min_weight_ +
          (1.0 - radar_near_heatmap_min_weight_) * ratio;
        vote *= std::clamp(multiplier, 0.0, 1.0);
      }
      for (const auto & object : detected_objects_) {
        const double d = std::hypot(point.x - object.state[0], point.y - object.state[1]);
        if (d < detected_suppression_radius_) {
          vote *= 0.08;
        }
      }
      const int ix = std::clamp(static_cast<int>((point.x - heatmap_x_min_) / heatmap_resolution_m_), 0, width - 1);
      const int iy = std::clamp(static_cast<int>((point.y - heatmap_y_min_) / heatmap_resolution_m_), 0, height - 1);
      const double sigma = std::max(0.1, heatmap_cluster_sigma_m_);
      const int radius = std::max(0, static_cast<int>(std::ceil(
        std::max(0.0, heatmap_cluster_radius_m_) / heatmap_resolution_m_)));
      // Spread every modal point over neighbouring 1 m cells.  This is a spatial
      // point-cluster fusion, not a change to the 1 m output grid resolution.
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          const int nx = ix + dx;
          const int ny = iy + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
            continue;
          }
          const double distance_squared =
            std::pow(static_cast<double>(dx) * heatmap_resolution_m_, 2) +
            std::pow(static_cast<double>(dy) * heatmap_resolution_m_, 2);
          if (distance_squared > heatmap_cluster_radius_m_ * heatmap_cluster_radius_m_) {
            continue;
          }
          const double kernel = std::exp(-0.5 * distance_squared / (sigma * sigma));
          source_grid[static_cast<std::size_t>(ny * width + nx)][source_slot(point.source_id)] += vote * kernel;
        }
      }
    }
    for (std::size_t i = 0; i < grid.size(); ++i) {
      for (const double source_vote : source_grid[i]) {
        grid[i] += std::min(source_vote, heatmap_source_cell_cap_);
      }
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

    const auto peak_indices = select_peak_indices(grid, width, height, heatmap_threshold_);
    if (peak_indices.empty()) {
      last_heatmap_peaks_.clear();
      return std::nullopt;
    }
    std::vector<HeatmapPeak> peaks;
    peaks.reserve(peak_indices.size());
    for (const int peak_index : peak_indices) {
      const int peak_x = peak_index % width;
      const int peak_y = peak_index / width;
      peaks.push_back(HeatmapPeak{
        heatmap_x_min_ + (static_cast<double>(peak_x) + 0.5) * heatmap_resolution_m_,
        heatmap_y_min_ + (static_cast<double>(peak_y) + 0.5) * heatmap_resolution_m_,
        grid[static_cast<std::size_t>(peak_index)], -1});
    }
    return finalize_heatmap_peaks(std::move(peaks), header);
  }

  double source_weight(float source_id) const
  {
    if (std::abs(source_id - kSrcRadar) < 0.5) {
      return 0.50;
    }
    if (std::abs(source_id - kSrcSonar) < 0.5) {
      return 0.70;
    }
    if (std::abs(source_id - kSrcVisionGated) < 0.5 || std::abs(source_id - kSrcVisionUav) < 0.5) {
      return 1.25;
    }
    if (std::abs(source_id - kSrcVisionUavDepthYolo) < 0.5) {
      return 1.00;
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

  static const char * source_name(float source_id)
  {
    if (std::abs(source_id - kSrcVisionUav) < 0.5F) {
      return "uav_gated_camera";
    }
    if (std::abs(source_id - kSrcVisionUavDepthYolo) < 0.5F) {
      return "uav_depth_camera";
    }
    if (std::abs(source_id - kSrcVisionNormal) < 0.5F ||
      std::abs(source_id - kSrcVisionGated) < 0.5F ||
      std::abs(source_id - kSrcVisionDepthYolo) < 0.5F)
    {
      return "main_camera";
    }
    if (std::abs(source_id - kSrcSonar) < 0.5F) {
      return "sonar";
    }
    if (std::abs(source_id - kSrcRadar) < 0.5F) {
      return "radar";
    }
    return "fusion";
  }

  static std::size_t source_slot(float source_id)
  {
    const std::array<float, 10> ids{
      kSrcRadar, kSrcSonar, kSrcDepth, kSrcVisionNormal, kSrcVisionGated,
      kSrcVisionStf, kSrcVisionBev, kSrcVisionUav, kSrcVisionDepthYolo,
      kSrcVisionUavDepthYolo};
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (std::abs(source_id - ids[i]) < 0.5F) {
        return i;
      }
    }
    return ids.size();
  }

  void publish_goal(double x, double y, const rclcpp::Time & stamp)
  {
    const double grid = std::max(0.1, drone_goal_grid_m_);
    // The flight controller receives only the center of a 1 m x 1 m world-frame cell.
    const double coarse_x = (std::floor(x / grid) + 0.5) * grid;
    const double coarse_y = (std::floor(y / grid) + 0.5) * grid;
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = stamp;
    goal.header.frame_id = "world";
    goal.pose.position.x = coarse_x;
    goal.pose.position.y = coarse_y;
    goal.pose.position.z = drone_goal_altitude_;
    goal.pose.orientation.w = 1.0;
    goal_pub_->publish(goal);
    mission_goal_pub_->publish(goal);
  }

  bool confirm_or_update_object(
    double candidate_x,
    double candidate_y,
    const std::vector<StandardPoint> & points,
    const rclcpp::Time & stamp,
    double minimum_semantic_confidence = 0.18,
    int required_class_id = -1)
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
    double weighted_x2 = 0.0;
    double weighted_y2 = 0.0;
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
        weighted_x2 += weight * point.x * point.x;
        weighted_y2 += weight * point.y * point.y;
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
      const int point_class_id = static_cast<int>(std::lround(point.class_id));
      if (required_class_id >= 0 && point_class_id != required_class_id) {
        continue;
      }
      const bool is_bev = std::abs(point.source_id - kSrcVisionBev) < 0.5;
      const bool is_uav = is_uav_vision_source(point.source_id);
      const bool is_local_vision =
        std::abs(point.source_id - kSrcVisionNormal) < 0.5 ||
        std::abs(point.source_id - kSrcVisionGated) < 0.5 ||
        std::abs(point.source_id - kSrcVisionStf) < 0.5 ||
        std::abs(point.source_id - kSrcVisionDepthYolo) < 0.5;
      if (candidate_range > close_range_m_) {
        if (!is_uav && !is_local_vision) {
          continue;
        }
      } else if ((!is_local_vision && !is_uav) || is_bev) {
        continue;
      }
      if (unscaled_visual_confidence(point) < minimum_semantic_confidence) {
        continue;
      }
      if (!best_semantic ||
        unscaled_visual_confidence(point) > unscaled_visual_confidence(*best_semantic))
      {
        best_semantic = point;
      }
    }

    auto measurement = best_semantic;
    if (!measurement && required_class_id < 0 && enable_spatial_fallback_confirmation_) {
      const int source_count =
        static_cast<int>(has_radar) + static_cast<int>(has_sonar) + static_cast<int>(has_vision) + static_cast<int>(has_depth);
      const double fallback_x = weighted_sum > 1e-6 ? weighted_x / weighted_sum : candidate_x;
      const double fallback_y = weighted_sum > 1e-6 ? weighted_y / weighted_sum : candidate_y;
      const double mean_x2 = weighted_sum > 1e-6 ? weighted_x2 / weighted_sum : fallback_x * fallback_x;
      const double mean_y2 = weighted_sum > 1e-6 ? weighted_y2 / weighted_sum : fallback_y * fallback_y;
      const double spread = std::sqrt(std::max(0.0,
        mean_x2 - fallback_x * fallback_x + mean_y2 - fallback_y * fallback_y));
      if (nearby_points >= spatial_confirmation_min_points_ &&
        source_count >= spatial_confirmation_min_sources_ &&
        spread <= spatial_confirmation_max_spread_m_ &&
        spatial_candidate_is_temporally_confirmed(fallback_x, fallback_y, stamp))
      {
        StandardPoint fallback;
        fallback.x = static_cast<float>(fallback_x);
        fallback.y = static_cast<float>(fallback_y);
        fallback.z = static_cast<float>(weighted_sum > 1e-6 ? std::max(0.0, weighted_z / weighted_sum) : 0.0);
        fallback.intensity = static_cast<float>(
          std::clamp(last_heatmap_probability_ / std::max(1.0, heatmap_threshold_ * 10.0), 0.35, 0.85));
        fallback.class_id = kUnknownClass;
        fallback.source_id = 90.0F + static_cast<float>(source_count);
        measurement = fallback;
      }
    }

    if (!measurement) {
      return false;
    }
    const bool strong_semantic =
      measurement->class_id >= 0.0F &&
      unscaled_visual_confidence(*measurement) >= minimum_semantic_confidence;
    if (candidate_range <= close_range_m_ && !has_depth_support && !strong_semantic) {
      return false;
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
    return true;
  }

  void maintain_detected_objects_from_semantics(
    const std::vector<StandardPoint> & vision, const rclcpp::Time & stamp)
  {
    if (detected_objects_.empty()) {
      return;
    }
    std::vector<bool> updated(detected_objects_.size(), false);
    for (const auto & point : vision) {
      if (point.class_id < 0.0F || point.intensity < object_semantic_maintenance_min_confidence_) {
        continue;
      }
      const bool is_uav = is_uav_vision_source(point.source_id);
      const bool is_bev = std::abs(point.source_id - kSrcVisionBev) < 0.5F;
      const bool is_local_camera =
        std::abs(point.source_id - kSrcVisionNormal) < 0.5F ||
        std::abs(point.source_id - kSrcVisionGated) < 0.5F ||
        std::abs(point.source_id - kSrcVisionStf) < 0.5F ||
        std::abs(point.source_id - kSrcVisionDepthYolo) < 0.5F;
      const double range = std::hypot(point.x, point.y);
      if (is_bev || (!is_uav && !is_local_camera) ||
        (range > close_range_m_ && !is_uav))
      {
        continue;
      }

      const int class_id = static_cast<int>(std::lround(point.class_id));
      int best_index = -1;
      double best_distance = object_semantic_maintenance_radius_;
      for (std::size_t index = 0; index < detected_objects_.size(); ++index) {
        const auto & object = detected_objects_[index];
        if (updated[index] || (object.class_id >= 0 && object.class_id != class_id)) {
          continue;
        }
        const double distance = std::hypot(object.state[0] - point.x, object.state[1] - point.y);
        if (distance < best_distance) {
          best_distance = distance;
          best_index = static_cast<int>(index);
        }
      }
      if (best_index < 0) {
        continue;
      }

      // This path only refreshes an object confirmed earlier by the heatmap;
      // a single camera frame can never create a new tracked object here.
      auto & object = detected_objects_[static_cast<std::size_t>(best_index)];
      object.class_id = class_id;
      object.name = class_name(class_id);
      object.update_ekf(point, stamp, ekf_measurement_noise_);
      updated[static_cast<std::size_t>(best_index)] = true;
    }
  }

  static bool is_vision_source(float source_id)
  {
    return std::abs(source_id - kSrcVisionNormal) < 0.5 ||
           std::abs(source_id - kSrcVisionGated) < 0.5 ||
           std::abs(source_id - kSrcVisionStf) < 0.5 ||
           std::abs(source_id - kSrcVisionBev) < 0.5 ||
           is_uav_vision_source(source_id) ||
           std::abs(source_id - kSrcVisionDepthYolo) < 0.5;
  }

  static bool is_uav_vision_source(float source_id)
  {
    return std::abs(source_id - kSrcVisionUav) < 0.5F ||
           std::abs(source_id - kSrcVisionUavDepthYolo) < 0.5F;
  }

  std::vector<StandardPoint> collect_near(const std::deque<TimedCloud> & buffer, const rclcpp::Time & stamp) const
  {
    std::vector<StandardPoint> output;
    const double tolerance = &buffer == &sonar_buffer_ ? sonar_sync_tolerance_s_ : sync_tolerance_s_;
    for (const auto & frame : buffer) {
      if (std::abs((frame.stamp - stamp).seconds()) <= tolerance) {
        output.insert(output.end(), frame.points.begin(), frame.points.end());
      }
    }
    return output;
  }

  std::vector<StandardPoint> cluster_radar_for_display(
    const std::vector<StandardPoint> & points) const
  {
    struct DisplayCluster
    {
      double x{0.0};
      double y{0.0};
      double intensity_sum{0.0};
      std::vector<StandardPoint> points;
    };

    std::vector<StandardPoint> sorted = points;
    std::sort(sorted.begin(), sorted.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.intensity > rhs.intensity;
    });

    const double radius = std::max(0.5, radar_display_cluster_radius_m_);
    std::vector<DisplayCluster> clusters;
    for (const auto & point : sorted) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        continue;
      }
      int best = -1;
      double best_distance = radius;
      for (std::size_t i = 0; i < clusters.size(); ++i) {
        const double distance = std::hypot(point.x - clusters[i].x, point.y - clusters[i].y);
        if (distance < best_distance) {
          best = static_cast<int>(i);
          best_distance = distance;
        }
      }
      if (best < 0) {
        DisplayCluster cluster;
        cluster.x = point.x;
        cluster.y = point.y;
        cluster.intensity_sum = point.intensity;
        cluster.points.push_back(point);
        clusters.push_back(std::move(cluster));
        continue;
      }
      auto & cluster = clusters[static_cast<std::size_t>(best)];
      const double count = static_cast<double>(cluster.points.size());
      cluster.x = (cluster.x * count + point.x) / (count + 1.0);
      cluster.y = (cluster.y * count + point.y) / (count + 1.0);
      cluster.intensity_sum += point.intensity;
      cluster.points.push_back(point);
    }

    std::sort(clusters.begin(), clusters.end(), [](const auto & lhs, const auto & rhs) {
      const double lhs_score = lhs.intensity_sum + 0.15 * lhs.points.size();
      const double rhs_score = rhs.intensity_sum + 0.15 * rhs.points.size();
      return lhs_score > rhs_score;
    });

    const std::size_t maximum = static_cast<std::size_t>(std::max(1, radar_display_max_points_));
    const std::size_t per_cluster = static_cast<std::size_t>(
      std::max(1, radar_display_max_points_per_cluster_));
    const std::size_t min_cluster = static_cast<std::size_t>(
      std::max(1, radar_display_min_cluster_points_));
    std::vector<StandardPoint> output;
    output.reserve(std::min(maximum, points.size()));
    const auto append_clusters = [&](bool require_support) {
        for (const auto & cluster : clusters) {
          const bool has_support = cluster.points.size() >= min_cluster;
          if ((require_support != has_support) || output.size() >= maximum) {
            continue;
          }
          const std::size_t available = std::min(per_cluster, maximum - output.size());
          const std::size_t count = std::min(available, cluster.points.size());
          output.insert(output.end(), cluster.points.begin(), cluster.points.begin() + count);
        }
      };
    // The display contains measured returns only. Spatially supported clusters
    // are emitted first so vessel returns remain obvious in RViz.
    append_clusters(true);
    if (output.size() < maximum) {
      append_clusters(false);
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
    return latest;
  }

  void prune_all(const rclcpp::Time & stamp)
  {
    prune_buffer(radar_buffer_, stamp);
    prune_buffer(sonar_buffer_, stamp, sonar_buffer_keep_s_);
    prune_buffer(vision_buffer_, stamp);
    prune_buffer(depth_buffer_, stamp);
  }

  void prune_buffer(
    std::deque<TimedCloud> & buffer, const rclcpp::Time & stamp, double keep_s = -1.0) const
  {
    const double retention = keep_s > 0.0 ? keep_s : buffer_keep_s_;
    while (!buffer.empty() && (stamp - buffer.front().stamp).seconds() > retention) {
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
      if (std::hypot(object.x, object.y) <= max_range_m_) {
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
    bool first = true;
    for (const auto & object : detected_objects_) {
      if (object.last_measurement.nanoseconds() == 0 ||
        (stamp - object.last_measurement).seconds() > detected_object_timeout_s_)
      {
        continue;
      }
      if (!first) {
        out << ",";
      }
      first = false;
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
          << ",\"updates\":" << object.updates
          << ",\"last_source\":\"" << source_name(object.last_source_id) << "\"}";
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
    const bool evaluation_target_detected = tp > 0;
    const double processing_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    const auto inspected_clusters = static_cast<std::size_t>(std::count_if(
      inspection_clusters_.begin(), inspection_clusters_.end(), [](const InspectionCluster & cluster) {
        return cluster.inspected;
      }));
    const auto uav_recognized_clusters = static_cast<std::size_t>(std::count_if(
      inspection_clusters_.begin(), inspection_clusters_.end(), [](const InspectionCluster & cluster) {
        return cluster.uav_recognized;
      }));

    std::ostringstream out;
    out.setf(std::ios::fixed, std::ios::floatfield);
    out << std::setprecision(3)
        << "{\"stamp\":" << stamp.seconds()
        << ",\"integrated_points\":" << integrated_points
        << ",\"neural_radar_input_points\":" << last_neural_radar_input_points_
        << ",\"neural_sonar_input_points\":" << last_neural_sonar_input_points_
        << ",\"neural_clutter_tracks\":" << neural_clutter_tracks_.size()
        << ",\"detected_objects\":" << detected_objects_.size()
        << ",\"active_detected_objects\":" << active_indices.size()
        << ",\"evaluation_scope\":\"target_models\""
        << ",\"evaluation_target\":\""
        << (!evaluation_target_model_name_.empty() ? evaluation_target_model_name_ :
      (evaluation_target_model_names_.empty() ? "all" : evaluation_target_model_names_.front())) << "\""
        << ",\"scene_ground_truth_objects\":" << scene_gts.size()
        << ",\"ground_truth_objects\":" << gts.size()
        << ",\"evaluation_target_detected\":" << (evaluation_target_detected ? "true" : "false")
        << ",\"tp\":" << tp
        << ",\"fp\":" << fp
        << ",\"fn\":" << fn
        << ",\"detection_precision\":" << precision
        << ",\"false_positive_rate\":" << false_positive_rate
        << ",\"miss_rate\":" << miss_rate
        << ",\"classification_accuracy\":" << class_accuracy
        << ",\"single_frame_processing_ms\":" << processing_ms
        << ",\"process_memory_mb\":" << process_memory_mb()
        << ",\"heatmap_mode\":\"" << heatmap_mode_ << "\""
        << ",\"neural_execution_provider\":\"" << neural_active_execution_provider_ << "\""
        << ",\"heatmap_best_probability\":" << last_heatmap_probability_
        << ",\"candidate_x\":" << last_candidate_x_
        << ",\"candidate_y\":" << last_candidate_y_
        << ",\"uav_goal_stability_ratio\":" << last_uav_goal_stability_ratio_
        << ",\"uav_goal_large_cell_x\":" << last_uav_goal_large_cell_x_
        << ",\"uav_goal_large_cell_y\":" << last_uav_goal_large_cell_y_
        << ",\"heatmap_cluster_count\":" << inspection_clusters_.size()
        << ",\"uav_inspected_cluster_count\":" << inspected_clusters
        << ",\"uav_recognized_cluster_count\":" << uav_recognized_clusters
        << ",\"uav_target_recognized\":" << (uav_recognized_clusters > 0 ? "true" : "false")
        << ",\"radar_points\":" << last_radar_points_
        << ",\"sonar_points\":" << last_sonar_points_
        << ",\"vision_points\":" << last_vision_points_
        << ",\"depth_points\":" << last_depth_points_
        << ",\"normal_camera_points\":" << last_normal_camera_points_
        << ",\"gated_camera_points\":" << last_gated_camera_points_
        << ",\"stf_points\":" << last_stf_points_
        << ",\"bev_points\":" << last_bev_points_
        << ",\"uav_gated_points\":" << last_uav_gated_points_
        << ",\"uav_depth_yolo_points\":" << last_uav_depth_points_
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
  std::string sonar_detect_topic_;
  std::string normal_camera_topic_;
  std::string gated_camera_topic_;
  std::vector<std::string> normal_camera_topics_;
  std::vector<std::string> gated_camera_topics_;
  std::string stf_camera_topic_;
  std::string bev_topic_;
  std::string uav_gated_topic_;
  std::vector<std::string> uav_gated_topics_;
  std::vector<std::string> uav_depth_topics_;
  std::string uav_observation_active_topic_;
  std::string depth_camera_detection_topic_;
  std::string depth_topic_;
  std::string model_states_topic_;
  std::vector<std::string> evaluation_model_names_;
  std::vector<std::string> evaluation_target_model_names_;
  std::string evaluation_target_model_name_;
  double buffer_keep_s_{0.65};
  double sync_tolerance_s_{0.35};
  int sonar_fusion_window_frames_{3};
  double publish_rate_hz_{8.0};
  double diagnostic_image_rate_hz_{3.0};
  int max_points_per_cloud_{2500};
  int radar_display_max_points_{100};
  double radar_display_cluster_radius_m_{3.0};
  int radar_display_min_cluster_points_{2};
  int radar_display_max_points_per_cluster_{6};
  double max_range_m_{160.0};
  double radar_min_range_m_{1.0};
  double radar_near_heatmap_radius_m_{35.0};
  double radar_near_heatmap_min_weight_{0.10};
  double close_range_m_{30.0};
  double heatmap_resolution_m_{1.0};
  double heatmap_x_min_{0.0};
  double heatmap_x_max_{150.0};
  double heatmap_y_min_{-75.0};
  double heatmap_y_max_{75.0};
  double pointcloud_visual_x_min_{-150.0};
  double pointcloud_visual_x_max_{150.0};
  double pointcloud_visual_y_min_{-150.0};
  double pointcloud_visual_y_max_{150.0};
  double heatmap_threshold_{0.55};
  double heatmap_source_cell_cap_{1.25};
  double heatmap_cluster_sigma_m_{1.0};
  double heatmap_cluster_radius_m_{2.0};
  double heatmap_peak_nms_radius_m_{5.0};
  int heatmap_max_peaks_{5};
  std::string heatmap_mode_{"vote"};
  std::string neural_heatmap_model_path_;
  double neural_heatmap_threshold_{0.50};
  int neural_heatmap_nms_kernel_{5};
  double heatmap_temporal_smoothing_alpha_{0.70};
  int neural_max_points_per_modality_{1200};
  std::string neural_execution_provider_{"auto"};
  std::string neural_active_execution_provider_{"vote"};
  int neural_intra_op_threads_{4};
  int neural_cuda_device_id_{0};
  bool neural_depth_optical_frame_{true};
  double neural_depth_x_offset_m_{1.65};
  double neural_depth_y_offset_m_{0.0};
  double neural_depth_z_offset_m_{0.98};
  double sonar_buffer_keep_s_{1.5};
  double sonar_sync_tolerance_s_{1.0};
  bool neural_clutter_filter_enabled_{true};
  double neural_clutter_cluster_radius_m_{2.5};
  int neural_clutter_min_cluster_points_{2};
  double neural_clutter_mahalanobis_gate_{9.21};
  double neural_clutter_measurement_noise_m_{2.0};
  double neural_clutter_velocity_noise_mps_{1.2};
  double neural_clutter_confirmation_interval_s_{0.30};
  int neural_clutter_min_track_hits_{3};
  double neural_clutter_track_timeout_s_{2.0};
  double neural_clutter_residual_mean_max_{6.0};
  double neural_clutter_cfar_scale_{1.10};
  int neural_radar_min_cluster_points_{3};
  double neural_radar_mahalanobis_gate_{6.0};
  double neural_radar_confirmation_interval_s_{0.45};
  int neural_radar_min_track_hits_{4};
  double neural_radar_residual_mean_max_{3.5};
  double neural_radar_cfar_scale_{1.35};
  bool radar_point_filter_enabled_{true};
  double radar_filter_zone_a_max_m_{1.0};
  double radar_filter_zone_b_max_m_{100.0};
  double radar_filter_zone_c_max_m_{300.0};
  double radar_filter_zone_b_min_snr_{0.4};
  double radar_filter_zone_c_min_snr_{0.3};
  double radar_filter_zone_d_min_snr_{0.3};
  double radar_filter_zone_b_min_power_{0.0005};
  double radar_filter_zone_c_min_power_{0.0003};
  double radar_filter_zone_d_min_power_{0.0002};
  double radar_filter_min_point_score_{0.8};
  double radar_filter_score_intensity_scale_{8.0};
  int radar_filter_history_frames_{3};
  bool radar_filter_fail_open_{true};
  bool radar_filter_local_cfar_enabled_{true};
  double radar_filter_local_range_bin_m_{5.0};
  double radar_filter_local_azimuth_bin_deg_{1.0};
  int radar_filter_local_guard_cells_{1};
  int radar_filter_local_train_cells_{3};
  double radar_filter_local_os_quantile_{0.75};
  double radar_filter_local_cfar_scale_{1.25};
  double detected_suppression_radius_{9.0};
  double confirmation_radius_{7.0};
  double semantic_confirmation_radius_{12.0};
  bool enable_spatial_fallback_confirmation_{true};
  int spatial_confirmation_min_points_{8};
  int spatial_confirmation_min_sources_{2};
  int spatial_confirmation_min_frames_{3};
  double spatial_confirmation_max_spread_m_{4.0};
  double spatial_confirmation_association_m_{3.0};
  double spatial_confirmation_timeout_s_{1.2};
  double object_association_radius_{14.0};
  double object_semantic_maintenance_radius_{12.0};
  double object_semantic_maintenance_min_confidence_{0.22};
  double detected_object_timeout_s_{30.0};
  double ekf_process_noise_{0.35};
  double ekf_measurement_noise_{0.9};
  double drone_goal_altitude_{10.0};
  double drone_goal_grid_m_{1.0};
  double drone_dispatch_min_range_m_{30.0};
  double uav_goal_stability_window_s_{5.0};
  double uav_goal_update_period_s_{5.0};
  double uav_goal_min_stability_ratio_{0.65};
  int uav_goal_min_samples_{12};
  double inspection_cluster_association_m_{7.0};
  double inspection_cluster_memory_s_{600.0};
  double uav_inspection_dwell_s_{4.0};
  std::vector<double> uav_confirmation_class_ids_{0.0, 1.0, 3.0};
  double uav_confirmation_min_confidence_{0.15};
  double uav_direct_confirmation_min_confidence_{0.35};
  double main_direct_confirmation_min_confidence_{0.30};
  double uav_gated_confidence_weight_{1.20};
  double uav_depth_confidence_weight_{0.90};
  double uav_dispatch_min_yolo_confidence_{0.15};
  double uav_dispatch_association_m_{12.0};
  double visual_candidate_cluster_radius_m_{4.0};
  bool require_uav_observation_active_{true};
  bool uav_observation_active_{false};
  rclcpp::Time active_observation_started_{0, 0, RCL_ROS_TIME};

  std::deque<TimedCloud> radar_buffer_;
  std::deque<TimedCloud> sonar_buffer_;
  std::deque<TimedCloud> vision_buffer_;
  std::deque<TimedCloud> depth_buffer_;
  std::vector<std::deque<TimedCloud>> sonar_frame_windows_;
  std::vector<NeuralClutterTrack> neural_clutter_tracks_;
  std::deque<std::vector<StandardPoint>> radar_filter_history_;
  std::vector<DetectedObject> detected_objects_;
  int next_detected_object_id_{1};

  bool has_model_states_{false};
  bool has_usv_pose_{false};
  gazebo_msgs::msg::ModelStates last_model_states_;
  geometry_msgs::msg::Pose usv_pose_;
  double last_heatmap_probability_{0.0};
  double last_candidate_x_{0.0};
  double last_candidate_y_{0.0};
  int last_candidate_cluster_id_{-1};
  double last_uav_goal_stability_ratio_{0.0};
  int last_uav_goal_large_cell_x_{-1};
  int last_uav_goal_large_cell_y_{-1};
  rclcpp::Time last_uav_goal_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diagnostic_image_publish_time_{0, 0, RCL_ROS_TIME};
  std::deque<HeatmapCandidateSample> uav_goal_history_;
  std::vector<HeatmapPeak> last_heatmap_peaks_;
  std::vector<InspectionCluster> inspection_clusters_;
  std::vector<SpatialConfirmationCandidate> spatial_confirmation_candidates_;
  std::vector<double> previous_neural_scores_;
  int next_inspection_cluster_id_{1};
  int active_inspection_cluster_id_{-1};
  std::size_t last_radar_points_{0};
  std::size_t last_sonar_points_{0};
  std::size_t last_vision_points_{0};
  std::size_t last_depth_points_{0};
  std::size_t last_normal_camera_points_{0};
  std::size_t last_gated_camera_points_{0};
  std::size_t last_stf_points_{0};
  std::size_t last_bev_points_{0};
  std::size_t last_uav_gated_points_{0};
  std::size_t last_uav_depth_points_{0};
  std::size_t last_depth_camera_yolo_points_{0};
  std::size_t last_neural_radar_input_points_{0};
  std::size_t last_neural_sonar_input_points_{0};

  Ort::Env neural_ort_env_{ORT_LOGGING_LEVEL_WARNING, "c3_pos_confidence"};
  Ort::MemoryInfo neural_memory_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
  std::unique_ptr<Ort::Session> neural_session_;
  const std::array<const char *, 10> neural_input_names_{
    "dehaze_points", "dehaze_batch", "gated_points", "gated_batch", "sonar_points", "sonar_batch",
    "radar_points", "radar_batch", "radar_velocity_points", "radar_velocity_batch"};
  const std::array<const char *, 4> neural_output_names_{
    "confidence_map", "offset", "sigma", "modality_weights"};
  bool neural_heatmap_ready_{false};

  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> radar_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> vision_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> sonar_subs_;
  rclcpp::Subscription<c3_sonar_driver::msg::SonarDetect>::SharedPtr sonar_detect_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr uav_observation_active_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vision_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr integrated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr heatmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cluster_overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr inspection_clusters_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr neural_weights_pub_;
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
