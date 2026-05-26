#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"

class MmwaveRadarFilterNode : public rclcpp::Node
{
public:
  MmwaveRadarFilterNode()
  : Node("mmwave_radar_filter_node")
  {
    declareParameters();
    loadParameters();

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&MmwaveRadarFilterNode::cloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "mmWave radar filter subscribed: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "mmWave radar filter publishing: %s", output_topic_.c_str());
  }

private:
  enum class RangeZone
  {
    A_NEAR,
    B_SHORT,
    C_MID,
    D_LONG
  };

  enum class TrackState
  {
    TENTATIVE,
    CONFIRMED
  };

  struct RadarPoint
  {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float range = 0.0f;
    float azimuth_rad = 0.0f;
    float azimuth_deg = 0.0f;

    float snr = 0.0f;
    float power = 0.0f;
    float rcs = 0.0f;
    float radial_velocity = 0.0f;

    // Evaluation/debug fields only. Do not use in filtering decisions.
    float source_id = 0.0f;
    float beam_count = 0.0f;
    float angular_extent_deg = 0.0f;
    float target_type = 0.0f;

    RangeZone zone = RangeZone::A_NEAR;
    float point_score = 0.0f;
  };

  struct Cluster
  {
    int id = -1;
    std::vector<RadarPoint> points;

    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float mean_range = 0.0f;
    float mean_azimuth_deg = 0.0f;

    float min_range = 0.0f;
    float max_range = 0.0f;
    float range_span = 0.0f;

    float min_azimuth_deg = 0.0f;
    float max_azimuth_deg = 0.0f;
    float azimuth_span_deg = 0.0f;

    float max_snr = 0.0f;
    float mean_snr = 0.0f;
    float max_power = 0.0f;
    float mean_power = 0.0f;

    float max_point_score = 0.0f;
    float mean_point_score = 0.0f;

    float max_abs_radial_velocity = 0.0f;
    float mean_abs_radial_velocity = 0.0f;

    // Evaluation/debug fields only. Do not use in filtering decisions.
    float max_beam_count = 0.0f;
    float max_angular_extent_deg = 0.0f;

    float cluster_score = 0.0f;
  };

  struct Track
  {
    int id = -1;

    // EKF state: [x, y, vx, vy]
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;

    // Covariance P, 4x4
    double P[4][4] = {};

    rclcpp::Time last_stamp;

    int age = 0;
    int hits = 0;
    int missed = 0;
    int consecutive_missed = 0;

    TrackState state = TrackState::TENTATIVE;
    std::deque<bool> hit_history;

    std::deque<double> nis_history;
    double last_nis = 0.0;

    float score = 0.0f;
    float power = 0.0f;
    float snr = 0.0f;
    float rcs = 0.0f;

    // Evaluation/debug fields only. Do not use in filtering decisions.
    float source_id = 0.0f;
    float target_type = 0.0f;
    float beam_count = 0.0f;
    float angular_extent_deg = 0.0f;
  };

  // =========================
  // Parameters
  // =========================

  std::string input_topic_;
  std::string output_topic_;

  double zone_a_max_m_ = 30.0;
  double zone_b_max_m_ = 100.0;
  double zone_c_max_m_ = 300.0;

  double zone_b_min_snr_ = 2.0;
  double zone_c_min_snr_ = 1.8;
  double zone_d_min_snr_ = 1.8;

  double zone_b_min_power_ = 0.01;
  double zone_c_min_power_ = 0.005;
  double zone_d_min_power_ = 0.003;

  double point_score_keep_min_ = 1.5;

  double cluster_range_eps_m_ = 8.0;
  double cluster_azimuth_eps_deg_ = 2.0;
  int cluster_min_points_ = 2;

  double cluster_max_range_span_m_ = 25.0;
  double cluster_max_azimuth_span_deg_ = 8.0;
  double cluster_score_keep_min_ = 2.0;

  double far_single_point_min_range_m_ = 150.0;
  double far_single_point_min_score_ = 3.5;
  double far_single_point_min_snr_ = 2.5;


  int track_max_missed_ = 5;
  int track_min_hits_to_output_ = 3;

  double kf_process_noise_ = 4.0;
  double kf_measurement_noise_ = 1.0;

  // EKF measurement noise.
  double kf_range_noise_m_ = 1.0;
  double kf_azimuth_noise_rad_ = 0.5 * M_PI / 180.0;
  double kf_radial_velocity_noise_mps_ = 0.60;

  // Mahalanobis gating.
  // 3D chi-square 99% threshold ≈ 11.34.
  double mahalanobis_gate_threshold_ = 11.34;

  // M/N confirmation.
  int track_confirm_window_ = 5;
  int track_confirm_min_hits_ = 2;
  int track_delete_missed_ = 5;

  // NIS innovation history.
  int nis_history_window_ = 6;
  double nis_mean_max_ = 9.0;
  double nis_var_max_ = 40.0;

  // Local adaptive detection / point-cloud OS-CFAR approximation.
  bool enable_local_adaptive_detection_ = true;
  double local_range_bin_m_ = 5.0;
  double local_azimuth_bin_deg_ = 1.0;
  int local_cfar_guard_cells_ = 1;
  int local_cfar_train_cells_ = 3;
  double local_cfar_os_quantile_ = 0.75;
  double local_cfar_scale_ = 1.8;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::vector<std::vector<RadarPoint>> raw_points_history_;
  std::vector<Track> tracks_;
  int next_track_id_ = 1;

  void declareParameters()
  {
    this->declare_parameter<std::string>("input_topic", "/mmwave/detections");
    this->declare_parameter<std::string>("output_topic", "/mmwave/filtered_detections");

    this->declare_parameter<double>("zone_a_max_m", 30.0);
    this->declare_parameter<double>("zone_b_max_m", 100.0);
    this->declare_parameter<double>("zone_c_max_m", 300.0);

    this->declare_parameter<double>("zone_b_min_snr", 2.0);
    this->declare_parameter<double>("zone_c_min_snr", 1.8);
    this->declare_parameter<double>("zone_d_min_snr", 1.8);

    this->declare_parameter<double>("zone_b_min_power", 0.01);
    this->declare_parameter<double>("zone_c_min_power", 0.005);
    this->declare_parameter<double>("zone_d_min_power", 0.003);

    this->declare_parameter<double>("point_score_keep_min", 1.5);

    this->declare_parameter<double>("cluster_range_eps_m", 8.0);
    this->declare_parameter<double>("cluster_azimuth_eps_deg", 2.0);
    this->declare_parameter<int>("cluster_min_points", 2);

    this->declare_parameter<double>("cluster_max_range_span_m", 25.0);
    this->declare_parameter<double>("cluster_max_azimuth_span_deg", 8.0);
    this->declare_parameter<double>("cluster_score_keep_min", 2.0);
    this->declare_parameter<double>("far_single_point_min_range_m", 150.0);
    this->declare_parameter<double>("far_single_point_min_score", 3.5);
    this->declare_parameter<double>("far_single_point_min_snr", 2.5);


    this->declare_parameter<int>("track_max_missed", 5);
    this->declare_parameter<int>("track_min_hits_to_output", 2);

    this->declare_parameter<double>("kf_process_noise", 4.0);
    this->declare_parameter<double>("kf_measurement_noise", 1.0);

    this->declare_parameter<double>("kf_range_noise_m", 1.0);
    this->declare_parameter<double>("kf_azimuth_noise_deg", 0.5);
    this->declare_parameter<double>("kf_radial_velocity_noise_mps", 0.60);

    this->declare_parameter<double>("mahalanobis_gate_threshold", 11.34);

    this->declare_parameter<int>("track_confirm_window", 5);
    this->declare_parameter<int>("track_confirm_min_hits", 3);
    this->declare_parameter<int>("track_delete_missed", 5);

    this->declare_parameter<int>("nis_history_window", 6);
    this->declare_parameter<double>("nis_mean_max", 9.0);
    this->declare_parameter<double>("nis_var_max", 40.0);

    this->declare_parameter<bool>("enable_local_adaptive_detection", true);
    this->declare_parameter<double>("local_range_bin_m", 5.0);
    this->declare_parameter<double>("local_azimuth_bin_deg", 1.0);
    this->declare_parameter<int>("local_cfar_guard_cells", 1);
    this->declare_parameter<int>("local_cfar_train_cells", 3);
    this->declare_parameter<double>("local_cfar_os_quantile", 0.75);
    this->declare_parameter<double>("local_cfar_scale", 1.8);
  }

  void loadParameters()
  {
    zone_a_max_m_ = this->get_parameter("zone_a_max_m").as_double();
    zone_b_max_m_ = this->get_parameter("zone_b_max_m").as_double();
    zone_c_max_m_ = this->get_parameter("zone_c_max_m").as_double();

    zone_b_min_snr_ = this->get_parameter("zone_b_min_snr").as_double();
    zone_c_min_snr_ = this->get_parameter("zone_c_min_snr").as_double();
    zone_d_min_snr_ = this->get_parameter("zone_d_min_snr").as_double();

    zone_b_min_power_ = this->get_parameter("zone_b_min_power").as_double();
    zone_c_min_power_ = this->get_parameter("zone_c_min_power").as_double();
    zone_d_min_power_ = this->get_parameter("zone_d_min_power").as_double();

    point_score_keep_min_ = this->get_parameter("point_score_keep_min").as_double();

    cluster_range_eps_m_ = this->get_parameter("cluster_range_eps_m").as_double();
    cluster_azimuth_eps_deg_ = this->get_parameter("cluster_azimuth_eps_deg").as_double();
    cluster_min_points_ = this->get_parameter("cluster_min_points").as_int();

    cluster_max_range_span_m_ = this->get_parameter("cluster_max_range_span_m").as_double();
    cluster_max_azimuth_span_deg_ = this->get_parameter("cluster_max_azimuth_span_deg").as_double();
    cluster_score_keep_min_ = this->get_parameter("cluster_score_keep_min").as_double();

    far_single_point_min_range_m_ = this->get_parameter("far_single_point_min_range_m").as_double();
    far_single_point_min_score_ = this->get_parameter("far_single_point_min_score").as_double();
    far_single_point_min_snr_ = this->get_parameter("far_single_point_min_snr").as_double();


    track_max_missed_ = this->get_parameter("track_max_missed").as_int();
    track_min_hits_to_output_ = this->get_parameter("track_min_hits_to_output").as_int();

    kf_process_noise_ = this->get_parameter("kf_process_noise").as_double();
    kf_measurement_noise_ = this->get_parameter("kf_measurement_noise").as_double();

    kf_range_noise_m_ = this->get_parameter("kf_range_noise_m").as_double();
    kf_azimuth_noise_rad_ = this->get_parameter("kf_azimuth_noise_deg").as_double() * M_PI / 180.0;
    kf_radial_velocity_noise_mps_ = this->get_parameter("kf_radial_velocity_noise_mps").as_double();

    mahalanobis_gate_threshold_ = this->get_parameter("mahalanobis_gate_threshold").as_double();

    track_confirm_window_ = this->get_parameter("track_confirm_window").as_int();
    track_confirm_min_hits_ = this->get_parameter("track_confirm_min_hits").as_int();
    track_delete_missed_ = this->get_parameter("track_delete_missed").as_int();

    nis_history_window_ = this->get_parameter("nis_history_window").as_int();
    nis_mean_max_ = this->get_parameter("nis_mean_max").as_double();
    nis_var_max_ = this->get_parameter("nis_var_max").as_double();

    enable_local_adaptive_detection_ =
      this->get_parameter("enable_local_adaptive_detection").as_bool();
    local_range_bin_m_ = this->get_parameter("local_range_bin_m").as_double();
    local_azimuth_bin_deg_ = this->get_parameter("local_azimuth_bin_deg").as_double();
    local_cfar_guard_cells_ = this->get_parameter("local_cfar_guard_cells").as_int();
    local_cfar_train_cells_ = this->get_parameter("local_cfar_train_cells").as_int();
    local_cfar_os_quantile_ = this->get_parameter("local_cfar_os_quantile").as_double();
    local_cfar_scale_ = this->get_parameter("local_cfar_scale").as_double();

    track_confirm_window_ = std::max(1, track_confirm_window_);
    track_confirm_min_hits_ = std::clamp(track_confirm_min_hits_, 1, track_confirm_window_);
    track_delete_missed_ = std::max(1, track_delete_missed_);
    nis_history_window_ = std::max(1, nis_history_window_);
    far_single_point_min_range_m_ = std::max(0.0, far_single_point_min_range_m_);
    far_single_point_min_score_ = std::max(0.0, far_single_point_min_score_);
    far_single_point_min_snr_ = std::max(0.0, far_single_point_min_snr_);

  }

  // =========================
  // Main callback
  // =========================

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    loadParameters();

    std::vector<RadarPoint> current_points = readPointCloud(*msg);
    std::vector<RadarPoint> raw_points = three_cloudpoints(current_points);

    std::vector<RadarPoint> candidate_points = processPoints(raw_points);

    if (enable_local_adaptive_detection_) {
      candidate_points = localAdaptiveDetection(candidate_points);
    }

    std::vector<Cluster> output_clusters = processClusters(candidate_points);
    std::vector<RadarPoint> output_points = updateTracks(output_clusters, msg->header.stamp);

    auto output_msg = createOutputCloud(msg->header, output_points);
    pub_->publish(output_msg);

    RCLCPP_DEBUG(
      this->get_logger(),
      "current=%zu fused_raw=%zu candidate_points=%zu clusters=%zu tracks=%zu output=%zu",
      current_points.size(),
      raw_points.size(),
      candidate_points.size(),
      output_clusters.size(),
      tracks_.size(),
      output_points.size());
  }

  // =========================
  // Input cloud
  // =========================

  std::vector<RadarPoint> readPointCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    std::vector<RadarPoint> points;

    if (cloud.width * cloud.height == 0) {
      return points;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_power(cloud, "power");
    sensor_msgs::PointCloud2ConstIterator<float> iter_velocity(cloud, "radial_velocity");
    sensor_msgs::PointCloud2ConstIterator<float> iter_snr(cloud, "snr");
    sensor_msgs::PointCloud2ConstIterator<float> iter_rcs(cloud, "rcs");
    sensor_msgs::PointCloud2ConstIterator<float> iter_range(cloud, "range");
    sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth(cloud, "azimuth_deg");
    sensor_msgs::PointCloud2ConstIterator<float> iter_timestamp(cloud, "timestamp");
    sensor_msgs::PointCloud2ConstIterator<float> iter_source_id(cloud, "source_id");
    sensor_msgs::PointCloud2ConstIterator<float> iter_beam_count(cloud, "beam_count");
    sensor_msgs::PointCloud2ConstIterator<float> iter_extent(cloud, "angular_extent_deg");
    sensor_msgs::PointCloud2ConstIterator<float> iter_target_type(cloud, "target_type");

    for (; iter_x != iter_x.end();
      ++iter_x, ++iter_y, ++iter_z,
      ++iter_range, ++iter_azimuth,
      ++iter_snr, ++iter_power, ++iter_rcs,
      ++iter_velocity, ++iter_timestamp,
      ++iter_source_id, ++iter_beam_count,
      ++iter_extent, ++iter_target_type)
    {
      RadarPoint p;
      p.x = *iter_x;
      p.y = *iter_y;
      p.z = *iter_z;
      p.power = *iter_power;
      p.radial_velocity = *iter_velocity;
      p.snr = *iter_snr;
      p.rcs = *iter_rcs;
      p.range = *iter_range;
      p.azimuth_deg = *iter_azimuth;
      p.azimuth_rad = static_cast<float>(p.azimuth_deg * M_PI / 180.0);
      (void)iter_timestamp;

      // Evaluation/debug fields only.
      p.source_id = *iter_source_id;
      p.beam_count = *iter_beam_count;
      p.angular_extent_deg = *iter_extent;
      p.target_type = *iter_target_type;

      if (!std::isfinite(p.range) || p.range <= 0.0f) {
        p.range = std::hypot(p.x, p.y);
      }
      if (!std::isfinite(p.azimuth_rad)) {
        p.azimuth_rad = std::atan2(p.y, p.x);
        p.azimuth_deg = static_cast<float>(p.azimuth_rad * 180.0 / M_PI);
      }

      points.push_back(p);
    }

    return points;
  }

  std::vector<RadarPoint> three_cloudpoints(const std::vector<RadarPoint> & current_points)
  {
    raw_points_history_.push_back(current_points);

    while (raw_points_history_.size() > 2) {
      raw_points_history_.erase(raw_points_history_.begin());
    }

    size_t total_size = 0;
    for (const auto & frame_points : raw_points_history_) {
      total_size += frame_points.size();
    }

    std::vector<RadarPoint> fused_points;
    fused_points.reserve(total_size);

    for (const auto & frame_points : raw_points_history_) {
      fused_points.insert(fused_points.end(), frame_points.begin(), frame_points.end());
    }

    return fused_points;
  }

  // =========================
  // Point filtering
  // =========================

  RangeZone getZone(float range_m) const
  {
    if (range_m < zone_a_max_m_) {
      return RangeZone::A_NEAR;
    }
    if (range_m < zone_b_max_m_) {
      return RangeZone::B_SHORT;
    }
    if (range_m < zone_c_max_m_) {
      return RangeZone::C_MID;
    }
    return RangeZone::D_LONG;
  }

  bool pointHardFilter(const RadarPoint & p) const
{
  if (!std::isfinite(p.range) || p.range <= 0.0f) {
    return false;
  }

  if (!std::isfinite(p.azimuth_rad) || !std::isfinite(p.radial_velocity)) {
    return false;
  }

  // Filter out all points in Zone A / near range.
  if (p.zone == RangeZone::A_NEAR) {
    return false;
  }

  if (p.zone == RangeZone::B_SHORT) {
    if (p.snr < zone_b_min_snr_ || p.power < zone_b_min_power_) {
      return false;
    }
  } else if (p.zone == RangeZone::C_MID) {
    if (p.snr < zone_c_min_snr_ || p.power < zone_c_min_power_) {
      return false;
    }
  } else if (p.zone == RangeZone::D_LONG) {
    if (p.snr < zone_d_min_snr_ || p.power < zone_d_min_power_) {
      return false;
    }
  }

  return true;
}


  std::pair<float, float> getPowerScoreThresholds(RangeZone zone) const
  {
    if (zone == RangeZone::B_SHORT) {
      return {static_cast<float>(zone_b_min_power_), static_cast<float>(zone_b_min_power_ * 2.0)};
    }
    if (zone == RangeZone::C_MID) {
      return {static_cast<float>(zone_c_min_power_), static_cast<float>(zone_c_min_power_ * 2.0)};
    }
    return {static_cast<float>(zone_d_min_power_), static_cast<float>(zone_d_min_power_ * 2.0)};
  }

  float calcPointScore(const RadarPoint & p) const
  {
    float score = 0.0f;

    if (p.zone == RangeZone::B_SHORT) {
      if (p.snr >= 5.5f) score += 4.0f;
      else if (p.snr >= 4.0f) score += 3.0f;
      else if (p.snr >= 3.0f) score += 1.5f;
    } else {
      if (p.snr >= 5.5f) score += 4.0f;
      else if (p.snr >= 4.0f) score += 3.0f;
      else if (p.snr >= 2.5f) score += 1.5f;
    }

    const auto [low_power, mid_power] = getPowerScoreThresholds(p.zone);
    if (p.power >= mid_power) score += 3.0f;
    else if (p.power >= low_power) score += 1.5f;

    if (p.zone == RangeZone::C_MID) score += 0.5f;
    else if (p.zone == RangeZone::D_LONG) score += 1.0f;

    // Only use measured radial velocity, not labels.
    if (std::abs(p.radial_velocity) >= 0.2f) {
      score += 0.5f;
    }

    return score;
  }

  std::vector<RadarPoint> processPoints(const std::vector<RadarPoint> & raw_points) const
  {
    std::vector<RadarPoint> kept;
    kept.reserve(raw_points.size());

    for (auto p : raw_points) {
      p.zone = getZone(p.range);

      if (!pointHardFilter(p)) {
        continue;
      }

      p.point_score = calcPointScore(p);
      if (p.point_score < point_score_keep_min_) {
        continue;
      }

      kept.push_back(p);
    }

    return kept;
  }

  // =========================
  // Local adaptive detection / OS-CFAR approximation
  // =========================

  double detectionStrength(const RadarPoint & p) const
  {
    // Only measured fields. No source_id / target_type / beam_count / angular_extent_deg.
    return static_cast<double>(p.power) + 0.05 * static_cast<double>(p.snr);
  }

  std::vector<RadarPoint> localAdaptiveDetection(const std::vector<RadarPoint> & points) const
  {
    if (points.empty()) {
      return points;
    }

    std::vector<RadarPoint> kept;
    kept.reserve(points.size());

    const double az_bin_rad = std::max(1e-6, local_azimuth_bin_deg_ * M_PI / 180.0);
    const double range_bin = std::max(1e-6, local_range_bin_m_);

    for (const auto & p : points) {
      std::vector<double> reference_values;
      reference_values.reserve(points.size());

      const int p_r_bin = static_cast<int>(std::floor(static_cast<double>(p.range) / range_bin));
      const int p_a_bin = static_cast<int>(std::floor(static_cast<double>(p.azimuth_rad) / az_bin_rad));

      for (const auto & q : points) {
        const int q_r_bin = static_cast<int>(std::floor(static_cast<double>(q.range) / range_bin));
        const int q_a_bin = static_cast<int>(std::floor(static_cast<double>(q.azimuth_rad) / az_bin_rad));

        const int dr = std::abs(q_r_bin - p_r_bin);
        const int da = std::abs(q_a_bin - p_a_bin);

        const bool in_guard =
          dr <= local_cfar_guard_cells_ && da <= local_cfar_guard_cells_;

        const bool in_train =
          dr <= local_cfar_guard_cells_ + local_cfar_train_cells_ &&
          da <= local_cfar_guard_cells_ + local_cfar_train_cells_;

        if (!in_train || in_guard) {
          continue;
        }

        reference_values.push_back(detectionStrength(q));
      }

      // Not enough reference cells: keep it to avoid deleting isolated real targets.
      if (reference_values.size() < 6) {
        kept.push_back(p);
        continue;
      }

      std::sort(reference_values.begin(), reference_values.end());

      const double q_clamped = std::clamp(local_cfar_os_quantile_, 0.0, 1.0);
      const size_t os_idx = std::min(
        reference_values.size() - 1,
        static_cast<size_t>(std::floor(q_clamped * static_cast<double>(reference_values.size() - 1))));

      const double local_background = reference_values[os_idx];
      const double threshold = local_cfar_scale_ * local_background;
      const double current = detectionStrength(p);

      if (current >= threshold) {
        kept.push_back(p);
      }
    }

    return kept;
  }

  // =========================
  // Clustering
  // =========================

  std::vector<Cluster> clusterPoints(const std::vector<RadarPoint> & points) const
  {
    std::vector<Cluster> clusters;
    int next_cluster_id = 1;

    for (const auto & p : points) {
      int best_idx = -1;
      double best_dist = std::numeric_limits<double>::max();

      for (size_t i = 0; i < clusters.size(); ++i) {
        const auto & c = clusters[i];

        const double range_diff = std::abs(p.range - c.mean_range);
        const double az_diff = std::abs(p.azimuth_deg - c.mean_azimuth_deg);

        if (range_diff <= cluster_range_eps_m_ && az_diff <= cluster_azimuth_eps_deg_) {
          const double normalized_dist =
            range_diff / std::max(cluster_range_eps_m_, 1e-3) +
            az_diff / std::max(cluster_azimuth_eps_deg_, 1e-3);

          if (normalized_dist < best_dist) {
            best_dist = normalized_dist;
            best_idx = static_cast<int>(i);
          }
        }
      }

      if (best_idx >= 0) {
        clusters[best_idx].points.push_back(p);
        computeClusterFeatures(clusters[best_idx]);
      } else {
        Cluster c;
        c.id = next_cluster_id++;
        c.points.push_back(p);
        computeClusterFeatures(c);
        clusters.push_back(c);
      }
    }

    return clusters;
  }

  void computeClusterFeatures(Cluster & c) const
  {
    if (c.points.empty()) {
      return;
    }

    const float n = static_cast<float>(c.points.size());

    c.mean_x = 0.0f;
    c.mean_y = 0.0f;
    c.mean_range = 0.0f;
    c.mean_azimuth_deg = 0.0f;
    c.mean_snr = 0.0f;
    c.mean_power = 0.0f;
    c.mean_point_score = 0.0f;
    c.mean_abs_radial_velocity = 0.0f;

    c.min_range = std::numeric_limits<float>::max();
    c.max_range = -std::numeric_limits<float>::max();
    c.min_azimuth_deg = std::numeric_limits<float>::max();
    c.max_azimuth_deg = -std::numeric_limits<float>::max();

    c.max_snr = -std::numeric_limits<float>::max();
    c.max_power = -std::numeric_limits<float>::max();
    c.max_point_score = -std::numeric_limits<float>::max();
    c.max_abs_radial_velocity = 0.0f;
    c.max_beam_count = 0.0f;
    c.max_angular_extent_deg = 0.0f;

    for (const auto & p : c.points) {
      c.mean_x += p.x;
      c.mean_y += p.y;
      c.mean_range += p.range;
      c.mean_azimuth_deg += p.azimuth_deg;
      c.mean_snr += p.snr;
      c.mean_power += p.power;
      c.mean_point_score += p.point_score;
      c.mean_abs_radial_velocity += std::abs(p.radial_velocity);

      c.min_range = std::min(c.min_range, p.range);
      c.max_range = std::max(c.max_range, p.range);
      c.min_azimuth_deg = std::min(c.min_azimuth_deg, p.azimuth_deg);
      c.max_azimuth_deg = std::max(c.max_azimuth_deg, p.azimuth_deg);

      c.max_snr = std::max(c.max_snr, p.snr);
      c.max_power = std::max(c.max_power, p.power);
      c.max_point_score = std::max(c.max_point_score, p.point_score);
      c.max_abs_radial_velocity = std::max(c.max_abs_radial_velocity, std::abs(p.radial_velocity));

      // Computed only for output/debug compatibility, not for decisions.
      c.max_beam_count = std::max(c.max_beam_count, p.beam_count);
      c.max_angular_extent_deg = std::max(c.max_angular_extent_deg, p.angular_extent_deg);
    }

    c.mean_x /= n;
    c.mean_y /= n;
    c.mean_range /= n;
    c.mean_azimuth_deg /= n;
    c.mean_snr /= n;
    c.mean_power /= n;
    c.mean_point_score /= n;
    c.mean_abs_radial_velocity /= n;

    c.range_span = c.max_range - c.min_range;
    c.azimuth_span_deg = c.max_azimuth_deg - c.min_azimuth_deg;
  }

    bool isStrongFarSinglePointCluster(const Cluster & c) const
  {
    return c.points.size() == 1 &&
      c.mean_range >= far_single_point_min_range_m_ &&
      c.max_snr >= far_single_point_min_snr_ &&
      c.max_point_score >= far_single_point_min_score_;
  }

  bool clusterHardFilter(const Cluster & c) const
  {
    const bool strong_far_single = isStrongFarSinglePointCluster(c);

    if (static_cast<int>(c.points.size()) < cluster_min_points_ && !strong_far_single) {
      return false;
    }

    if (c.range_span > cluster_max_range_span_m_) {
      return false;
    }

    if (c.azimuth_span_deg > cluster_max_azimuth_span_deg_) {
      return false;
    }

    if (c.max_snr < 2.0f && c.mean_snr < 1.5f) {
      return false;
    }

    return true;
  }


  float calcClusterScore(const Cluster & c) const
  {
    float score = 0.0f;

    const int size = static_cast<int>(c.points.size());
    if (size >= 4) score += 3.0f;
    else if (size == 3) score += 2.5f;
    else if (size == 2) score += 1.5f;

    if (c.mean_snr >= 5.0f || c.max_snr >= 6.0f) score += 3.0f;
    else if (c.mean_snr >= 4.0f || c.max_snr >= 5.0f) score += 2.0f;
    else if (c.mean_snr >= 3.0f || c.max_snr >= 4.0f) score += 1.0f;

    if (c.range_span < 8.0f && c.azimuth_span_deg < 2.0f) score += 2.0f;
    else if (c.range_span < 15.0f && c.azimuth_span_deg < 4.0f) score += 1.0f;

    if (c.mean_point_score >= 6.0f) score += 1.0f;
    else if (c.mean_point_score >= 4.0f) score += 0.5f;

    if (c.mean_abs_radial_velocity >= 0.3f || c.max_abs_radial_velocity >= 0.5f) {
      score += 1.0f;
    }

    // Do not use beam_count / angular_extent_deg here.
    return score;
  }

  std::vector<Cluster> processClusters(const std::vector<RadarPoint> & points) const
  {
    std::vector<Cluster> raw_clusters = clusterPoints(points);
    std::vector<Cluster> kept;

    for (auto & c : raw_clusters) {
      computeClusterFeatures(c);

      if (!clusterHardFilter(c)) {
        continue;
      }

      c.cluster_score = calcClusterScore(c);
      if (c.cluster_score < cluster_score_keep_min_ && !isStrongFarSinglePointCluster(c)) {
        continue;
      }


      kept.push_back(c);
    }

    return kept;
  }

  // =========================
  // Detection representation
  // =========================

  RadarPoint selectRepresentativePoint(const Cluster & c) const
  {
    if (c.points.empty()) {
      return RadarPoint{};
    }

    return *std::max_element(
      c.points.begin(), c.points.end(),
      [](const RadarPoint & a, const RadarPoint & b) {
        return a.point_score < b.point_score;
      });
  }

  RadarPoint clusterToWeightedDetection(const Cluster & c) const
  {
    RadarPoint p = selectRepresentativePoint(c);

    double sum_w = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_vr = 0.0;

    for (const auto & q : c.points) {
      const double w = std::max(0.1f, q.point_score);
      sum_w += w;
      sum_x += static_cast<double>(q.x) * w;
      sum_y += static_cast<double>(q.y) * w;
      sum_vr += static_cast<double>(q.radial_velocity) * w;
    }

    if (sum_w > 1e-6) {
      p.x = static_cast<float>(sum_x / sum_w);
      p.y = static_cast<float>(sum_y / sum_w);
      p.radial_velocity = static_cast<float>(sum_vr / sum_w);
    } else {
      p.x = c.mean_x;
      p.y = c.mean_y;
    }

    p.z = 0.0f;
    p.range = static_cast<float>(std::hypot(p.x, p.y));
    p.azimuth_rad = static_cast<float>(std::atan2(p.y, p.x));
    p.azimuth_deg = static_cast<float>(p.azimuth_rad * 180.0 / M_PI);
    p.power = c.max_power;
    p.snr = c.max_snr;
    p.point_score = c.cluster_score;

    return p;
  }

  // =========================
  // Math helpers
  // =========================

  static double sqr(double x)
  {
    return x * x;
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static bool invert3x3(const double A[3][3], double invA[3][3])
  {
    const double a = A[0][0];
    const double b = A[0][1];
    const double c = A[0][2];
    const double d = A[1][0];
    const double e = A[1][1];
    const double f = A[1][2];
    const double g = A[2][0];
    const double h = A[2][1];
    const double i = A[2][2];

    const double A00 = e * i - f * h;
    const double A01 = -(d * i - f * g);
    const double A02 = d * h - e * g;

    const double A10 = -(b * i - c * h);
    const double A11 = a * i - c * g;
    const double A12 = -(a * h - b * g);

    const double A20 = b * f - c * e;
    const double A21 = -(a * f - c * d);
    const double A22 = a * e - b * d;

    const double det = a * A00 + b * A01 + c * A02;

    if (std::abs(det) < 1e-9) {
      return false;
    }

    const double inv_det = 1.0 / det;

    invA[0][0] = A00 * inv_det;
    invA[0][1] = A10 * inv_det;
    invA[0][2] = A20 * inv_det;

    invA[1][0] = A01 * inv_det;
    invA[1][1] = A11 * inv_det;
    invA[1][2] = A21 * inv_det;

    invA[2][0] = A02 * inv_det;
    invA[2][1] = A12 * inv_det;
    invA[2][2] = A22 * inv_det;

    return true;
  }

  // =========================
  // Track history helpers
  // =========================

  void pushLimitedBool(std::deque<bool> & history, bool value, int max_size) const
  {
    history.push_back(value);
    while (static_cast<int>(history.size()) > max_size) {
      history.pop_front();
    }
  }

  void pushLimitedDouble(std::deque<double> & history, double value, int max_size) const
  {
    history.push_back(value);
    while (static_cast<int>(history.size()) > max_size) {
      history.pop_front();
    }
  }

  int countHits(const std::deque<bool> & history) const
  {
    int count = 0;
    for (bool h : history) {
      if (h) {
        ++count;
      }
    }
    return count;
  }

  double meanNis(const Track & t) const
  {
    if (t.nis_history.empty()) {
      return 0.0;
    }

    const double sum = std::accumulate(t.nis_history.begin(), t.nis_history.end(), 0.0);
    return sum / static_cast<double>(t.nis_history.size());
  }

  double varNis(const Track & t) const
  {
    if (t.nis_history.size() < 2) {
      return 0.0;
    }

    const double mean = meanNis(t);
    double acc = 0.0;

    for (double v : t.nis_history) {
      acc += sqr(v - mean);
    }

    return acc / static_cast<double>(t.nis_history.size() - 1);
  }

  bool nisHistoryLooksStable(const Track & t) const
  {
    if (static_cast<int>(t.nis_history.size()) < std::min(3, nis_history_window_)) {
      return true;
    }

    return meanNis(t) <= nis_mean_max_ && varNis(t) <= nis_var_max_;
  }

  void updateTrackConfirmationState(Track & t) const
  {
    if (t.state == TrackState::CONFIRMED) {
      return;
    }

    if (static_cast<int>(t.hit_history.size()) < track_confirm_window_) {
      return;
    }

    const int recent_hits = countHits(t.hit_history);

    if (recent_hits >= track_confirm_min_hits_ && nisHistoryLooksStable(t)) {
      t.state = TrackState::CONFIRMED;
    }
  }

  // =========================
  // EKF measurement model and gating
  // =========================

  void computeMeasurementPredictionAndJacobian(
    const Track & t,
    double z_pred[3],
    double H[3][4]) const
  {
    const double px = t.x;
    const double py = t.y;
    const double vx = t.vx;
    const double vy = t.vy;

    const double rho2 = std::max(1e-6, px * px + py * py);
    const double rho = std::sqrt(rho2);
    const double rho3 = rho2 * rho;

    z_pred[0] = rho;
    z_pred[1] = std::atan2(py, px);
    z_pred[2] = (px * vx + py * vy) / rho;

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 4; ++c) {
        H[r][c] = 0.0;
      }
    }

    // range = sqrt(px^2 + py^2)
    H[0][0] = px / rho;
    H[0][1] = py / rho;

    // azimuth = atan2(py, px)
    H[1][0] = -py / rho2;
    H[1][1] = px / rho2;

    // radial_velocity = (px * vx + py * vy) / rho
    const double dot = px * vx + py * vy;
    H[2][0] = vx / rho - dot * px / rho3;
    H[2][1] = vy / rho - dot * py / rho3;
    H[2][2] = px / rho;
    H[2][3] = py / rho;
  }

  void buildInnovationCovariance(
    const Track & t,
    const double H[3][4],
    double S[3][3]) const
  {
    double HP[3][4] = {};

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 4; ++c) {
        for (int k = 0; k < 4; ++k) {
          HP[r][c] += H[r][k] * t.P[k][c];
        }
      }
    }

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        S[r][c] = 0.0;
        for (int k = 0; k < 4; ++k) {
          S[r][c] += HP[r][k] * H[c][k];
        }
      }
    }

    S[0][0] += sqr(kf_range_noise_m_);
    S[1][1] += sqr(kf_azimuth_noise_rad_);
    S[2][2] += sqr(kf_radial_velocity_noise_mps_);
  }

  double computeInnovationAndNis(
    const Track & t,
    const RadarPoint & det,
    double innovation[3]) const
  {
    double z_pred[3] = {};
    double H[3][4] = {};
    double S[3][3] = {};
    double invS[3][3] = {};

    computeMeasurementPredictionAndJacobian(t, z_pred, H);
    buildInnovationCovariance(t, H, S);

    if (!invert3x3(S, invS)) {
      return std::numeric_limits<double>::max();
    }

    innovation[0] = static_cast<double>(det.range) - z_pred[0];
    innovation[1] = normalizeAngle(static_cast<double>(det.azimuth_rad) - z_pred[1]);
    innovation[2] = static_cast<double>(det.radial_velocity) - z_pred[2];

    double nis = 0.0;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        nis += innovation[r] * invS[r][c] * innovation[c];
      }
    }

    return nis;
  }

  bool passMahalanobisGate(
    const Track & t,
    const RadarPoint & det,
    double * out_nis = nullptr) const
  {
    double innovation[3] = {};
    const double nis = computeInnovationAndNis(t, det, innovation);

    if (out_nis != nullptr) {
      *out_nis = nis;
    }

    return nis <= mahalanobis_gate_threshold_;
  }

  // =========================
  // Tracking
  // =========================

  Track createTrack(const RadarPoint & det, const rclcpp::Time & stamp)
  {
    Track t;
    t.id = next_track_id_++;

    t.x = det.x;
    t.y = det.y;

    const double init_range = std::max(1e-3, std::hypot(t.x, t.y));
    t.vx = static_cast<double>(det.radial_velocity) * t.x / init_range;
    t.vy = static_cast<double>(det.radial_velocity) * t.y / init_range;

    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        t.P[r][c] = 0.0;
      }
    }

    t.P[0][0] = 4.0;
    t.P[1][1] = 4.0;
    t.P[2][2] = 9.0;
    t.P[3][3] = 9.0;

    t.last_stamp = stamp;
    t.age = 1;
    t.hits = 1;
    t.missed = 0;
    t.consecutive_missed = 0;
    t.state = TrackState::TENTATIVE;

    pushLimitedBool(t.hit_history, true, track_confirm_window_);

    t.score = det.point_score;
    t.power = det.power;
    t.snr = det.snr;
    t.rcs = det.rcs;

    // Pass-through only.
    t.source_id = det.source_id;
    t.target_type = det.target_type;
    t.beam_count = det.beam_count;
    t.angular_extent_deg = det.angular_extent_deg;

    return t;
  }

  void predictTrack(Track & t, const rclcpp::Time & stamp)
  {
    double dt = (stamp - t.last_stamp).seconds();

    if (!std::isfinite(dt) || dt <= 0.0) {
      dt = 0.05;
    }

    dt = std::clamp(dt, 1e-3, 0.5);

    t.x += t.vx * dt;
    t.y += t.vy * dt;

    double F[4][4] = {
      {1.0, 0.0, dt, 0.0},
      {0.0, 1.0, 0.0, dt},
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0}
    };

    double FP[4][4] = {};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        for (int k = 0; k < 4; ++k) {
          FP[r][c] += F[r][k] * t.P[k][c];
        }
      }
    }

    double newP[4][4] = {};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        for (int k = 0; k < 4; ++k) {
          newP[r][c] += FP[r][k] * F[c][k];
        }
      }
    }

    const double q = kf_process_noise_;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    double Q[4][4] = {
      {0.25 * dt4 * q, 0.0, 0.5 * dt3 * q, 0.0},
      {0.0, 0.25 * dt4 * q, 0.0, 0.5 * dt3 * q},
      {0.5 * dt3 * q, 0.0, dt2 * q, 0.0},
      {0.0, 0.5 * dt3 * q, 0.0, dt2 * q}
    };

    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        t.P[r][c] = newP[r][c] + Q[r][c];
      }
    }

    t.last_stamp = stamp;
  }

  void markTrackMissed(Track & t)
  {
    t.missed += 1;
    t.consecutive_missed += 1;
    t.age += 1;
    pushLimitedBool(t.hit_history, false, track_confirm_window_);
  }

  void updateTrack(Track & t, const RadarPoint & det)
  {
    double z_pred[3] = {};
    double H[3][4] = {};
    double S[3][3] = {};
    double invS[3][3] = {};

    computeMeasurementPredictionAndJacobian(t, z_pred, H);
    buildInnovationCovariance(t, H, S);

    if (!invert3x3(S, invS)) {
      markTrackMissed(t);
      return;
    }

    double innovation[3] = {};
    innovation[0] = static_cast<double>(det.range) - z_pred[0];
    innovation[1] = normalizeAngle(static_cast<double>(det.azimuth_rad) - z_pred[1]);
    innovation[2] = static_cast<double>(det.radial_velocity) - z_pred[2];

    double nis = 0.0;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        nis += innovation[r] * invS[r][c] * innovation[c];
      }
    }

    t.last_nis = nis;
    pushLimitedDouble(t.nis_history, nis, nis_history_window_);

    double PHt[4][3] = {};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 4; ++k) {
          PHt[r][c] += t.P[r][k] * H[c][k];
        }
      }
    }

    double K[4][3] = {};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 3; ++k) {
          K[r][c] += PHt[r][k] * invS[k][c];
        }
      }
    }

    double dx[4] = {};
    for (int r = 0; r < 4; ++r) {
      for (int k = 0; k < 3; ++k) {
        dx[r] += K[r][k] * innovation[k];
      }
    }

    t.x += dx[0];
    t.y += dx[1];
    t.vx += dx[2];
    t.vy += dx[3];

    // Joseph-form covariance update.
    double KH[4][4] = {};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        for (int k = 0; k < 3; ++k) {
          KH[r][c] += K[r][k] * H[k][c];
        }
      }
    }

    double I_KH[4][4] = {};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        I_KH[r][c] = (r == c ? 1.0 : 0.0) - KH[r][c];
      }
    }

    double temp[4][4] = {};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        for (int k = 0; k < 4; ++k) {
          temp[r][c] += I_KH[r][k] * t.P[k][c];
        }
      }
    }

    double newP[4][4] = {};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        for (int k = 0; k < 4; ++k) {
          newP[r][c] += temp[r][k] * I_KH[c][k];
        }
      }
    }

    double R[3][3] = {};
    R[0][0] = sqr(kf_range_noise_m_);
    R[1][1] = sqr(kf_azimuth_noise_rad_);
    R[2][2] = sqr(kf_radial_velocity_noise_mps_);

    double KR[4][3] = {};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 3; ++k) {
          KR[r][c] += K[r][k] * R[k][c];
        }
      }
    }

    double KRKt[4][4] = {};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        for (int k = 0; k < 3; ++k) {
          KRKt[r][c] += KR[r][k] * K[c][k];
        }
      }
    }

    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        t.P[r][c] = newP[r][c] + KRKt[r][c];
      }
    }

    t.age += 1;
    t.hits += 1;
    t.missed = 0;
    t.consecutive_missed = 0;

    pushLimitedBool(t.hit_history, true, track_confirm_window_);
    updateTrackConfirmationState(t);

    t.score = 0.8f * t.score + 0.2f * det.point_score;
    t.power = 0.8f * t.power + 0.2f * det.power;
    t.snr = 0.8f * t.snr + 0.2f * det.snr;
    t.rcs = 0.8f * t.rcs + 0.2f * det.rcs;

    // Pass-through only.
    t.source_id = det.source_id;
    t.target_type = det.target_type;
    t.beam_count = det.beam_count;
    t.angular_extent_deg = det.angular_extent_deg;
  }

  std::vector<RadarPoint> updateTracks(
    const std::vector<Cluster> & clusters,
    const rclcpp::Time & stamp)
  {
    std::vector<RadarPoint> detections;
    detections.reserve(clusters.size());

    for (const auto & c : clusters) {
      detections.push_back(clusterToWeightedDetection(c));
    }

    // Predict all existing tracks.
    for (auto & t : tracks_) {
      predictTrack(t, stamp);
    }

    std::vector<bool> det_used(detections.size(), false);
    std::vector<bool> track_updated(tracks_.size(), false);

    // Associate by Mahalanobis distance in radar measurement space.
    for (size_t ti = 0; ti < tracks_.size(); ++ti) {
      Track & t = tracks_[ti];

      double best_nis = std::numeric_limits<double>::max();
      int best_idx = -1;

      for (size_t di = 0; di < detections.size(); ++di) {
        if (det_used[di]) {
          continue;
        }

        double nis = 0.0;
        if (!passMahalanobisGate(t, detections[di], &nis)) {
          continue;
        }

        if (nis < best_nis) {
          best_nis = nis;
          best_idx = static_cast<int>(di);
        }
      }

      if (best_idx >= 0) {
        updateTrack(t, detections[best_idx]);
        det_used[best_idx] = true;
        track_updated[ti] = true;
      }
    }

    // Mark unmatched tracks missed.
    for (size_t ti = 0; ti < tracks_.size(); ++ti) {
      if (!track_updated[ti]) {
        markTrackMissed(tracks_[ti]);
      }
    }

    // Create tentative tracks from unmatched detections.
    // No source_id / target_type / artificial-label based start rule.
    for (size_t di = 0; di < detections.size(); ++di) {
      if (!det_used[di]) {
        tracks_.push_back(createTrack(detections[di], stamp));
      }
    }

    // Delete bad or stale tracks.
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this](const Track & t) {
          if (t.consecutive_missed > track_delete_missed_) {
            return true;
          }

          if (t.missed > track_max_missed_) {
            return true;
          }

          if (t.state == TrackState::TENTATIVE &&
              static_cast<int>(t.hit_history.size()) >= track_confirm_window_ &&
              countHits(t.hit_history) < track_confirm_min_hits_) {
            return true;
          }

          if (t.state == TrackState::CONFIRMED && !nisHistoryLooksStable(t)) {
            return true;
          }

          return false;
        }),
      tracks_.end());

    std::vector<RadarPoint> output;
    output.reserve(tracks_.size());

    for (const auto & t : tracks_) {
      if (t.state != TrackState::CONFIRMED) {
        continue;
      }

      if (t.hits < track_min_hits_to_output_) {
        continue;
      }

      if (t.consecutive_missed > 3) {
        continue;
      }

      if (!nisHistoryLooksStable(t)) {
        continue;
      }

      RadarPoint p;
      p.x = static_cast<float>(t.x);
      p.y = static_cast<float>(t.y);
      p.z = 0.0f;

      p.range = static_cast<float>(std::hypot(t.x, t.y));
      p.azimuth_rad = static_cast<float>(std::atan2(t.y, t.x));
      p.azimuth_deg = static_cast<float>(p.azimuth_rad * 180.0 / M_PI);

      const double out_range = std::max(1e-3, std::hypot(t.x, t.y));
      p.radial_velocity = static_cast<float>((t.x * t.vx + t.y * t.vy) / out_range);

      p.point_score = t.score;
      p.power = t.power;
      p.snr = t.snr;
      p.rcs = t.rcs;

      // Pass-through only for evaluation/debug.
      p.source_id = t.source_id;
      p.target_type = t.target_type;
      p.beam_count = t.beam_count;
      p.angular_extent_deg = t.angular_extent_deg;

      output.push_back(p);
    }

    return output;
  }

  // =========================
  // Output cloud
  // =========================

  sensor_msgs::msg::PointCloud2 createOutputCloud(
    const std_msgs::msg::Header & header,
    const std::vector<RadarPoint> & points) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      14,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "power", 1, sensor_msgs::msg::PointField::FLOAT32,
      "radial_velocity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "snr", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rcs", 1, sensor_msgs::msg::PointField::FLOAT32,
      "range", 1, sensor_msgs::msg::PointField::FLOAT32,
      "azimuth_deg", 1, sensor_msgs::msg::PointField::FLOAT32,
      "timestamp", 1, sensor_msgs::msg::PointField::FLOAT32,
      "source_id", 1, sensor_msgs::msg::PointField::FLOAT32,
      "target_type", 1, sensor_msgs::msg::PointField::FLOAT32,
      "beam_count", 1, sensor_msgs::msg::PointField::FLOAT32,
      "angular_extent_deg", 1, sensor_msgs::msg::PointField::FLOAT32);

    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_power(cloud, "power");
    sensor_msgs::PointCloud2Iterator<float> iter_velocity(cloud, "radial_velocity");
    sensor_msgs::PointCloud2Iterator<float> iter_snr(cloud, "snr");
    sensor_msgs::PointCloud2Iterator<float> iter_rcs(cloud, "rcs");
    sensor_msgs::PointCloud2Iterator<float> iter_range(cloud, "range");
    sensor_msgs::PointCloud2Iterator<float> iter_azimuth(cloud, "azimuth_deg");
    sensor_msgs::PointCloud2Iterator<float> iter_timestamp(cloud, "timestamp");
    sensor_msgs::PointCloud2Iterator<float> iter_source_id(cloud, "source_id");
    sensor_msgs::PointCloud2Iterator<float> iter_target_type(cloud, "target_type");
    sensor_msgs::PointCloud2Iterator<float> iter_beam_count(cloud, "beam_count");
    sensor_msgs::PointCloud2Iterator<float> iter_extent(cloud, "angular_extent_deg");

    const float timestamp = static_cast<float>(header.stamp.sec + header.stamp.nanosec * 1e-9);

    for (const auto & p : points) {
      *iter_x = p.x;
      *iter_y = p.y;
      *iter_z = p.z;
      *iter_power = p.power;
      *iter_velocity = p.radial_velocity;
      *iter_snr = p.snr;
      *iter_rcs = p.rcs;
      *iter_range = p.range;
      *iter_azimuth = p.azimuth_deg;
      *iter_timestamp = timestamp;
      *iter_source_id = p.source_id;
      *iter_target_type = p.target_type;
      *iter_beam_count = p.beam_count;
      *iter_extent = p.angular_extent_deg;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_power;
      ++iter_velocity;
      ++iter_snr;
      ++iter_rcs;
      ++iter_range;
      ++iter_azimuth;
      ++iter_timestamp;
      ++iter_source_id;
      ++iter_target_type;
      ++iter_beam_count;
      ++iter_extent;
    }

    return cloud;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MmwaveRadarFilterNode>());
  rclcpp::shutdown();
  return 0;
}
