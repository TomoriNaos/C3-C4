#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "c3_drone_driver/msg/gimbal_visual_command.hpp"
#include "c3_drone_driver/msg/target_observation.hpp"

namespace c3_drone_driver
{

class TargetProcessorNode : public rclcpp::Node
{
public:
  TargetProcessorNode()
  : Node("target_processor_node")
  {
    time_sync_threshold_s_ = declare_parameter<double>("time_sync_threshold_s", 0.1);
    default_confidence_ = declare_parameter<double>("default_confidence", 0.6);
    target_type_default_ = declare_parameter<int>("target_type_default", 0);
    visual_cmd_gain_ = declare_parameter<double>("visual_cmd_gain", 1.0);
    roi_margin_px_ = declare_parameter<int>("roi_margin_px", 16);
    image_width_ = declare_parameter<int>("image_width", 1280);
    image_height_ = declare_parameter<int>("image_height", 720);
    depth_min_ = declare_parameter<double>("depth_min", 0.5);
    depth_max_ = declare_parameter<double>("depth_max", 120.0);
    min_roi_points_ = declare_parameter<int>("min_roi_points", 20);
    low_conf_threshold_ = declare_parameter<double>("low_conf_threshold", 0.55);
    tc_noise_std_ = declare_parameter<double>("tc_noise_std", 0.10);
    gc_noise_std_ = declare_parameter<double>("gc_noise_std", 0.05);
    smoothing_alpha_ = declare_parameter<double>("smoothing_alpha", 0.35);
    bbox_timeout_s_ = declare_parameter<double>("bbox_timeout_s", 0.3);

    tc_bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/tc/bbox", 10,
      std::bind(&TargetProcessorNode::onTcBbox, this, std::placeholders::_1));
    tc_pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/tc/points", rclcpp::SensorDataQoS(),
      std::bind(&TargetProcessorNode::onTcPointCloud, this, std::placeholders::_1));
    gc_pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/gc/points", rclcpp::SensorDataQoS(),
      std::bind(&TargetProcessorNode::onGcPointCloud, this, std::placeholders::_1));

    observation_pub_ = create_publisher<msg::TargetObservation>("/target/observation_body", 10);
    visual_cmd_pub_ = create_publisher<msg::GimbalVisualCommand>("/gimbal/visual_command", 10);

    RCLCPP_INFO(get_logger(), "target_processor_node started");
  }

private:
  void onTcBbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    last_bbox_ = *msg;
    has_bbox_ = true;
    last_bbox_time_ = now();
    publishIfReady();
  }

  void onTcPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_tc_pc_ = *msg;
    has_tc_pc_ = true;
    publishIfReady();
  }

  void onGcPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_gc_pc_ = *msg;
    has_gc_pc_ = true;
    publishIfReady();
  }

  void publishIfReady()
  {
    if (!has_bbox_ || !has_tc_pc_ || !has_gc_pc_) {
      return;
    }
    if ((now() - last_bbox_time_).seconds() > bbox_timeout_s_) {
      return;
    }

    const auto t_tc = rclcpp::Time(last_tc_pc_.header.stamp);
    const auto t_gc = rclcpp::Time(last_gc_pc_.header.stamp);
    if (std::abs((t_tc - t_gc).seconds()) > time_sync_threshold_s_) {
      return;
    }

    float cx = 0.0F;
    float cy = 0.0F;
    float w = 0.0F;
    float det_conf = static_cast<float>(default_confidence_);

    if (last_bbox_.data.size() >= 4) {
      cx = last_bbox_.data[0];
      cy = last_bbox_.data[1];
      w = last_bbox_.data[2];
    }
    if (last_bbox_.data.size() >= 5) {
      det_conf = last_bbox_.data[4];
    }
    uint32_t target_id = 0;
    if (last_bbox_.data.size() >= 6) {
      target_id = static_cast<uint32_t>(std::max(0.0F, last_bbox_.data[5]));
    }
    uint8_t target_type = static_cast<uint8_t>(target_type_default_);
    if (last_bbox_.data.size() >= 7) {
      target_type = static_cast<uint8_t>(std::max(0.0F, last_bbox_.data[6]));
    }

    const float half_w = std::max(1.0F, w * 0.5F);
    const float half_h = std::max(1.0F, half_w);
    const int x_min = std::max(0, static_cast<int>(std::floor(cx - half_w)) - roi_margin_px_);
    const int x_max = std::min(image_width_ - 1, static_cast<int>(std::ceil(cx + half_w)) + roi_margin_px_);
    const int y_min = std::max(0, static_cast<int>(std::floor(cy - half_h)) - roi_margin_px_);
    const int y_max = std::min(image_height_ - 1, static_cast<int>(std::ceil(cy + half_h)) + roi_margin_px_);

    geometry_msgs::msg::Point tc_center;
    geometry_msgs::msg::Point gc_center;
    bool tc_ok = extractRoiCentroid(last_tc_pc_, x_min, x_max, y_min, y_max, tc_center);
    bool gc_ok = extractRoiCentroid(last_gc_pc_, x_min, x_max, y_min, y_max, gc_center);

    geometry_msgs::msg::Point fused_center;
    uint8_t source = msg::TargetObservation::SOURCE_FUSED;
    if (!tc_ok && !gc_ok) {
      publishLostObservation(target_id, target_type, det_conf);
      return;
    } else if (tc_ok && gc_ok) {
      const double tc_w = 1.0 / std::max(1e-6, tc_noise_std_ * tc_noise_std_);
      const double gc_w = 1.0 / std::max(1e-6, gc_noise_std_ * gc_noise_std_);
      const double sum_w = tc_w + gc_w;
      fused_center.x = static_cast<float>((tc_center.x * tc_w + gc_center.x * gc_w) / sum_w);
      fused_center.y = static_cast<float>((tc_center.y * tc_w + gc_center.y * gc_w) / sum_w);
      fused_center.z = static_cast<float>((tc_center.z * tc_w + gc_center.z * gc_w) / sum_w);
      source = msg::TargetObservation::SOURCE_FUSED;
    } else if (gc_ok) {
      fused_center = gc_center;
      source = msg::TargetObservation::SOURCE_GC_ONLY;
    } else {
      fused_center = tc_center;
      source = msg::TargetObservation::SOURCE_TC_ONLY;
    }

    if (!has_smoothed_position_) {
      smoothed_position_ = fused_center;
      has_smoothed_position_ = true;
    } else {
      smoothed_position_.x = static_cast<float>(
        smoothing_alpha_ * fused_center.x + (1.0 - smoothing_alpha_) * smoothed_position_.x);
      smoothed_position_.y = static_cast<float>(
        smoothing_alpha_ * fused_center.y + (1.0 - smoothing_alpha_) * smoothed_position_.y);
      smoothed_position_.z = static_cast<float>(
        smoothing_alpha_ * fused_center.z + (1.0 - smoothing_alpha_) * smoothed_position_.z);
    }

    msg::TargetObservation obs;
    obs.header.stamp = now();
    obs.header.frame_id = "base_link";
    obs.obs_id = ++obs_id_;
    obs.track_id = track_id_;
    obs.target_id = target_id;
    obs.target_type = target_type;
    obs.position = smoothed_position_;
    obs.range = static_cast<float>(
      std::sqrt(obs.position.x * obs.position.x + obs.position.y * obs.position.y +
      obs.position.z * obs.position.z));
    obs.yaw = static_cast<float>(std::atan2(obs.position.y, obs.position.x));
    obs.pitch = static_cast<float>(
      std::atan2(obs.position.z, std::sqrt(obs.position.x * obs.position.x + obs.position.y * obs.position.y)));
    obs.confidence = std::clamp(det_conf, 0.0F, 1.0F);
    obs.source = source;
    obs.status = (obs.confidence >= low_conf_threshold_) ?
      msg::TargetObservation::STATUS_VALID : msg::TargetObservation::STATUS_LOW_CONF;

    msg::GimbalVisualCommand gimbal_cmd;
    gimbal_cmd.header = obs.header;
    gimbal_cmd.yaw = static_cast<float>(visual_cmd_gain_ * obs.yaw);
    gimbal_cmd.pitch = static_cast<float>(visual_cmd_gain_ * obs.pitch);
    gimbal_cmd.confidence = obs.confidence;

    observation_pub_->publish(obs);
    visual_cmd_pub_->publish(gimbal_cmd);
  }

  bool extractRoiCentroid(
    const sensor_msgs::msg::PointCloud2 & cloud, int x_min, int x_max, int y_min, int y_max,
    geometry_msgs::msg::Point & centroid) const
  {
    if (cloud.height == 0 || cloud.width == 0 || cloud.height == 1) {
      return false;
    }

    const int cloud_w = static_cast<int>(cloud.width);
    const int cloud_h = static_cast<int>(cloud.height);
    const int xs = std::max(0, std::min(x_min, cloud_w - 1));
    const int xe = std::max(0, std::min(x_max, cloud_w - 1));
    const int ys = std::max(0, std::min(y_min, cloud_h - 1));
    const int ye = std::max(0, std::min(y_max, cloud_h - 1));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    const std::size_t total = static_cast<std::size_t>(cloud_w * cloud_h);
    std::vector<float> roi_x;
    std::vector<float> roi_y;
    std::vector<float> roi_z;
    roi_x.reserve(total / 20);
    roi_y.reserve(total / 20);
    roi_z.reserve(total / 20);

    for (std::size_t idx = 0; idx < total; ++idx, ++iter_x, ++iter_y, ++iter_z) {
      const int px = static_cast<int>(idx % cloud_w);
      const int py = static_cast<int>(idx / cloud_w);
      if (px < xs || px > xe || py < ys || py > ye) {
        continue;
      }

      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < depth_min_ || z > depth_max_) {
        continue;
      }
      roi_x.push_back(x);
      roi_y.push_back(y);
      roi_z.push_back(z);
    }

    if (static_cast<int>(roi_x.size()) < min_roi_points_) {
      return false;
    }

    auto robustMean = [](std::vector<float> & values) -> double {
        std::sort(values.begin(), values.end());
        const std::size_t n = values.size();
        const std::size_t l = n / 10;
        const std::size_t r = (n * 9) / 10;
        const std::size_t begin = std::min(l, n - 1);
        const std::size_t end = std::max(begin + 1, r);
        double sum = 0.0;
        std::size_t cnt = 0;
        for (std::size_t i = begin; i < end && i < n; ++i) {
          sum += values[i];
          ++cnt;
        }
        return (cnt > 0) ? (sum / static_cast<double>(cnt)) : 0.0;
      };

    centroid.x = robustMean(roi_x);
    centroid.y = robustMean(roi_y);
    centroid.z = robustMean(roi_z);
    return true;
  }

  void publishLostObservation(uint32_t target_id, uint8_t target_type, float det_conf)
  {
    msg::TargetObservation obs;
    obs.header.stamp = now();
    obs.header.frame_id = "base_link";
    obs.obs_id = ++obs_id_;
    obs.track_id = track_id_;
    obs.target_id = target_id;
    obs.target_type = target_type;
    obs.position.x = std::numeric_limits<float>::quiet_NaN();
    obs.position.y = std::numeric_limits<float>::quiet_NaN();
    obs.position.z = std::numeric_limits<float>::quiet_NaN();
    obs.range = std::numeric_limits<float>::quiet_NaN();
    obs.yaw = 0.0F;
    obs.pitch = 0.0F;
    obs.confidence = std::clamp(det_conf, 0.0F, 1.0F);
    obs.source = 0;
    obs.status = msg::TargetObservation::STATUS_LOST;
    observation_pub_->publish(obs);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tc_bbox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tc_pc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr gc_pc_sub_;

  rclcpp::Publisher<msg::TargetObservation>::SharedPtr observation_pub_;
  rclcpp::Publisher<msg::GimbalVisualCommand>::SharedPtr visual_cmd_pub_;

  std_msgs::msg::Float32MultiArray last_bbox_;
  sensor_msgs::msg::PointCloud2 last_tc_pc_;
  sensor_msgs::msg::PointCloud2 last_gc_pc_;

  bool has_bbox_{false};
  bool has_tc_pc_{false};
  bool has_gc_pc_{false};

  double time_sync_threshold_s_{0.1};
  double default_confidence_{0.6};
  int target_type_default_{0};
  double visual_cmd_gain_{1.0};
  int roi_margin_px_{16};
  int image_width_{1280};
  int image_height_{720};
  double depth_min_{0.5};
  double depth_max_{120.0};
  int min_roi_points_{20};
  double low_conf_threshold_{0.55};
  double tc_noise_std_{0.10};
  double gc_noise_std_{0.05};
  double smoothing_alpha_{0.35};
  double bbox_timeout_s_{0.3};

  uint32_t obs_id_{0};
  uint32_t track_id_{1};
  rclcpp::Time last_bbox_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Point smoothed_position_{};
  bool has_smoothed_position_{false};
};

}  // namespace c3_drone_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<c3_drone_driver::TargetProcessorNode>());
  rclcpp::shutdown();
  return 0;
}
