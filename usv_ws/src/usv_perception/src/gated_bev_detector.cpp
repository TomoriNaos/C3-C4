#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

namespace usv_perception
{

struct BevCluster
{
  int count{0};
  double sum_x{0.0};
  double sum_y{0.0};
  double min_x{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};
};

struct BevDetection
{
  float x{0.0F};
  float y{0.0F};
  float z{0.8F};
  float confidence{0.0F};
  float class_id{5.0F};
};

class GatedBevDetector : public rclcpp::Node
{
public:
  GatedBevDetector()
  : Node("gated_bev_detector")
  {
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth_camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/depth_camera/camera_info");
    output_topic_ = declare_parameter<std::string>("output_topic", "/gated_camera/bev_detection_points");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    camera_x_offset_ = declare_parameter<double>("camera_x_offset", 1.55);
    camera_y_offset_ = declare_parameter<double>("camera_y_offset", 0.0);
    camera_z_offset_ = declare_parameter<double>("camera_z_offset", 1.20);
    camera_horizontal_fov_ = declare_parameter<double>("camera_horizontal_fov", 1.3962634);
    min_range_ = declare_parameter<double>("min_range", 2.0);
    max_range_ = declare_parameter<double>("max_range", 90.0);
    min_z_ = declare_parameter<double>("min_z", 0.45);
    max_z_ = declare_parameter<double>("max_z", 4.0);
    lateral_range_ = declare_parameter<double>("lateral_range", 35.0);
    cell_size_ = declare_parameter<double>("cell_size", 0.65);
    pixel_stride_ = std::max(1, static_cast<int>(declare_parameter<int>("pixel_stride", 4)));
    min_cell_points_ = std::max(1, static_cast<int>(declare_parameter<int>("min_cell_points", 2)));
    min_cluster_cells_ = std::max(1, static_cast<int>(declare_parameter<int>("min_cluster_cells", 3)));
    min_cluster_points_ = std::max(1, static_cast<int>(declare_parameter<int>("min_cluster_points", 18)));

    grid_x_count_ = std::max(1, static_cast<int>(std::ceil((max_range_ - min_range_) / cell_size_)));
    grid_y_count_ = std::max(1, static_cast<int>(std::ceil((2.0 * lateral_range_) / cell_size_)));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10, std::bind(&GatedBevDetector::on_camera_info, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GatedBevDetector::on_depth, this, std::placeholders::_1));
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/gated_camera/bev_detector/status", 10);
    RCLCPP_INFO(get_logger(), "Gated BEV detector depth=%s output=%s", depth_topic_.c_str(), output_topic_.c_str());
  }

private:
  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (msg->k[0] > 0.0 && msg->k[4] > 0.0) {
      fx_ = msg->k[0];
      fy_ = msg->k[4];
      cx_ = msg->k[2];
      cy_ = msg->k[5];
      has_camera_info_ = true;
    }
  }

  void on_depth(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_like_depth_.clear();
    const std::string encoding = msg->encoding;
    if (encoding != "32FC1" && encoding != "16UC1" && encoding != "mono16") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Unsupported depth encoding: %s", encoding.c_str());
      return;
    }

    if (!has_camera_info_) {
      fx_ = static_cast<double>(msg->width) / (2.0 * std::tan(camera_horizontal_fov_ * 0.5));
      fy_ = fx_;
      cx_ = 0.5 * static_cast<double>(msg->width - 1);
      cy_ = 0.5 * static_cast<double>(msg->height - 1);
    }

    std::vector<BevCluster> cells(static_cast<std::size_t>(grid_x_count_ * grid_y_count_));
    for (int v = 0; v < static_cast<int>(msg->height); v += pixel_stride_) {
      for (int u = 0; u < static_cast<int>(msg->width); u += pixel_stride_) {
        const float depth = read_depth(*msg, u, v);
        if (!std::isfinite(depth) || depth < min_range_ || depth > max_range_) {
          continue;
        }
        const double lateral = (static_cast<double>(u) - cx_) * depth / fx_;
        const double vertical = (static_cast<double>(v) - cy_) * depth / fy_;
        const double x = camera_x_offset_ + depth;
        const double y = camera_y_offset_ - lateral;
        const double z = camera_z_offset_ - vertical;
        if (z < min_z_ || z > max_z_ || std::abs(y) > lateral_range_) {
          continue;
        }
        const int ix = static_cast<int>((depth - min_range_) / cell_size_);
        const int iy = static_cast<int>((y + lateral_range_) / cell_size_);
        if (ix < 0 || ix >= grid_x_count_ || iy < 0 || iy >= grid_y_count_) {
          continue;
        }
        auto & cell = cells[cell_index(ix, iy)];
        ++cell.count;
        cell.sum_x += x;
        cell.sum_y += y;
        cell.min_x = std::min(cell.min_x, x);
        cell.max_x = std::max(cell.max_x, x);
        cell.min_y = std::min(cell.min_y, y);
        cell.max_y = std::max(cell.max_y, y);
      }
    }

    const auto detections = cluster_cells(cells);
    points_pub_->publish(to_cloud(detections, msg->header));
    publish_status(detections);
  }

  float read_depth(const sensor_msgs::msg::Image & msg, int u, int v) const
  {
    const auto * row = msg.data.data() + static_cast<std::size_t>(v) * msg.step;
    if (msg.encoding == "32FC1") {
      float value = 0.0F;
      std::memcpy(&value, row + static_cast<std::size_t>(u) * sizeof(float), sizeof(float));
      return value;
    }
    std::uint16_t value = 0;
    std::memcpy(&value, row + static_cast<std::size_t>(u) * sizeof(std::uint16_t), sizeof(std::uint16_t));
    return static_cast<float>(value) * 0.001F;
  }

  std::vector<BevDetection> cluster_cells(const std::vector<BevCluster> & cells) const
  {
    std::vector<unsigned char> visited(cells.size(), 0);
    std::vector<BevDetection> detections;
    constexpr int dx[4]{1, -1, 0, 0};
    constexpr int dy[4]{0, 0, 1, -1};

    for (int ix = 0; ix < grid_x_count_; ++ix) {
      for (int iy = 0; iy < grid_y_count_; ++iy) {
        const auto start = cell_index(ix, iy);
        if (visited[start] || cells[start].count < min_cell_points_) {
          continue;
        }

        std::deque<std::pair<int, int>> queue;
        queue.push_back({ix, iy});
        visited[start] = 1;
        BevCluster merged;
        int cell_count = 0;
        while (!queue.empty()) {
          const auto [cx, cy] = queue.front();
          queue.pop_front();
          const auto index = cell_index(cx, cy);
          const auto & cell = cells[index];
          ++cell_count;
          merged.count += cell.count;
          merged.sum_x += cell.sum_x;
          merged.sum_y += cell.sum_y;
          merged.min_x = std::min(merged.min_x, cell.min_x);
          merged.max_x = std::max(merged.max_x, cell.max_x);
          merged.min_y = std::min(merged.min_y, cell.min_y);
          merged.max_y = std::max(merged.max_y, cell.max_y);

          for (int dir = 0; dir < 4; ++dir) {
            const int nx = cx + dx[dir];
            const int ny = cy + dy[dir];
            if (nx < 0 || nx >= grid_x_count_ || ny < 0 || ny >= grid_y_count_) {
              continue;
            }
            const auto next = cell_index(nx, ny);
            if (!visited[next] && cells[next].count >= min_cell_points_) {
              visited[next] = 1;
              queue.push_back({nx, ny});
            }
          }
        }

        if (cell_count < min_cluster_cells_ || merged.count < min_cluster_points_) {
          continue;
        }
        const double confidence = std::clamp(0.32 + 0.012 * merged.count + 0.035 * cell_count, 0.35, 0.94);
        detections.push_back(BevDetection{
          static_cast<float>(merged.sum_x / merged.count),
          static_cast<float>(merged.sum_y / merged.count),
          0.8F,
          static_cast<float>(confidence),
          5.0F});
      }
    }
    return detections;
  }

  sensor_msgs::msg::PointCloud2 to_cloud(
    const std::vector<BevDetection> & detections,
    const std_msgs::msg::Header & header) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.header.frame_id = base_frame_;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(detections.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 36;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(9);
    set_field(cloud.fields[0], "x", 0);
    set_field(cloud.fields[1], "y", 4);
    set_field(cloud.fields[2], "z", 8);
    set_field(cloud.fields[3], "intensity", 12);
    set_field(cloud.fields[4], "class_id", 16);
    set_field(cloud.fields[5], "bbox_cx", 20);
    set_field(cloud.fields[6], "bbox_cy", 24);
    set_field(cloud.fields[7], "bbox_w", 28);
    set_field(cloud.fields[8], "bbox_h", 32);
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
    constexpr float no_bbox = 0.0F;
    for (std::size_t i = 0; i < detections.size(); ++i) {
      unsigned char * dst = cloud.data.data() + i * cloud.point_step;
      std::memcpy(dst + 0, &detections[i].x, sizeof(float));
      std::memcpy(dst + 4, &detections[i].y, sizeof(float));
      std::memcpy(dst + 8, &detections[i].z, sizeof(float));
      std::memcpy(dst + 12, &detections[i].confidence, sizeof(float));
      std::memcpy(dst + 16, &detections[i].class_id, sizeof(float));
      std::memcpy(dst + 20, &no_bbox, sizeof(float));
      std::memcpy(dst + 24, &no_bbox, sizeof(float));
      std::memcpy(dst + 28, &no_bbox, sizeof(float));
      std::memcpy(dst + 32, &no_bbox, sizeof(float));
    }
    return cloud;
  }

  static void set_field(sensor_msgs::msg::PointField & field, const std::string & name, std::uint32_t offset)
  {
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
  }

  void publish_status(const std::vector<BevDetection> & detections)
  {
    std_msgs::msg::String msg;
    std::ostringstream text;
    text << "bev_detections=" << detections.size();
    for (const auto & det : detections) {
      text << " | (" << det.x << "," << det.y << ") c=" << det.confidence;
    }
    msg.data = text.str();
    status_pub_->publish(msg);
  }

  std::size_t cell_index(int ix, int iy) const
  {
    return static_cast<std::size_t>(iy * grid_x_count_ + ix);
  }

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  std::string base_frame_;
  double camera_x_offset_{1.55};
  double camera_y_offset_{0.0};
  double camera_z_offset_{1.20};
  double camera_horizontal_fov_{1.3962634};
  double min_range_{2.0};
  double max_range_{90.0};
  double min_z_{0.45};
  double max_z_{4.0};
  double lateral_range_{35.0};
  double cell_size_{0.65};
  int pixel_stride_{4};
  int min_cell_points_{2};
  int min_cluster_cells_{3};
  int min_cluster_points_{18};
  int grid_x_count_{1};
  int grid_y_count_{1};
  double fx_{1.0};
  double fy_{1.0};
  double cx_{0.0};
  double cy_{0.0};
  bool has_camera_info_{false};
  std::vector<unsigned char> cv_like_depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::GatedBevDetector>());
  rclcpp::shutdown();
  return 0;
}
