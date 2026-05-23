#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

struct PointXYZ
{
  float x;
  float y;
  float z;
  float range;
};

struct Cluster
{
  std::vector<PointXYZ> points;
  float avg_depth;
  float center_x;
  float center_y;
  float center_z;
};

class LidarDepthNode : public rclcpp::Node
{
public:
  LidarDepthNode() : Node("lidar_depth_node")
  {
    RCLCPP_INFO(this->get_logger(), "激光雷达点云分簇节点已启动");

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&LidarDepthNode::get_cloud_point, this, std::placeholders::_1));

    gap_max_ = 2;
    max_diff_x_ = 0.5f;
    max_diff_y_ = 0.5f;
    max_diff_range_ = 0.8f;
    max_diff_euclidean_ = 1.0f;
    min_cluster_size_ = 3;
  }

private:
  // =========================
  // 回调函数：scan -> cloud_points -> clusters
  // =========================
  void get_cloud_point(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<PointXYZ> cloud_points;

    float angle = msg->angle_min;

    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float range = msg->ranges[i];

      if (std::isnan(range) || std::isinf(range))
      {
        angle += msg->angle_increment;
        continue;
      }

      if (range < msg->range_min || range > msg->range_max)
      {
        angle += msg->angle_increment;
        continue;
      }

      PointXYZ point;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);
      point.z = 0.0f;
      point.range = range;

      cloud_points.push_back(point);
      angle += msg->angle_increment;
    }

    std::vector<Cluster> clusters = spilt_point(cloud_points);

    RCLCPP_INFO(this->get_logger(), "当前帧有效点数: %zu, 分割得到点簇数: %zu",
                cloud_points.size(), clusters.size());

    for (size_t i = 0; i < clusters.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(),
                  "点簇[%zu] 点数=%zu, 平均深度=%.2f, 中心=(%.2f, %.2f, %.2f)",
                  i,
                  clusters[i].points.size(),
                  clusters[i].avg_depth,
                  clusters[i].center_x,
                  clusters[i].center_y,
                  clusters[i].center_z);
    }
  }

  // =========================
  // 分割点簇函数
  // 输入：点云 points
  // 输出：点簇 clusters
  // =========================
  std::vector<Cluster> spilt_point(const std::vector<PointXYZ> & points)
  {
    std::vector<Cluster> clusters;

    if (points.empty())
    {
      return clusters;
    }

    std::vector<PointXYZ> current_points_vector;
    int gap_count = 0;

    for (size_t i = 0; i < points.size(); ++i)
    {
      const PointXYZ & current_point = points[i];

      if (current_points_vector.empty())
      {
        current_points_vector.push_back(current_point);
        continue;
      }

      const PointXYZ & last_point = current_points_vector.back();

      PointXYZ center_point = get_center_xyz(current_points_vector);
      float avg_depth = get_depth(current_points_vector);

      float ref_x = 0.5f * last_point.x + 0.5f * center_point.x;
      float ref_y = 0.5f * last_point.y + 0.5f * center_point.y;

      float diff_x = std::fabs(current_point.x - ref_x);
      float diff_y = std::fabs(current_point.y - ref_y);
      float diff_range = std::fabs(current_point.range - avg_depth);

      float dx = current_point.x - last_point.x;
      float dy = current_point.y - last_point.y;
      float euclidean_dist = std::sqrt(dx * dx + dy * dy);

      if (diff_x < max_diff_x_ &&
          diff_y < max_diff_y_ &&
          diff_range < max_diff_range_ &&
          euclidean_dist < max_diff_euclidean_)
      {
        current_points_vector.push_back(current_point);
        gap_count = 0;
      }
      else
      {
        gap_count++;

        if (gap_count > gap_max_)
        {
          if (current_points_vector.size() >= min_cluster_size_)
          {
            Cluster cluster;
            cluster.points = current_points_vector;
            cluster.avg_depth = get_depth(current_points_vector);

            PointXYZ center = get_center_xyz(current_points_vector);
            cluster.center_x = center.x;
            cluster.center_y = center.y;
            cluster.center_z = center.z;

            clusters.push_back(cluster);
          }

          current_points_vector.clear();
          current_points_vector.push_back(current_point);
          gap_count = 0;
        }
      }
    }

    if (current_points_vector.size() >= min_cluster_size_)
    {
      Cluster cluster;
      cluster.points = current_points_vector;
      cluster.avg_depth = get_depth(current_points_vector);

      PointXYZ center = get_center_xyz(current_points_vector);
      cluster.center_x = center.x;
      cluster.center_y = center.y;
      cluster.center_z = center.z;

      clusters.push_back(cluster);
    }

    // 再过滤一次：少于3个点的点簇视为噪点
    clusters.erase(
      std::remove_if(clusters.begin(), clusters.end(),
        [this](const Cluster & c)
        {
          return c.points.size() < min_cluster_size_;
        }),
      clusters.end());

    // 相邻点簇合并
    clusters = merge_clusters(clusters);

    return clusters;
  }

  // =========================
  // 获取点组平均深度
  // =========================
  float get_depth(const std::vector<PointXYZ> & points)
  {
    if (points.empty())
    {
      return 0.0f;
    }

    float sum = 0.0f;
    for (size_t i = 0; i < points.size(); ++i)
    {
      sum += points[i].range;
    }

    return sum / static_cast<float>(points.size());
  }

  // =========================
  // 获取点组中心点坐标
  // =========================
  PointXYZ get_center_xyz(const std::vector<PointXYZ> & points)
  {
    PointXYZ center{0.0f, 0.0f, 0.0f, 0.0f};

    if (points.empty())
    {
      return center;
    }

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    float sum_range = 0.0f;

    for (size_t i = 0; i < points.size(); ++i)
    {
      sum_x += points[i].x;
      sum_y += points[i].y;
      sum_z += points[i].z;
      sum_range += points[i].range;
    }

    center.x = sum_x / static_cast<float>(points.size());
    center.y = sum_y / static_cast<float>(points.size());
    center.z = sum_z / static_cast<float>(points.size());
    center.range = sum_range / static_cast<float>(points.size());

    return center;
  }

  // =========================
// 合并相邻点簇
// 只比较前后两个点簇，不做全量两两比较
// 条件：中心欧式距离近 + 平均深度接近
// =========================
std::vector<Cluster> merge_clusters(const std::vector<Cluster> & input_clusters)
{
  if (input_clusters.empty())
  {
    return input_clusters;
  }

  std::vector<Cluster> merged_clusters;

  // 先把第一个点簇作为当前点簇
  Cluster current_cluster = input_clusters[0];

  // 从第二个点簇开始，逐个和前一个点簇比较
  for (size_t i = 1; i < input_clusters.size(); ++i)
  {
    const Cluster & next_cluster = input_clusters[i];

    // 计算前后两个点簇中心点之间的欧式距离
    float dx = current_cluster.center_x - next_cluster.center_x;
    float dy = current_cluster.center_y - next_cluster.center_y;
    float dz = current_cluster.center_z - next_cluster.center_z;
    float cluster_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 计算前后两个点簇平均深度差
    float range_diff = std::fabs(current_cluster.avg_depth - next_cluster.avg_depth);

    // 如果两个点簇足够接近，就合并
    if (cluster_distance < max_diff_euclidean_ && range_diff < max_diff_range_)
    {
      // 把 next_cluster 的点加入 current_cluster
      current_cluster.points.insert(
        current_cluster.points.end(),
        next_cluster.points.begin(),
        next_cluster.points.end());

      // 重新计算合并后点簇的平均深度和中心点
      current_cluster.avg_depth = get_depth(current_cluster.points);
      PointXYZ new_center = get_center_xyz(current_cluster.points);
      current_cluster.center_x = new_center.x;
      current_cluster.center_y = new_center.y;
      current_cluster.center_z = new_center.z;
    }
    else
    {
      // 如果不能合并，就把当前点簇存起来
      merged_clusters.push_back(current_cluster);

      // 然后把下一个点簇作为新的当前点簇
      current_cluster = next_cluster;
    }
  }

  // 最后一个 current_cluster 别忘了放进去
  merged_clusters.push_back(current_cluster);

  return merged_clusters;
}


private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  int gap_max_;
  int min_cluster_size_;
  float max_diff_x_;
  float max_diff_y_;
  float max_diff_range_;
  float max_diff_euclidean_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDepthNode>());
  rclcpp::shutdown();
  return 0;
}
