#ifndef USV_RVIZ_DASHBOARD__PERCEPTION_DASHBOARD_HPP_
#define USV_RVIZ_DASHBOARD__PERCEPTION_DASHBOARD_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QImage>
#include <QLabel>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

namespace usv_rviz_dashboard
{

class PerceptionDashboard : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PerceptionDashboard(QWidget * parent = nullptr);
  void onInitialize() override;

Q_SIGNALS:
  void image_ready(int slot, const QImage & image);

private Q_SLOTS:
  void set_image(int slot, const QImage & image);

private:
  using ImageSubscription = rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr;
  using CloudSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
  void subscribe_image(const std::string & topic, int slot, bool pseudo_color);
  void on_image(int slot, bool pseudo_color, const sensor_msgs::msg::Image::SharedPtr msg);
  void on_cloud(int slot, const sensor_msgs::msg::PointCloud2::SharedPtr msg, const QString & title);
  bool should_render(int slot, std::chrono::milliseconds minimum_period);
  void refresh_image_pixmap(int slot);

  static QImage to_qimage(const sensor_msgs::msg::Image & msg, bool pseudo_color);
  static QImage false_color(const QImage & image);
  static QImage cloud_to_qimage(const sensor_msgs::msg::PointCloud2 & msg, const QString & title);
  static QLabel * make_image_label(const QString & title);

  rclcpp::Node::SharedPtr node_;
  std::array<QLabel *, 11> image_labels_{};
  std::array<QImage, 11> latest_images_{};
  std::array<std::chrono::steady_clock::time_point, 11> last_render_times_{};
  std::mutex render_mutex_;
  std::vector<ImageSubscription> image_subscriptions_;
  std::vector<CloudSubscription> cloud_subscriptions_;
};

}  // namespace usv_rviz_dashboard

#endif  // USV_RVIZ_DASHBOARD__PERCEPTION_DASHBOARD_HPP_
