#ifndef USV_RVIZ_DASHBOARD__PERCEPTION_DASHBOARD_HPP_
#define USV_RVIZ_DASHBOARD__PERCEPTION_DASHBOARD_HPP_

#include <array>
#include <memory>
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
  void metrics_ready(const QString & text);

private Q_SLOTS:
  void set_image(int slot, const QImage & image);
  void set_metrics(const QString & text);

private:
  using ImageSubscription = rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr;
  using CloudSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
  void subscribe_image(const std::string & topic, int slot, bool pseudo_color);
  void on_image(int slot, bool pseudo_color, const sensor_msgs::msg::Image::SharedPtr msg);
  void on_cloud(int slot, const sensor_msgs::msg::PointCloud2::SharedPtr msg, const QString & title);
  void on_metrics(const std_msgs::msg::String::SharedPtr msg);

  static QImage to_qimage(const sensor_msgs::msg::Image & msg, bool pseudo_color);
  static QImage false_color(const QImage & image);
  static QImage cloud_to_qimage(const sensor_msgs::msg::PointCloud2 & msg, const QString & title);
  static double json_number(const std::string & text, const std::string & key, double fallback);
  static QLabel * make_image_label(const QString & title);
  static QLabel * make_metric_label(const QString & title);

  rclcpp::Node::SharedPtr node_;
  std::array<QLabel *, 9> image_labels_{};
  std::array<QLabel *, 4> metric_labels_{};
  std::vector<ImageSubscription> image_subscriptions_;
  std::vector<CloudSubscription> cloud_subscriptions_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metrics_sub_;
};

}  // namespace usv_rviz_dashboard

#endif  // USV_RVIZ_DASHBOARD__PERCEPTION_DASHBOARD_HPP_
