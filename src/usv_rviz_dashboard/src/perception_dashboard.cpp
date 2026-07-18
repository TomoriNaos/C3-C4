#include "usv_rviz_dashboard/perception_dashboard.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include <QDockWidget>
#include <QMainWindow>
#include <QPainter>
#include <QPixmap>
#include <QSizePolicy>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace usv_rviz_dashboard
{

namespace
{
constexpr int kShipGated = 0;
constexpr int kShipDepth = 1;
constexpr int kUavGated = 2;
constexpr int kRadar = 3;
constexpr int kSonar = 4;
constexpr int kHeatmap = 5;
constexpr int kShipGatedBev = 6;
constexpr int kUavGatedBev = 7;
constexpr int kDepthBev = 8;
constexpr int kClusterOverlay = 9;
constexpr int kInspectionClusters = 10;

int field_offset(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}
}  // namespace

PerceptionDashboard::PerceptionDashboard(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(6, 6, 6, 6);
  root->setSpacing(6);

  auto * image_rows = new QSplitter(Qt::Vertical);
  image_rows->setChildrenCollapsible(false);
  image_rows->setHandleWidth(7);
  std::array<QSplitter *, 3> rows{};
  for (auto & row : rows) {
    row = new QSplitter(Qt::Horizontal, image_rows);
    row->setChildrenCollapsible(false);
    row->setHandleWidth(7);
    connect(row, &QSplitter::splitterMoved, this, [this](int, int) {
      for (int slot = 0; slot < static_cast<int>(image_labels_.size()); ++slot) {
        refresh_image_pixmap(slot);
      }
    });
  }
  connect(image_rows, &QSplitter::splitterMoved, this, [this](int, int) {
    for (int slot = 0; slot < static_cast<int>(image_labels_.size()); ++slot) {
      refresh_image_pixmap(slot);
    }
  });
  const std::array<QString, 11> titles{
    "船载门控相机（三色伪彩色）", "船载深度相机图像", "飞控载门控相机（三色伪彩色）",
    "船载毫米波雷达 BEV", "船载声呐 BEV", "多模态融合热力图",
    "船载门控相机 BEV 点云", "飞控载门控相机 BEV 点云", "船载深度相机 BEV 点云",
    "多模态融合点云点簇与真实物体（红框）", "已识别追踪目标（亮绿）与运动预测"};
  for (std::size_t i = 0; i < image_labels_.size(); ++i) {
    image_labels_[i] = make_image_label(titles[i]);
    rows[i / 4]->addWidget(image_labels_[i]);
  }
  root->addWidget(image_rows, 1);
  setMinimumSize(1100, 620);

  connect(this, &PerceptionDashboard::image_ready, this, &PerceptionDashboard::set_image, Qt::QueuedConnection);
}

void PerceptionDashboard::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    return;
  }
  node_ = abstraction->get_raw_node();
  subscribe_image("/gated_camera/range_view", kShipGated, false);
  subscribe_image("/depth_camera/image_raw", kShipDepth, false);
  subscribe_image("/uav/gated_camera/range_view", kUavGated, false);
  subscribe_image("/c3/heatmap/image", kHeatmap, false);
  subscribe_image("/c3/heatmap/cluster_overlay", kClusterOverlay, false);
  subscribe_image("/c3/inspection_clusters/image", kInspectionClusters, false);
  cloud_subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/c3/buffer/radar_cloud", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_cloud(kRadar, msg, "船载毫米波雷达 BEV");}));
  cloud_subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/c3/buffer/sonar_cloud", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_cloud(kSonar, msg, "船载声呐 BEV");}));
  cloud_subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/gated_camera/detection_points", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_cloud(kShipGatedBev, msg, "船载门控相机 BEV 点云");}));
  cloud_subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/uav/gated_camera/detection_points", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_cloud(kUavGatedBev, msg, "飞控载门控相机 BEV 点云");}));
  cloud_subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/depth_camera/detection_points", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_cloud(kDepthBev, msg, "船载深度相机 BEV 点云");}));
  // Make the dashboard visible at the top without hiding any RViz panels.
  QTimer::singleShot(0, this, [this]() {
    QDockWidget * dashboard_dock = nullptr;
    for (QWidget * parent = parentWidget(); parent != nullptr; parent = parent->parentWidget()) {
      dashboard_dock = qobject_cast<QDockWidget *>(parent);
      if (dashboard_dock) {
        break;
      }
    }
    auto * main_window = qobject_cast<QMainWindow *>(window());
    if (!dashboard_dock || !main_window) {
      return;
    }
    dashboard_dock->setAllowedAreas(Qt::TopDockWidgetArea | Qt::BottomDockWidgetArea);
    main_window->addDockWidget(Qt::TopDockWidgetArea, dashboard_dock);
    dashboard_dock->setMinimumHeight(620);
    dashboard_dock->show();
  });
}

void PerceptionDashboard::subscribe_image(const std::string & topic, int slot, bool pseudo_color)
{
  image_subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::SensorDataQoS(),
      [this, slot, pseudo_color](sensor_msgs::msg::Image::SharedPtr msg) {on_image(slot, pseudo_color, msg);}));
}

void PerceptionDashboard::on_image(int slot, bool pseudo_color, const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg && should_render(slot, std::chrono::milliseconds(200))) {
    emit image_ready(slot, to_qimage(*msg, pseudo_color));
  }
}

void PerceptionDashboard::on_cloud(
  int slot, const sensor_msgs::msg::PointCloud2::SharedPtr msg, const QString & title)
{
  if (msg && should_render(slot, std::chrono::milliseconds(250))) {
    emit image_ready(slot, cloud_to_qimage(*msg, title));
  }
}

bool PerceptionDashboard::should_render(int slot, std::chrono::milliseconds minimum_period)
{
  if (slot < 0 || slot >= static_cast<int>(last_render_times_.size())) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(render_mutex_);
  auto & last = last_render_times_[static_cast<std::size_t>(slot)];
  if (last.time_since_epoch().count() != 0 && now - last < minimum_period) {
    return false;
  }
  last = now;
  return true;
}

void PerceptionDashboard::set_image(int slot, const QImage & image)
{
  if (slot < 0 || slot >= static_cast<int>(image_labels_.size()) || image.isNull()) {
    return;
  }
  latest_images_[static_cast<std::size_t>(slot)] = image;
  refresh_image_pixmap(slot);
}

void PerceptionDashboard::refresh_image_pixmap(int slot)
{
  if (slot < 0 || slot >= static_cast<int>(image_labels_.size())) {
    return;
  }
  const auto & image = latest_images_[static_cast<std::size_t>(slot)];
  if (image.isNull()) {
    return;
  }
  auto * label = image_labels_[static_cast<std::size_t>(slot)];
  QImage titled = image.convertToFormat(QImage::Format_RGB32);
  QPainter painter(&titled);
  painter.setPen(QColor(245, 245, 245));
  painter.setBrush(QColor(0, 0, 0, 155));
  painter.drawRect(0, 0, titled.width(), 26);
  painter.drawText(8, 19, label->property("image_title").toString());
  label->setPixmap(QPixmap::fromImage(titled).scaled(label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

QImage PerceptionDashboard::to_qimage(const sensor_msgs::msg::Image & msg, bool pseudo_color)
{
  if (msg.width == 0 || msg.height == 0 || msg.data.empty()) {
    return {};
  }
  const int width = static_cast<int>(msg.width);
  const int height = static_cast<int>(msg.height);
  if (msg.encoding == "mono8") {
    const QImage image(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_Grayscale8);
    return pseudo_color ? false_color(image) : image.copy();
  }
  if (msg.encoding == "rgb8" || msg.encoding == "bgr8") {
    QImage image(width, height, QImage::Format_RGB888);
    for (int y = 0; y < height; ++y) {
      const auto * source = msg.data.data() + static_cast<std::size_t>(y) * msg.step;
      auto * target = image.scanLine(y);
      for (int x = 0; x < width; ++x) {
        const auto * pixel = source + x * 3;
        target[x * 3 + 0] = msg.encoding == "rgb8" ? pixel[0] : pixel[2];
        target[x * 3 + 1] = pixel[1];
        target[x * 3 + 2] = msg.encoding == "rgb8" ? pixel[2] : pixel[0];
      }
    }
    return pseudo_color ? false_color(image) : image;
  }

  QImage image(width, height, QImage::Format_Grayscale8);
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  const bool float_depth = msg.encoding == "32FC1";
  const bool uint16_depth = msg.encoding == "16UC1";
  if (!float_depth && !uint16_depth) {
    return image;
  }
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const auto * source = msg.data.data() + static_cast<std::size_t>(y) * msg.step + x * (float_depth ? 4 : 2);
      double value = 0.0;
      if (float_depth) {
        float raw = 0.0F;
        std::memcpy(&raw, source, sizeof(float));
        value = raw;
      } else {
        std::uint16_t raw = 0;
        std::memcpy(&raw, source, sizeof(std::uint16_t));
        value = raw * 0.001;
      }
      if (std::isfinite(value) && value > 0.0) {
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
      }
    }
  }
  const double span = std::max(1e-4, maximum - minimum);
  for (int y = 0; y < height; ++y) {
    auto * target = image.scanLine(y);
    for (int x = 0; x < width; ++x) {
      const auto * source = msg.data.data() + static_cast<std::size_t>(y) * msg.step + x * (float_depth ? 4 : 2);
      double value = 0.0;
      if (float_depth) {
        float raw = 0.0F;
        std::memcpy(&raw, source, sizeof(float));
        value = raw;
      } else {
        std::uint16_t raw = 0;
        std::memcpy(&raw, source, sizeof(std::uint16_t));
        value = raw * 0.001;
      }
      target[x] = static_cast<unsigned char>(std::isfinite(value) && value > 0.0 ? 255.0 * (1.0 - (value - minimum) / span) : 0.0);
    }
  }
  return pseudo_color ? false_color(image) : image;
}

QImage PerceptionDashboard::false_color(const QImage & image)
{
  if (image.isNull()) {
    return {};
  }
  QImage result(image.width(), image.height(), QImage::Format_RGB32);
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      const int value = qGray(image.pixel(x, y));
      const int red = value > 127 ? 2 * (value - 127) : 0;
      const int green = value <= 127 ? 2 * value : 2 * (255 - value);
      const int blue = value < 127 ? 255 - 2 * value : 0;
      result.setPixel(x, y, qRgb(red, green, blue));
    }
  }
  return result;
}

QImage PerceptionDashboard::cloud_to_qimage(const sensor_msgs::msg::PointCloud2 & msg, const QString & title)
{
  constexpr int width = 420;
  constexpr int height = 250;
  constexpr double max_range = 150.0;
  QImage image(width, height, QImage::Format_RGB32);
  image.fill(QColor(10, 20, 30));
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QColor(70, 110, 135));
  const QPoint origin(width / 2, height / 2);
  const double pixel_radius = 0.48 * std::min(width, height);
  for (int radius = 30; radius <= 150; radius += 30) {
    const int pixels = static_cast<int>(radius / max_range * pixel_radius);
    painter.drawEllipse(origin, pixels, pixels);
  }
  painter.drawLine(origin.x(), 0, origin.x(), height);
  painter.drawLine(0, origin.y(), width, origin.y());
  painter.setPen(QColor(220, 235, 245));
  painter.drawText(8, 18, title);
  painter.setPen(QColor(100, 160, 185));
  painter.drawText(8, height - 5, "360 deg / range 150 m");

  const int x_offset = field_offset(msg, "x");
  const int y_offset = field_offset(msg, "y");
  if (x_offset < 0 || y_offset < 0 || msg.point_step == 0) {
    return image;
  }
  const std::size_t point_count = std::min<std::size_t>(msg.width * msg.height, msg.data.size() / msg.point_step);
  painter.setPen(QPen(QColor(255, 205, 65), 4));
  for (std::size_t index = 0; index < point_count; ++index) {
    const auto * point = msg.data.data() + index * msg.point_step;
    float x = 0.0F;
    float y = 0.0F;
    std::memcpy(&x, point + x_offset, sizeof(float));
    std::memcpy(&y, point + y_offset, sizeof(float));
    if (!std::isfinite(x) || !std::isfinite(y) || std::hypot(x, y) > max_range) {
      continue;
    }
    const int px = origin.x() - static_cast<int>(y / max_range * pixel_radius);
    const int py = origin.y() - static_cast<int>(x / max_range * pixel_radius);
    painter.drawPoint(px, py);
  }
  return image;
}

QLabel * PerceptionDashboard::make_image_label(const QString & title)
{
  auto * label = new QLabel(title + "\nwaiting for topic...");
  label->setProperty("image_title", title);
  label->setAlignment(Qt::AlignCenter);
  label->setMinimumSize(220, 140);
  label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  label->setStyleSheet("QLabel { background: #0b1720; color: #c9e7f5; border: 1px solid #315466; font-weight: bold; }");
  return label;
}

}  // namespace usv_rviz_dashboard

PLUGINLIB_EXPORT_CLASS(usv_rviz_dashboard::PerceptionDashboard, rviz_common::Panel)
