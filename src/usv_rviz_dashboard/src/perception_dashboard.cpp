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
constexpr int kHeatmap = 3;
constexpr int kInspectionClusters = 4;
}  // namespace

PerceptionDashboard::PerceptionDashboard(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(6, 6, 6, 6);
  root->setSpacing(6);

  auto * rows = new QSplitter(Qt::Vertical, this);
  rows->setChildrenCollapsible(false);
  rows->setHandleWidth(7);
  auto * top_row = new QSplitter(Qt::Horizontal, rows);
  auto * bottom_row = new QSplitter(Qt::Horizontal, rows);
  top_row->setChildrenCollapsible(false);
  bottom_row->setChildrenCollapsible(false);
  top_row->setHandleWidth(7);
  bottom_row->setHandleWidth(7);
  const auto refresh_all = [this](int, int) {
      for (int slot = 0; slot < static_cast<int>(image_labels_.size()); ++slot) {
        refresh_image_pixmap(slot);
      }
    };
  connect(rows, &QSplitter::splitterMoved, this, refresh_all);
  connect(top_row, &QSplitter::splitterMoved, this, refresh_all);
  connect(bottom_row, &QSplitter::splitterMoved, this, refresh_all);

  const std::array<QString, 5> titles{
    "船载门控相机（三色伪彩色）", "船载深度相机图像", "飞控载门控相机（三色伪彩色）",
    "多模态融合热力图", "已识别追踪目标（亮绿）与运动预测"};
  for (std::size_t i = 0; i < image_labels_.size(); ++i) {
    auto * row = i < 3U ? top_row : bottom_row;
    row->addWidget(make_image_tile(titles[i], &image_labels_[i]));
    row->setStretchFactor(static_cast<int>(i < 3U ? i : i - 3U), 1);
  }
  rows->setStretchFactor(0, 1);
  rows->setStretchFactor(1, 1);
  root->addWidget(rows, 1);
  setMinimumSize(980, 560);

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
  subscribe_image("/c3/inspection_clusters/image", kInspectionClusters, false);
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
    dashboard_dock->setMinimumHeight(560);
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
  label->setPixmap(QPixmap::fromImage(image).scaled(label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
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

QLabel * PerceptionDashboard::make_image_label(const QString & title)
{
  (void)title;
  auto * label = new QLabel();
  label->setProperty("image_title", title);
  label->setAlignment(Qt::AlignCenter);
  label->setMinimumSize(220, 140);
  label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  label->setStyleSheet("QLabel { background: #0b1720; border: 1px solid #315466; }");
  return label;
}

QWidget * PerceptionDashboard::make_image_tile(const QString & title, QLabel ** image_label)
{
  auto * tile = new QWidget();
  auto * layout = new QVBoxLayout(tile);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(2);

  auto * title_label = new QLabel(title);
  title_label->setAlignment(Qt::AlignCenter);
  title_label->setFixedHeight(22);
  title_label->setStyleSheet(
    "QLabel { background: #102230; color: #c9e7f5; border: 1px solid #315466; "
    "font-weight: bold; }");
  layout->addWidget(title_label, 0);

  *image_label = make_image_label(title);
  layout->addWidget(*image_label, 1);
  return tile;
}

}  // namespace usv_rviz_dashboard

PLUGINLIB_EXPORT_CLASS(usv_rviz_dashboard::PerceptionDashboard, rviz_common::Panel)
