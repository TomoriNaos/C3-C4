#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

namespace usv_perception
{

struct SliceGate
{
  double near{0.0};
  double far{0.0};
};

struct SliceDetection
{
  cv::Rect box;
  double score{0.0};
  double range{0.0};
  std::string label;
};

class GatedSliceFusionRecognizer : public rclcpp::Node
{
public:
  GatedSliceFusionRecognizer()
  : Node("gated_slice_fusion_recognizer")
  {
    near_topic_ = declare_parameter<std::string>("near_topic", "/gated_camera/slice_near");
    mid_topic_ = declare_parameter<std::string>("mid_topic", "/gated_camera/slice_mid");
    far_topic_ = declare_parameter<std::string>("far_topic", "/gated_camera/slice_far");
    output_prefix_ = normalize_prefix(declare_parameter<std::string>("output_prefix", "gated_camera/stf_fusion"));
    frame_id_ = declare_parameter<std::string>("frame_id", "gated_camera_link");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    process_stride_ = std::max(1, static_cast<int>(declare_parameter<int>("process_stride", 2)));
    min_contour_area_ = declare_parameter<double>("min_contour_area", 260.0);
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.22);
    camera_x_offset_ = declare_parameter<double>("camera_x_offset", 1.55);
    camera_y_offset_ = declare_parameter<double>("camera_y_offset", 0.0);
    camera_z_offset_ = declare_parameter<double>("camera_z_offset", 1.20);
    camera_horizontal_fov_ = declare_parameter<double>("camera_horizontal_fov", 1.20);
    camera_fx_ = declare_parameter<double>("camera_fx", 0.0);
    camera_fy_ = declare_parameter<double>("camera_fy", 0.0);
    camera_cx_ = declare_parameter<double>("camera_cx", 0.0);
    camera_cy_ = declare_parameter<double>("camera_cy", 0.0);
    camera_calibration_width_ = declare_parameter<double>("camera_calibration_width", 0.0);
    camera_calibration_height_ = declare_parameter<double>("camera_calibration_height", 0.0);
    range_blend_with_size_prior_ =
      std::clamp(declare_parameter<double>("range_blend_with_size_prior", 0.25), 0.0, 0.85);
    default_object_height_ = declare_parameter<double>("default_object_height", 1.6);
    gates_[0] = parse_gate(declare_parameter<std::vector<double>>("gate_near", {2.0, 18.0}), 2.0, 18.0);
    gates_[1] = parse_gate(declare_parameter<std::vector<double>>("gate_mid", {12.0, 42.0}), 12.0, 42.0);
    gates_[2] = parse_gate(declare_parameter<std::vector<double>>("gate_far", {32.0, 85.0}), 32.0, 85.0);

    near_sub_ = create_subscription<sensor_msgs::msg::Image>(
      near_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        near_ = image_to_gray(*msg);
        near_header_ = msg->header;
        has_near_ = true;
      });
    mid_sub_ = create_subscription<sensor_msgs::msg::Image>(
      mid_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        mid_ = image_to_gray(*msg);
        mid_header_ = msg->header;
        has_mid_ = true;
      });
    far_sub_ = create_subscription<sensor_msgs::msg::Image>(
      far_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        far_ = image_to_gray(*msg);
        far_header_ = msg->header;
        has_far_ = true;
        process_if_ready(msg->header);
      });

    fused_pub_ = create_publisher<sensor_msgs::msg::Image>(topic("fused"), 10);
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(topic("annotated"), 10);
    detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(topic("detections"), 10);
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/gated_camera/stf_detection_points", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(topic("status"), 10);

    RCLCPP_INFO(
      get_logger(), "STF-style slice fusion recognizer near=%s mid=%s far=%s points=/gated_camera/stf_detection_points",
      near_topic_.c_str(), mid_topic_.c_str(), far_topic_.c_str());
  }

private:
  static std::string normalize_prefix(std::string prefix)
  {
    prefix.erase(prefix.begin(), std::find_if(prefix.begin(), prefix.end(), [](unsigned char ch) {
      return !std::isspace(ch) && ch != '/';
    }));
    prefix.erase(std::find_if(prefix.rbegin(), prefix.rend(), [](unsigned char ch) {
      return !std::isspace(ch) && ch != '/';
    }).base(), prefix.end());
    return prefix;
  }

  std::string topic(const std::string & leaf) const
  {
    return output_prefix_.empty() ? leaf : output_prefix_ + "/" + leaf;
  }

  static SliceGate parse_gate(const std::vector<double> & values, double default_near, double default_far)
  {
    if (values.size() >= 2) {
      return SliceGate{values[0], values[1]};
    }
    return SliceGate{default_near, default_far};
  }

  static std::string lower_copy(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return value;
  }

  void process_if_ready(const std_msgs::msg::Header & header)
  {
    ++frame_index_;
    if (!has_near_ || !has_mid_ || !has_far_ || frame_index_ % process_stride_ != 0) {
      return;
    }
    if (near_.empty() || mid_.empty() || far_.empty()) {
      return;
    }

    cv::Mat mid_resized = mid_;
    cv::Mat far_resized = far_;
    if (mid_resized.size() != near_.size()) {
      cv::resize(mid_resized, mid_resized, near_.size(), 0.0, 0.0, cv::INTER_LINEAR);
    }
    if (far_resized.size() != near_.size()) {
      cv::resize(far_resized, far_resized, near_.size(), 0.0, 0.0, cv::INTER_LINEAR);
    }

    cv::Mat pseudo = build_pseudo_color(near_, mid_resized, far_resized);
    const auto detections = detect_from_slices(near_, mid_resized, far_resized);
    const auto detection_msg = detections_to_msg(detections, header);
    const auto cloud = detections_to_cloud(detections, near_.cols, near_.rows, header);
    const auto annotated = draw_detections(pseudo, detections, near_.cols, near_.rows);

    fused_pub_->publish(bgr_to_msg(pseudo, header));
    annotated_pub_->publish(bgr_to_msg(annotated, header));
    detection_pub_->publish(detection_msg);
    points_pub_->publish(cloud);
    publish_status(detections);
  }

  static cv::Mat image_to_gray(const sensor_msgs::msg::Image & msg)
  {
    const std::string encoding = lower_copy(msg.encoding);
    if (encoding == "mono8" || encoding == "8uc1") {
      cv::Mat full(msg.height, msg.step, CV_8UC1, const_cast<unsigned char *>(msg.data.data()));
      return full(cv::Rect(0, 0, msg.width, msg.height)).clone();
    }
    if (encoding == "bgr8" || encoding == "rgb8") {
      cv::Mat full(msg.height, msg.step / 3, CV_8UC3, const_cast<unsigned char *>(msg.data.data()));
      cv::Mat bgr = full(cv::Rect(0, 0, msg.width, msg.height)).clone();
      if (encoding == "rgb8") {
        cv::cvtColor(bgr, bgr, cv::COLOR_RGB2BGR);
      }
      cv::Mat gray;
      cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
      return gray;
    }
    throw std::runtime_error("Unsupported slice image encoding: " + msg.encoding);
  }

  static cv::Mat build_pseudo_color(const cv::Mat & near, const cv::Mat & mid, const cv::Mat & far)
  {
    std::vector<cv::Mat> channels(3);
    channels[0] = far;
    channels[1] = mid;
    channels[2] = near;
    cv::Mat pseudo;
    cv::merge(channels, pseudo);
    return pseudo;
  }

  std::vector<SliceDetection> detect_from_slices(const cv::Mat & near, const cv::Mat & mid, const cv::Mat & far) const
  {
    cv::Mat max_response;
    cv::max(near, mid, max_response);
    cv::max(max_response, far, max_response);

    cv::Mat contrast_a;
    cv::Mat contrast_b;
    cv::absdiff(mid, near, contrast_a);
    cv::absdiff(far, mid, contrast_b);
    cv::Mat contrast;
    cv::max(contrast_a, contrast_b, contrast);

    cv::Mat fused;
    cv::addWeighted(max_response, 0.72, contrast, 0.55, 0.0, fused);
    cv::GaussianBlur(fused, fused, cv::Size(5, 5), 0.0);

    cv::Mat mask;
    cv::threshold(fused, mask, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);
    const cv::Mat kernel = cv::Mat::ones(5, 5, CV_8UC1);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 1);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<SliceDetection> detections;
    const double image_area = static_cast<double>(near.rows * near.cols);
    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < min_contour_area_) {
        continue;
      }
      const cv::Rect rect = cv::boundingRect(contour);
      if (rect.width < 8 || rect.height < 8 || rect.y < 2 || rect.y + rect.height > near.rows - 2) {
        continue;
      }

      const auto label = classify(rect, area, image_area);
      const double score = std::clamp(0.22 + 22.0 * area / std::max(image_area, 1.0), 0.0, 0.96);
      if (score < confidence_threshold_) {
        continue;
      }
      const double range = estimate_range(rect, near, mid, far, label, near.cols, near.rows);
      detections.push_back(SliceDetection{rect, score, range, label});
    }
    return detections;
  }

  static std::string classify(const cv::Rect & rect, double area, double image_area)
  {
    const double aspect = static_cast<double>(rect.width) / std::max(1.0, static_cast<double>(rect.height));
    const double area_ratio = area / std::max(image_area, 1.0);
    if (aspect > 1.55 && area_ratio > 0.0008) {
      return "vessel";
    }
    if (rect.height > rect.width * 1.15 && area_ratio < 0.018) {
      return "buoy";
    }
    if (area_ratio < 0.006) {
      return "floating_obstacle";
    }
    return "maritime_obstacle";
  }

  double estimate_range(
    const cv::Rect & rect, const cv::Mat & near, const cv::Mat & mid, const cv::Mat & far,
    const std::string & label, int image_width, int image_height) const
  {
    const double near_energy = mean_intensity(near(rect));
    const double mid_energy = mean_intensity(mid(rect));
    const double far_energy = mean_intensity(far(rect));
    const std::array<double, 3> energies{near_energy, mid_energy, far_energy};

    const double total = std::max(1e-3, near_energy + mid_energy + far_energy);
    double gated_range = 0.0;
    for (std::size_t index = 0; index < energies.size(); ++index) {
      const double center = 0.5 * (gates_[index].near + gates_[index].far);
      gated_range += (energies[index] / total) * center;
    }

    const double height_prior = label == "buoy" ? 1.2 :
      (label == "floating_obstacle" ? 0.6 : default_object_height_);
    const double size_range = height_prior * focal_y(image_width, image_height) /
      std::max(1.0, static_cast<double>(rect.height));
    const double max_gate_far = std::max({gates_[0].far, gates_[1].far, gates_[2].far});
    const double prior = std::clamp(size_range, gates_[0].near, max_gate_far);
    return (1.0 - range_blend_with_size_prior_) * gated_range +
      range_blend_with_size_prior_ * prior;
  }

  static double mean_intensity(const cv::Mat & roi)
  {
    if (roi.empty()) {
      return 0.0;
    }
    return cv::mean(roi)[0];
  }

  double focal_y(int image_width_hint, int image_height_hint) const
  {
    if (camera_fy_ > 0.0) {
      if (camera_calibration_height_ > 0.0 && image_height_hint > 0) {
        return camera_fy_ * static_cast<double>(image_height_hint) / camera_calibration_height_;
      }
      return camera_fy_;
    }
    (void)image_width_hint;
    return static_cast<double>(image_width_hint) /
      (2.0 * std::tan(std::max(camera_horizontal_fov_, 0.01) * 0.5));
  }

  vision_msgs::msg::Detection2DArray detections_to_msg(
    const std::vector<SliceDetection> & detections,
    const std_msgs::msg::Header & header) const
  {
    vision_msgs::msg::Detection2DArray msg;
    msg.header = header;
    msg.header.frame_id = frame_id_;
    for (const auto & det : detections) {
      vision_msgs::msg::Detection2D out;
      out.header = msg.header;
      out.bbox.center.position.x = det.box.x + 0.5 * det.box.width;
      out.bbox.center.position.y = det.box.y + 0.5 * det.box.height;
      out.bbox.size_x = det.box.width;
      out.bbox.size_y = det.box.height;
      vision_msgs::msg::ObjectHypothesisWithPose result;
      result.hypothesis.class_id = det.label;
      result.hypothesis.score = det.score;
      out.results.push_back(result);
      msg.detections.push_back(out);
    }
    return msg;
  }

  sensor_msgs::msg::PointCloud2 detections_to_cloud(
    const std::vector<SliceDetection> & detections,
    int image_width,
    int image_height,
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
    set_cloud_field(cloud.fields[0], "x", 0);
    set_cloud_field(cloud.fields[1], "y", 4);
    set_cloud_field(cloud.fields[2], "z", 8);
    set_cloud_field(cloud.fields[3], "intensity", 12);
    set_cloud_field(cloud.fields[4], "class_id", 16);
    set_cloud_field(cloud.fields[5], "bbox_cx", 20);
    set_cloud_field(cloud.fields[6], "bbox_cy", 24);
    set_cloud_field(cloud.fields[7], "bbox_w", 28);
    set_cloud_field(cloud.fields[8], "bbox_h", 32);
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));

    const double fx = focal_x(image_width, image_height);
    const double cx = principal_x(image_width);
    for (std::size_t i = 0; i < detections.size(); ++i) {
      const auto & det = detections[i];
      const double u = det.box.x + 0.5 * det.box.width;
      const double lateral = (u - cx) * det.range / std::max(1.0, fx);
      const float x = static_cast<float>(camera_x_offset_ + det.range);
      const float y = static_cast<float>(camera_y_offset_ - lateral);
      const float z = static_cast<float>(camera_z_offset_);
      const float score = static_cast<float>(det.score);
      const float class_id = static_cast<float>(class_id_for_label(det.label));
      const float bbox_cx = static_cast<float>(u);
      const float bbox_cy = static_cast<float>(det.box.y + 0.5 * det.box.height);
      const float bbox_w = static_cast<float>(det.box.width);
      const float bbox_h = static_cast<float>(det.box.height);
      unsigned char * dst = cloud.data.data() + i * cloud.point_step;
      std::memcpy(dst + 0, &x, sizeof(float));
      std::memcpy(dst + 4, &y, sizeof(float));
      std::memcpy(dst + 8, &z, sizeof(float));
      std::memcpy(dst + 12, &score, sizeof(float));
      std::memcpy(dst + 16, &class_id, sizeof(float));
      std::memcpy(dst + 20, &bbox_cx, sizeof(float));
      std::memcpy(dst + 24, &bbox_cy, sizeof(float));
      std::memcpy(dst + 28, &bbox_w, sizeof(float));
      std::memcpy(dst + 32, &bbox_h, sizeof(float));
    }
    return cloud;
  }

  double focal_x(int image_width, int image_height) const
  {
    if (camera_fx_ > 0.0) {
      if (camera_calibration_width_ > 0.0) {
        return camera_fx_ * static_cast<double>(image_width) / camera_calibration_width_;
      }
      return camera_fx_;
    }
    (void)image_height;
    return static_cast<double>(image_width) /
      (2.0 * std::tan(std::max(camera_horizontal_fov_, 0.01) * 0.5));
  }

  double principal_x(int image_width) const
  {
    if (camera_cx_ > 0.0) {
      if (camera_calibration_width_ > 0.0) {
        return camera_cx_ * static_cast<double>(image_width) / camera_calibration_width_;
      }
      return camera_cx_;
    }
    return 0.5 * static_cast<double>(image_width - 1);
  }

  int class_id_for_label(const std::string & label) const
  {
    if (label == "vessel") {
      return 0;
    }
    if (label == "buoy") {
      return 2;
    }
    if (label == "floating_obstacle") {
      return 3;
    }
    return 5;
  }

  cv::Mat draw_detections(
    const cv::Mat & image,
    const std::vector<SliceDetection> & detections,
    int image_width,
    int image_height) const
  {
    (void)image_width;
    (void)image_height;
    cv::Mat out = image.clone();
    for (const auto & det : detections) {
      const cv::Scalar color = det.label == "buoy" ? cv::Scalar(0, 220, 255) : cv::Scalar(80, 255, 80);
      cv::rectangle(out, det.box, color, 2);
      std::ostringstream text;
      text << det.label << " " << std::fixed << std::setprecision(1) << det.range << "m";
      cv::putText(
        out, text.str(), cv::Point(det.box.x, std::max(18, det.box.y - 6)),
        cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv::LINE_AA);
    }
    return out;
  }

  sensor_msgs::msg::Image bgr_to_msg(const cv::Mat & bgr, const std_msgs::msg::Header & header) const
  {
    cv::Mat contiguous = bgr.isContinuous() ? bgr : bgr.clone();
    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<std::uint32_t>(contiguous.rows);
    msg.width = static_cast<std::uint32_t>(contiguous.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = 0;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(contiguous.cols * 3);
    msg.data.assign(contiguous.datastart, contiguous.dataend);
    return msg;
  }

  static void set_cloud_field(sensor_msgs::msg::PointField & field, const std::string & name, std::uint32_t offset)
  {
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
  }

  void publish_status(const std::vector<SliceDetection> & detections)
  {
    std_msgs::msg::String msg;
    std::ostringstream text;
    text << "stf_slice_fusion detections=" << detections.size();
    for (const auto & det : detections) {
      text << " | " << det.label << " range=" << std::fixed << std::setprecision(1) << det.range
           << " score=" << std::setprecision(2) << det.score;
    }
    msg.data = text.str();
    status_pub_->publish(msg);
  }

  std::string near_topic_;
  std::string mid_topic_;
  std::string far_topic_;
  std::string output_prefix_;
  std::string frame_id_;
  std::string base_frame_;
  int process_stride_{2};
  int frame_index_{0};
  double min_contour_area_{260.0};
  double confidence_threshold_{0.22};
  double camera_x_offset_{1.55};
  double camera_y_offset_{0.0};
  double camera_z_offset_{1.20};
  double camera_horizontal_fov_{1.20};
  double camera_fx_{0.0};
  double camera_fy_{0.0};
  double camera_cx_{0.0};
  double camera_cy_{0.0};
  double camera_calibration_width_{0.0};
  double camera_calibration_height_{0.0};
  double range_blend_with_size_prior_{0.25};
  double default_object_height_{1.6};
  SliceGate gates_[3];
  cv::Mat near_;
  cv::Mat mid_;
  cv::Mat far_;
  std_msgs::msg::Header near_header_;
  std_msgs::msg::Header mid_header_;
  std_msgs::msg::Header far_header_;
  bool has_near_{false};
  bool has_mid_{false};
  bool has_far_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr near_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr far_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fused_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::GatedSliceFusionRecognizer>());
  rclcpp::shutdown();
  return 0;
}
