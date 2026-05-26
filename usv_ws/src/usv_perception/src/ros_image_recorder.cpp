#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace usv_perception
{

class RosImageRecorder : public rclcpp::Node
{
public:
  RosImageRecorder()
  : Node("ros_image_recorder")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/gated_camera/range_view");
    output_dir_ = std::filesystem::path(
      expand_user(declare_parameter<std::string>("output_dir", "~/usv_captures")));
    prefix_ = declare_parameter<std::string>("prefix", "gated");
    every_n_ = std::max(1, static_cast<int>(declare_parameter<int>("every_n", 10)));
    max_images_ = std::max(0, static_cast<int>(declare_parameter<int>("max_images", 0)));
    extension_ = strip_extension_dot(declare_parameter<std::string>("extension", "jpg"));
    crop_x_ = std::max(0, static_cast<int>(declare_parameter<int>("crop_x", 0)));
    crop_y_ = std::max(0, static_cast<int>(declare_parameter<int>("crop_y", 0)));
    crop_width_ = std::max(0, static_cast<int>(declare_parameter<int>("crop_width", 0)));
    crop_height_ = std::max(0, static_cast<int>(declare_parameter<int>("crop_height", 0)));
    resize_width_ = std::max(0, static_cast<int>(declare_parameter<int>("resize_width", 0)));
    resize_height_ = std::max(0, static_cast<int>(declare_parameter<int>("resize_height", 0)));

    std::filesystem::create_directories(output_dir_);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RosImageRecorder::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Recording %s to %s every_n=%d max_images=%d",
      image_topic_.c_str(), output_dir_.string().c_str(), every_n_, max_images_);
  }

private:
  static std::string expand_user(const std::string & path)
  {
    if (path.empty() || path[0] != '~') {
      return path;
    }
    const char * home = std::getenv("HOME");
    if (home == nullptr) {
      return path;
    }
    if (path.size() == 1) {
      return std::string(home);
    }
    if (path[1] == '/') {
      return std::string(home) + path.substr(1);
    }
    return path;
  }

  static std::string strip_extension_dot(std::string extension)
  {
    while (!extension.empty() && extension.front() == '.') {
      extension.erase(extension.begin());
    }
    std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return extension.empty() ? "jpg" : extension;
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    ++frame_count_;
    if (frame_count_ % every_n_ != 0) {
      return;
    }
    if (max_images_ > 0 && saved_count_ >= max_images_) {
      return;
    }

    cv::Mat bgr;
    try {
      bgr = image_msg_to_bgr(*msg);
      bgr = postprocess_image(bgr);
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", exc.what());
      return;
    }

    const std::uint64_t stamp =
      static_cast<std::uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nanosec;
    std::ostringstream filename;
    filename << prefix_ << "_" << std::setw(6) << std::setfill('0') << saved_count_ << "_"
             << stamp << "." << extension_;
    const std::filesystem::path path = output_dir_ / filename.str();

    if (!cv::imwrite(path.string(), bgr)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to write image: %s", path.string().c_str());
      return;
    }
    ++saved_count_;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Saved %s", path.string().c_str());

    if (max_images_ > 0 && saved_count_ >= max_images_) {
      RCLCPP_INFO(get_logger(), "Reached max_images=%d; stopping recorder", max_images_);
      rclcpp::shutdown();
    }
  }

  static cv::Mat image_msg_to_bgr(const sensor_msgs::msg::Image & msg)
  {
    const std::string encoding = lower_copy(msg.encoding);
    if (encoding == "bgr8" || encoding == "rgb8") {
      cv::Mat full(msg.height, msg.step / 3, CV_8UC3, const_cast<unsigned char *>(msg.data.data()));
      cv::Mat cropped = full(cv::Rect(0, 0, msg.width, msg.height)).clone();
      if (encoding == "rgb8") {
        cv::cvtColor(cropped, cropped, cv::COLOR_RGB2BGR);
      }
      return cropped;
    }
    if (encoding == "bgra8" || encoding == "rgba8") {
      cv::Mat full(msg.height, msg.step / 4, CV_8UC4, const_cast<unsigned char *>(msg.data.data()));
      cv::Mat cropped = full(cv::Rect(0, 0, msg.width, msg.height)).clone();
      cv::Mat bgr;
      cv::cvtColor(cropped, bgr, encoding == "bgra8" ? cv::COLOR_BGRA2BGR : cv::COLOR_RGBA2BGR);
      return bgr;
    }
    if (encoding == "mono8" || encoding == "8uc1") {
      cv::Mat full(msg.height, msg.step, CV_8UC1, const_cast<unsigned char *>(msg.data.data()));
      cv::Mat cropped = full(cv::Rect(0, 0, msg.width, msg.height)).clone();
      cv::Mat bgr;
      cv::cvtColor(cropped, bgr, cv::COLOR_GRAY2BGR);
      return bgr;
    }
    if (encoding == "16uc1" || encoding == "mono16") {
      cv::Mat full(msg.height, msg.step / 2, CV_16UC1, const_cast<unsigned char *>(msg.data.data()));
      cv::Mat cropped = full(cv::Rect(0, 0, msg.width, msg.height)).clone();
      double max_value = 1.0;
      cv::minMaxLoc(cropped, nullptr, &max_value);
      cv::Mat gray;
      cropped.convertTo(gray, CV_8UC1, 255.0 / std::max(max_value, 1.0));
      cv::Mat bgr;
      cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
      return bgr;
    }
    if (encoding == "32fc1") {
      cv::Mat full(msg.height, msg.step / 4, CV_32FC1, const_cast<unsigned char *>(msg.data.data()));
      cv::Mat depth = full(cv::Rect(0, 0, msg.width, msg.height)).clone();
      return depth_to_bgr(depth);
    }
    throw std::runtime_error("Unsupported image encoding: " + msg.encoding);
  }

  cv::Mat postprocess_image(const cv::Mat & input) const
  {
    cv::Mat output = input;
    if (crop_width_ > 0 && crop_height_ > 0) {
      const int x = std::clamp(crop_x_, 0, std::max(0, input.cols - 1));
      const int y = std::clamp(crop_y_, 0, std::max(0, input.rows - 1));
      const int width = std::min(crop_width_, input.cols - x);
      const int height = std::min(crop_height_, input.rows - y);
      if (width > 0 && height > 0) {
        output = input(cv::Rect(x, y, width, height)).clone();
      }
    }
    if (resize_width_ > 0 && resize_height_ > 0) {
      cv::Mat resized;
      cv::resize(output, resized, cv::Size(resize_width_, resize_height_), 0.0, 0.0, cv::INTER_AREA);
      output = resized;
    }
    return output;
  }

  static cv::Mat depth_to_bgr(const cv::Mat & depth)
  {
    std::vector<float> positive_values;
    positive_values.reserve(static_cast<std::size_t>(depth.rows * depth.cols));
    for (int row = 0; row < depth.rows; ++row) {
      const float * ptr = depth.ptr<float>(row);
      for (int col = 0; col < depth.cols; ++col) {
        if (std::isfinite(ptr[col]) && ptr[col] > 0.0F) {
          positive_values.push_back(ptr[col]);
        }
      }
    }
    float clip_value = 1.0F;
    if (!positive_values.empty()) {
      const std::size_t rank = static_cast<std::size_t>(0.95 * static_cast<double>(positive_values.size() - 1));
      std::nth_element(positive_values.begin(), positive_values.begin() + rank, positive_values.end());
      clip_value = std::max(positive_values[rank], 1.0F);
    }

    cv::Mat clipped(depth.rows, depth.cols, CV_32FC1, cv::Scalar(0.0F));
    for (int row = 0; row < depth.rows; ++row) {
      const float * in = depth.ptr<float>(row);
      float * out = clipped.ptr<float>(row);
      for (int col = 0; col < depth.cols; ++col) {
        out[col] = std::isfinite(in[col]) ? std::clamp(in[col], 0.0F, clip_value) : 0.0F;
      }
    }

    double min_value = 0.0;
    double max_value = 1.0;
    cv::minMaxLoc(clipped, &min_value, &max_value);
    cv::Mat gray;
    if (max_value - min_value < 1e-6) {
      gray = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC1);
    } else {
      clipped.convertTo(gray, CV_8UC1, 255.0 / (max_value - min_value), -min_value * 255.0 / (max_value - min_value));
    }
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }

  static std::string lower_copy(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return value;
  }

  std::string image_topic_;
  std::filesystem::path output_dir_;
  std::string prefix_{"gated"};
  std::string extension_{"jpg"};
  int every_n_{10};
  int max_images_{0};
  int crop_x_{0};
  int crop_y_{0};
  int crop_width_{0};
  int crop_height_{0};
  int resize_width_{0};
  int resize_height_{0};
  int frame_count_{0};
  int saved_count_{0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::RosImageRecorder>());
  rclcpp::shutdown();
  return 0;
}
