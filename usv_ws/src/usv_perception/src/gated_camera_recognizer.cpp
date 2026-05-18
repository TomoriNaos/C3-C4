#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

namespace usv_perception
{

struct GateBounds
{
  double near{0.0};
  double far{0.0};
};

struct YoloCandidate
{
  cv::Rect box;
  float score{0.0F};
  int class_index{0};
};

class GatedCameraRecognizer : public rclcpp::Node
{
public:
  GatedCameraRecognizer()
  : Node("gated_camera_recognizer")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/gated_camera/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth_camera/depth/image_raw");
    output_prefix_ = normalize_prefix(declare_parameter<std::string>("output_prefix", "gated_camera"));
    frame_id_ = declare_parameter<std::string>("frame_id", "gated_camera_link");
    yolo_model_path_ = declare_parameter<std::string>("yolo_model_path", "");
    yolo_device_ = declare_parameter<std::string>("yolo_device", "auto");
    yolo_half_precision_ = declare_parameter<bool>("half_precision", true);
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.35);
    min_contour_area_ = declare_parameter<double>("min_contour_area", 450.0);
    process_stride_ = std::max(1, static_cast<int>(declare_parameter<int>("process_stride", 3)));
    yolo_input_width_ = std::max(32, static_cast<int>(declare_parameter<int>("yolo_input_width", 640)));
    yolo_input_height_ = std::max(32, static_cast<int>(declare_parameter<int>("yolo_input_height", 640)));
    yolo_nms_threshold_ = declare_parameter<double>("yolo_nms_threshold", 0.45);
    class_names_ = declare_parameter<std::vector<std::string>>(
      "class_names",
      {"vessel", "fishing_boat", "buoy", "fishnet_buoy", "floating_obstacle", "maritime_obstacle"});
    gates_[0] = parse_gate(declare_parameter<std::vector<double>>("gate_near", {2.0, 18.0}), 2.0, 18.0);
    gates_[1] = parse_gate(declare_parameter<std::vector<double>>("gate_mid", {12.0, 42.0}), 12.0, 42.0);
    gates_[2] = parse_gate(declare_parameter<std::vector<double>>("gate_far", {32.0, 85.0}), 32.0, 85.0);

    load_yolo_model();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GatedCameraRecognizer::on_image, this, std::placeholders::_1));
    if (!depth_topic_.empty()) {
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, rclcpp::SensorDataQoS(),
        std::bind(&GatedCameraRecognizer::on_depth, this, std::placeholders::_1));
    }

    near_pub_ = create_publisher<sensor_msgs::msg::Image>(topic("slice_near"), 10);
    mid_pub_ = create_publisher<sensor_msgs::msg::Image>(topic("slice_mid"), 10);
    far_pub_ = create_publisher<sensor_msgs::msg::Image>(topic("slice_far"), 10);
    range_view_pub_ = create_publisher<sensor_msgs::msg::Image>(topic("range_view"), 10);
    detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(topic("detections"), 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(topic("status"), 10);

    RCLCPP_INFO(
      get_logger(), "Gated recognizer image=%s depth=%s output=%s frame=%s backend=%s",
      image_topic_.c_str(), depth_topic_.empty() ? "none" : depth_topic_.c_str(),
      output_prefix_.empty() ? "." : output_prefix_.c_str(), frame_id_.c_str(), yolo_backend_.c_str());
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

  static GateBounds parse_gate(const std::vector<double> & values, double default_near, double default_far)
  {
    if (values.size() >= 2) {
      return GateBounds{values[0], values[1]};
    }
    return GateBounds{default_near, default_far};
  }

  static std::string lower_copy(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return value;
  }

  void load_yolo_model()
  {
    if (yolo_model_path_.empty()) {
      RCLCPP_INFO(get_logger(), "No YOLO model path configured; using contour fallback recognizer");
      return;
    }

    const auto path = std::filesystem::path(yolo_model_path_);
    const std::string extension = lower_copy(path.extension().string());
    if (extension != ".onnx") {
      RCLCPP_WARN(
        get_logger(),
        "C++ gated recognizer supports ONNX YOLO models. Got '%s'; using contour fallback. "
        "Export with: yolo export model=best.pt format=onnx imgsz=640",
        yolo_model_path_.c_str());
      return;
    }
    if (!std::filesystem::exists(path)) {
      RCLCPP_WARN(get_logger(), "YOLO ONNX model does not exist: %s; using contour fallback", yolo_model_path_.c_str());
      return;
    }

    try {
      yolo_net_ = cv::dnn::readNetFromONNX(yolo_model_path_);
      const std::string device = lower_copy(yolo_device_);
      if (device != "cpu" && device != "none") {
        yolo_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        yolo_net_.setPreferableTarget(
          yolo_half_precision_ ? cv::dnn::DNN_TARGET_CUDA_FP16 : cv::dnn::DNN_TARGET_CUDA);
        yolo_backend_ = yolo_half_precision_ ? "opencv_dnn_cuda_fp16" : "opencv_dnn_cuda";
        yolo_uses_cuda_ = true;
      } else {
        set_yolo_cpu_backend();
      }
      yolo_loaded_ = true;
      RCLCPP_INFO(get_logger(), "Loaded YOLO ONNX model: %s backend=%s", yolo_model_path_.c_str(), yolo_backend_.c_str());
    } catch (const cv::Exception & exc) {
      yolo_loaded_ = false;
      yolo_backend_ = "contour_fallback";
      RCLCPP_WARN(get_logger(), "Could not load YOLO ONNX model '%s': %s", yolo_model_path_.c_str(), exc.what());
      RCLCPP_WARN(get_logger(), "Falling back to contour recognizer");
    }
  }

  void set_yolo_cpu_backend()
  {
    yolo_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    yolo_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    yolo_backend_ = "opencv_dnn_cpu";
    yolo_uses_cuda_ = false;
  }

  void on_depth(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      latest_depth_ = depth_msg_to_mat(*msg);
      has_depth_ = true;
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", exc.what());
    }
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    ++frame_index_;
    if (frame_index_ % process_stride_ != 0) {
      return;
    }

    cv::Mat bgr;
    try {
      bgr = color_msg_to_bgr(*msg);
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", exc.what());
      return;
    }

    cv::Mat depth;
    if (has_depth_) {
      depth = latest_depth_;
      if (!depth.empty() && depth.size() != bgr.size()) {
        cv::resize(depth, depth, bgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
      }
    }

    cv::Mat near_slice;
    cv::Mat mid_slice;
    cv::Mat far_slice;
    cv::Mat range_view;
    build_gated_view(bgr, depth, near_slice, mid_slice, far_slice, range_view);

    near_pub_->publish(bgr_to_msg(near_slice, msg->header));
    mid_pub_->publish(bgr_to_msg(mid_slice, msg->header));
    far_pub_->publish(bgr_to_msg(far_slice, msg->header));
    range_view_pub_->publish(bgr_to_msg(range_view, msg->header));

    auto detections = detect_objects(range_view, msg->header);
    const std::size_t detection_count = detections.detections.size();
    detection_pub_->publish(detections);

    std_msgs::msg::String status;
    std::ostringstream text;
    text << "detections=" << detection_count
         << ", yolo=" << (yolo_loaded_ ? "onnx" : "off")
         << ", backend=" << yolo_backend_;
    status.data = text.str();
    status_pub_->publish(status);
  }

  void build_gated_view(
    const cv::Mat & bgr, const cv::Mat & depth,
    cv::Mat & near_slice, cv::Mat & mid_slice, cv::Mat & far_slice, cv::Mat & range_view) const
  {
    if (depth.empty()) {
      cv::Mat gray;
      cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
      cv::cvtColor(gray, near_slice, cv::COLOR_GRAY2BGR);
      mid_slice = near_slice.clone();
      far_slice = near_slice.clone();
      range_view = bgr.clone();
      return;
    }

    cv::Mat clean_depth = depth.clone();
    for (int row = 0; row < clean_depth.rows; ++row) {
      float * ptr = clean_depth.ptr<float>(row);
      for (int col = 0; col < clean_depth.cols; ++col) {
        if (!std::isfinite(ptr[col])) {
          ptr[col] = 0.0F;
        }
      }
    }

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    gray.convertTo(gray, CV_32FC1);

    cv::Mat near_gray = apply_gate(gray, clean_depth, gates_[0]);
    cv::Mat mid_gray = apply_gate(gray, clean_depth, gates_[1]);
    cv::Mat far_gray = apply_gate(gray, clean_depth, gates_[2]);
    cv::cvtColor(near_gray, near_slice, cv::COLOR_GRAY2BGR);
    cv::cvtColor(mid_gray, mid_slice, cv::COLOR_GRAY2BGR);
    cv::cvtColor(far_gray, far_slice, cv::COLOR_GRAY2BGR);

    std::vector<cv::Mat> channels(3);
    channels[0] = far_gray;
    channels[1] = mid_gray;
    channels[2] = near_gray;
    cv::merge(channels, range_view);
    cv::addWeighted(range_view, 0.85, bgr, 0.15, 0.0, range_view);
  }

  static cv::Mat apply_gate(const cv::Mat & gray_float, const cv::Mat & depth, const GateBounds & gate)
  {
    const float center = static_cast<float>(0.5 * (gate.near + gate.far));
    const float sigma = static_cast<float>(std::max((gate.far - gate.near) / 2.35, 0.1));
    cv::Mat out(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0));
    for (int row = 0; row < depth.rows; ++row) {
      const float * depth_ptr = depth.ptr<float>(row);
      const float * gray_ptr = gray_float.ptr<float>(row);
      unsigned char * out_ptr = out.ptr<unsigned char>(row);
      for (int col = 0; col < depth.cols; ++col) {
        const float d = depth_ptr[col];
        if (d < gate.near || d > gate.far) {
          out_ptr[col] = 0;
          continue;
        }
        const float normalized = (d - center) / sigma;
        const float weight = std::exp(-0.5F * normalized * normalized);
        const float value = std::clamp(gray_ptr[col] * weight * 1.8F, 0.0F, 255.0F);
        out_ptr[col] = static_cast<unsigned char>(value);
      }
    }
    return out;
  }

  vision_msgs::msg::Detection2DArray detect_objects(const cv::Mat & bgr, const std_msgs::msg::Header & header)
  {
    if (yolo_loaded_) {
      return detect_with_yolo(bgr, header);
    }
    return detect_with_contours(bgr, header);
  }

  vision_msgs::msg::Detection2DArray detect_with_yolo(const cv::Mat & bgr, const std_msgs::msg::Header & header)
  {
    vision_msgs::msg::Detection2DArray detections;
    detections.header = header;
    detections.header.frame_id = frame_id_;

    std::vector<cv::Mat> outputs;
    try {
      const cv::Mat blob = cv::dnn::blobFromImage(
        bgr, 1.0 / 255.0, cv::Size(yolo_input_width_, yolo_input_height_), cv::Scalar(), true, false);
      yolo_net_.setInput(blob);
      yolo_net_.forward(outputs, yolo_net_.getUnconnectedOutLayersNames());
    } catch (const cv::Exception & exc) {
      if (yolo_uses_cuda_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "YOLO CUDA inference failed, falling back to OpenCV CPU: %s", exc.what());
        set_yolo_cpu_backend();
        try {
          const cv::Mat blob = cv::dnn::blobFromImage(
            bgr, 1.0 / 255.0, cv::Size(yolo_input_width_, yolo_input_height_), cv::Scalar(), true, false);
          yolo_net_.setInput(blob);
          yolo_net_.forward(outputs, yolo_net_.getUnconnectedOutLayersNames());
        } catch (const cv::Exception & cpu_exc) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "YOLO CPU inference failed: %s", cpu_exc.what());
          return detections;
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "YOLO inference failed: %s", exc.what());
        return detections;
      }
    }

    if (outputs.empty()) {
      return detections;
    }

    const auto candidates = parse_yolo_output(outputs.front(), bgr.cols, bgr.rows);
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    boxes.reserve(candidates.size());
    scores.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      boxes.push_back(candidate.box);
      scores.push_back(candidate.score);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(confidence_threshold_), static_cast<float>(yolo_nms_threshold_), keep);
    for (int index : keep) {
      const auto & candidate = candidates[static_cast<std::size_t>(index)];
      const std::string class_id = candidate.class_index >= 0 &&
          candidate.class_index < static_cast<int>(class_names_.size()) ?
        class_names_[static_cast<std::size_t>(candidate.class_index)] :
        std::to_string(candidate.class_index);
      detections.detections.push_back(make_detection(
        header,
        candidate.box.x,
        candidate.box.y,
        candidate.box.x + candidate.box.width,
        candidate.box.y + candidate.box.height,
        class_id,
        candidate.score));
    }
    return detections;
  }

  std::vector<YoloCandidate> parse_yolo_output(const cv::Mat & output, int image_width, int image_height) const
  {
    cv::Mat proposals;
    if (output.dims == 3) {
      const int rows = output.size[1];
      const int cols = output.size[2];
      cv::Mat matrix(rows, cols, CV_32F, const_cast<float *>(reinterpret_cast<const float *>(output.data)));
      proposals = rows < cols && rows <= 256 ? matrix.t() : matrix;
    } else if (output.dims == 2) {
      proposals = output;
    } else {
      return {};
    }

    if (!proposals.isContinuous()) {
      proposals = proposals.clone();
    }

    const int cols = proposals.cols;
    const bool has_known_no_objectness = cols == 4 + static_cast<int>(class_names_.size()) || cols == 84;
    const bool has_known_objectness = cols == 5 + static_cast<int>(class_names_.size()) || cols == 85;
    const int class_start = has_known_no_objectness ? 4 : (has_known_objectness ? 5 : 4);
    const bool use_objectness = class_start == 5;
    if (cols <= class_start) {
      return {};
    }

    const float x_factor = static_cast<float>(image_width) / static_cast<float>(yolo_input_width_);
    const float y_factor = static_cast<float>(image_height) / static_cast<float>(yolo_input_height_);
    std::vector<YoloCandidate> candidates;
    for (int row = 0; row < proposals.rows; ++row) {
      const float * data = proposals.ptr<float>(row);
      float best_score = 0.0F;
      int best_class = 0;
      for (int col = class_start; col < cols; ++col) {
        if (data[col] > best_score) {
          best_score = data[col];
          best_class = col - class_start;
        }
      }

      const float objectness = use_objectness ? data[4] : 1.0F;
      const float score = objectness * best_score;
      if (score < confidence_threshold_) {
        continue;
      }

      const float cx = data[0] * x_factor;
      const float cy = data[1] * y_factor;
      const float width = data[2] * x_factor;
      const float height = data[3] * y_factor;
      const int left = std::clamp(static_cast<int>(cx - width * 0.5F), 0, std::max(image_width - 1, 0));
      const int top = std::clamp(static_cast<int>(cy - height * 0.5F), 0, std::max(image_height - 1, 0));
      const int right = std::clamp(static_cast<int>(cx + width * 0.5F), 0, image_width);
      const int bottom = std::clamp(static_cast<int>(cy + height * 0.5F), 0, image_height);
      if (right <= left || bottom <= top) {
        continue;
      }
      candidates.push_back(YoloCandidate{cv::Rect(left, top, right - left, bottom - top), score, best_class});
    }
    return candidates;
  }

  vision_msgs::msg::Detection2DArray detect_with_contours(const cv::Mat & bgr, const std_msgs::msg::Header & header) const
  {
    vision_msgs::msg::Detection2DArray detections;
    detections.header = header;
    detections.header.frame_id = frame_id_;

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    cv::Mat mask = (channels[1] > 45) & (channels[2] > 35);
    cv::medianBlur(mask, mask, 5);
    const cv::Mat kernel = cv::Mat::ones(5, 5, CV_8UC1);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 1);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const double image_area = static_cast<double>(bgr.rows * bgr.cols);
    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < min_contour_area_) {
        continue;
      }
      const cv::Rect rect = cv::boundingRect(contour);
      if (rect.width < 10 || rect.height < 10) {
        continue;
      }
      const double score = std::min(0.92, 0.38 + 18.0 * area / std::max(image_area, 1.0));
      detections.detections.push_back(make_detection(
        header, rect.x, rect.y, rect.x + rect.width, rect.y + rect.height,
        classify_fallback_target(rect.width, rect.height, area, image_area), score));
    }
    return detections;
  }

  static std::string classify_fallback_target(int width, int height, double area, double image_area)
  {
    const double aspect = static_cast<double>(width) / std::max(static_cast<double>(height), 1.0);
    const double area_ratio = area / std::max(image_area, 1.0);
    if (aspect > 1.45 && area_ratio > 0.0015) {
      return "vessel";
    }
    if (height > width * 1.15 && area_ratio < 0.02) {
      return "buoy";
    }
    if (area_ratio < 0.006) {
      return "floating_obstacle";
    }
    return "maritime_obstacle";
  }

  vision_msgs::msg::Detection2D make_detection(
    const std_msgs::msg::Header & header,
    double x1, double y1, double x2, double y2,
    const std::string & class_id, double score) const
  {
    vision_msgs::msg::Detection2D detection;
    detection.header = header;
    detection.header.frame_id = frame_id_;
    detection.bbox.center.position.x = 0.5 * (x1 + x2);
    detection.bbox.center.position.y = 0.5 * (y1 + y2);
    detection.bbox.center.theta = 0.0;
    detection.bbox.size_x = std::max(1.0, x2 - x1);
    detection.bbox.size_y = std::max(1.0, y2 - y1);

    vision_msgs::msg::ObjectHypothesisWithPose result;
    result.hypothesis.class_id = class_id;
    result.hypothesis.score = score;
    detection.results.push_back(result);
    return detection;
  }

  static cv::Mat color_msg_to_bgr(const sensor_msgs::msg::Image & msg)
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
    throw std::runtime_error("Unsupported color image encoding: " + msg.encoding);
  }

  static cv::Mat depth_msg_to_mat(const sensor_msgs::msg::Image & msg)
  {
    const std::string encoding = lower_copy(msg.encoding);
    if (encoding == "32fc1") {
      cv::Mat full(msg.height, msg.step / 4, CV_32FC1, const_cast<unsigned char *>(msg.data.data()));
      return full(cv::Rect(0, 0, msg.width, msg.height)).clone();
    }
    if (encoding == "16uc1" || encoding == "mono16") {
      cv::Mat full(msg.height, msg.step / 2, CV_16UC1, const_cast<unsigned char *>(msg.data.data()));
      cv::Mat cropped = full(cv::Rect(0, 0, msg.width, msg.height)).clone();
      cv::Mat depth;
      cropped.convertTo(depth, CV_32FC1, 0.001);
      return depth;
    }
    throw std::runtime_error("Unsupported depth image encoding: " + msg.encoding);
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

  std::string image_topic_;
  std::string depth_topic_;
  std::string output_prefix_;
  std::string frame_id_;
  std::string yolo_model_path_;
  std::string yolo_device_{"auto"};
  bool yolo_half_precision_{true};
  double confidence_threshold_{0.35};
  double min_contour_area_{450.0};
  int process_stride_{3};
  int frame_index_{0};
  int yolo_input_width_{640};
  int yolo_input_height_{640};
  double yolo_nms_threshold_{0.45};
  std::vector<std::string> class_names_;
  GateBounds gates_[3];
  cv::Mat latest_depth_;
  bool has_depth_{false};
  cv::dnn::Net yolo_net_;
  bool yolo_loaded_{false};
  bool yolo_uses_cuda_{false};
  std::string yolo_backend_{"contour_fallback"};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr near_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr far_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr range_view_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::GatedCameraRecognizer>());
  rclcpp::shutdown();
  return 0;
}
