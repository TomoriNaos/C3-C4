//
// Created by hakuroyo on 2026/5/11.
//
#include <depth_image_to_pointcloud2/depth_image_to_pointcloud2_node.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace depth_image_to_pointcloud2
{
namespace
{

cv::Mat convertRgbToFloat01(const cv::Mat &rgb)
{
    if (rgb.empty()) {
        throw std::invalid_argument("输入 RGB 图像为空");
    }
    if (rgb.channels() != 3) {
        throw std::invalid_argument("输入 RGB 图像必须是 3 通道 cv::Mat");
    }

    cv::Mat rgb_float;
    if (rgb.depth() == CV_8U) {
        rgb.convertTo(rgb_float, CV_32FC3, 1.0 / 255.0);
    } else if (rgb.depth() == CV_16U) {
        rgb.convertTo(rgb_float, CV_32FC3, 1.0 / 65535.0);
    } else if (rgb.depth() == CV_32F || rgb.depth() == CV_64F) {
        rgb.convertTo(rgb_float, CV_32FC3);
    } else {
        throw std::invalid_argument("不支持的 RGB 图像深度类型");
    }

    cv::patchNaNs(rgb_float, 0.0);
    cv::min(cv::max(rgb_float, 0.0), 1.0, rgb_float);
    return rgb_float;
}

cv::Mat computeDarkChannel(const cv::Mat &rgb_float, const int radius)
{
    std::vector<cv::Mat> channels;
    cv::split(rgb_float, channels);

    cv::Mat min_channel;
    cv::min(channels[0], channels[1], min_channel);
    cv::min(min_channel, channels[2], min_channel);

    const int kernel_size = std::max(1, radius * 2 + 1);
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));

    cv::Mat dark_channel;
    cv::erode(min_channel, dark_channel, kernel);
    return dark_channel;
}

cv::Vec3f estimateAtmosphericLight(
    const cv::Mat &rgb_float, const cv::Mat &dark_channel, const double top_percent)
{
    const int pixel_count = dark_channel.rows * dark_channel.cols;
    const int sample_count = std::clamp(
        static_cast<int>(std::ceil(pixel_count * top_percent)), 1, pixel_count);

    std::vector<int> indices(pixel_count);
    for (int i = 0; i < pixel_count; ++i) {
        indices[i] = i;
    }

    std::partial_sort(
        indices.begin(),
        indices.begin() + sample_count,
        indices.end(),
        [&dark_channel](const int lhs, const int rhs) {
            return dark_channel.at<float>(lhs / dark_channel.cols, lhs % dark_channel.cols) >
                   dark_channel.at<float>(rhs / dark_channel.cols, rhs % dark_channel.cols);
        });

    double best_intensity = -1.0;
    cv::Vec3f atmospheric_light(1.0F, 1.0F, 1.0F);
    for (int i = 0; i < sample_count; ++i) {
        const int index = indices[i];
        const cv::Vec3f color = rgb_float.at<cv::Vec3f>(index / rgb_float.cols, index % rgb_float.cols);
        const double intensity = color[0] + color[1] + color[2];
        if (intensity > best_intensity) {
            best_intensity = intensity;
            atmospheric_light = color;
        }
    }

    // 防止极暗图像导致除零；这里通常不需要调参。
    for (int channel = 0; channel < 3; ++channel) {
        atmospheric_light[channel] = std::max(atmospheric_light[channel], 1.0e-3F);
    }
    return atmospheric_light;
}

cv::Mat recoverRgb(
    const cv::Mat &rgb_float, const cv::Mat &transmission, const cv::Vec3f &atmospheric_light)
{
    cv::Mat recovered(rgb_float.size(), CV_32FC3);

    for (int row = 0; row < rgb_float.rows; ++row) {
        const cv::Vec3f *rgb_row = rgb_float.ptr<cv::Vec3f>(row);
        const float *transmission_row = transmission.ptr<float>(row);
        cv::Vec3f *recovered_row = recovered.ptr<cv::Vec3f>(row);

        for (int col = 0; col < rgb_float.cols; ++col) {
            const float t = transmission_row[col];
            for (int channel = 0; channel < 3; ++channel) {
                recovered_row[col][channel] =
                    (rgb_row[col][channel] - atmospheric_light[channel]) / t + atmospheric_light[channel];
                recovered_row[col][channel] = std::clamp(recovered_row[col][channel], 0.0F, 1.0F);
            }
        }
    }

    return recovered;
}

cv::Mat recoverDepth(
    const cv::Mat &depth, const cv::Mat &transmission, const DehazeParams &params)
{
    if (depth.empty() || params.depth_compensation_strength <= 0.0) {
        return depth.clone();
    }
    if (depth.size() != transmission.size()) {
        throw std::invalid_argument("Depth 图像尺寸必须与 RGB 图像一致");
    }
    if (depth.channels() != 1) {
        throw std::invalid_argument("Depth 图像必须是单通道 cv::Mat");
    }

    cv::Mat depth_float;
    depth.convertTo(depth_float, CV_32FC1);

    cv::Mat recovered_depth(depth.size(), CV_32FC1);
    const float max_scale = static_cast<float>(std::max(1.0, params.max_depth_scale));
    const float strength = static_cast<float>(std::clamp(params.depth_compensation_strength, 0.0, 1.0));

    for (int row = 0; row < depth.rows; ++row) {
        const float *depth_row = depth_float.ptr<float>(row);
        const float *transmission_row = transmission.ptr<float>(row);
        float *recovered_row = recovered_depth.ptr<float>(row);

        for (int col = 0; col < depth.cols; ++col) {
            if (!std::isfinite(depth_row[col]) || depth_row[col] <= 0.0F) {
                recovered_row[col] = depth_row[col];
                continue;
            }

            const float scale = std::min(max_scale, 1.0F / std::max(transmission_row[col], 1.0e-3F));
            recovered_row[col] = depth_row[col] * (1.0F + strength * (scale - 1.0F));
        }
    }

    cv::Mat output_depth;
    recovered_depth.convertTo(output_depth, depth.type());
    return output_depth;
}

}  // namespace

RgbdImage dehazeRgbdImage(const RgbdImage &input, const DehazeParams &params)
{
    if (input.depth.empty()) {
        throw std::invalid_argument("输入 Depth 图像为空");
    }
    if (input.rgb.size() != input.depth.size()) {
        throw std::invalid_argument("RGB 与 Depth 图像尺寸必须一致");
    }
    if (params.dark_channel_radius < 0) {
        throw std::invalid_argument("dark_channel_radius 不能小于 0");
    }

    const cv::Mat rgb_float = convertRgbToFloat01(input.rgb);
    const cv::Mat dark_channel = computeDarkChannel(rgb_float, params.dark_channel_radius);
    const cv::Vec3f atmospheric_light =
        estimateAtmosphericLight(rgb_float, dark_channel, params.atmospheric_light_percent);

    cv::Mat normalized_by_airlight(rgb_float.size(), CV_32FC3);
    for (int row = 0; row < rgb_float.rows; ++row) {
        const cv::Vec3f *rgb_row = rgb_float.ptr<cv::Vec3f>(row);
        cv::Vec3f *normalized_row = normalized_by_airlight.ptr<cv::Vec3f>(row);
        for (int col = 0; col < rgb_float.cols; ++col) {
            for (int channel = 0; channel < 3; ++channel) {
                normalized_row[col][channel] = rgb_row[col][channel] / atmospheric_light[channel];
            }
        }
    }

    const cv::Mat normalized_dark_channel =
        computeDarkChannel(normalized_by_airlight, params.dark_channel_radius);

    cv::Mat transmission =
        1.0 - std::clamp(params.omega, 0.0, 1.0) * normalized_dark_channel;
    cv::max(transmission, std::clamp(params.min_transmission, 1.0e-3, 1.0), transmission);

    // 可选增强：如果后续发现边缘有光晕，可在这里加入 guided filter 优化透射率。
    cv::Mat recovered_float = recoverRgb(rgb_float, transmission, atmospheric_light);

    cv::Mat output_rgb;
    if (input.rgb.depth() == CV_8U) {
        recovered_float.convertTo(output_rgb, input.rgb.type(), 255.0);
    } else if (input.rgb.depth() == CV_16U) {
        recovered_float.convertTo(output_rgb, input.rgb.type(), 65535.0);
    } else {
        recovered_float.convertTo(output_rgb, input.rgb.type());
    }

    return RgbdImage{output_rgb, recoverDepth(input.depth, transmission, params)};
}

DepthImageToPointCloud2Node::DepthImageToPointCloud2Node()
: Node("depth_image_to_pointcloud2_node")
{
    frame_stride_ = declare_parameter<int>("frame_stride", 4);
    depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
    max_valid_depth_ = declare_parameter<double>("max_valid_depth", 20.0);
    show_images_ = declare_parameter<bool>("show_images", true);

    // 需要调参：这些参数会直接传给 RGB-D 去雾算法，可在 launch.py 中覆盖。
    dehaze_params_.dark_channel_radius = declare_parameter<int>("dark_channel_radius", 7);
    dehaze_params_.omega = declare_parameter<double>("omega", 0.95);
    dehaze_params_.min_transmission = declare_parameter<double>("min_transmission", 0.10);
    dehaze_params_.atmospheric_light_percent =
        declare_parameter<double>("atmospheric_light_percent", 0.001);
    dehaze_params_.depth_compensation_strength =
        declare_parameter<double>("depth_compensation_strength", 0.35);
    dehaze_params_.max_depth_scale = declare_parameter<double>("max_depth_scale", 1.50);

    const auto qos = rclcpp::SensorDataQoS();
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", qos,
        std::bind(&DepthImageToPointCloud2Node::rgbCallback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_raw", qos,
        std::bind(&DepthImageToPointCloud2Node::depthCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", qos,
        std::bind(&DepthImageToPointCloud2Node::cameraInfoCallback, this, std::placeholders::_1));

    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

    if (show_images_ && std::getenv("DISPLAY") == nullptr) {
        RCLCPP_WARN(
            get_logger(),
            "未检测到 DISPLAY，跳过 OpenCV 实时图像窗口；如需显示窗口请为容器配置 GUI 转发");
        show_images_ = false;
    }

    if (show_images_) {
        cv::namedWindow("RGB-D 原始 RGB 图像", cv::WINDOW_NORMAL);
        cv::namedWindow("RGB-D 去雾后 RGB 图像", cv::WINDOW_NORMAL);
    }
}

void DepthImageToPointCloud2Node::rgbCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    ++frame_count_;
    if (frame_stride_ > 1 && frame_count_ % frame_stride_ != 0) {
        return;
    }
    if (!latest_depth_msg_ || !latest_camera_info_msg_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "等待 depth image 和 camera_info 后再生成点云");
        return;
    }

    processFrame(*msg);
}

void DepthImageToPointCloud2Node::depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    latest_depth_msg_ = msg;
}

void DepthImageToPointCloud2Node::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
    latest_camera_info_msg_ = msg;
}

void DepthImageToPointCloud2Node::processFrame(const sensor_msgs::msg::Image &rgb_msg)
{
    try {
        // Gazebo 输出的 RGB 图像通常是 bgr8，这里统一转为 BGR8 便于 OpenCV 显示。
        const cv_bridge::CvImagePtr rgb_cv =
            cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        const cv::Mat depth_meters = convertDepthToMeters(*latest_depth_msg_);

        const RgbdImage dehazed = dehazeRgbdImage(
            RgbdImage{rgb_cv->image, depth_meters}, dehaze_params_);

        if (show_images_) {
            cv::imshow("RGB-D 原始 RGB 图像", rgb_cv->image);
            cv::imshow("RGB-D 去雾后 RGB 图像", dehazed.rgb);
            cv::waitKey(1);
        }

        auto pointcloud = buildPointCloud(
            dehazed.rgb,
            dehazed.depth,
            *latest_camera_info_msg_,
            rgb_msg.header.stamp);
        pointcloud_pub_->publish(pointcloud);
    } catch (const std::exception &error) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "RGB-D 去雾或点云转换失败: %s", error.what());
    }
}

cv::Mat DepthImageToPointCloud2Node::convertDepthToMeters(
    const sensor_msgs::msg::Image &depth_msg) const
{
    cv_bridge::CvImagePtr depth_cv;
    if (depth_msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        depth_cv = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        return depth_cv->image.clone();
    }
    if (depth_msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        depth_msg.encoding == sensor_msgs::image_encodings::MONO16) {
        depth_cv = cv_bridge::toCvCopy(depth_msg);
        cv::Mat depth_meters;
        depth_cv->image.convertTo(depth_meters, CV_32FC1, depth_scale_);
        return depth_meters;
    }

    depth_cv = cv_bridge::toCvCopy(depth_msg);
    cv::Mat depth_meters;
    depth_cv->image.convertTo(depth_meters, CV_32FC1);
    return depth_meters;
}

sensor_msgs::msg::PointCloud2 DepthImageToPointCloud2Node::buildPointCloud(
    const cv::Mat &rgb,
    const cv::Mat &depth_meters,
    const sensor_msgs::msg::CameraInfo &camera_info,
    const rclcpp::Time &stamp) const
{
    if (rgb.empty() || depth_meters.empty()) {
        throw std::invalid_argument("RGB 或 Depth 图像为空，无法生成点云");
    }
    if (rgb.size() != depth_meters.size()) {
        throw std::invalid_argument("RGB 与 Depth 图像尺寸不一致，无法生成点云");
    }
    if (rgb.type() != CV_8UC3) {
        throw std::invalid_argument("点云颜色需要 CV_8UC3 图像");
    }

    const double fx = camera_info.k[0] > 0.0 ? camera_info.k[0] : camera_info.p[0];
    const double fy = camera_info.k[4] > 0.0 ? camera_info.k[4] : camera_info.p[5];
    const double cx = camera_info.k[2] > 0.0 ? camera_info.k[2] : camera_info.p[2];
    const double cy = camera_info.k[5] > 0.0 ? camera_info.k[5] : camera_info.p[6];
    if (fx <= 0.0 || fy <= 0.0) {
        throw std::invalid_argument("CameraInfo 中缺少有效内参 fx/fy");
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = camera_info.header.frame_id.empty() ? "camera_link" : camera_info.header.frame_id;
    cloud.height = static_cast<uint32_t>(depth_meters.rows);
    cloud.width = static_cast<uint32_t>(depth_meters.cols);
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(static_cast<size_t>(cloud.row_step) * cloud.height);

    cloud.fields.resize(4);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;
    cloud.fields[3].name = "rgb";
    cloud.fields[3].offset = 12;
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[3].count = 1;

    const float quiet_nan = std::numeric_limits<float>::quiet_NaN();
    for (int v = 0; v < depth_meters.rows; ++v) {
        const auto *depth_row = depth_meters.ptr<float>(v);
        const auto *rgb_row = rgb.ptr<cv::Vec3b>(v);
        for (int u = 0; u < depth_meters.cols; ++u) {
            float z = depth_row[u];
            float x = quiet_nan;
            float y = quiet_nan;
            if (std::isfinite(z) && z > 0.0F && z <= static_cast<float>(max_valid_depth_)) {
                x = static_cast<float>((static_cast<double>(u) - cx) * z / fx);
                y = static_cast<float>((static_cast<double>(v) - cy) * z / fy);
            } else {
                z = quiet_nan;
            }

            const cv::Vec3b bgr = rgb_row[u];
            const uint32_t rgb_packed =
                (static_cast<uint32_t>(bgr[2]) << 16U) |
                (static_cast<uint32_t>(bgr[1]) << 8U) |
                static_cast<uint32_t>(bgr[0]);
            float rgb_float = 0.0F;
            std::memcpy(&rgb_float, &rgb_packed, sizeof(rgb_float));

            const size_t offset =
                (static_cast<size_t>(v) * cloud.width + static_cast<size_t>(u)) * cloud.point_step;
            std::memcpy(&cloud.data[offset + 0], &x, sizeof(float));
            std::memcpy(&cloud.data[offset + 4], &y, sizeof(float));
            std::memcpy(&cloud.data[offset + 8], &z, sizeof(float));
            std::memcpy(&cloud.data[offset + 12], &rgb_float, sizeof(float));
        }
    }

    return cloud;
}

}  // namespace depth_image_to_pointcloud2
