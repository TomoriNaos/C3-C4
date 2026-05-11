//
// Created by hakuroyo on 2026/5/11.
//
#include <depth_image_to_pointcloud2/depth_image_to_pointcloud2_node.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

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

}  // namespace depth_image_to_pointcloud2
