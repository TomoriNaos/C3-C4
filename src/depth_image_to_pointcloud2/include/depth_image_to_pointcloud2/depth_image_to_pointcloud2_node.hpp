//
// Created by hakuroyo on 2026/5/11.
//

#ifndef ROS2_WS_DEPTH_IMAGE_TO_POINTCLOUD2_NODE_HPP
#define ROS2_WS_DEPTH_IMAGE_TO_POINTCLOUD2_NODE_HPP

#include <opencv2/core.hpp>

namespace depth_image_to_pointcloud2
{

struct RgbdImage
{
    cv::Mat rgb;
    cv::Mat depth;
};

struct DehazeParams
{
    // 需要调参：暗通道窗口半径，雾越浓或图像分辨率越高，可以适当增大。
    int dark_channel_radius = 7;

    // 需要调参：去雾强度，典型范围 0.75 ~ 0.98；越大去雾越强，但可能过饱和。
    double omega = 0.95;

    // 需要调参：透射率下限，越大图像越稳定但远处去雾会变弱。
    double min_transmission = 0.10;

    // 需要调参：用于估计大气光的最亮暗通道像素比例，典型范围 0.001 ~ 0.01。
    double atmospheric_light_percent = 0.001;

    // 需要调参：深度修正强度；0 表示不修正深度，1 表示按透射率完全补偿。
    double depth_compensation_strength = 0.35;

    // 需要调参：深度补偿最大倍数，避免雾很浓时把深度值放大得过多。
    double max_depth_scale = 1.50;
};

RgbdImage dehazeRgbdImage(const RgbdImage &input, const DehazeParams &params = DehazeParams());

}  // namespace depth_image_to_pointcloud2

#endif //ROS2_WS_DEPTH_IMAGE_TO_POINTCLOUD2_NODE_HPP
