//
// Created by hakuroyo on 2026/5/11.
//

#ifndef ROS2_WS_DEPTH_IMAGE_TO_POINTCLOUD2_NODE_HPP
#define ROS2_WS_DEPTH_IMAGE_TO_POINTCLOUD2_NODE_HPP

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

class DepthImageToPointCloud2Node : public rclcpp::Node
{
public:
    DepthImageToPointCloud2Node();

private:
    void rgbCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void processFrame(const sensor_msgs::msg::Image &rgb_msg);

    cv::Mat convertDepthToMeters(const sensor_msgs::msg::Image &depth_msg) const;
    sensor_msgs::msg::PointCloud2 buildPointCloud(
        const cv::Mat &rgb,
        const cv::Mat &depth_meters,
        const sensor_msgs::msg::CameraInfo &camera_info,
        const rclcpp::Time &stamp) const;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    sensor_msgs::msg::Image::ConstSharedPtr latest_depth_msg_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_msg_;

    DehazeParams dehaze_params_;
    int frame_stride_ = 4;
    int frame_count_ = 0;
    double depth_scale_ = 0.001;
    double max_valid_depth_ = 20.0;
    bool show_images_ = true;
};

}  // namespace depth_image_to_pointcloud2

#endif //ROS2_WS_DEPTH_IMAGE_TO_POINTCLOUD2_NODE_HPP
