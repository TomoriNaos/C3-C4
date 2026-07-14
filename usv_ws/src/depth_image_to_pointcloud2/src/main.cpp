//
// Created by hakuroyo on 2026/5/11.
//

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <depth_image_to_pointcloud2/depth_image_to_pointcloud2_node.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<depth_image_to_pointcloud2::DepthImageToPointCloud2Node>());
    rclcpp::shutdown();
    return 0;
}
