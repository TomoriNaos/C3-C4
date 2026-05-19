#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace c3_drone_driver
{

class Px4PoseBridgeNode : public rclcpp::Node
{
public:
  Px4PoseBridgeNode()
  : Node("px4_pose_bridge_node")
  {
    use_odom_input_ = declare_parameter<bool>("use_odom_input", true);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/pose");
    output_topic_ = declare_parameter<std::string>("output_topic", "/px4/vehicle_pose");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "ned");

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);

    if (use_odom_input_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&Px4PoseBridgeNode::onOdom, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(), "px4_pose_bridge_node uses Odometry input: %s -> %s",
        odom_topic_.c_str(), output_topic_.c_str());
    } else {
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, 10, std::bind(&Px4PoseBridgeNode::onPose, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(), "px4_pose_bridge_node uses PoseStamped input: %s -> %s",
        pose_topic_.c_str(), output_topic_.c_str());
    }
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    geometry_msgs::msg::PoseStamped out;
    out.header = msg->header;
    if (!output_frame_id_.empty()) {
      out.header.frame_id = output_frame_id_;
    }
    out.pose = msg->pose.pose;
    pose_pub_->publish(out);
  }

  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
  {
    auto out = *msg;
    if (!output_frame_id_.empty()) {
      out.header.frame_id = output_frame_id_;
    }
    pose_pub_->publish(out);
  }

  bool use_odom_input_{true};
  std::string odom_topic_;
  std::string pose_topic_;
  std::string output_topic_;
  std::string output_frame_id_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

}  // namespace c3_drone_driver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<c3_drone_driver::Px4PoseBridgeNode>());
  rclcpp::shutdown();
  return 0;
}

