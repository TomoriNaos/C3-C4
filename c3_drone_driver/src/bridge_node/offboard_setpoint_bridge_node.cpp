#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace c3_drone_driver
{

class OffboardSetpointBridgeNode : public rclcpp::Node
{
public:
  OffboardSetpointBridgeNode()
  : Node("offboard_setpoint_bridge_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/px4/offboard_goal");
    output_topic_ = declare_parameter<std::string>("output_topic", "/px4/setpoint_pose");
    force_frame_id_ = declare_parameter<std::string>("force_frame_id", "ned");

    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);
    setpoint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, 10, std::bind(&OffboardSetpointBridgeNode::onSetpoint, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "offboard_setpoint_bridge_node started: %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void onSetpoint(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
  {
    auto out = *msg;
    if (!force_frame_id_.empty()) {
      out.header.frame_id = force_frame_id_;
    }
    setpoint_pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string force_frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
};

}  // namespace c3_drone_driver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<c3_drone_driver::OffboardSetpointBridgeNode>());
  rclcpp::shutdown();
  return 0;
}

