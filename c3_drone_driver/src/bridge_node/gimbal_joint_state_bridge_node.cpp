#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "c3_drone_driver/msg/gimbal_state.hpp"

namespace c3_drone_driver
{

class GimbalJointStateBridgeNode : public rclcpp::Node
{
public:
  GimbalJointStateBridgeNode()
  : Node("gimbal_joint_state_bridge_node")
  {
    yaw_joint_name_ = declare_parameter<std::string>("yaw_joint_name", "gimbal_yaw_joint");
    pitch_joint_name_ = declare_parameter<std::string>("pitch_joint_name", "gimbal_pitch_joint");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    gimbal_state_topic_ = declare_parameter<std::string>("gimbal_state_topic", "/gimbal/state");

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 20);
    gimbal_state_sub_ = create_subscription<msg::GimbalState>(
      gimbal_state_topic_, 20,
      std::bind(&GimbalJointStateBridgeNode::onGimbalState, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "gimbal_joint_state_bridge_node started, %s -> %s",
      gimbal_state_topic_.c_str(),
      joint_state_topic_.c_str());
  }

private:
  void onGimbalState(const msg::GimbalState::SharedPtr msg)
  {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = msg->header.stamp;
    joint_state.name = {yaw_joint_name_, pitch_joint_name_};
    joint_state.position = {msg->yaw, msg->pitch};
    joint_state.velocity = {msg->yaw_rate, msg->pitch_rate};
    joint_state.effort = {0.0, 0.0};
    joint_state_pub_->publish(joint_state);
  }

  rclcpp::Subscription<msg::GimbalState>::SharedPtr gimbal_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::string yaw_joint_name_;
  std::string pitch_joint_name_;
  std::string joint_state_topic_;
  std::string gimbal_state_topic_;
};

}  // namespace c3_drone_driver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<c3_drone_driver::GimbalJointStateBridgeNode>());
  rclcpp::shutdown();
  return 0;
}

