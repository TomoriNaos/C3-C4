#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

namespace c3_drone_driver
{

class OffboardSetpointPx4BridgeNode : public rclcpp::Node
{
public:
  OffboardSetpointPx4BridgeNode()
  : Node("offboard_setpoint_px4_bridge_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/px4/offboard_goal");
    offboard_mode_topic_ = declare_parameter<std::string>("offboard_mode_topic", "/fmu/in/offboard_control_mode");
    trajectory_topic_ = declare_parameter<std::string>("trajectory_topic", "/fmu/in/trajectory_setpoint");
    vehicle_cmd_topic_ = declare_parameter<std::string>("vehicle_cmd_topic", "/fmu/in/vehicle_command");
    setpoint_hz_ = declare_parameter<double>("setpoint_hz", 20.0);
    auto_arm_ = declare_parameter<bool>("auto_arm", true);
    auto_enter_offboard_ = declare_parameter<bool>("auto_enter_offboard", true);
    sys_id_ = declare_parameter<int>("vehicle_system_id", 1);
    comp_id_ = declare_parameter<int>("vehicle_component_id", 1);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, 10, std::bind(&OffboardSetpointPx4BridgeNode::onGoal, this, std::placeholders::_1));

    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_mode_topic_, 10);
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(trajectory_topic_, 10);
    vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(vehicle_cmd_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(2.0, setpoint_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&OffboardSetpointPx4BridgeNode::onTick, this));

    RCLCPP_INFO(
      get_logger(), "offboard_setpoint_px4_bridge_node started: %s -> [%s, %s, %s]",
      input_topic_.c_str(), offboard_mode_topic_.c_str(), trajectory_topic_.c_str(),
      vehicle_cmd_topic_.c_str());
  }

private:
  static uint64_t nowUs(rclcpp::Clock &clock)
  {
    return static_cast<uint64_t>(clock.now().nanoseconds() / 1000ULL);
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_goal_ = *msg;
    has_goal_ = true;
  }

  void publishVehicleCommand(uint16_t cmd, float p1 = 0.0F, float p2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand vc{};
    vc.timestamp = nowUs(*get_clock());
    vc.param1 = p1;
    vc.param2 = p2;
    vc.command = cmd;
    vc.target_system = static_cast<uint8_t>(sys_id_);
    vc.target_component = static_cast<uint8_t>(comp_id_);
    vc.source_system = static_cast<uint8_t>(sys_id_);
    vc.source_component = static_cast<uint8_t>(comp_id_);
    vc.from_external = true;
    vehicle_cmd_pub_->publish(vc);
  }

  void onTick()
  {
    if (!has_goal_) {
      return;
    }

    const uint64_t t_us = nowUs(*get_clock());

    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = t_us;
    mode.position = true;
    mode.velocity = false;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    offboard_mode_pub_->publish(mode);

    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = t_us;
    sp.position[0] = static_cast<float>(last_goal_.pose.position.x);
    sp.position[1] = static_cast<float>(last_goal_.pose.position.y);
    sp.position[2] = static_cast<float>(last_goal_.pose.position.z);
    sp.yaw = 0.0F;
    trajectory_pub_->publish(sp);

    if (setpoint_count_ < 20U) {
      ++setpoint_count_;
      return;
    }

    if (!offboard_sent_ && auto_enter_offboard_) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
      offboard_sent_ = true;
    }
    if (!arm_sent_ && auto_arm_) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
      arm_sent_ = true;
    }
  }

  std::string input_topic_;
  std::string offboard_mode_topic_;
  std::string trajectory_topic_;
  std::string vehicle_cmd_topic_;
  double setpoint_hz_{20.0};
  bool auto_arm_{true};
  bool auto_enter_offboard_{true};
  int sys_id_{1};
  int comp_id_{1};

  bool has_goal_{false};
  bool offboard_sent_{false};
  bool arm_sent_{false};
  uint32_t setpoint_count_{0U};
  geometry_msgs::msg::PoseStamped last_goal_{};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace c3_drone_driver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<c3_drone_driver::OffboardSetpointPx4BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
