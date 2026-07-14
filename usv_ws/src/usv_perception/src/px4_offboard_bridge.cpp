#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace usv_perception
{

constexpr double kHalfPi = 1.57079632679489661923;

class Px4OffboardBridge : public rclcpp::Node
{
public:
  Px4OffboardBridge()
  : Node("px4_offboard_bridge")
  {
    input_goal_topic_ = declare_parameter<std::string>("input_goal_topic", "/uav/offboard_goal");
    vehicle_pose_topic_ = declare_parameter<std::string>("vehicle_pose_topic", "/px4/vehicle_pose");
    setpoint_hz_ = declare_parameter<double>("setpoint_hz", 20.0);
    prestream_setpoints_ = declare_parameter<int>("prestream_setpoints", 20);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    auto_enter_offboard_ = declare_parameter<bool>("auto_enter_offboard", false);
    vehicle_system_id_ = declare_parameter<int>("vehicle_system_id", 1);
    vehicle_component_id_ = declare_parameter<int>("vehicle_component_id", 1);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_goal_topic_, 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (msg) {
          last_goal_ = *msg;
          has_goal_ = true;
        }
      });
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
      std::bind(&Px4OffboardBridge::on_local_position, this, std::placeholders::_1));
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) { vehicle_status_ = *msg; });

    vehicle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(vehicle_pose_topic_, 10);
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(2.0, setpoint_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Px4OffboardBridge::on_timer, this));
  }

private:
  static uint64_t now_us(rclcpp::Clock & clock)
  {
    return static_cast<uint64_t>(clock.now().nanoseconds() / 1000ULL);
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg || !msg->xy_valid || !msg->z_valid) {
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "world";
    // PX4 local position is NED; the mission planner is expressed in ROS ENU.
    pose.pose.position.x = msg->y;
    pose.pose.position.y = msg->x;
    pose.pose.position.z = -msg->z;
    const double yaw_enu = kHalfPi - msg->heading;
    pose.pose.orientation.z = std::sin(yaw_enu * 0.5);
    pose.pose.orientation.w = std::cos(yaw_enu * 0.5);
    vehicle_pose_pub_->publish(pose);
  }

  void publish_vehicle_command(uint16_t command, float param1, float param2)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = now_us(*get_clock());
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = static_cast<uint8_t>(vehicle_system_id_);
    msg.target_component = static_cast<uint8_t>(vehicle_component_id_);
    msg.source_system = static_cast<uint8_t>(vehicle_system_id_);
    msg.source_component = static_cast<uint8_t>(vehicle_component_id_);
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void on_timer()
  {
    if (!has_goal_) {
      return;
    }
    const uint64_t timestamp = now_us(*get_clock());
    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = timestamp;
    mode.position = true;
    offboard_mode_pub_->publish(mode);

    const auto & enu = last_goal_.pose.position;
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = timestamp;
    setpoint.position[0] = static_cast<float>(enu.y);
    setpoint.position[1] = static_cast<float>(enu.x);
    setpoint.position[2] = static_cast<float>(-enu.z);
    setpoint.velocity.fill(std::numeric_limits<float>::quiet_NaN());
    setpoint.acceleration.fill(std::numeric_limits<float>::quiet_NaN());
    setpoint.jerk.fill(std::numeric_limits<float>::quiet_NaN());
    setpoint.yaw = static_cast<float>(kHalfPi - yaw_from_quaternion(last_goal_.pose.orientation));
    setpoint.yawspeed = std::numeric_limits<float>::quiet_NaN();
    trajectory_pub_->publish(setpoint);

    if (setpoint_count_++ < static_cast<uint32_t>(std::max(1, prestream_setpoints_))) {
      return;
    }
    if (auto_enter_offboard_ && !offboard_command_sent_) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
      offboard_command_sent_ = true;
    }
    if (auto_arm_ && !arm_command_sent_) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F, 0.0F);
      arm_command_sent_ = true;
    }
  }

  std::string input_goal_topic_;
  std::string vehicle_pose_topic_;
  double setpoint_hz_{20.0};
  int prestream_setpoints_{20};
  bool auto_arm_{false};
  bool auto_enter_offboard_{false};
  int vehicle_system_id_{1};
  int vehicle_component_id_{1};
  bool has_goal_{false};
  bool offboard_command_sent_{false};
  bool arm_command_sent_{false};
  uint32_t setpoint_count_{0};
  geometry_msgs::msg::PoseStamped last_goal_;
  px4_msgs::msg::VehicleStatus vehicle_status_{};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::Px4OffboardBridge>());
  rclcpp::shutdown();
  return 0;
}
