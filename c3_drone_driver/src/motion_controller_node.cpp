#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "c3_drone_driver/msg/mission_command.hpp"

namespace c3_drone_driver
{

class MotionControllerNode : public rclcpp::Node
{
public:
  MotionControllerNode()
  : Node("motion_controller_node")
  {
    control_hz_ = declare_parameter<double>("control_hz", 50.0);
    cruise_speed_mps_ = declare_parameter<double>("cruise_speed_mps", 3.0);
    max_accel_mps2_ = declare_parameter<double>("max_accel_mps2", 1.5);
    arrive_radius_m_ = declare_parameter<double>("arrive_radius_m", 2.0);
    hover_altitude_m_ = declare_parameter<double>("hover_altitude_m", 20.0);
    home_x_ = declare_parameter<double>("home_x", 0.0);
    home_y_ = declare_parameter<double>("home_y", 0.0);
    home_z_ = declare_parameter<double>("home_z", 0.0);

    current_position_ = {home_x_, home_y_, home_z_};
    current_velocity_ = {0.0, 0.0, 0.0};
    target_position_ = current_position_;
    mode_ = Mode::HOLD;

    mission_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mission/goal", 10, std::bind(&MotionControllerNode::onMissionGoal, this, std::placeholders::_1));
    mission_cmd_sub_ = create_subscription<msg::MissionCommand>(
      "/mission/cmd", 10, std::bind(&MotionControllerNode::onMissionCommand, this, std::placeholders::_1));

    vehicle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/px4/vehicle_pose", 10);
    offboard_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/px4/offboard_goal", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(control_hz_, 5.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MotionControllerNode::onTick, this));
    last_tick_time_ = now();

    RCLCPP_INFO(get_logger(), "motion_controller_node started");
  }

private:
  enum class Mode : uint8_t
  {
    HOLD = 0,
    TRANSIT = 1,
    RETURN = 2,
    ABORT = 3
  };

  static std::array<double, 3> add(
    const std::array<double, 3> &a, const std::array<double, 3> &b)
  {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
  }

  static std::array<double, 3> sub(
    const std::array<double, 3> &a, const std::array<double, 3> &b)
  {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
  }

  static std::array<double, 3> scale(const std::array<double, 3> &v, double s)
  {
    return {v[0] * s, v[1] * s, v[2] * s};
  }

  static double norm(const std::array<double, 3> &v)
  {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  }

  void onMissionGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    target_position_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    if (std::abs(target_position_[2]) < 1e-6) {
      target_position_[2] = hover_altitude_m_;
    }
    mode_ = Mode::TRANSIT;
  }

  void onMissionCommand(const msg::MissionCommand::SharedPtr msg)
  {
    if (msg->command == msg::MissionCommand::CMD_BACK) {
      target_position_ = {home_x_, home_y_, home_z_};
      mode_ = Mode::RETURN;
      return;
    }
    if (msg->command == msg::MissionCommand::CMD_CLOSE) {
      target_position_ = current_position_;
      current_velocity_ = {0.0, 0.0, 0.0};
      mode_ = Mode::ABORT;
      return;
    }
    if (msg->command == msg::MissionCommand::CMD_HOLD) {
      target_position_ = current_position_;
      current_velocity_ = {0.0, 0.0, 0.0};
      mode_ = Mode::HOLD;
      return;
    }
  }

  void onTick()
  {
    const auto now_time = now();
    const double dt = std::clamp((now_time - last_tick_time_).seconds(), 0.001, 0.2);
    last_tick_time_ = now_time;

    const std::array<double, 3> delta = sub(target_position_, current_position_);
    const double distance = norm(delta);

    std::array<double, 3> desired_velocity{0.0, 0.0, 0.0};
    if (distance > arrive_radius_m_ && mode_ != Mode::ABORT) {
      const auto direction = scale(delta, 1.0 / std::max(distance, 1e-6));
      desired_velocity = scale(direction, cruise_speed_mps_);
    } else {
      mode_ = Mode::HOLD;
    }

    const std::array<double, 3> vel_error = sub(desired_velocity, current_velocity_);
    const double vel_error_norm = norm(vel_error);
    const double max_dv = max_accel_mps2_ * dt;
    std::array<double, 3> dv = vel_error;
    if (vel_error_norm > max_dv && vel_error_norm > 1e-9) {
      dv = scale(vel_error, max_dv / vel_error_norm);
    }

    current_velocity_ = add(current_velocity_, dv);
    current_position_ = add(current_position_, scale(current_velocity_, dt));

    publishVehiclePose(now_time);
    publishOffboardGoal(now_time);
  }

  void publishVehiclePose(const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "ned";
    pose.pose.position.x = current_position_[0];
    pose.pose.position.y = current_position_[1];
    pose.pose.position.z = current_position_[2];
    pose.pose.orientation.w = 1.0;
    vehicle_pose_pub_->publish(pose);
  }

  void publishOffboardGoal(const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = stamp;
    goal.header.frame_id = "ned";
    goal.pose.position.x = target_position_[0];
    goal.pose.position.y = target_position_[1];
    goal.pose.position.z = target_position_[2];
    goal.pose.orientation.w = 1.0;
    offboard_goal_pub_->publish(goal);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mission_goal_sub_;
  rclcpp::Subscription<msg::MissionCommand>::SharedPtr mission_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr offboard_goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double control_hz_{50.0};
  double cruise_speed_mps_{3.0};
  double max_accel_mps2_{1.5};
  double arrive_radius_m_{2.0};
  double hover_altitude_m_{20.0};
  double home_x_{0.0};
  double home_y_{0.0};
  double home_z_{0.0};

  Mode mode_{Mode::HOLD};
  std::array<double, 3> current_position_{0.0, 0.0, 0.0};
  std::array<double, 3> current_velocity_{0.0, 0.0, 0.0};
  std::array<double, 3> target_position_{0.0, 0.0, 0.0};
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace c3_drone_driver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<c3_drone_driver::MotionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
