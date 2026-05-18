#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "c3_drone_driver/config.h"
#include "c3_drone_driver/msg/drone_status.hpp"
#include "c3_drone_driver/msg/gimbal_motion_command.hpp"
#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/mission_command.hpp"
#include "c3_drone_driver/msg/target_observation.hpp"
#include "c3_drone_driver/pose_estimator.h"

namespace c3_drone_driver
{

class DroneMainControllerNode : public rclcpp::Node
{
public:
  DroneMainControllerNode()
  : Node("drone_main_controller_node")
  {
    const std::string default_cfg =
      ament_index_cpp::get_package_share_directory("c3_drone_driver") +
      "/config/pose_estimator_default.yaml";
    const std::string config_file = declare_parameter<std::string>("pose_config_file", default_cfg);
    if (!Config::SetParameterFile(config_file)) {
      RCLCPP_WARN(get_logger(), "Failed to load %s, fallback to built-in defaults", config_file.c_str());
    }

    PoseEstimator::Config pose_cfg;
    pose_cfg.body_to_gimbal_x = Config::GetOr<double>("body_to_gimbal_x", 0.15);
    pose_cfg.body_to_gimbal_y = Config::GetOr<double>("body_to_gimbal_y", 0.0);
    pose_cfg.body_to_gimbal_z = Config::GetOr<double>("body_to_gimbal_z", -0.05);
    motion_command_hz_ = declare_parameter<double>("motion_command_hz", 100.0);
    observation_valid_timeout_s_ = declare_parameter<double>("observation_valid_timeout_s", 2.0);

    pose_estimator_ = std::make_unique<PoseEstimator>(pose_cfg);

    mission_cmd_sub_ = create_subscription<msg::MissionCommand>(
      "/mission/cmd", 10, std::bind(&DroneMainControllerNode::onMissionCmd, this, std::placeholders::_1));
    ship_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ship/pose_world", 10, std::bind(&DroneMainControllerNode::onShipPose, this, std::placeholders::_1));
    ship_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ship/target_point", 10, std::bind(&DroneMainControllerNode::onShipTarget, this, std::placeholders::_1));
    observation_sub_ = create_subscription<msg::TargetObservation>(
      "/target/observation_body", 10, std::bind(&DroneMainControllerNode::onObservation, this, std::placeholders::_1));
    gimbal_state_sub_ = create_subscription<msg::GimbalState>(
      "/gimbal/state", 10, std::bind(&DroneMainControllerNode::onGimbalState, this, std::placeholders::_1));
    vehicle_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/px4/vehicle_pose", 10, std::bind(&DroneMainControllerNode::onVehiclePose, this, std::placeholders::_1));

    observation_ned_pub_ = create_publisher<msg::TargetObservation>("/target/observation_ned", 10);
    gimbal_motion_pub_ = create_publisher<msg::GimbalMotionCommand>("/gimbal/motion_command", 10);
    mission_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mission/goal", 10);
    status_pub_ = create_publisher<msg::DroneStatus>("/main_controller/status", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(motion_command_hz_, 5.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DroneMainControllerNode::onTick, this));

    RCLCPP_INFO(get_logger(), "drone_main_controller_node started");
  }

private:
  void onMissionCmd(const msg::MissionCommand::SharedPtr msg)
  {
    last_mission_cmd_ = *msg;
    has_mission_cmd_ = true;
  }

  void onShipPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    ship_pose_world_ = *msg;
    has_ship_pose_ = true;
  }

  void onShipTarget(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // 约定：输入为船坐标系相对目标点（frame_id=ship）
    if (msg->header.frame_id != "ship") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ship target frame_id should be 'ship'");
      return;
    }
    ship_target_rel_ = *msg;
    has_ship_target_ = true;
  }

  void onObservation(const msg::TargetObservation::SharedPtr msg)
  {
    const auto transformed = pose_estimator_->transformObservation(*msg);
    if (!transformed.has_value()) {
      return;
    }

    msg::TargetObservation ned_obs = *msg;
    ned_obs.header.stamp = now();
    ned_obs.frame = msg::TargetObservation::FRAME_NED_DRONE;
    ned_obs.position.x = transformed->position_ned[0];
    ned_obs.position.y = transformed->position_ned[1];
    ned_obs.position.z = transformed->position_ned[2];
    ned_obs.range = static_cast<float>(transformed->range);
    ned_obs.yaw = static_cast<float>(transformed->yaw_body);
    ned_obs.pitch = static_cast<float>(transformed->pitch_body);
    observation_ned_pub_->publish(ned_obs);

    last_target_body_ = transformed->position_body;
    has_target_body_ = true;
    last_obs_time_ = now();
  }

  void onGimbalState(const msg::GimbalState::SharedPtr msg)
  {
    pose_estimator_->updateGimbalState(*msg);
  }

  void onVehiclePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pose_estimator_->updateVehiclePose(*msg);
    last_vehicle_pose_ = *msg;
    has_vehicle_pose_ = true;
  }

  void onTick()
  {
    publishMissionGoal();
    publishMotionCommand();
    publishStatus();
  }

  void publishMissionGoal()
  {
    if (!has_ship_pose_) {
      return;
    }
    if (!has_mission_cmd_) {
      return;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "ned";
    goal.pose.orientation.w = 1.0;

    if (last_mission_cmd_.command == msg::MissionCommand::CMD_BACK) {
      // 回船：直接跟随母船当前世界系位置
      goal.pose.position = ship_pose_world_.pose.position;
      mission_goal_pub_->publish(goal);
      return;
    }

    if (last_mission_cmd_.command != msg::MissionCommand::CMD_START) {
      return;
    }
    if (!has_ship_target_) {
      return;
    }

    const auto &sp = ship_pose_world_.pose.position;
    const auto &sq = ship_pose_world_.pose.orientation;
    const double ship_yaw = quatYaw(sq.x, sq.y, sq.z, sq.w);
    const double c = std::cos(ship_yaw);
    const double s = std::sin(ship_yaw);

    const double rx = ship_target_rel_.pose.position.x;
    const double ry = ship_target_rel_.pose.position.y;
    const double rz = ship_target_rel_.pose.position.z;

    goal.pose.position.x = sp.x + c * rx - s * ry;
    goal.pose.position.y = sp.y + s * rx + c * ry;
    goal.pose.position.z = sp.z + rz;
    mission_goal_pub_->publish(goal);
  }

  static double quatYaw(double x, double y, double z, double w)
  {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void publishMotionCommand()
  {
    if (!has_target_body_) {
      return;
    }
    const auto cmd = pose_estimator_->bodyPointToGimbalYawPitch(last_target_body_);
    if (!cmd.has_value()) {
      return;
    }
    msg::GimbalMotionCommand motion;
    motion.header.stamp = now();
    motion.yaw = static_cast<float>(cmd->first);
    motion.pitch = static_cast<float>(cmd->second);
    gimbal_motion_pub_->publish(motion);
  }

  void publishStatus()
  {
    msg::DroneStatus s;
    s.header.stamp = now();
    s.t_usec = static_cast<uint64_t>(s.header.stamp.sec) * 1000000ULL +
      static_cast<uint64_t>(s.header.stamp.nanosec / 1000U);
    s.mission_mode = msg::DroneStatus::MODE_IDLE;
    if (has_mission_cmd_) {
      if (last_mission_cmd_.command == msg::MissionCommand::CMD_BACK) {
        s.mission_mode = msg::DroneStatus::MODE_RETURN;
      } else if (has_target_body_) {
        s.mission_mode = msg::DroneStatus::MODE_TRACK;
      } else if (last_mission_cmd_.command == msg::MissionCommand::CMD_START) {
        s.mission_mode = msg::DroneStatus::MODE_TRANSIT;
      }
    }
    if (has_mission_cmd_ && last_mission_cmd_.command == msg::MissionCommand::CMD_DETECTING) {
      s.gimbal_mode = msg::GimbalState::MODE_DETECTING;
    } else {
      s.gimbal_mode = msg::GimbalState::MODE_TRACKING;
    }
    s.link_state = msg::DroneStatus::LINK_OK;
    s.battery_remain = 1.0F;
    s.nav_health = has_vehicle_pose_ ? 1 : 0;
    s.vision_health =
      (has_target_body_ && (now() - last_obs_time_).seconds() < observation_valid_timeout_s_) ? 1 : 0;
    status_pub_->publish(s);
  }

  std::unique_ptr<PoseEstimator> pose_estimator_;

  rclcpp::Subscription<msg::MissionCommand>::SharedPtr mission_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ship_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ship_target_sub_;
  rclcpp::Subscription<msg::TargetObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<msg::GimbalState>::SharedPtr gimbal_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_sub_;

  rclcpp::Publisher<msg::TargetObservation>::SharedPtr observation_ned_pub_;
  rclcpp::Publisher<msg::GimbalMotionCommand>::SharedPtr gimbal_motion_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mission_goal_pub_;
  rclcpp::Publisher<msg::DroneStatus>::SharedPtr status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  double motion_command_hz_{50.0};
  double observation_valid_timeout_s_{2.0};

  bool has_mission_cmd_{false};
  bool has_ship_pose_{false};
  bool has_ship_target_{false};
  bool has_target_body_{false};
  bool has_vehicle_pose_{false};
  msg::MissionCommand last_mission_cmd_{};
  geometry_msgs::msg::PoseStamped ship_pose_world_{};
  geometry_msgs::msg::PoseStamped ship_target_rel_{};
  geometry_msgs::msg::PoseStamped last_vehicle_pose_{};
  std::array<double, 3> last_target_body_{0.0, 0.0, 0.0};
  rclcpp::Time last_obs_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace c3_drone_driver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<c3_drone_driver::DroneMainControllerNode>());
  rclcpp::shutdown();
  return 0;
}
