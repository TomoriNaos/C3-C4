#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "c3_drone_driver/msg/drone_status.hpp"
#include "c3_drone_driver/msg/gimbal_motion_command.hpp"
#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/mission_command.hpp"
#include "c3_drone_driver/msg/target_hint.hpp"
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
    PoseEstimator::Config pose_cfg;
    pose_cfg.body_to_gimbal_x = declare_parameter<double>("body_to_gimbal_x", 0.15);
    pose_cfg.body_to_gimbal_y = declare_parameter<double>("body_to_gimbal_y", 0.0);
    pose_cfg.body_to_gimbal_z = declare_parameter<double>("body_to_gimbal_z", -0.05);
    motion_command_hz_ = declare_parameter<double>("motion_command_hz", 100.0);
    observation_valid_timeout_s_ = declare_parameter<double>("observation_valid_timeout_s", 2.0);

    pose_estimator_ = std::make_unique<PoseEstimator>(pose_cfg);

    mission_cmd_sub_ = create_subscription<msg::MissionCommand>(
      "/mission/cmd", 10, std::bind(&DroneMainControllerNode::onMissionCmd, this, std::placeholders::_1));
    target_hint_sub_ = create_subscription<msg::TargetHint>(
      "/mission/target_hint", 10, std::bind(&DroneMainControllerNode::onTargetHint, this, std::placeholders::_1));
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

  void onTargetHint(const msg::TargetHint::SharedPtr msg)
  {
    last_target_hint_ = *msg;
    has_target_hint_ = true;
    geometry_msgs::msg::PoseStamped goal;
    goal.header = msg->header;
    // Preserve hint source timestamp for downstream temporal alignment.
    if (msg->t_usec != 0ULL) {
      const int64_t sec = static_cast<int64_t>(msg->t_usec / 1000000ULL);
      const int64_t nsec = static_cast<int64_t>((msg->t_usec % 1000000ULL) * 1000ULL);
      goal.header.stamp.sec = static_cast<int32_t>(sec);
      goal.header.stamp.nanosec = static_cast<uint32_t>(nsec);
    }
    // Carry frame semantics forward for mission/trajectory modules.
    goal.header.frame_id =
      (msg->frame == msg::TargetHint::FRAME_NED_REL_MOTHERSHIP) ? "mother_ned" : "mother_body";
    goal.pose.position = msg->position;
    goal.pose.orientation.w = 1.0;
    mission_goal_pub_->publish(goal);
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
    publishMotionCommand();
    publishStatus();
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
      } else if (last_mission_cmd_.command == msg::MissionCommand::CMD_CLOSE) {
        s.mission_mode = msg::DroneStatus::MODE_ABORT;
      } else if (has_target_body_) {
        s.mission_mode = msg::DroneStatus::MODE_TRACK;
      } else if (has_target_hint_) {
        s.mission_mode = msg::DroneStatus::MODE_SEARCH;
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
  rclcpp::Subscription<msg::TargetHint>::SharedPtr target_hint_sub_;
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
  bool has_target_hint_{false};
  bool has_target_body_{false};
  bool has_vehicle_pose_{false};
  msg::MissionCommand last_mission_cmd_{};
  msg::TargetHint last_target_hint_{};
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
