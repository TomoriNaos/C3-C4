#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "c3_drone_driver/config.h"
#include "c3_drone_driver/msg/drone_status.hpp"
#include "c3_drone_driver/msg/gimbal_motion_command.hpp"
#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/mission_command.hpp"
#include "c3_drone_driver/msg/target_observation.hpp"
#include "c3_drone_driver/srv/set_gimbal_mode.hpp"
#include "c3_drone_driver/pose_estimator.h"

namespace c3_drone_driver
{

	class DroneMainControllerNode : public rclcpp::Node
	{
	public:
		DroneMainControllerNode()
			: Node("drone_main_controller_node")
		{
			// config/drone_main_controller_default.yaml
			const std::string default_cfg =
				ament_index_cpp::get_package_share_directory("c3_drone_driver") +
				"/config/pose_estimator_default.yaml";
			const std::string config_file = declare_parameter<std::string>("pose_config_file", default_cfg);
			if (!Config::SetParameterFile(config_file))
			{
				RCLCPP_WARN(get_logger(), "Failed to load %s, fallback to built-in defaults", config_file.c_str());
			}

			PoseEstimator::Config pose_cfg;
			pose_cfg.body_to_gimbal_x = Config::GetOr<double>("body_to_gimbal_x", 0.15);
			pose_cfg.body_to_gimbal_y = Config::GetOr<double>("body_to_gimbal_y", 0.0);
			pose_cfg.body_to_gimbal_z = Config::GetOr<double>("body_to_gimbal_z", -0.05);
			motion_command_hz_ = declare_parameter<double>("motion_command_hz", 100.0);
			observation_valid_timeout_s_ = declare_parameter<double>("observation_valid_timeout_s", 2.0);
			status_keepalive_s_ = declare_parameter<double>("status_keepalive_s", 1.0);
			gimbal_enable_distance_m_ = declare_parameter<double>("gimbal_enable_distance_m", 8.0);

			// 位姿计算器实例化
			pose_estimator_ = std::make_unique<PoseEstimator>(pose_cfg);

			// ============== mavlink_bridge_node通信接口 ==============
			// 命令接收器
			mission_cmd_sub_ = create_subscription<msg::MissionCommand>(
				"/mission/cmd", 10, [this](const msg::MissionCommand::SharedPtr msg)
				{ 
					state_.mission_cmd = *msg;
					state_.has_mission_cmd = true; });

			// 母船位姿（位置）订阅
			ship_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
				"/ship/pose_world", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
				{ 
					state_.ship_pose_world = *msg;
					state_.has_ship_pose = true; });

			// 目标点订阅
			ship_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
				"/ship/target_point", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
				{ 
					// 约定：输入为船坐标系相对目标点（frame_id=ship）
					if (msg->header.frame_id != "ship")
					{
						RCLCPP_WARN_THROTTLE(
							get_logger(), *get_clock(), 1000, "Ship target frame_id should be 'ship'");
						return;
					}
					state_.ship_target_rel = *msg;
					state_.has_ship_target = true; });

			// ============= target_processor_node通信接口 =============
			// 目标观测订阅
			observation_sub_ = create_subscription<msg::TargetObservation>(
				"/target/observation_body", 10, [this](const msg::TargetObservation::SharedPtr msg)
				{ 			
					// 将目标观测解算到机体系
					const auto transformed = pose_estimator_->transformObservation(*msg);
					if (!transformed.has_value())
					{
						return;
					}

					state_.target_body = transformed->position_body;
					state_.has_target_body = true;
					state_.last_obs_time = now(); });

			// ============= gimbal_controller_node通信接口 =============
			// 云台状态订阅
			gimbal_state_sub_ = create_subscription<msg::GimbalState>(
				"/gimbal/state", 10, [this](const msg::GimbalState::SharedPtr msg)
				{
					pose_estimator_->updateGimbalState(*msg);
					state_.gimbal_state = *msg;
					state_.has_gimbal_state = true; });

			// ============= px4_pose_bridge_node通信接口 =============
			// 无人机位姿订阅
			vehicle_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
				"/px4/vehicle_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
				{ pose_estimator_->updateVehiclePose(*msg); });

			// ============= motion_controller_node通信接口 =============
			// 实际任务模式订阅
			motion_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
				"/motion/mission_mode", 10, [this](const std_msgs::msg::UInt8::SharedPtr msg)
				{
					state_.motion_mode = msg->data;
					state_.has_motion_mode = true; });

			// 云台控制发布器
			gimbal_motion_pub_ = create_publisher<msg::GimbalMotionCommand>("/gimbal/motion_command", 10);
			gimbal_mode_client_ = create_client<srv::SetGimbalMode>("/gimbal/set_mode");

			// 任务目标发布器
			mission_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mission/goal", 10);

			// 状态发布器
			status_pub_ = create_publisher<msg::DroneStatus>("/main_controller/status", 10);

			const auto period = std::chrono::duration<double>(1.0 / std::max(motion_command_hz_, 5.0));
			// 定时器
			timer_ = create_wall_timer(
				std::chrono::duration_cast<std::chrono::nanoseconds>(period),
				[this]()
				{ onTick(); });

			RCLCPP_INFO(get_logger(), "drone_main_controller_node started");
		}

	private:
		struct MainState
		{
			bool has_mission_cmd{false};
			bool has_ship_pose{false};
			bool has_ship_target{false};
			bool has_target_body{false};
			bool has_gimbal_state{false};
			bool has_motion_mode{false};
			msg::MissionCommand mission_cmd{};
			geometry_msgs::msg::PoseStamped ship_pose_world{};
			geometry_msgs::msg::PoseStamped ship_target_rel{};
			msg::GimbalState gimbal_state{};
			uint8_t motion_mode{msg::DroneStatus::MODE_HOLD};
			std::array<double, 3> target_body{0.0, 0.0, 0.0};
			rclcpp::Time last_obs_time{0, 0, RCL_ROS_TIME};
		};

		void onTick()
		{
			publishMissionGoal();
			publishMotionCommand();
			publishStatus();
		}

		void publishMissionGoal()
		{
			if (!state_.has_ship_pose)
				return;
			if (!state_.has_mission_cmd)
				return;

			// 1. 根据当前任务命令和输入信息，计算任务目标点（世界系）
			geometry_msgs::msg::PoseStamped goal;
			goal.header.stamp = now();
			goal.header.frame_id = "ned";
			goal.pose.orientation.w = 1.0;

			// 2a. 回船命令：直接发布母船当前世界系位置
			if (state_.mission_cmd.command == msg::MissionCommand::CMD_BACK)
			{
				goal.pose.position = state_.ship_pose_world.pose.position;
				mission_goal_pub_->publish(goal);
				return;
			}

			// 2b. 其他命令：如果有相对目标点输入，则发布转换到世界系的目标点；否则不发布
			if (state_.mission_cmd.command != msg::MissionCommand::CMD_START)
				return;
			if (!state_.has_ship_target)
				return;

			// 2c. TRANSIT态下，将船坐标系相对目标点转换到世界系并发布
			const std::array<double, 3> rel_ship{
				state_.ship_target_rel.pose.position.x,
				state_.ship_target_rel.pose.position.y,
				state_.ship_target_rel.pose.position.z};
			const auto target_ned = PoseEstimator::shipRelativePointToNed(state_.ship_pose_world, rel_ship);
			goal.pose.position.x = target_ned[0];
			goal.pose.position.y = target_ned[1];
			goal.pose.position.z = target_ned[2];
			mission_goal_pub_->publish(goal);
		}

		void publishMotionCommand()
		{
			if (!state_.has_target_body)
				return;
			if ((now() - state_.last_obs_time).seconds() > observation_valid_timeout_s_)
				return;

			const auto cmd = pose_estimator_->bodyPointToGimbalYawPitch(state_.target_body);
			if (!cmd.has_value())
				return;

			const double target_range_m = targetRangeMeters(state_.target_body);
			const bool enable_visual = target_range_m <= gimbal_enable_distance_m_;
			requestGimbalMode(enable_visual ? msg::GimbalState::MODE_DETECTING : msg::GimbalState::MODE_TRACKING);
			if (!enable_visual)
				return;

			msg::GimbalMotionCommand motion;
			motion.header.stamp = now();
			motion.yaw = static_cast<float>(cmd->first);
			motion.pitch = static_cast<float>(cmd->second);
			gimbal_motion_pub_->publish(motion);
		}

		void publishStatus()
		{
			const auto stamp = now();
			const auto status = buildStatus(stamp);
			if (!shouldPublishStatus(status, status.header.stamp))
			{
				return;
			}

			status_pub_->publish(status);
			last_status_ = status;
			has_last_status_ = true;
			last_status_pub_time_ = status.header.stamp;
		}

		msg::DroneStatus buildStatus(const rclcpp::Time &stamp) const
		{
			msg::DroneStatus s;
			s.header.stamp = stamp;
			s.t_usec = static_cast<uint64_t>(stamp.nanoseconds() / 1000ULL);
			s.mission_mode = state_.has_motion_mode ? state_.motion_mode : msg::DroneStatus::MODE_HOLD;
			s.gimbal_mode = computeGimbalMode();
			return s;
		}

		uint8_t computeGimbalMode() const
		{
			if (state_.has_gimbal_state)
			{
				return state_.gimbal_state.mode;
			}
			return msg::GimbalState::MODE_TRACKING;
		}

		double targetRangeMeters(const std::array<double, 3> &point_body) const
		{
			const double dx = point_body[0];
			const double dy = point_body[1];
			const double dz = point_body[2];
			return std::sqrt(dx * dx + dy * dy + dz * dz);
		}

		void requestGimbalMode(uint8_t mode)
		{
			if (requested_gimbal_mode_ && *requested_gimbal_mode_ == mode)
				return;
			if (!gimbal_mode_client_->service_is_ready())
				return;
				
			auto req = std::make_shared<srv::SetGimbalMode::Request>();
			req->mode = mode;
			(void)gimbal_mode_client_->async_send_request(req);
			requested_gimbal_mode_ = mode;
		}

		/**
		 * @brief 是否应发布无人机当前状态
		 * @details
		 * - 第一次一定发布：!has_last_status_
		 * - 任一关键状态变化就发布：mission_mode / gimbal_mode / link_state
		 * - 即使没变化，也每隔 status_keepalive_s_ 秒强制发布一次，防止下游以为链路断了((HERATBEAT)
		 */
		bool shouldPublishStatus(const msg::DroneStatus &status, const rclcpp::Time &stamp) const
		{
			const bool status_changed = !has_last_status_ ||
										status.mission_mode != last_status_.mission_mode ||
										status.gimbal_mode != last_status_.gimbal_mode ||
										status.link_state != last_status_.link_state;
			const bool keepalive_due = !has_last_status_ ||
									   (stamp - last_status_pub_time_).seconds() >= status_keepalive_s_;
			return status_changed || keepalive_due;
		}

		std::unique_ptr<PoseEstimator> pose_estimator_;

		rclcpp::Subscription<msg::MissionCommand>::SharedPtr mission_cmd_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ship_pose_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ship_target_sub_;
		rclcpp::Subscription<msg::TargetObservation>::SharedPtr observation_sub_;
		rclcpp::Subscription<msg::GimbalState>::SharedPtr gimbal_state_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_sub_;
		rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr motion_mode_sub_;

		rclcpp::Publisher<msg::GimbalMotionCommand>::SharedPtr gimbal_motion_pub_;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mission_goal_pub_;
		rclcpp::Publisher<msg::DroneStatus>::SharedPtr status_pub_;
		rclcpp::Client<srv::SetGimbalMode>::SharedPtr gimbal_mode_client_;

		rclcpp::TimerBase::SharedPtr timer_;
		double motion_command_hz_{50.0};
		double observation_valid_timeout_s_{2.0};
		double status_keepalive_s_{1.0};
		double gimbal_enable_distance_m_{8.0};
		rclcpp::Time last_status_pub_time_{0, 0, RCL_ROS_TIME};
		std::optional<uint8_t> requested_gimbal_mode_{};
		msg::DroneStatus last_status_{};
		bool has_last_status_{false};

		MainState state_{};
	};

} // namespace c3_drone_driver

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<c3_drone_driver::DroneMainControllerNode>());
	rclcpp::shutdown();
	return 0;
}
