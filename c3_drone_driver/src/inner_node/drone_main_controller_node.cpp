#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "c3_drone_driver/msg/drone_status.hpp"
#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/mission_command.hpp"
#include "c3_drone_driver/srv/set_drone_lifecycle.hpp"

namespace c3_drone_driver
{

	class DroneMainControllerNode : public rclcpp::Node
	{
	public:
		DroneMainControllerNode()
			: Node("drone_main_controller_node")
		{
			motion_command_hz_ = declare_parameter<double>("motion_command_hz", 100.0);
			status_keepalive_s_ = declare_parameter<double>("status_keepalive_s", 1.0);
			tc_lifecycle_service_name_ = declare_parameter<std::string>("tc_lifecycle_service_name", "/tc_camera_node/change_state");
			gc_lifecycle_service_name_ = declare_parameter<std::string>("gc_lifecycle_service_name", "/gated_camera_node/change_state");

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

			// ============= gimbal_controller_node通信接口 =============
			// 云台状态订阅
			gimbal_state_sub_ = create_subscription<msg::GimbalState>(
				"/gimbal/state", 10, [this](const msg::GimbalState::SharedPtr msg)
				{
					state_.gimbal_state = *msg;
					state_.has_gimbal_state = true; });

			// ============= px4_pose_bridge_node通信接口 =============
			// 实际任务模式订阅
			motion_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
				"/motion/mission_mode", 10, [this](const std_msgs::msg::UInt8::SharedPtr msg)
				{
					state_.motion_mode = msg->data;
					state_.has_motion_mode = true; });

			// 任务目标发布器
			mission_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mission/goal", 10);

			// 状态发布器
			status_pub_ = create_publisher<msg::DroneStatus>("/main_controller/status", 10);
			drone_lifecycle_srv_ = create_service<srv::SetDroneLifecycle>(
				"/main_controller/set_lifecycle",
				[this](
					const srv::SetDroneLifecycle::Request::SharedPtr req,
					srv::SetDroneLifecycle::Response::SharedPtr res)
				{
					onSetDroneLifecycle(req, res);
				});

			// 相机生命周期客户端
			tc_lifecycle_client_ = create_client<lifecycle_msgs::srv::ChangeState>(tc_lifecycle_service_name_);
			gc_lifecycle_client_ = create_client<lifecycle_msgs::srv::ChangeState>(gc_lifecycle_service_name_);
			tc_camera_lifecycle_.service_name = tc_lifecycle_service_name_;
			tc_camera_lifecycle_.client = tc_lifecycle_client_;
			gc_camera_lifecycle_.service_name = gc_lifecycle_service_name_;
			gc_camera_lifecycle_.client = gc_lifecycle_client_;

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
			bool has_gimbal_state{false};
			bool has_motion_mode{false};
			msg::MissionCommand mission_cmd{};
			geometry_msgs::msg::PoseStamped ship_pose_world{};
			geometry_msgs::msg::PoseStamped ship_target_rel{};
			msg::GimbalState gimbal_state{};
			uint8_t motion_mode{msg::DroneStatus::MODE_HOLD};
		};
		
		struct LifecycleEndpoint
		{
			std::string service_name;
			rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client;
			std::optional<bool> requested_enabled{};
		};

		void onTick()
		{
			publishMissionGoal();
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
			const auto target_ned = shipRelativePointToNed(state_.ship_pose_world, rel_ship);
			goal.pose.position.x = target_ned[0];
			goal.pose.position.y = target_ned[1];
			goal.pose.position.z = target_ned[2];
			mission_goal_pub_->publish(goal);
		}

		void publishStatus()
		{
			const auto stamp = now();
			const auto status = buildStatus(stamp);
			if (!shouldPublishStatus(status, status.header.stamp))
				return;

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
			if (state_.has_gimbal_state){
				s.gimbal_mode = state_.gimbal_state.mode;
			} else {
				s.gimbal_mode = msg::GimbalState::MODE_TRACKING;
			}
			return s;
		}

		static std::array<double, 3> shipRelativePointToNed(
			const geometry_msgs::msg::PoseStamped &ship_pose_world,
			const std::array<double, 3> &target_rel_ship)
		{
			const auto &sp = ship_pose_world.pose.position;
			const auto &sq = ship_pose_world.pose.orientation;
			const double siny_cosp = 2.0 * (sq.w * sq.z + sq.x * sq.y);
			const double cosy_cosp = 1.0 - 2.0 * (sq.y * sq.y + sq.z * sq.z);
			const double ship_yaw = std::atan2(siny_cosp, cosy_cosp);
			const double c = std::cos(ship_yaw);
			const double s = std::sin(ship_yaw);
			return {
				sp.x + c * target_rel_ship[0] - s * target_rel_ship[1],
				sp.y + s * target_rel_ship[0] + c * target_rel_ship[1],
				sp.z + target_rel_ship[2]};
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
		
		void onSetDroneLifecycle(
			const srv::SetDroneLifecycle::Request::SharedPtr req,
			srv::SetDroneLifecycle::Response::SharedPtr res)
		{
			if (!req)
			{
				return;
			}

			bool accepted = true;
			std::string message = "ok";

			switch (req->command)
			{
			case srv::SetDroneLifecycle::Request::CMD_ACTIVATE:
				requestCameraLifecycle(true);
				break;
			case srv::SetDroneLifecycle::Request::CMD_DEACTIVATE:
				requestCameraLifecycle(false);
				break;
			default:
				accepted = false;
				message = "unsupported lifecycle command";
				break;
			}

			res->success = accepted;
			res->message = message;
			res->mission_mode = state_.has_motion_mode
				? state_.motion_mode
				: msg::DroneStatus::MODE_HOLD;
		}
		
		void requestCameraLifecycle(bool enable)
		{
			requestLifecycleState(tc_camera_lifecycle_, enable);
			requestLifecycleState(gc_camera_lifecycle_, enable);
		}

		void requestLifecycleState(LifecycleEndpoint &endpoint, bool enable)
		{
			if (!endpoint.client)
				return;
			if (endpoint.requested_enabled && *endpoint.requested_enabled == enable)
				return;
			if (!enable && !endpoint.requested_enabled)
				return;
			if (!endpoint.client->service_is_ready())
			{
				RCLCPP_WARN_THROTTLE(
					get_logger(), *get_clock(), 2000,
					"Lifecycle service not ready: %s", endpoint.service_name.c_str());
				return;
			}

			auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
			req->transition.id = enable
				? lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE
				: lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
			req->transition.label = enable ? "activate" : "deactivate";
			(void)endpoint.client->async_send_request(req);
			endpoint.requested_enabled = enable;
		}

		rclcpp::Subscription<msg::MissionCommand>::SharedPtr mission_cmd_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ship_pose_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ship_target_sub_;
		rclcpp::Subscription<msg::GimbalState>::SharedPtr gimbal_state_sub_;
		rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr motion_mode_sub_;

		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mission_goal_pub_;
		rclcpp::Publisher<msg::DroneStatus>::SharedPtr status_pub_;
		rclcpp::Service<srv::SetDroneLifecycle>::SharedPtr drone_lifecycle_srv_;
		rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr tc_lifecycle_client_;
		rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr gc_lifecycle_client_;

		rclcpp::TimerBase::SharedPtr timer_;
		double motion_command_hz_{50.0};
		double status_keepalive_s_{1.0};
		std::string tc_lifecycle_service_name_{"/tc_camera_node/change_state"};
		std::string gc_lifecycle_service_name_{"/gated_camera_node/change_state"};
		rclcpp::Time last_status_pub_time_{0, 0, RCL_ROS_TIME};
		msg::DroneStatus last_status_{};
		bool has_last_status_{false};
		LifecycleEndpoint tc_camera_lifecycle_{};
		LifecycleEndpoint gc_camera_lifecycle_{};
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
