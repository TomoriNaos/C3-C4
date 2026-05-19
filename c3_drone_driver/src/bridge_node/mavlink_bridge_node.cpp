#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "c3_drone_driver/msg/drone_status.hpp"
#include "c3_drone_driver/msg/command_ack.hpp"
#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/mission_command.hpp"
#include "c3_drone_driver/msg/target_observation.hpp"

namespace c3_drone_driver
{

class MavlinkBridgeNode : public rclcpp::Node
{
public:
	MavlinkBridgeNode()
	: Node("mavlink_bridge_node")
	{
		link_degraded_s_ = declare_parameter<double>("link_degraded_s", 3.0);
		link_lost_s_ = declare_parameter<double>("link_lost_s", 8.0);
		status_hz_ = declare_parameter<double>("status_hz", 1.0);
		heartbeat_hz_ = declare_parameter<double>("heartbeat_hz", 1.0);
		status_keepalive_s_ = declare_parameter<double>("status_keepalive_s", 5.0);
		target_obs_valid_timeout_s_ = declare_parameter<double>("target_obs_valid_timeout_s", 2.0);

		heartbeat_rx_sub_ = create_subscription<std_msgs::msg::Bool>(
			"/mavlink/heartbeat_rx", 10,
			[this](const std_msgs::msg::Bool::SharedPtr msg) { onHeartbeatRx(msg); });
		mission_cmd_rx_sub_ = create_subscription<msg::MissionCommand>(
			"/mavlink/mission_cmd_rx", 10,
			[this](const msg::MissionCommand::SharedPtr msg) { onMissionCmdRx(msg); });
		target_obs_sub_ = create_subscription<msg::TargetObservation>(
			"/target/observation_body", 10,
			[this](const msg::TargetObservation::SharedPtr msg) { onTargetObservation(msg); });
		ship_pose_rx_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
			"/mavlink/ship_pose_world_rx", 10,
			[this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { onShipPoseRx(msg); });
		ship_target_rx_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
			"/mavlink/ship_target_point_rx", 10,
			[this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { onShipTargetRx(msg); });

		mission_cmd_pub_ = create_publisher<msg::MissionCommand>("/mission/cmd", 10);
		drone_status_pub_ = create_publisher<msg::DroneStatus>("/mission/state", 10);
		command_ack_pub_ = create_publisher<msg::CommandAck>("/mavlink/mission_cmd_ack", 10);
		heartbeat_tx_pub_ = create_publisher<std_msgs::msg::Bool>("/mavlink/heartbeat_tx", 10);
		ship_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/ship/pose_world", 10);
		ship_target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/ship/target_point", 10);
		mavlink_target_obs_pub_ = create_publisher<msg::TargetObservation>("/mavlink/target_obs", 10);

		const auto status_period = std::chrono::duration<double>(1.0 / std::max(status_hz_, 0.2));
		timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
			[this]() { onStatusTimer(); });

		const auto hb_period = std::chrono::duration<double>(1.0 / std::max(heartbeat_hz_, 0.2));
		hb_timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(hb_period),
			[this]() { onHeartbeatTimer(); });

		last_heartbeat_rx_ = now();
		RCLCPP_INFO(get_logger(), "mavlink_bridge_node started (ROS-side placeholder)");
	}

private:
	void onHeartbeatRx(const std_msgs::msg::Bool::SharedPtr msg)
	{
		if (msg->data)
		{
			last_heartbeat_rx_ = now();
		}
	}

	void onMissionCmdRx(const msg::MissionCommand::SharedPtr msg)
	{
		// 纯ROS占位网关：仅透传消息，不在Bridge内维护任务/云台状态机。
		mission_cmd_pub_->publish(*msg);
		publishCommandAck(msg->command, msg::CommandAck::RESULT_ACCEPTED, 0U, "accepted");
	}

	void onTargetObservation(const msg::TargetObservation::SharedPtr msg)
	{
		if (!msg)
		{
			return;
		}
		const auto stamp = rclcpp::Time(msg->header.stamp);
		if ((now() - stamp).seconds() > target_obs_valid_timeout_s_)
		{
			return;
		}
		if (has_last_obs_ && msg->obs_id == last_obs_id_)
		{
			return;
		}

		last_obs_id_ = msg->obs_id;
		has_last_obs_ = true;
		mavlink_target_obs_pub_->publish(*msg);
	}

	void onShipPoseRx(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		ship_pose_pub_->publish(*msg);
	}

	void onShipTargetRx(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		ship_target_pub_->publish(*msg);
	}

	void publishCommandAck(uint8_t command, uint8_t result, uint8_t retry_count, const std::string &text)
	{
		msg::CommandAck ack;
		ack.header.stamp = now();
		ack.command = command;
		ack.result = result;
		ack.retry_count = retry_count;
		ack.message = text;
		command_ack_pub_->publish(ack);
	}

	void onHeartbeatTimer()
	{
		std_msgs::msg::Bool hb;
		hb.data = true;
		heartbeat_tx_pub_->publish(hb);
	}

	void onStatusTimer()
	{
		const auto t_now = now();
		const double since_rx = (t_now - last_heartbeat_rx_).seconds();

		const uint8_t link_state = (since_rx >= link_lost_s_) ? msg::DroneStatus::LINK_LOST
			: (since_rx >= link_degraded_s_) ? msg::DroneStatus::LINK_DEGRADED
											   : msg::DroneStatus::LINK_OK;
		msg::DroneStatus status;
		status.header.stamp = t_now;
		status.t_usec = static_cast<uint64_t>(t_now.nanoseconds() / 1000ULL);
		status.mission_mode = msg::DroneStatus::MODE_HOLD;
		status.gimbal_mode = msg::GimbalState::MODE_TRACKING;
		status.link_state = link_state;

		const bool status_changed = !has_last_status_ ||
			status.link_state != last_status_.link_state ||
			status.mission_mode != last_status_.mission_mode ||
			status.gimbal_mode != last_status_.gimbal_mode;
		const bool keepalive_due = !has_last_status_ ||
			(t_now - last_status_pub_time_).seconds() >= status_keepalive_s_;
		if (!status_changed && !keepalive_due)
		{
			return;
		}

		drone_status_pub_->publish(status);
		last_status_ = status;
		has_last_status_ = true;
		last_status_pub_time_ = t_now;
	}

	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_rx_sub_;
	rclcpp::Subscription<msg::MissionCommand>::SharedPtr mission_cmd_rx_sub_;
	rclcpp::Subscription<msg::TargetObservation>::SharedPtr target_obs_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ship_pose_rx_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ship_target_rx_sub_;
		rclcpp::Publisher<msg::MissionCommand>::SharedPtr mission_cmd_pub_;
		rclcpp::Publisher<msg::DroneStatus>::SharedPtr drone_status_pub_;
		rclcpp::Publisher<msg::CommandAck>::SharedPtr command_ack_pub_;
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_tx_pub_;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ship_pose_pub_;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ship_target_pub_;
		rclcpp::Publisher<msg::TargetObservation>::SharedPtr mavlink_target_obs_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr hb_timer_;

	double link_degraded_s_{3.0};
	double link_lost_s_{8.0};
	double status_hz_{1.0};
	double heartbeat_hz_{1.0};
	double status_keepalive_s_{5.0};
	double target_obs_valid_timeout_s_{2.0};

	rclcpp::Time last_heartbeat_rx_{0, 0, RCL_ROS_TIME};
	rclcpp::Time last_status_pub_time_{0, 0, RCL_ROS_TIME};
	uint32_t last_obs_id_{0};
	msg::DroneStatus last_status_{};
	bool has_last_obs_{false};
	bool has_last_status_{false};

};

} // namespace c3_drone_driver

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<c3_drone_driver::MavlinkBridgeNode>());
	rclcpp::shutdown();
	return 0;
}
