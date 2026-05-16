#include <algorithm>
#include <cstdint>
#include <deque>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "c3_drone_driver/msg/drone_status.hpp"
#include "c3_drone_driver/msg/command_ack.hpp"
#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/mission_command.hpp"
#include "c3_drone_driver/msg/target_hint.hpp"
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
		target_obs_valid_timeout_s_ = declare_parameter<double>("target_obs_valid_timeout_s", 2.0);
		heartbeat_hz_ = declare_parameter<double>("heartbeat_hz", 1.0);
		cmd_ack_timeout_ms_ = declare_parameter<int>("cmd_ack_timeout_ms", 300);
		cmd_max_retry_ = declare_parameter<int>("cmd_max_retry", 5);

		obs_sub_ = create_subscription<msg::TargetObservation>(
			"/mavlink/target_obs", 10,
			std::bind(&MavlinkBridgeNode::onObservation, this, std::placeholders::_1));
		gimbal_state_sub_ = create_subscription<msg::GimbalState>(
			"/gimbal/state", 10,
			std::bind(&MavlinkBridgeNode::onGimbalState, this, std::placeholders::_1));
		heartbeat_rx_sub_ = create_subscription<std_msgs::msg::Bool>(
			"/mavlink/heartbeat_rx", 10,
			std::bind(&MavlinkBridgeNode::onHeartbeatRx, this, std::placeholders::_1));
		mission_cmd_rx_sub_ = create_subscription<msg::MissionCommand>(
			"/mavlink/mission_cmd_rx", 10,
			std::bind(&MavlinkBridgeNode::onMissionCmdRx, this, std::placeholders::_1));
		target_hint_rx_sub_ = create_subscription<msg::TargetHint>(
			"/mavlink/target_hint_rx", 10,
			std::bind(&MavlinkBridgeNode::onTargetHintRx, this, std::placeholders::_1));

			mission_cmd_pub_ = create_publisher<msg::MissionCommand>("/mission/cmd", 10);
			target_hint_pub_ = create_publisher<msg::TargetHint>("/mission/target_hint", 10);
			drone_status_pub_ = create_publisher<msg::DroneStatus>("/mission/state", 10);
			command_ack_pub_ = create_publisher<msg::CommandAck>("/mavlink/mission_cmd_ack", 10);
			heartbeat_tx_pub_ = create_publisher<std_msgs::msg::Bool>("/mavlink/heartbeat_tx", 10);

		const auto status_period = std::chrono::duration<double>(1.0 / std::max(status_hz_, 0.2));
		timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
			std::bind(&MavlinkBridgeNode::onStatusTimer, this));

		const auto hb_period = std::chrono::duration<double>(1.0 / std::max(heartbeat_hz_, 0.2));
		hb_timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(hb_period),
			std::bind(&MavlinkBridgeNode::onHeartbeatTimer, this));

		last_heartbeat_rx_ = now();
		RCLCPP_INFO(get_logger(), "mavlink_bridge_node started (ROS-side placeholder)");
	}

private:
	void onObservation(const msg::TargetObservation::SharedPtr msg)
	{
		last_obs_time_ = rclcpp::Time(msg->header.stamp);
		last_obs_status_ = msg->status;
		last_obs_id_ = msg->obs_id;
		if (msg->status == msg::TargetObservation::STATUS_VALID)
		{
			mission_mode_ = msg::DroneStatus::MODE_TRACK;
		}
		else if (msg->status == msg::TargetObservation::STATUS_LOST && mission_mode_ == msg::DroneStatus::MODE_TRACK)
		{
			mission_mode_ = msg::DroneStatus::MODE_SEARCH;
		}
	}

	void onGimbalState(const msg::GimbalState::SharedPtr msg)
	{
		gimbal_mode_ = msg->mode;
		last_gimbal_state_time_ = rclcpp::Time(msg->header.stamp);
	}

	void onHeartbeatRx(const std_msgs::msg::Bool::SharedPtr msg)
	{
		if (msg->data)
		{
			last_heartbeat_rx_ = now();
		}
	}

	void onMissionCmdRx(const msg::MissionCommand::SharedPtr msg)
	{
			last_mission_cmd_ = *msg;
			mission_cmd_pub_->publish(*msg);
			publishCommandAck(msg->command, msg::CommandAck::RESULT_ACCEPTED, 0U, "accepted");
			switch (msg->command)
		{
		case msg::MissionCommand::CMD_START:
			mission_mode_ = msg::DroneStatus::MODE_TRANSIT;
			break;
		case msg::MissionCommand::CMD_BACK:
			mission_mode_ = msg::DroneStatus::MODE_RETURN;
			break;
		case msg::MissionCommand::CMD_CLOSE:
			mission_mode_ = msg::DroneStatus::MODE_ABORT;
			break;
		case msg::MissionCommand::CMD_HOLD:
			mission_mode_ = msg::DroneStatus::MODE_SEARCH;
			break;
		case msg::MissionCommand::CMD_TRACKING:
			gimbal_mode_ = msg::GimbalState::MODE_TRACKING;
			break;
		case msg::MissionCommand::CMD_DETECTING:
			gimbal_mode_ = msg::GimbalState::MODE_DETECTING;
			break;
		default:
			break;
		}
			last_cmd_rx_time_ = now();
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

	void onTargetHintRx(const msg::TargetHint::SharedPtr msg)
	{
		target_hint_pub_->publish(*msg);
		last_target_hint_time_ = rclcpp::Time(msg->header.stamp);
		if (mission_mode_ == msg::DroneStatus::MODE_IDLE)
		{
			mission_mode_ = msg::DroneStatus::MODE_TRANSIT;
		}
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

		msg::DroneStatus status;
		status.header.stamp = t_now;
		status.t_usec = static_cast<uint64_t>(t_now.nanoseconds() / 1000ULL);
		status.mission_mode = mission_mode_;
		status.gimbal_mode = gimbal_mode_;
		status.battery_remain = 1.0F;
		status.nav_health = 1;
		status.vision_health = ((t_now - last_obs_time_).seconds() < target_obs_valid_timeout_s_ &&
			last_obs_status_ != msg::TargetObservation::STATUS_LOST) ? 1 : 0;

		if (since_rx >= link_lost_s_)
		{
			status.link_state = msg::DroneStatus::LINK_LOST;
			if (mission_mode_ != msg::DroneStatus::MODE_ABORT)
			{
				mission_mode_ = msg::DroneStatus::MODE_RETURN;
			}
		}
		else if (since_rx >= link_degraded_s_)
		{
			status.link_state = msg::DroneStatus::LINK_DEGRADED;
		}
		else
		{
			status.link_state = msg::DroneStatus::LINK_OK;
		}

		drone_status_pub_->publish(status);
	}

	rclcpp::Subscription<msg::TargetObservation>::SharedPtr obs_sub_;
	rclcpp::Subscription<msg::GimbalState>::SharedPtr gimbal_state_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_rx_sub_;
	rclcpp::Subscription<msg::MissionCommand>::SharedPtr mission_cmd_rx_sub_;
	rclcpp::Subscription<msg::TargetHint>::SharedPtr target_hint_rx_sub_;
		rclcpp::Publisher<msg::MissionCommand>::SharedPtr mission_cmd_pub_;
		rclcpp::Publisher<msg::TargetHint>::SharedPtr target_hint_pub_;
		rclcpp::Publisher<msg::DroneStatus>::SharedPtr drone_status_pub_;
		rclcpp::Publisher<msg::CommandAck>::SharedPtr command_ack_pub_;
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_tx_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr hb_timer_;

	double link_degraded_s_{3.0};
	double link_lost_s_{8.0};
	double status_hz_{1.0};
	double heartbeat_hz_{1.0};
	double target_obs_valid_timeout_s_{2.0};
	int cmd_ack_timeout_ms_{300};
	int cmd_max_retry_{5};

	rclcpp::Time last_heartbeat_rx_{0, 0, RCL_ROS_TIME};
	rclcpp::Time last_obs_time_{0, 0, RCL_ROS_TIME};
	rclcpp::Time last_gimbal_state_time_{0, 0, RCL_ROS_TIME};
	rclcpp::Time last_cmd_rx_time_{0, 0, RCL_ROS_TIME};
	rclcpp::Time last_target_hint_time_{0, 0, RCL_ROS_TIME};

	msg::MissionCommand last_mission_cmd_;
	uint32_t last_obs_id_{0};
	uint8_t mission_mode_{msg::DroneStatus::MODE_IDLE};
	uint8_t gimbal_mode_{msg::GimbalState::MODE_TRACKING};
	uint8_t last_obs_status_{msg::TargetObservation::STATUS_LOST};
};

} // namespace c3_drone_driver

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<c3_drone_driver::MavlinkBridgeNode>());
	rclcpp::shutdown();
	return 0;
}
