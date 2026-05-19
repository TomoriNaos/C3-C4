#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "c3_drone_driver/msg/gimbal_motion_command.hpp"
#include "c3_drone_driver/msg/drone_status.hpp"
#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/gimbal_visual_command.hpp"
#include "c3_drone_driver/srv/set_gimbal_mode.hpp"

namespace c3_drone_driver
{

class GimbalControllerNode : public rclcpp::Node
{
public:
	GimbalControllerNode()
	: Node("gimbal_controller_node")
	{
		//gimbal.yaml
		yaw_min_ = declare_parameter<double>("yaw_min", -2.4);
		yaw_max_ = declare_parameter<double>("yaw_max", 2.4);
		pitch_min_ = declare_parameter<double>("pitch_min", -0.8);
		pitch_max_ = declare_parameter<double>("pitch_max", 0.6);
		yaw_rate_max_ = declare_parameter<double>("yaw_rate_max", 1.2);
		pitch_rate_max_ = declare_parameter<double>("pitch_rate_max", 1.0);
		visual_conf_threshold_ = declare_parameter<double>("visual_conf_threshold", 0.55);
		visual_valid_timeout_s_ = declare_parameter<double>("visual_valid_timeout_s", 0.1);
		visual_loss_to_tracking_s_ = declare_parameter<double>("visual_loss_to_tracking_s", 2.0);
		motion_valid_timeout_s_ = declare_parameter<double>("motion_valid_timeout_s", 0.3);
		control_hz_ = declare_parameter<double>("control_hz", 50.0);

		mode_ = msg::GimbalState::MODE_TRACKING;

		// 订阅来自动作模块（主控发送的）消息
		motion_sub_ = create_subscription<msg::GimbalMotionCommand>(
			"/gimbal/motion_command", rclcpp::SensorDataQoS(),
			[this](const msg::GimbalMotionCommand::SharedPtr msg) { onMotionCommand(msg); });

		// 订阅来自视觉模块（目标处理器发送的）消息
		visual_sub_ = create_subscription<msg::GimbalVisualCommand>(
			"/gimbal/visual_command", rclcpp::SensorDataQoS(),
			[this](const msg::GimbalVisualCommand::SharedPtr msg) { onVisualCommand(msg); });

		// 订阅来自状态模块（主控发送的）消息
		mission_state_sub_ = create_subscription<msg::DroneStatus>(
			"/main_controller/status", 10,
			[this](const msg::DroneStatus::SharedPtr msg) { onMissionStatus(msg); });

		// 发布云台状态消息
		state_pub_ = create_publisher<msg::GimbalState>("/gimbal/state", 10);

		// 提供设置云台模式的服务
		mode_srv_ = create_service<srv::SetGimbalMode>(
			"/gimbal/set_mode",
			[this](const srv::SetGimbalMode::Request::SharedPtr req, srv::SetGimbalMode::Response::SharedPtr res) {
				onSetMode(req, res);
			});

		// 创建控制定时器,运行仲裁系统
		const auto period = std::chrono::duration<double>(1.0 / std::max(control_hz_, 1.0));
		timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(period),
			[this]() { onControlTick(); });

		last_tick_time_ = now();
		RCLCPP_INFO(get_logger(), "gimbal_controller_node started");
	}

private:
	static constexpr double kMinDtSec = 1e-6;
	static constexpr double kLimitEpsilon = 1e-4;

    /**
	 * @brief 动作命令回调函数
	 * @param msg 来自主控的云台动作命令消息
	 * 接收来自主控的云台动作命令消息。
	 */
	void onMotionCommand(const msg::GimbalMotionCommand::SharedPtr msg)
	{
		last_motion_ = *msg;
		has_motion_ = true;
	}

	/**
	 * @brief 视觉命令回调函数
	 * @param msg 来自目标处理器的视觉命令消息
	 * 接收来自目标处理器的视觉命令消息。
	 */
	void onVisualCommand(const msg::GimbalVisualCommand::SharedPtr msg)
	{
		last_visual_ = *msg;
		has_visual_ = true;
	}

	/**
	 * @brief 任务状态回调函数
	 * @param msg 来自主控的任务状态消息
	 * 接收来自主控的任务状态消息，并根据任务模式更新云台模式
	 */
	void onMissionStatus(const msg::DroneStatus::SharedPtr msg)
	{
		if (msg->gimbal_mode == msg::GimbalState::MODE_TRACKING || msg->gimbal_mode == msg::GimbalState::MODE_DETECTING)
		{
			requested_mode_ = msg->gimbal_mode;
		}
	}

	/**
	 * @brief 设置云台模式服务回调函数
	 * @param req 来自服务请求的设置云台模式请求消息
	 * @param res 用于响应服务请求的设置云台模式响应消息
	 * 处理设置云台模式的服务请求，根据请求的模式参数切换云台模式
	 */
	void onSetMode(const srv::SetGimbalMode::Request::SharedPtr req, srv::SetGimbalMode::Response::SharedPtr res)
	{
		if (req->mode != msg::GimbalState::MODE_TRACKING && req->mode != msg::GimbalState::MODE_DETECTING)
		{
			res->success = false;
			res->message = "unsupported mode";
			return;
		}
		requested_mode_ = req->mode;
		res->success = true;
		res->message = "mode switched";
	}

	bool visualValid(const rclcpp::Time &now_time) const
	{
		if (!has_visual_ || last_visual_.confidence < visual_conf_threshold_)
		{
			return false;
		}
		const auto visual_stamp = rclcpp::Time(last_visual_.header.stamp);
		return (now_time - visual_stamp).seconds() <= visual_valid_timeout_s_;
	}

	bool motionValid(const rclcpp::Time &now_time) const
	{
		if (!has_motion_)
		{
			return false;
		}
		const auto motion_stamp = rclcpp::Time(last_motion_.header.stamp);
		return (now_time - motion_stamp).seconds() <= motion_valid_timeout_s_;
	}

	void updateGimbalMode(const rclcpp::Time &now_time, bool visual_ok)
	{
		// 主控未授权视觉接管时，强制跟踪模式
		if (requested_mode_ != msg::GimbalState::MODE_DETECTING)
		{
			mode_ = msg::GimbalState::MODE_TRACKING;
			visual_loss_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
			return;
		}

		mode_ = msg::GimbalState::MODE_DETECTING;
		if (visual_ok)
		{
			visual_loss_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
			return;
		}

		if (visual_loss_start_.nanoseconds() == 0)
		{
			visual_loss_start_ = now_time;
		}
		if ((now_time - visual_loss_start_).seconds() >= visual_loss_to_tracking_s_)
		{
			mode_ = msg::GimbalState::MODE_TRACKING;
			requested_mode_ = msg::GimbalState::MODE_TRACKING;
		}
	}

	void selectTargetAngles(bool visual_ok, double &target_yaw, double &target_pitch) const
	{
		target_yaw = current_yaw_;
		target_pitch = current_pitch_;
		if (mode_ == msg::GimbalState::MODE_DETECTING && visual_ok)
		{
			target_yaw = last_visual_.yaw;
			target_pitch = last_visual_.pitch;
		}
		else if (motionValid(last_tick_time_))
		{
			target_yaw = last_motion_.yaw;
			target_pitch = last_motion_.pitch;
		}
	}

	void applyAngleAndRateLimits(double &target_yaw, double &target_pitch, double dt, double &yaw_rate, double &pitch_rate) const
	{
		target_yaw = std::clamp(target_yaw, yaw_min_, yaw_max_);
		target_pitch = std::clamp(target_pitch, pitch_min_, pitch_max_);
		yaw_rate = std::clamp((target_yaw - current_yaw_) / dt, -yaw_rate_max_, yaw_rate_max_);
		pitch_rate = std::clamp((target_pitch - current_pitch_) / dt, -pitch_rate_max_, pitch_rate_max_);
	}

	void publishState(const rclcpp::Time &stamp, double dt)
	{
		msg::GimbalState state;
		state.header.stamp = stamp;
		state.yaw = static_cast<float>(current_yaw_);
		state.pitch = static_cast<float>(current_pitch_);
		state.yaw_rate = static_cast<float>((current_yaw_ - last_yaw_) / dt);
		state.pitch_rate = static_cast<float>((current_pitch_ - last_pitch_) / dt);
		state.mode = mode_;
		state.yaw_at_limit = std::abs(current_yaw_ - yaw_min_) < kLimitEpsilon ||
							 std::abs(current_yaw_ - yaw_max_) < kLimitEpsilon;
		state.pitch_at_limit = std::abs(current_pitch_ - pitch_min_) < kLimitEpsilon ||
							   std::abs(current_pitch_ - pitch_max_) < kLimitEpsilon;
		state_pub_->publish(state);
	}

	/**
	 * @brief 定时器回调函数 仲裁系统
	 */
	void onControlTick()
	{
		const auto t_now = now();
		double dt = (t_now - last_tick_time_).seconds();
		if (dt <= kMinDtSec)
		{
			dt = 1.0 / std::max(control_hz_, 1.0);
		}
		last_tick_time_ = t_now;

		const bool visual_ok = visualValid(t_now);
		updateGimbalMode(t_now, visual_ok);
		double target_yaw   = current_yaw_;
		double target_pitch = current_pitch_;
		selectTargetAngles(visual_ok, target_yaw, target_pitch);
		double yaw_rate = 0.0;
		double pitch_rate = 0.0;
		applyAngleAndRateLimits(target_yaw, target_pitch, dt, yaw_rate, pitch_rate);

		last_yaw_   = current_yaw_;
		last_pitch_ = current_pitch_;
		current_yaw_   += yaw_rate   * dt;
		current_pitch_ += pitch_rate * dt;
		publishState(t_now, dt);
	}

	rclcpp::Subscription<msg::GimbalMotionCommand>::SharedPtr motion_sub_;
	rclcpp::Subscription<msg::GimbalVisualCommand>::SharedPtr visual_sub_;
	rclcpp::Subscription<msg::DroneStatus>::SharedPtr mission_state_sub_;
	rclcpp::Publisher<msg::GimbalState>::SharedPtr state_pub_;
	rclcpp::Service<srv::SetGimbalMode>::SharedPtr mode_srv_;
	rclcpp::TimerBase::SharedPtr timer_;

	msg::GimbalMotionCommand last_motion_;
	msg::GimbalVisualCommand last_visual_;
	bool has_motion_{false};
	bool has_visual_{false};

	uint8_t mode_{msg::GimbalState::MODE_TRACKING};
	uint8_t requested_mode_{msg::GimbalState::MODE_TRACKING};

	double yaw_min_{-2.4};
	double yaw_max_{2.4};
	double pitch_min_{-0.8};
	double pitch_max_{0.6};
	double yaw_rate_max_{1.2};
	double pitch_rate_max_{1.0};
	double visual_conf_threshold_{0.55};
	double visual_valid_timeout_s_{0.1};
	double visual_loss_to_tracking_s_{2.0};
	double motion_valid_timeout_s_{0.3};
	double control_hz_{100.0};

	double current_yaw_{0.0};
	double current_pitch_{0.0};
	double last_yaw_{0.0};
	double last_pitch_{0.0};

	rclcpp::Time visual_loss_start_{0, 0, RCL_ROS_TIME};
	rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
};

} // namespace c3_drone_driver

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<c3_drone_driver::GimbalControllerNode>());
	rclcpp::shutdown();
	return 0;
}
