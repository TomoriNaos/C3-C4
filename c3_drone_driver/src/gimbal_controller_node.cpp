#include <algorithm>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "c3_drone_driver/msg/gimbal_motion_command.hpp"
#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/gimbal_visual_command.hpp"
#include "c3_drone_driver/srv/set_gimbal_mode.hpp"

namespace c3_drone_driver
{
	/**
	 * @brief 云台控制节点
	 * 云台是一个（yaw,pitch）两轴旋转系统 
	 * 其接收到自身运动模块发来的运动指令（msg），以及视觉模块发布的视觉指令（msg）。经过仲裁后，旋转相应角度以完成目标跟踪
	 */
	class GimbalControllerNode : public rclcpp::Node
	{
	public:
		GimbalControllerNode()
			: Node("gimbal_controller_node")
		{
			yaw_min_ = declare_parameter<double>("yaw_min", -2.4);
			yaw_max_ = declare_parameter<double>("yaw_max", 2.4);
			pitch_min_ = declare_parameter<double>("pitch_min", -0.8);
			pitch_max_ = declare_parameter<double>("pitch_max", 0.6);
			yaw_rate_max_ = declare_parameter<double>("yaw_rate_max", 1.2);
			pitch_rate_max_ = declare_parameter<double>("pitch_rate_max", 1.0);
			visual_conf_threshold_ = declare_parameter<double>("visual_conf_threshold", 0.55);
			visual_valid_timeout_s_ = declare_parameter<double>("visual_valid_timeout_s", 2.0);
			control_hz_ = declare_parameter<double>("control_hz", 100.0);

			// 运动指令接收
			motion_sub_ = create_subscription<msg::GimbalMotionCommand>(
				"/gimbal/motion_command", rclcpp::SensorDataQoS(),
				std::bind(&GimbalControllerNode::onMotionCommand, this, std::placeholders::_1));

			// 视觉指令接收
			visual_sub_ = create_subscription<msg::GimbalVisualCommand>(
				"/gimbal/visual_command", rclcpp::SensorDataQoS(),
				std::bind(&GimbalControllerNode::onVisualCommand, this, std::placeholders::_1));
			
			// 云台状态发布
			state_pub_ = create_publisher<msg::GimbalState>("/gimbal/state", 10);

			// 模式切换服务
			mode_srv_ = create_service<srv::SetGimbalMode>(
				"/gimbal/set_mode",
				std::bind(&GimbalControllerNode::onSetMode, this,std::placeholders::_1, std::placeholders::_2));

			// 云台默认处于跟踪态，姿态回正
			mode_ = msg::GimbalState::MODE_TRACKING;
			current_yaw_ = 0.0;
			current_pitch_ = 0.0;
			last_yaw_ = 0.0;
			last_pitch_ = 0.0;

			// 循环计时器
			const auto period = std::chrono::duration<double>(1.0 / std::max(control_hz_, 1.0));
			// 定时器回调函数为onControlTick(仲裁系统)
			timer_ = create_wall_timer(
				std::chrono::duration_cast<std::chrono::nanoseconds>(period),
				std::bind(&GimbalControllerNode::onControlTick, this));

			last_tick_time_ = now();
			RCLCPP_INFO(get_logger(), "gimbal_controller_node started");
		}

	private:
		/** 
		 * @brief 运动指令回调，获得运动消息
		 * @param msg 运动指令消息
		 * 对应GimbalMotionCommand
		*/
		void onMotionCommand(const msg::GimbalMotionCommand::SharedPtr msg)
		{
			last_motion_ = *msg;
			has_motion_ = true;
		}

		/**
		 * @brief 视觉指令回调，获得视觉消息
		 * @param msg 视觉指令消息
		 * 对应GimbalVisualCommand
		 */
		void onVisualCommand(const msg::GimbalVisualCommand::SharedPtr msg)
		{
			last_visual_ = *msg;
			has_visual_ = true;
		}

		/**
		 * @brief 服务端切换云台模式
		 */
		void onSetMode(
			const srv::SetGimbalMode::Request::SharedPtr req,
			srv::SetGimbalMode::Response::SharedPtr res)
		{
			if (req->mode != msg::GimbalState::MODE_TRACKING &&
				req->mode != msg::GimbalState::MODE_DETECTING)
			{
				res->success = false;
				res->message = "unsupported mode";
				return;
			}

			mode_ = req->mode;
			res->success = true;
			res->message = "mode switched";
		}

		bool visualValid(const rclcpp::Time &now_time) const
		{
			if (!has_visual_)
			{
				return false;
			}
			if (last_visual_.confidence < visual_conf_threshold_)
			{
				return false;
			}
			const auto visual_stamp = rclcpp::Time(last_visual_.header.stamp);
			return (now_time - visual_stamp).seconds() <= visual_valid_timeout_s_;
		}

		/**
		 * @brief 云台控制循环，定时执行
			 * 根据当前模式和指令，计算目标角度，并以一定速率旋转云台
			 * 最后发布当前云台状态
			 * 其中，视觉指令优先级高于运动指令；在检测模式下，如果视觉指令不可用，则保持当前姿态不变；在跟踪模式下，如果视觉指令不可用，则使用运动指令进行控制
			 * 姿态更新时会考虑角度范围和速率限制
		 */
			void onControlTick()
			{
				// 1) 计算控制周期 dt，用于后续角速度限幅与状态反馈。
				const auto t_now = now();
				double dt = (t_now - last_tick_time_).seconds();
				if (dt <= 1e-6)
				{
					dt = 1.0 / std::max(control_hz_, 1.0);
				}
				last_tick_time_ = t_now;

				// 2) 判断视觉命令当前是否有效：
				//    - 已经收到过视觉消息
				//    - 置信度高于阈值
				//    - 时间戳未超时
				const bool visual_ok = visualValid(t_now);

				// 3) 模式管理：
				//    当前处于 DETECTING 但视觉失效时，开始累计“视觉丢失时间”。
				//    若连续丢失超过阈值，则自动退回 TRACKING，
				//    让云台重新服从运动模块前馈命令。
				if (mode_ == msg::GimbalState::MODE_DETECTING && !visual_ok)
				{
					if (visual_loss_start_.nanoseconds() == 0)
					{
						visual_loss_start_ = t_now;
				}
				if ((t_now - visual_loss_start_).seconds() >= visual_valid_timeout_s_)
				{
						mode_ = msg::GimbalState::MODE_TRACKING;
					}
				}
				// 只要当前不满足“DETECTING 且视觉失效”，就清空丢失计时器。
				// 这里包括两种情况：
				// - 视觉重新恢复有效
				// - 当前本来就不在 DETECTING 模式
				else
				{
					visual_loss_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
				}

				// 4) 先默认“保持当前姿态不动”。
				//    如果后续没有拿到可用控制源，就不会产生突变。
				double target_yaw = current_yaw_;
				double target_pitch = current_pitch_;

				// 5) 仲裁逻辑：
				//    - 只有在 DETECTING 模式且视觉有效时，视觉命令才真正接管云台
				//    - 否则退回运动模块前馈命令
				//    - 如果运动命令也还没收到，则保持当前姿态
				if (mode_ == msg::GimbalState::MODE_DETECTING && visual_ok)
				{
					target_yaw = last_visual_.yaw;
					target_pitch = last_visual_.pitch;
				}
			else if (has_motion_)
			{
				target_yaw = last_motion_.yaw;
					target_pitch = last_motion_.pitch;
				}

				// 6) 软件角度限位，保证目标角不会超出结构允许范围。
				target_yaw = std::clamp(target_yaw, yaw_min_, yaw_max_);
				target_pitch = std::clamp(target_pitch, pitch_min_, pitch_max_);

				// 7) 角速度限幅：
				//    先按“当前位置 -> 目标角”的误差计算理想角速度，
				//    再限制在参数给定的最大角速度内，避免云台抖动或突跳。
				const double yaw_rate = std::clamp((target_yaw - current_yaw_) / dt, -yaw_rate_max_, yaw_rate_max_);
				const double pitch_rate = std::clamp((target_pitch - current_pitch_) / dt, -pitch_rate_max_, pitch_rate_max_);

				// 8) 根据限幅后的角速度，积分更新当前云台姿态。
				last_yaw_ = current_yaw_;
				last_pitch_ = current_pitch_;
				current_yaw_ += yaw_rate * dt;
				current_pitch_ += pitch_rate * dt;

				// 9) 发布状态：
				//    对外反馈当前姿态、角速度、控制模式以及是否打到软限位，
				//    供运动模块/融合模块做坐标变换和状态判断。
				msg::GimbalState state;
				state.header.stamp = t_now;
				state.yaw = static_cast<float>(current_yaw_);
				state.pitch = static_cast<float>(current_pitch_);
			state.yaw_rate = static_cast<float>((current_yaw_ - last_yaw_) / dt);
			state.pitch_rate = static_cast<float>((current_pitch_ - last_pitch_) / dt);
			state.mode = mode_;
			state.yaw_at_limit = std::abs(current_yaw_ - yaw_min_) < 1e-4 || std::abs(current_yaw_ - yaw_max_) < 1e-4;
			state.pitch_at_limit = std::abs(current_pitch_ - pitch_min_) < 1e-4 || std::abs(current_pitch_ - pitch_max_) < 1e-4;

			state_pub_->publish(state);
		}

		rclcpp::Subscription<msg::GimbalMotionCommand>::SharedPtr motion_sub_;
		rclcpp::Subscription<msg::GimbalVisualCommand>::SharedPtr visual_sub_;
		rclcpp::Publisher<msg::GimbalState>::SharedPtr state_pub_;
		rclcpp::Service<srv::SetGimbalMode>::SharedPtr mode_srv_;
		rclcpp::TimerBase::SharedPtr timer_;

		msg::GimbalMotionCommand last_motion_;
		msg::GimbalVisualCommand last_visual_;
		bool has_motion_ = false;
		bool has_visual_ = false;

		uint8_t mode_{msg::GimbalState::MODE_TRACKING};

		double yaw_min_{-2.4};
		double yaw_max_{2.4};
		double pitch_min_{-0.8};
		double pitch_max_{0.6};
		double yaw_rate_max_{1.2};
		double pitch_rate_max_{1.0};
		double visual_conf_threshold_{0.55};
		double visual_valid_timeout_s_{2.0};
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
