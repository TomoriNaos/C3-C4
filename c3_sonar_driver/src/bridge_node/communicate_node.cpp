#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "c3_sonar_driver/msg/sonar_detect.hpp"
#include "c3_sonar_driver/msg/sonar_status.hpp"

namespace c3_sonar_driver
{

	class CommunicateNode : public rclcpp::Node
	{
	public:
		CommunicateNode()
			: Node("communicate_node")
		{
			// config/conmmunicate.yaml
			status_hz_ = declare_parameter<double>("status_hz", 1.0);
			detect_valid_timeout_s_ = declare_parameter<double>("detect_valid_timeout_s", 2.0);

			//接受声呐检测结果 
			sonar_detect_sub_ = create_subscription<msg::SonarDetect>(
				"/sonar/detect", 10,
				[this](const msg::SonarDetect::SharedPtr msg)
				{ onSonarDetect(msg); });

			//接受声呐主控状态
			sonar_status_sub_ = create_subscription<msg::SonarStatus>(
				"/main_controller/status", 10,
				[this](const msg::SonarStatus::SharedPtr msg){ 
					if (!msg) return;
					main_status_ = *msg;
					has_main_status_ = true; });

			//============ 向母船发送检测结果 ==============
			ship_tx_detect_pub_ = create_publisher<msg::SonarDetect>("/sonar_link/ship_tx/detect", 10);
			ship_tx_status_pub_ = create_publisher<msg::SonarStatus>("/sonar_link/ship_tx/status", 10);

			const auto status_period = std::chrono::duration<double>(1.0 / std::max(status_hz_, 0.2));
			status_timer_ = create_wall_timer(
				std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
				[this]()
				{ onStatusTimer(); });

			RCLCPP_INFO(get_logger(), "communicate_node started");
		}

	private:
		void onSonarDetect(const msg::SonarDetect::SharedPtr msg)
		{
			if (!msg) return;

			const auto stamp = rclcpp::Time(msg->header.stamp);
			if ((now() - stamp).seconds() > detect_valid_timeout_s_) return;

			if (has_last_detect_ && msg->detect_id == last_detect_id_) return;

			last_detect_id_ = msg->detect_id;
			has_last_detect_ = true;
			ship_tx_detect_pub_->publish(*msg);
		}

		void onStatusTimer()
		{
			if (!has_main_status_) return;

			const auto t_now = now();

			msg::SonarStatus status = main_status_;
			status.header.stamp = t_now;
			status.t_usec = static_cast<uint64_t>(t_now.nanoseconds() / 1000ULL);

			const bool status_changed = !has_last_status_ ||
			                            status.sonar_active != last_status_.sonar_active;

			if (!status_changed) return;

			ship_tx_status_pub_->publish(status);
			last_status_ = status;
			has_last_status_ = true;
		}

		rclcpp::Subscription<msg::SonarDetect>::SharedPtr sonar_detect_sub_;
		rclcpp::Subscription<msg::SonarStatus>::SharedPtr sonar_status_sub_;

		rclcpp::Publisher<msg::SonarDetect>::SharedPtr ship_tx_detect_pub_;
		rclcpp::Publisher<msg::SonarStatus>::SharedPtr ship_tx_status_pub_;

		rclcpp::TimerBase::SharedPtr status_timer_;

		double status_hz_{1.0};
		double detect_valid_timeout_s_{2.0};


		uint32_t last_detect_id_{0};
		bool has_last_detect_{false};
		bool has_main_status_{false};
		bool has_last_status_{false};

		msg::SonarStatus main_status_{};
		msg::SonarStatus last_status_{};
	};

} // namespace c3_sonar_driver

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<c3_sonar_driver::CommunicateNode>());
	rclcpp::shutdown();
	return 0;
}