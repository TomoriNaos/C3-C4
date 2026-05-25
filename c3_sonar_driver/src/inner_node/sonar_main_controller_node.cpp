#include <algorithm>
#include <string>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "c3_sonar_driver/msg/sonar_status.hpp"
#include "c3_sonar_driver/srv/set_sonar_lifecycle.hpp"

namespace c3_sonar_driver
{

	class SonarMainControllerNode : public rclcpp::Node
	{
	public:
		SonarMainControllerNode()
			: Node("sonar_main_controller_node")
		{
			sonar_lifecycle_service_name_ = declare_parameter<std::string>(
				"sonar_lifecycle_service_name", "/sonar_processor_node/change_state");
			status_hz_ = declare_parameter<double>("status_hz", 2.0);

			sonar_lifecycle_client_ = create_client<lifecycle_msgs::srv::ChangeState>(
				sonar_lifecycle_service_name_);

			lifecycle_srv_ = create_service<srv::SetSonarLifecycle>(
				"/main_controller/set_lifecycle",
				[this](const srv::SetSonarLifecycle::Request::SharedPtr req,
					   srv::SetSonarLifecycle::Response::SharedPtr res)
				{ onSetLifecycle(req, res); });

			status_pub_ = create_publisher<msg::SonarStatus>("/main_controller/status", 10);

			const auto period = std::chrono::duration<double>(1.0 / std::max(status_hz_, 0.2));
			timer_ = create_wall_timer(
				std::chrono::duration_cast<std::chrono::nanoseconds>(period),
				[this]()
				{ publishStatus(); });

			RCLCPP_INFO(get_logger(), "sonar_main_controller_node started");
		}

	private:
		void onSetLifecycle(
			const srv::SetSonarLifecycle::Request::SharedPtr req,
			srv::SetSonarLifecycle::Response::SharedPtr res)
		{
			if (!req || !res)
			{
				return;
			}

			bool accepted = true;
			std::string message = "ok";

			switch (req->command)
			{
			case srv::SetSonarLifecycle::Request::CMD_ACTIVATE:
				sonar_active_ = true;
				requestSonarLifecycle(true);
				break;
			case srv::SetSonarLifecycle::Request::CMD_DEACTIVATE:
				sonar_active_ = false;
				requestSonarLifecycle(false);
				break;
			default:
				accepted = false;
				message = "unsupported lifecycle command";
				break;
			}

			res->success = accepted;
			res->message = message;
		}

		void requestSonarLifecycle(bool enable)
		{
			if (!sonar_lifecycle_client_->wait_for_service(std::chrono::milliseconds(200)))
			{
				RCLCPP_WARN(get_logger(), "Lifecycle service not ready: %s", sonar_lifecycle_service_name_.c_str());
				return;
			}

			auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
			req->transition.id = enable
									 ? lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE
									 : lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
			sonar_lifecycle_client_->async_send_request(req);
		}

		void publishStatus()
		{
			msg::SonarStatus status;
			const auto stamp = now();
			status.header.stamp = stamp;
			status.t_usec = static_cast<uint64_t>(stamp.nanoseconds() / 1000ULL);
			status.link_state = msg::SonarStatus::LINK_OK;
			status.sonar_active = sonar_active_;
			status.estimated_sound_speed_mps = 1500.0F;
			status.estimated_latency_ms = 0.0F;
			status_pub_->publish(status);
		}

		rclcpp::Service<srv::SetSonarLifecycle>::SharedPtr lifecycle_srv_;
		rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr sonar_lifecycle_client_;
		rclcpp::Publisher<msg::SonarStatus>::SharedPtr status_pub_;
		rclcpp::TimerBase::SharedPtr timer_;

		std::string sonar_lifecycle_service_name_{"/sonar_processor_node/change_state"};
		double status_hz_{2.0};
		bool sonar_active_{false};
	};

} // namespace c3_sonar_driver

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<c3_sonar_driver::SonarMainControllerNode>());
	rclcpp::shutdown();
	return 0;
}
