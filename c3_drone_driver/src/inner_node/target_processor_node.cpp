#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "c3_drone_driver/config.h"
#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/gimbal_visual_command.hpp"
#include "c3_drone_driver/msg/tc_detection.hpp"
#include "c3_drone_driver/msg/target_observation.hpp"
#include "c3_drone_driver/srv/set_gimbal_mode.hpp"
#include "c3_drone_driver/target_fusion_processor.h"

namespace c3_drone_driver
{

class TargetProcessorNode : public rclcpp::Node
{
public:
	TargetProcessorNode()
	: Node("target_processor_node")
	{
		const std::string config_file =
			ament_index_cpp::get_package_share_directory("c3_drone_driver") +
			"/config/target_fusion_default.yaml";
		if (!Config::SetParameterFile(config_file))
		{
			RCLCPP_WARN(get_logger(), "Failed to load %s, fallback to built-in defaults", config_file.c_str());
		}

		//config/target_fusion_default.yaml
		TargetFusionProcessor::Config config;
		config.time_sync_threshold_s = Config::GetOr<double>("time_sync_threshold_s", 0.1);
		config.pending_wait_s = Config::GetOr<double>("pending_wait_s", 0.06);
		config.buffer_keep_s = Config::GetOr<double>("buffer_keep_s", 0.2);
		config.default_confidence = Config::GetOr<double>("default_confidence", 0.6);
		config.visual_cmd_gain = Config::GetOr<double>("visual_cmd_gain", 1.0);
		config.roi_margin_px = Config::GetOr<int>("roi_margin_px", 16);
		config.image_width = Config::GetOr<int>("image_width", 1280);
		config.image_height = Config::GetOr<int>("image_height", 720);
		config.body_to_gimbal_x = Config::GetOr<double>("body_to_gimbal_x", 0.15);
		config.body_to_gimbal_y = Config::GetOr<double>("body_to_gimbal_y", 0.0);
		config.body_to_gimbal_z = Config::GetOr<double>("body_to_gimbal_z", -0.05);
		config.tc_to_gimbal_x = Config::GetOr<double>("tc_to_gimbal_x", 0.05);
		config.tc_to_gimbal_y = Config::GetOr<double>("tc_to_gimbal_y", 0.02);
		config.tc_to_gimbal_z = Config::GetOr<double>("tc_to_gimbal_z", 0.0);
		config.gc_to_gimbal_x = Config::GetOr<double>("gc_to_gimbal_x", 0.05);
		config.gc_to_gimbal_y = Config::GetOr<double>("gc_to_gimbal_y", -0.02);
		config.gc_to_gimbal_z = Config::GetOr<double>("gc_to_gimbal_z", 0.0);
		config.depth_min = Config::GetOr<double>("depth_min", 0.5);
		config.depth_max = Config::GetOr<double>("depth_max", 120.0);
		config.min_roi_points = static_cast<std::size_t>(Config::GetOr<int>("min_roi_points", 20));
		config.low_conf_threshold = Config::GetOr<double>("low_conf_threshold", 0.55);
		config.tc_noise_std = Config::GetOr<double>("tc_noise_std", 0.10);
		config.gc_noise_std = Config::GetOr<double>("gc_noise_std", 0.05);
		config.smoothing_alpha = Config::GetOr<double>("smoothing_alpha", 0.35);
		config.max_tracking_loss_frames = static_cast<std::size_t>(Config::GetOr<int>("max_tracking_loss_frames", 30));
		config.confidence_weight_detection = Config::GetOr<double>("confidence_weight_detection", 0.5);
		config.confidence_weight_cluster = Config::GetOr<double>("confidence_weight_cluster", 0.2);
		config.confidence_weight_stability = Config::GetOr<double>("confidence_weight_stability", 0.3);

		mode_request_min_interval_s_ = declare_parameter<double>("mode_request_min_interval_s", 0.2);
		gimbal_mode_service_name_ = declare_parameter<std::string>("gimbal_mode_service_name", "/gimbal/set_mode");

		// target_fusion_processor实例化
		processor_ = std::make_unique<TargetFusionProcessor>(config);

		// ============ 摄像机通信接口 =============
		// traditional camera (TC)
		tc_detection_sub_ = create_subscription<msg::TcDetection>(
			"/tc/detection", rclcpp::SensorDataQoS(),
			[this](const msg::TcDetection::SharedPtr msg) { processor_->updateTcDetection(msg); });

		// gated camera (GC)
		gc_detection_sub_ = create_subscription<msg::TcDetection>(
			"/gc/detection", rclcpp::SensorDataQoS(),
			[this](const msg::TcDetection::SharedPtr msg) { processor_->updateGcDetection(msg); });

		// =========== gimbal_controller_node通信接口 ============
		// 云台状态接收器
		gimbal_state_sub_ = create_subscription<msg::GimbalState>(
			"/gimbal/state", rclcpp::SensorDataQoS(),
			[this](const msg::GimbalState::SharedPtr msg) { 		
				if (!msg) return;
				processor_->updateGimbalState(msg); });

		// 观测发布接口
		observation_pub_ = create_publisher<msg::TargetObservation>("/target/observation_body", 10);
		
		// 视觉命令发布接口
		visual_cmd_pub_ = create_publisher<msg::GimbalVisualCommand>("/gimbal/visual_command", 10);
		
		// 云台模式请求客户端
		gimbal_mode_client_ = create_client<srv::SetGimbalMode>(gimbal_mode_service_name_);

		// 定时处理器
		timer_ = create_wall_timer(
			std::chrono::milliseconds(20),
			[this]() { process(); });

		RCLCPP_INFO(get_logger(), "target_processor_node started");
	}

	private:

	/// 请求云台进入指定模式
	void requestGimbalMode(uint8_t mode, const rclcpp::Time &stamp)
	{
		// 1.不是同一次请求，并且距离上次请求已经超过最小间隔，才发出新请求
		if (last_requested_mode_.has_value() && *last_requested_mode_ == mode &&
			last_mode_request_time_.nanoseconds() != 0 &&
			(stamp - last_mode_request_time_).seconds() < mode_request_min_interval_s_)
		{
			return;
		}

		// 2.请求云台模式服务，如果服务不可用则跳过请求
		if (!gimbal_mode_client_->service_is_ready())
		{
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 2000,
				"Gimbal mode service not ready: %s", gimbal_mode_service_name_.c_str());
			return;
		}

		auto req = std::make_shared<srv::SetGimbalMode::Request>();
		req->mode = mode;
		(void)gimbal_mode_client_->async_send_request(req);
		last_requested_mode_ = mode;
		last_mode_request_time_ = stamp;
	}

	void process()
	{
		const auto stamp = now();
		const auto result = processor_->process(stamp);
		if (!result.has_value()) return;

		if (result->has_observation)
		{
			observation_pub_->publish(result->observation);
			if (result->observation.status == msg::TargetObservation::STATUS_VALID)
			{
				requestGimbalMode(msg::GimbalState::MODE_DETECTING, stamp);
			}
		}

		if (result->has_visual_command && !result->lost)
		{
			visual_cmd_pub_->publish(result->gimbal_command);
		}
	}

	std::unique_ptr<TargetFusionProcessor> processor_;

	rclcpp::Subscription<msg::TcDetection>::SharedPtr tc_detection_sub_;
	rclcpp::Subscription<msg::TcDetection>::SharedPtr gc_detection_sub_;
	rclcpp::Subscription<msg::GimbalState>::SharedPtr gimbal_state_sub_;

	rclcpp::Publisher<msg::TargetObservation>::SharedPtr observation_pub_;
	rclcpp::Publisher<msg::GimbalVisualCommand>::SharedPtr visual_cmd_pub_;
	rclcpp::Client<srv::SetGimbalMode>::SharedPtr gimbal_mode_client_;
	rclcpp::TimerBase::SharedPtr timer_;

	std::string gimbal_mode_service_name_{"/gimbal/set_mode"};
	double mode_request_min_interval_s_{0.2};
	rclcpp::Time last_mode_request_time_{0, 0, RCL_ROS_TIME};
	std::optional<uint8_t> last_requested_mode_{};
};

} // namespace c3_drone_driver

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<c3_drone_driver::TargetProcessorNode>());
	rclcpp::shutdown();
	return 0;
}
