#include <chrono>
#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "c3_drone_driver/config.h"
#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/gimbal_visual_command.hpp"
#include "c3_drone_driver/msg/tc_detection.hpp"
#include "c3_drone_driver/msg/target_observation.hpp"
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

		// TargetFusionProcessor 实例化
		processor_ = std::make_unique<TargetFusionProcessor>(config);

		// tc数据接收器
		tc_detection_sub_ = create_subscription<msg::TcDetection>(
			"/tc/detection", rclcpp::SensorDataQoS(), [this](const msg::TcDetection::SharedPtr msg) {
				processor_->updateTcDetection(msg);
			});

		// gc数据接收器（bbox + cloud）
		gc_detection_sub_ = create_subscription<msg::TcDetection>(
			"/gc/detection", rclcpp::SensorDataQoS(), [this](const msg::TcDetection::SharedPtr msg) {
				processor_->updateGcDetection(msg);
			});

		gimbal_state_sub_ = create_subscription<msg::GimbalState>(
			"/gimbal/state", rclcpp::SensorDataQoS(), [this](const msg::GimbalState::SharedPtr msg) {
				processor_->updateGimbalState(msg);
			});

		// 数据发布器
		observation_pub_ = create_publisher<msg::TargetObservation>("/target/observation_body", 10);

		// 基于观测结果生成的云台控制指令发布器
		visual_cmd_pub_ = create_publisher<msg::GimbalVisualCommand>("/gimbal/visual_command", 10);

		// 定时处理器，每20ms调用一次process函数，确保在没有新数据到达时也能及时更新状态（如目标丢失）
		timer_ = create_wall_timer(
			std::chrono::milliseconds(20),
			[this]() {
				process();
			});

		RCLCPP_INFO(get_logger(), "target_processor_node started");
	}

private:

	/**
	 * @brief 处理函数，调用TargetFusionProcessor的process方法获取融合结果，并根据结果发布相应的消息
	 */
	void process()
	{
		const auto result = processor_->process(now());
		if (!result.has_value())
		{
			return;
		}

		if (result->has_observation)
		{
			observation_pub_->publish(result->observation);
		}
		if (result->has_visual_command)
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
	rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace c3_drone_driver

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<c3_drone_driver::TargetProcessorNode>());
	rclcpp::shutdown();
	return 0;
}
