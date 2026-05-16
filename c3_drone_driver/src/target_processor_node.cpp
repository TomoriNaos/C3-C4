#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "c3_drone_driver/msg/gimbal_visual_command.hpp"
#include "c3_drone_driver/msg/target_hint.hpp"
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
		TargetFusionProcessor::Config config;
		config.time_sync_threshold_s = declare_parameter<double>("time_sync_threshold_s", 0.1);
		config.pending_wait_s = declare_parameter<double>("pending_wait_s", 0.06);
		config.buffer_keep_s = declare_parameter<double>("buffer_keep_s", 0.2);
		config.default_confidence = declare_parameter<double>("default_confidence", 0.6);
		config.target_type_default = static_cast<uint8_t>(declare_parameter<int>("target_type_default", 0));
		config.visual_cmd_gain = declare_parameter<double>("visual_cmd_gain", 1.0);
		config.roi_margin_px = declare_parameter<int>("roi_margin_px", 16);
		config.image_width = declare_parameter<int>("image_width", 1280);
		config.image_height = declare_parameter<int>("image_height", 720);
		config.depth_min = declare_parameter<double>("depth_min", 0.5);
		config.depth_max = declare_parameter<double>("depth_max", 120.0);
		config.min_roi_points = static_cast<std::size_t>(declare_parameter<int>("min_roi_points", 20));
		config.low_conf_threshold = declare_parameter<double>("low_conf_threshold", 0.55);
		config.tc_noise_std = declare_parameter<double>("tc_noise_std", 0.10);
		config.gc_noise_std = declare_parameter<double>("gc_noise_std", 0.05);
		config.smoothing_alpha = declare_parameter<double>("smoothing_alpha", 0.35);
		config.max_tracking_loss_frames = static_cast<std::size_t>(declare_parameter<int>("max_tracking_loss_frames", 30));
		config.confidence_weight_detection = declare_parameter<double>("confidence_weight_detection", 0.5);
		config.confidence_weight_cluster = declare_parameter<double>("confidence_weight_cluster", 0.2);
		config.confidence_weight_stability = declare_parameter<double>("confidence_weight_stability", 0.3);

		processor_ = std::make_unique<TargetFusionProcessor>(config, get_logger());

		tc_bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
			"/tc/bbox", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
				processor_->updateTcBbox(msg, now());
				process();
			});
		tc_pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
			"/tc/points", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
				processor_->updateTcPointCloud(msg);
				process();
			});
		gc_pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
			"/gc/points", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
				processor_->updateGcPointCloud(msg);
				process();
			});
		target_hint_sub_ = create_subscription<msg::TargetHint>(
			"/mission/target_hint", 10, [this](const msg::TargetHint::SharedPtr msg) {
				processor_->updateTargetHint(msg);
			});

		observation_pub_ = create_publisher<msg::TargetObservation>("/target/observation_body", 10);
		mavlink_observation_pub_ = create_publisher<msg::TargetObservation>("/mavlink/target_obs", 10);
		visual_cmd_pub_ = create_publisher<msg::GimbalVisualCommand>("/gimbal/visual_command", 10);
		lost_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/target/fusion_lost", 10);

		timer_ = create_wall_timer(
			std::chrono::milliseconds(20),
			[this]() {
				process();
			});

		RCLCPP_INFO(get_logger(), "target_processor_node started");
	}

private:
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
			mavlink_observation_pub_->publish(result->observation);
		}
		if (result->has_visual_command)
		{
			visual_cmd_pub_->publish(result->gimbal_command);
		}
		if (result->lost)
		{
			std_msgs::msg::Float32MultiArray lost_msg;
			lost_msg.data = {1.0F};
			lost_pub_->publish(lost_msg);
		}
	}

	std::unique_ptr<TargetFusionProcessor> processor_;

	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tc_bbox_sub_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tc_pc_sub_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr gc_pc_sub_;
	rclcpp::Subscription<msg::TargetHint>::SharedPtr target_hint_sub_;

	rclcpp::Publisher<msg::TargetObservation>::SharedPtr observation_pub_;
	rclcpp::Publisher<msg::TargetObservation>::SharedPtr mavlink_observation_pub_;
	rclcpp::Publisher<msg::GimbalVisualCommand>::SharedPtr visual_cmd_pub_;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lost_pub_;
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
