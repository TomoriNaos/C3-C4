#pragma once

#include <cstdint>
#include <deque>
#include <optional>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "c3_drone_driver/msg/gimbal_visual_command.hpp"
#include "c3_drone_driver/msg/target_hint.hpp"
#include "c3_drone_driver/msg/target_observation.hpp"

namespace c3_drone_driver
{

class TargetFusionProcessor
{
public:
	struct Config
	{
		double time_sync_threshold_s{0.1};
		double pending_wait_s{0.06};
		double buffer_keep_s{0.2};
		double default_confidence{0.6};
		uint8_t target_type_default{0};
		double visual_cmd_gain{1.0};
		int roi_margin_px{16};
		int image_width{1280};
		int image_height{720};
		double depth_min{0.5};
		double depth_max{120.0};
		std::size_t min_roi_points{20};
		double low_conf_threshold{0.55};
		double tc_noise_std{0.10};
		double gc_noise_std{0.05};
		double smoothing_alpha{0.35};
		std::size_t max_tracking_loss_frames{30};
		double confidence_weight_detection{0.5};
		double confidence_weight_cluster{0.2};
		double confidence_weight_stability{0.3};
	};

	struct Result
	{
		msg::TargetObservation observation;
		msg::GimbalVisualCommand gimbal_command;
		bool has_observation{false};
		bool has_visual_command{false};
		bool lost{false};
	};

	explicit TargetFusionProcessor(
		const Config &config,
		const rclcpp::Logger &logger = rclcpp::get_logger("TargetFusionProcessor"));

	void updateTcBbox(const std_msgs::msg::Float32MultiArray::SharedPtr &msg, const rclcpp::Time &stamp);
	void updateTcPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
	void updateGcPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
	void updateTargetHint(const msg::TargetHint::SharedPtr &msg);
	std::optional<Result> process(const rclcpp::Time &now);

private:
	struct TimestampedBbox
	{
		rclcpp::Time stamp;
		std_msgs::msg::Float32MultiArray data;
	};

	struct TimestampedCloud
	{
		rclcpp::Time stamp;
		sensor_msgs::msg::PointCloud2::SharedPtr cloud;
	};

	struct RoiResult
	{
		geometry_msgs::msg::Point centroid;
		double quality{0.0};
		std::size_t point_count{0};
	};

	struct RoiBounds
	{
		int x_min{0};
		int x_max{0};
		int y_min{0};
		int y_max{0};
	};

	const TimestampedBbox *latestBbox() const;
	std::optional<TimestampedCloud> findNearestCloud(const std::deque<TimestampedCloud> &buffer, const rclcpp::Time &stamp) const;
	void pruneBuffers(const rclcpp::Time &now);
	bool withinSyncThreshold(const rclcpp::Time &lhs, const rclcpp::Time &rhs) const;
	void parseBboxMeta(
		const std_msgs::msg::Float32MultiArray &data,
		uint32_t &target_id,
		float &detection_conf,
		uint8_t &target_type) const;
	std::optional<RoiBounds> buildRoi(const std_msgs::msg::Float32MultiArray &data) const;
	std::optional<RoiResult> extractRoiCentroid(const sensor_msgs::msg::PointCloud2 &cloud, const RoiBounds &roi) const;
	void updateTrack(const geometry_msgs::msg::Point &observed, const rclcpp::Time &now, bool has_observation);
	geometry_msgs::msg::Point predict(const rclcpp::Time &now) const;
	msg::TargetObservation buildObservation(
		const rclcpp::Time &stamp,
		uint32_t target_id,
		uint8_t target_type,
		float detection_conf,
		const geometry_msgs::msg::Point &position,
		double cluster_quality,
		uint8_t source,
		bool lost);
	msg::GimbalVisualCommand buildVisualCommand(const msg::TargetObservation &obs) const;
	std::optional<Result> buildLostResult(const rclcpp::Time &now, const std_msgs::msg::Float32MultiArray &bbox);
	double computeConfidence(float detection_conf, double cluster_quality, double stability, bool lost) const;
	uint64_t buildKey(
		const rclcpp::Time &bbox_stamp,
		const rclcpp::Time &tc_stamp,
		const rclcpp::Time &gc_stamp,
		uint8_t status) const;
	bool shouldSuppress(const msg::TargetObservation &obs);

	Config config_;
	rclcpp::Logger logger_;

	std::deque<TimestampedBbox> tc_bbox_buffer_;
	std::deque<TimestampedCloud> tc_pc_buffer_;
	std::deque<TimestampedCloud> gc_pc_buffer_;

	bool has_target_hint_{false};
	uint8_t target_type_hint_{0};
	rclcpp::Time target_hint_stamp_{0, 0, RCL_ROS_TIME};

	bool track_initialized_{false};
	geometry_msgs::msg::Point track_position_{};
	geometry_msgs::msg::Point track_velocity_{};
	double stability_{0.5};
	std::size_t loss_frames_{0};
	rclcpp::Time last_track_time_{0, 0, RCL_ROS_TIME};

	uint32_t obs_id_{0};
	uint32_t track_id_{1};
	uint64_t last_emitted_key_{std::numeric_limits<uint64_t>::max()};
};

} // namespace c3_drone_driver

