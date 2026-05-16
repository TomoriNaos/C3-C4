#include "c3_drone_driver/target_fusion_processor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace c3_drone_driver
{

TargetFusionProcessor::TargetFusionProcessor(const Config &config, const rclcpp::Logger &logger)
: config_(config), logger_(logger)
{
}

void TargetFusionProcessor::updateTcBbox(const std_msgs::msg::Float32MultiArray::SharedPtr &msg, const rclcpp::Time &stamp)
{
	if (!msg) return;
	tc_bbox_buffer_.push_back({stamp, *msg});
	pruneBuffers(stamp);
}

void TargetFusionProcessor::updateTcPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
{
	if (!msg) return;
	const rclcpp::Time stamp(msg->header.stamp);
	tc_pc_buffer_.push_back({stamp, msg});
	pruneBuffers(stamp);
}

void TargetFusionProcessor::updateGcPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
{
	if (!msg) return;
	const rclcpp::Time stamp(msg->header.stamp);
	gc_pc_buffer_.push_back({stamp, msg});
	pruneBuffers(stamp);
}

void TargetFusionProcessor::updateTargetHint(const msg::TargetHint::SharedPtr &msg)
{
	if (!msg) return;
	target_type_hint_ = msg->target_type_hint;
	target_hint_stamp_ = rclcpp::Time(msg->header.stamp);
	has_target_hint_ = true;
}

std::optional<TargetFusionProcessor::Result> TargetFusionProcessor::process(const rclcpp::Time &now)
{
	pruneBuffers(now);
	const auto *bbox = latestBbox();
	if (!bbox) return std::nullopt;

	const auto tc_match = findNearestCloud(tc_pc_buffer_, bbox->stamp);
	const auto gc_match = findNearestCloud(gc_pc_buffer_, bbox->stamp);
	const bool tc_ready = tc_match && withinSyncThreshold(tc_match->stamp, bbox->stamp);
	const bool gc_ready = gc_match && withinSyncThreshold(gc_match->stamp, bbox->stamp);

	if (!tc_ready && !gc_ready)
	{
		if ((now - bbox->stamp).seconds() < config_.pending_wait_s) return std::nullopt;
		return buildLostResult(now, bbox->data);
	}
	if ((now - bbox->stamp).seconds() < config_.pending_wait_s && !(tc_ready && gc_ready)) return std::nullopt;

	uint32_t target_id = 0;
	float detection_conf = static_cast<float>(config_.default_confidence);
	uint8_t target_type = config_.target_type_default;
	parseBboxMeta(bbox->data, target_id, detection_conf, target_type);

	const auto roi = buildRoi(bbox->data);
	if (!roi) return buildLostResult(now, bbox->data);

	const auto tc_result = tc_ready ? extractRoiCentroid(*tc_match->cloud, *roi) : std::nullopt;
	const auto gc_result = gc_ready ? extractRoiCentroid(*gc_match->cloud, *roi) : std::nullopt;
	if (!tc_result && !gc_result) return buildLostResult(now, bbox->data);

	geometry_msgs::msg::Point fused_center;
	uint8_t source = msg::TargetObservation::SOURCE_FUSED;
	double cluster_quality = 0.0;
	if (tc_result && gc_result)
	{
		const double tc_w = 1.0 / std::max(1e-6, config_.tc_noise_std * config_.tc_noise_std);
		const double gc_w = 1.0 / std::max(1e-6, config_.gc_noise_std * config_.gc_noise_std);
		const double sum_w = tc_w + gc_w;
		fused_center.x = static_cast<float>((tc_result->centroid.x * tc_w + gc_result->centroid.x * gc_w) / sum_w);
		fused_center.y = static_cast<float>((tc_result->centroid.y * tc_w + gc_result->centroid.y * gc_w) / sum_w);
		fused_center.z = static_cast<float>((tc_result->centroid.z * tc_w + gc_result->centroid.z * gc_w) / sum_w);
		cluster_quality = std::clamp(0.5 * (tc_result->quality + gc_result->quality), 0.0, 1.0);
	}
	else if (tc_result)
	{
		fused_center = tc_result->centroid;
		cluster_quality = tc_result->quality;
		source = msg::TargetObservation::SOURCE_TC_ONLY;
	}
	else
	{
		fused_center = gc_result->centroid;
		cluster_quality = gc_result->quality;
		source = msg::TargetObservation::SOURCE_GC_ONLY;
	}

	updateTrack(fused_center, now, true);

	Result result;
	result.observation = buildObservation(now, target_id, target_type, detection_conf, fused_center, cluster_quality, source, false);
	result.gimbal_command = buildVisualCommand(result.observation);
	result.has_observation = true;
	result.has_visual_command = true;
	if (shouldSuppress(result.observation)) return std::nullopt;

	last_emitted_key_ = buildKey(
		bbox->stamp,
		tc_result ? tc_match->stamp : rclcpp::Time(0, 0, RCL_ROS_TIME),
		gc_result ? gc_match->stamp : rclcpp::Time(0, 0, RCL_ROS_TIME),
		result.observation.status);
	return result;
}

const TargetFusionProcessor::TimestampedBbox *TargetFusionProcessor::latestBbox() const
{
	return tc_bbox_buffer_.empty() ? nullptr : &tc_bbox_buffer_.back();
}

std::optional<TargetFusionProcessor::TimestampedCloud> TargetFusionProcessor::findNearestCloud(
	const std::deque<TimestampedCloud> &buffer, const rclcpp::Time &stamp) const
{
	if (buffer.empty()) return std::nullopt;
	const auto it = std::min_element(buffer.begin(), buffer.end(), [&](const auto &a, const auto &b) {
		return std::abs((a.stamp - stamp).seconds()) < std::abs((b.stamp - stamp).seconds());
	});
	return *it;
}

void TargetFusionProcessor::pruneBuffers(const rclcpp::Time &now)
{
	auto prune = [&](auto &buffer)
	{
		while (!buffer.empty() && (now - buffer.front().stamp).seconds() > config_.buffer_keep_s)
		{
			buffer.pop_front();
		}
	};
	prune(tc_bbox_buffer_);
	prune(tc_pc_buffer_);
	prune(gc_pc_buffer_);
}

bool TargetFusionProcessor::withinSyncThreshold(const rclcpp::Time &lhs, const rclcpp::Time &rhs) const
{
	return std::abs((lhs - rhs).seconds()) <= config_.time_sync_threshold_s;
}

void TargetFusionProcessor::parseBboxMeta(
	const std_msgs::msg::Float32MultiArray &data, uint32_t &target_id, float &detection_conf, uint8_t &target_type) const
{
	if (data.data.size() >= 5) detection_conf = static_cast<float>(std::clamp<double>(data.data[4], 0.0, 1.0));
	if (data.data.size() >= 6) target_id = static_cast<uint32_t>(std::max(0.0f, data.data[5]));
	if (data.data.size() >= 7) target_type = static_cast<uint8_t>(std::max(0.0f, data.data[6]));
	else if (has_target_hint_) target_type = target_type_hint_;
}

std::optional<TargetFusionProcessor::RoiBounds> TargetFusionProcessor::buildRoi(const std_msgs::msg::Float32MultiArray &data) const
{
	if (data.data.size() < 4) return std::nullopt;
	const float x = data.data[0];
	const float y = data.data[1];
	const float w = std::max(1.0f, data.data[2]);
	const float h = std::max(1.0f, data.data[3]);
	RoiBounds roi;
	roi.x_min = std::max(0, static_cast<int>(std::floor(x)) - config_.roi_margin_px);
	roi.x_max = std::min(config_.image_width - 1, static_cast<int>(std::ceil(x + w)) + config_.roi_margin_px);
	roi.y_min = std::max(0, static_cast<int>(std::floor(y)) - config_.roi_margin_px);
	roi.y_max = std::min(config_.image_height - 1, static_cast<int>(std::ceil(y + h)) + config_.roi_margin_px);
	if (roi.x_min >= roi.x_max || roi.y_min >= roi.y_max) return std::nullopt;
	return roi;
}

std::optional<TargetFusionProcessor::RoiResult> TargetFusionProcessor::extractRoiCentroid(
	const sensor_msgs::msg::PointCloud2 &cloud, const RoiBounds &roi) const
{
	if (cloud.width == 0 || cloud.height <= 1) return std::nullopt;
	const int cloud_w = static_cast<int>(cloud.width);
	const int cloud_h = static_cast<int>(cloud.height);
	const int x_min = std::clamp(roi.x_min, 0, cloud_w - 1);
	const int x_max = std::clamp(roi.x_max, 0, cloud_w - 1);
	const int y_min = std::clamp(roi.y_min, 0, cloud_h - 1);
	const int y_max = std::clamp(roi.y_max, 0, cloud_h - 1);

	sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

	const std::size_t total = static_cast<std::size_t>(cloud_w * cloud_h);
	std::vector<float> xs;
	std::vector<float> ys;
	std::vector<float> zs;
	xs.reserve(total / 16 + 1);
	ys.reserve(total / 16 + 1);
	zs.reserve(total / 16 + 1);

	for (std::size_t idx = 0; idx < total; ++idx, ++iter_x, ++iter_y, ++iter_z)
	{
		const int px = static_cast<int>(idx % cloud_w);
		const int py = static_cast<int>(idx / cloud_w);
		if (px < x_min || px > x_max || py < y_min || py > y_max) continue;
		const float x = *iter_x;
		const float y = *iter_y;
		const float z = *iter_z;
		if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
		if (z < config_.depth_min || z > config_.depth_max) continue;
		xs.push_back(x);
		ys.push_back(y);
		zs.push_back(z);
	}
	if (xs.size() < config_.min_roi_points) return std::nullopt;

	auto trimmedMean = [](std::vector<float> &values) -> double
	{
		std::sort(values.begin(), values.end());
		const std::size_t n = values.size();
		const std::size_t begin = std::min<std::size_t>(n / 10, n - 1);
		const std::size_t end = std::max(begin + 1, (n * 9) / 10);
		double sum = 0.0;
		std::size_t count = 0;
		for (std::size_t i = begin; i < end && i < n; ++i) { sum += values[i]; ++count; }
		return count > 0 ? sum / static_cast<double>(count) : 0.0;
	};

	RoiResult result;
	result.centroid.x = static_cast<float>(trimmedMean(xs));
	result.centroid.y = static_cast<float>(trimmedMean(ys));
	result.centroid.z = static_cast<float>(trimmedMean(zs));
	result.point_count = xs.size();
	result.quality = std::clamp(static_cast<double>(result.point_count) / static_cast<double>(config_.min_roi_points * 4), 0.0, 1.0);
	return result;
}

void TargetFusionProcessor::updateTrack(const geometry_msgs::msg::Point &observed, const rclcpp::Time &now, bool has_observation)
{
	if (!track_initialized_)
	{
		track_position_ = observed;
		track_velocity_ = geometry_msgs::msg::Point();
		track_initialized_ = true;
		loss_frames_ = 0;
		stability_ = 0.5;
		last_track_time_ = now;
		return;
	}
	const double dt = std::max(1e-3, (now - last_track_time_).seconds());
	if (has_observation)
	{
		const geometry_msgs::msg::Point predicted = predict(now);
		const double vx = (observed.x - predicted.x) / dt;
		const double vy = (observed.y - predicted.y) / dt;
		const double vz = (observed.z - predicted.z) / dt;
		track_position_ = observed;
		track_velocity_.x = static_cast<float>(0.7 * track_velocity_.x + 0.3 * vx);
		track_velocity_.y = static_cast<float>(0.7 * track_velocity_.y + 0.3 * vy);
		track_velocity_.z = static_cast<float>(0.7 * track_velocity_.z + 0.3 * vz);
		loss_frames_ = 0;
		stability_ = std::min(1.0, stability_ + 0.08);
	}
	else
	{
		track_position_ = predict(now);
		++loss_frames_;
		stability_ = std::max(0.0, stability_ - 0.06);
	}
	last_track_time_ = now;
}

geometry_msgs::msg::Point TargetFusionProcessor::predict(const rclcpp::Time &now) const
{
	if (!track_initialized_) return geometry_msgs::msg::Point();
	const double dt = std::max(0.0, (now - last_track_time_).seconds());
	geometry_msgs::msg::Point predicted = track_position_;
	predicted.x = static_cast<float>(predicted.x + track_velocity_.x * dt);
	predicted.y = static_cast<float>(predicted.y + track_velocity_.y * dt);
	predicted.z = static_cast<float>(predicted.z + track_velocity_.z * dt);
	return predicted;
}

msg::TargetObservation TargetFusionProcessor::buildObservation(
	const rclcpp::Time &stamp, uint32_t target_id, uint8_t target_type, float detection_conf,
	const geometry_msgs::msg::Point &position, double cluster_quality, uint8_t source, bool lost)
{
	msg::TargetObservation obs;
	obs.header.stamp = stamp;
	obs.header.frame_id = "base_link";
	obs.t_usec = static_cast<uint64_t>(stamp.nanoseconds() / 1000ULL);
	obs.obs_id = ++obs_id_;
	obs.track_id = track_id_;
	obs.target_id = target_id;
	obs.frame = msg::TargetObservation::FRAME_BODY_DRONE;
	obs.target_type = target_type;
	obs.position = position;
	obs.range = static_cast<float>(std::sqrt(position.x * position.x + position.y * position.y + position.z * position.z));
	obs.yaw = static_cast<float>(std::atan2(position.y, position.x));
	obs.pitch = static_cast<float>(std::atan2(position.z, std::sqrt(position.x * position.x + position.y * position.y)));
	obs.confidence = computeConfidence(detection_conf, cluster_quality, stability_, lost);
	obs.source = source;
	obs.status = lost ? msg::TargetObservation::STATUS_LOST :
		(obs.confidence >= config_.low_conf_threshold ? msg::TargetObservation::STATUS_VALID : msg::TargetObservation::STATUS_LOW_CONF);
	return obs;
}

msg::GimbalVisualCommand TargetFusionProcessor::buildVisualCommand(const msg::TargetObservation &obs) const
{
	msg::GimbalVisualCommand cmd;
	cmd.header = obs.header;
	cmd.yaw = static_cast<float>(config_.visual_cmd_gain * obs.yaw);
	cmd.pitch = static_cast<float>(config_.visual_cmd_gain * obs.pitch);
	cmd.confidence = obs.confidence;
	return cmd;
}

std::optional<TargetFusionProcessor::Result> TargetFusionProcessor::buildLostResult(const rclcpp::Time &now, const std_msgs::msg::Float32MultiArray &bbox)
{
	uint32_t target_id = 0;
	float detection_conf = static_cast<float>(config_.default_confidence);
	uint8_t target_type = config_.target_type_default;
	parseBboxMeta(bbox, target_id, detection_conf, target_type);

	const geometry_msgs::msg::Point predicted = track_initialized_ ? predict(now) : geometry_msgs::msg::Point();
	updateTrack(predicted, now, false);

	Result result;
	result.observation = buildObservation(now, target_id, target_type, detection_conf, predicted, 0.0, 0, true);
	result.has_observation = true;
	result.lost = true;
	if (shouldSuppress(result.observation)) return std::nullopt;
	last_emitted_key_ = buildKey(now, rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Time(0, 0, RCL_ROS_TIME), result.observation.status);
	return result;
}

double TargetFusionProcessor::computeConfidence(float detection_conf, double cluster_quality, double stability, bool lost) const
{
	if (lost) return 0.0;
	const double fused = config_.confidence_weight_detection * std::clamp<double>(detection_conf, 0.0, 1.0) +
		config_.confidence_weight_cluster * std::clamp(cluster_quality, 0.0, 1.0) +
		config_.confidence_weight_stability * std::clamp(stability, 0.0, 1.0);
	return std::clamp(fused, 0.0, 1.0);
}

uint64_t TargetFusionProcessor::buildKey(
	const rclcpp::Time &bbox_stamp, const rclcpp::Time &tc_stamp, const rclcpp::Time &gc_stamp, uint8_t status) const
{
	const auto mix = [](uint64_t seed, uint64_t value)
	{
		return seed ^ (value + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U));
	};
	uint64_t key = 0;
	key = mix(key, static_cast<uint64_t>(bbox_stamp.nanoseconds()));
	key = mix(key, static_cast<uint64_t>(tc_stamp.nanoseconds()));
	key = mix(key, static_cast<uint64_t>(gc_stamp.nanoseconds()));
	key = mix(key, static_cast<uint64_t>(status));
	return key;
}

bool TargetFusionProcessor::shouldSuppress(const msg::TargetObservation &obs)
{
	const uint64_t key = buildKey(obs.header.stamp, rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Time(0, 0, RCL_ROS_TIME), obs.status);
	if (key == last_emitted_key_) return true;
	return false;
}

} // namespace c3_drone_driver
