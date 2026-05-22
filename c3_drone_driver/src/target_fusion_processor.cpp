#include "c3_drone_driver/target_fusion_processor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace c3_drone_driver
{

TargetFusionProcessor::TargetFusionProcessor(const Config &config)
: config_(config),
  pose_estimator_([&config]() {
	  PoseEstimator::Config pose_cfg;
	  pose_cfg.body_to_gimbal_x = config.body_to_gimbal_x;
	  pose_cfg.body_to_gimbal_y = config.body_to_gimbal_y;
	  pose_cfg.body_to_gimbal_z = config.body_to_gimbal_z;
	  return pose_cfg;
	}())
{
}

void TargetFusionProcessor::setTfBuffer(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer)
{
	pose_estimator_.setTfBuffer(tf_buffer);
}

std::optional<TargetFusionProcessor::Result> TargetFusionProcessor::process(const rclcpp::Time &now)
{
	//1. 清理过期数据
	pruneBuffers(now);

	//2. 选取锚点检测（TC/GC中时间更新的一侧）
	const auto *tc_latest = latestTcDetection();
	const auto *gc_latest = latestGcDetection();
	if ((!tc_latest || !tc_latest->data) && (!gc_latest || !gc_latest->data)) return std::nullopt;

	bool anchor_is_tc = false;
	const TimestampedDetection *anchor = nullptr;
	if (tc_latest && tc_latest->data &&
		(!gc_latest || !gc_latest->data || tc_latest->stamp >= gc_latest->stamp))
	{
		anchor = tc_latest;
		anchor_is_tc = true;
	}
	else
	{
		anchor = gc_latest;
		anchor_is_tc = false;
	}
	if (!anchor || !anchor->data) return std::nullopt;
	if (!pose_estimator_.hasGimbalState()) return std::nullopt;

	rclcpp::Time matched_stamp(0, 0, RCL_ROS_TIME);
	std::optional<msg::TcDetection::SharedPtr> counterpart;
	if (anchor_is_tc)
	{
		counterpart = findNearestGcDetection(anchor->stamp, &matched_stamp);
	}
	else
	{
		counterpart = findNearestTcDetection(anchor->stamp, &matched_stamp);
	}
	const bool counterpart_ready = counterpart.has_value() &&
		withinSyncThreshold(matched_stamp, anchor->stamp);

	//3. 等待短窗口内迟到帧，尽量争取TC+GC联合融合
	if (!counterpart_ready && (now - anchor->stamp).seconds() < config_.pending_wait_s)
	{
		return std::nullopt;
	}

	msg::TcDetection::SharedPtr tc_detection = nullptr;
	msg::TcDetection::SharedPtr gc_detection = nullptr;
	rclcpp::Time tc_stamp(0, 0, RCL_ROS_TIME);
	rclcpp::Time gc_stamp(0, 0, RCL_ROS_TIME);
	if (anchor_is_tc)
	{
		tc_detection = anchor->data;
		tc_stamp = anchor->stamp;
		if (counterpart_ready)
		{
			gc_detection = *counterpart;
			gc_stamp = matched_stamp;
		}
	}
	else
	{
		gc_detection = anchor->data;
		gc_stamp = anchor->stamp;
		if (counterpart_ready)
		{
			tc_detection = *counterpart;
			tc_stamp = matched_stamp;
		}
	}

	//4. 从每路检测提取ROI质心与元数据（各自使用自己的bbox+点云）
	struct CandidateObservation
	{
		RoiResult roi;
		uint32_t target_id{0};
		float detection_conf{0.0F};
		uint8_t target_type{kTargetTypeGcDetection};
		uint8_t source{msg::TargetObservation::SOURCE_GC_ONLY};
		rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
	};

	auto buildCandidate = [this](
		const msg::TcDetection::SharedPtr &detection,
		uint8_t target_type,
		uint8_t source,
		const rclcpp::Time &stamp) -> std::optional<CandidateObservation>
	{
		if (!detection) return std::nullopt;
		if (detection->bbox.data.empty()) return std::nullopt;
		if (detection->cloud.width == 0U || detection->cloud.height == 0U) return std::nullopt;

		CandidateObservation candidate;
		candidate.target_type = target_type;
		candidate.source = source;
		candidate.stamp = stamp;
		candidate.detection_conf = static_cast<float>(config_.default_confidence);
		parseDetectionMeta(detection->bbox.data, candidate.target_id, candidate.detection_conf);

		const auto roi = buildRoi(detection->bbox.data);
		if (!roi) return std::nullopt;
		std::optional<RoiResult> roi_result;
		const std::string camera_frame = detection->header.frame_id.empty()
			? (source == msg::TargetObservation::SOURCE_TC_ONLY
				? "tc_camera_optical_frame"
				: "gated_camera_optical_frame")
			: detection->header.frame_id;
		if (source == msg::TargetObservation::SOURCE_TC_ONLY)
		{
			roi_result = extractRoiCentroidInBody(
				detection->cloud, *roi, camera_frame, "base_link",
				config_.tc_to_gimbal_x, config_.tc_to_gimbal_y, config_.tc_to_gimbal_z);
		}
		else
		{
			roi_result = extractRoiCentroidInBody(
				detection->cloud, *roi, camera_frame, "base_link",
				config_.gc_to_gimbal_x, config_.gc_to_gimbal_y, config_.gc_to_gimbal_z);
		}
		if (!roi_result) return std::nullopt;
		candidate.roi = *roi_result;
		return candidate;
	};

	const auto tc_candidate = buildCandidate(
		tc_detection,
		kTargetTypeTcDetection,
		msg::TargetObservation::SOURCE_TC_ONLY,
		tc_stamp);
	const auto gc_candidate = buildCandidate(
		gc_detection,
		kTargetTypeGcDetection,
		msg::TargetObservation::SOURCE_GC_ONLY,
		gc_stamp);

	//5. 两路都无有效ROI时，按锚点源返回LOST结果
	if (!tc_candidate && !gc_candidate)
	{
		const uint8_t lost_type = anchor_is_tc ? kTargetTypeTcDetection : kTargetTypeGcDetection;
		const uint8_t lost_source = anchor_is_tc
			? msg::TargetObservation::SOURCE_TC_ONLY
			: msg::TargetObservation::SOURCE_GC_ONLY;
		return buildLostResult(now, *anchor->data, lost_type, lost_source);
	}

	//6. 融合结果并更新跟踪
	geometry_msgs::msg::Point fused_center;
	uint32_t target_id = 0U;
	float detection_conf = static_cast<float>(config_.default_confidence);
	uint8_t target_type = kTargetTypeTcDetection;
	uint8_t source = msg::TargetObservation::SOURCE_TC_ONLY;
	double cluster_quality = 0.0;
	if (tc_candidate && gc_candidate)
	{
		// 根据质心质量（即ROI内点的数量和分布情况）进行加权融合，质量更高的结果权重更大
		const double tc_w = 1.0 / std::max(1e-6, config_.tc_noise_std * config_.tc_noise_std);
		const double gc_w = 1.0 / std::max(1e-6, config_.gc_noise_std * config_.gc_noise_std);
		const double sum_w = tc_w + gc_w;
		fused_center.x = static_cast<float>((tc_candidate->roi.centroid.x * tc_w + gc_candidate->roi.centroid.x * gc_w) / sum_w);
		fused_center.y = static_cast<float>((tc_candidate->roi.centroid.y * tc_w + gc_candidate->roi.centroid.y * gc_w) / sum_w);
		fused_center.z = static_cast<float>((tc_candidate->roi.centroid.z * tc_w + gc_candidate->roi.centroid.z * gc_w) / sum_w);
		cluster_quality = std::clamp(0.5 * (tc_candidate->roi.quality + gc_candidate->roi.quality), 0.0, 1.0);
		detection_conf = static_cast<float>(
			std::clamp((tc_candidate->detection_conf * tc_w + gc_candidate->detection_conf * gc_w) / sum_w, 0.0, 1.0));

		const bool use_tc_meta =
			(tc_candidate->detection_conf > gc_candidate->detection_conf + 1e-4F) ||
			(std::abs(tc_candidate->detection_conf - gc_candidate->detection_conf) <= 1e-4F &&
			 tc_candidate->stamp >= gc_candidate->stamp);
		if (use_tc_meta)
		{
			target_id = tc_candidate->target_id;
			target_type = tc_candidate->target_type;
		}
		else
		{
			target_id = gc_candidate->target_id;
			target_type = gc_candidate->target_type;
		}
		source = msg::TargetObservation::SOURCE_FUSED;
	}
	else if (tc_candidate)
	{
		fused_center = tc_candidate->roi.centroid;
		cluster_quality = tc_candidate->roi.quality;
		target_id = tc_candidate->target_id;
		detection_conf = tc_candidate->detection_conf;
		target_type = tc_candidate->target_type;
		source = msg::TargetObservation::SOURCE_TC_ONLY;
	}
	else
	{
		fused_center = gc_candidate->roi.centroid;
		cluster_quality = gc_candidate->roi.quality;
		target_id = gc_candidate->target_id;
		detection_conf = gc_candidate->detection_conf;
		target_type = gc_candidate->target_type;
		source = msg::TargetObservation::SOURCE_GC_ONLY;
	}

	//7.更新跟踪状态
	updateTrack(fused_center, now, true);

	//8. 构建结果
	Result result;
	result.observation = buildObservation(now, target_id, target_type, detection_conf, fused_center, cluster_quality, source, false);
	result.gimbal_command = buildVisualCommand(result.observation);
	result.has_observation = true;
	result.has_visual_command = true;

	return result;
}

void TargetFusionProcessor::updateTcDetection(const msg::TcDetection::SharedPtr &msg)
{
	if (!msg) return;
	const rclcpp::Time stamp(msg->header.stamp);
	tc_detection_buffer_.push_back({stamp, msg});
	pruneBuffers(stamp);
}

void TargetFusionProcessor::updateGcDetection(const msg::TcDetection::SharedPtr &msg)
{
	if (!msg) return;
	const rclcpp::Time stamp(msg->header.stamp);
	gc_detection_buffer_.push_back({stamp, msg});
	pruneBuffers(stamp);
}

void TargetFusionProcessor::updateGimbalState(const msg::GimbalState::SharedPtr &msg)
{
	if (!msg) return;
	pose_estimator_.updateGimbalState(*msg);
}

const TargetFusionProcessor::TimestampedDetection *TargetFusionProcessor::latestTcDetection() const
{
	return tc_detection_buffer_.empty() ? nullptr : &tc_detection_buffer_.back();
}

const TargetFusionProcessor::TimestampedDetection *TargetFusionProcessor::latestGcDetection() const
{
	return gc_detection_buffer_.empty() ? nullptr : &gc_detection_buffer_.back();
}

std::optional<msg::TcDetection::SharedPtr> TargetFusionProcessor::findNearestTcDetection(
	const rclcpp::Time &stamp, rclcpp::Time *matched_stamp) const
{
	if (tc_detection_buffer_.empty()) return std::nullopt;
	const auto nearest_one = std::min_element(tc_detection_buffer_.begin(), tc_detection_buffer_.end(), [&](const auto &a, const auto &b) {
		return std::abs((a.stamp - stamp).seconds()) < std::abs((b.stamp - stamp).seconds());
	});
	if (matched_stamp) *matched_stamp = nearest_one->stamp;
	return nearest_one->data;
}

std::optional<msg::TcDetection::SharedPtr> TargetFusionProcessor::findNearestGcDetection(
	const rclcpp::Time &stamp, rclcpp::Time *matched_stamp) const
{
	if (gc_detection_buffer_.empty()) return std::nullopt;
	const auto nearest_one = std::min_element(gc_detection_buffer_.begin(), gc_detection_buffer_.end(), [&](const auto &a, const auto &b) {
		return std::abs((a.stamp - stamp).seconds()) < std::abs((b.stamp - stamp).seconds());
	});
	if (matched_stamp) *matched_stamp = nearest_one->stamp;
	return nearest_one->data;
}

void TargetFusionProcessor::pruneBuffers(const rclcpp::Time &now)
{
	// 当前时间与缓冲区中最旧数据的时间差超过buffer_keep_s时，丢弃最旧数据，直到缓冲区内数据的时间范围在buffer_keep_s内
	while (!tc_detection_buffer_.empty() && (now - tc_detection_buffer_.front().stamp).seconds() > config_.buffer_keep_s)
	{
		tc_detection_buffer_.pop_front();
	}
	while (!gc_detection_buffer_.empty() && (now - gc_detection_buffer_.front().stamp).seconds() > config_.buffer_keep_s)
	{
		gc_detection_buffer_.pop_front();
	}
}

bool TargetFusionProcessor::withinSyncThreshold(const rclcpp::Time &lhs, const rclcpp::Time &rhs) const
{
	return std::abs((lhs - rhs).seconds()) <= config_.time_sync_threshold_s;
}

std::optional<TargetFusionProcessor::RoiBounds> TargetFusionProcessor::buildRoi(const std::vector<float> &data) const
{
	if (data.size() < 4) return std::nullopt;
	const float x = data[0];
	const float y = data[1];
	const float w = std::max(1.0f, data[2]);
	const float h = std::max(1.0f, data[3]);
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
	//1. 基础输入检查与ROI边界裁剪
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

	//2. 遍历点云，筛选ROI内且深度/数值有效的三维点
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

	//3. 点数不足时直接判为无效ROI，避免稀疏噪点造成误判
	if (xs.size() < config_.min_roi_points) return std::nullopt;

	//4. 对每一维使用截尾均值，抑制离群点并估计质心
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

	//5. 输出ROI质心与质量分数（按点数相对阈值线性归一）
	RoiResult result;
	result.centroid.x = static_cast<float>(trimmedMean(xs));
	result.centroid.y = static_cast<float>(trimmedMean(ys));
	result.centroid.z = static_cast<float>(trimmedMean(zs));
	result.point_count = xs.size();
	result.quality = std::clamp(static_cast<double>(result.point_count) / static_cast<double>(config_.min_roi_points * 4), 0.0, 1.0);
	return result;
}

std::optional<TargetFusionProcessor::RoiResult> TargetFusionProcessor::extractRoiCentroidInBody(
	const sensor_msgs::msg::PointCloud2 &cloud,
	const RoiBounds &roi,
	const std::string &camera_optical_frame,
	const std::string &body_frame,
	double camera_to_gimbal_x,
	double camera_to_gimbal_y,
	double camera_to_gimbal_z) const
{
	const auto roi_result = extractRoiCentroid(cloud, roi);
	if (!roi_result) return std::nullopt;

	std::array<double, 3> p_cam{
		static_cast<double>(roi_result->centroid.x),
		static_cast<double>(roi_result->centroid.y),
		static_cast<double>(roi_result->centroid.z)};
	const auto body_point = pose_estimator_.cameraOpticalPointToBody(
		p_cam,
		camera_optical_frame,
		body_frame,
		camera_to_gimbal_x,
		camera_to_gimbal_y,
		camera_to_gimbal_z);
	if (!body_point) return std::nullopt;

	RoiResult out = *roi_result;
	out.centroid.x = static_cast<float>((*body_point)[0]);
	out.centroid.y = static_cast<float>((*body_point)[1]);
	out.centroid.z = static_cast<float>((*body_point)[2]);
	return out;
}

void TargetFusionProcessor::updateTrack(const geometry_msgs::msg::Point &observed, const rclcpp::Time &now, bool has_observation)
{
	//1. 首帧初始化轨迹状态
	if (!track_initialized_)
	{
		track_position_ = observed;
		track_velocity_ = geometry_msgs::msg::Point();
		track_initialized_ = true;
		stability_ = 0.5;
		last_track_time_ = now;
		return;
	}

	//2. 计算时间间隔，用于速度更新与状态外推
	const double dt = std::max(1e-3, (now - last_track_time_).seconds());
	if (has_observation)
	{
		//3. 有观测时：残差更新速度（指数平滑）并提升稳定度
		const geometry_msgs::msg::Point predicted = predict(now);
		const double vx = (observed.x - predicted.x) / dt;
		const double vy = (observed.y - predicted.y) / dt;
		const double vz = (observed.z - predicted.z) / dt;
		track_position_ = observed;
		track_velocity_.x = static_cast<float>(0.7 * track_velocity_.x + 0.3 * vx);
		track_velocity_.y = static_cast<float>(0.7 * track_velocity_.y + 0.3 * vy);
		track_velocity_.z = static_cast<float>(0.7 * track_velocity_.z + 0.3 * vz);
		stability_ = std::min(1.0, stability_ + 0.08);
	}
	else
	{
		//4. 无观测时：仅按运动模型外推并降低稳定度
		track_position_ = predict(now);
		stability_ = std::max(0.0, stability_ - 0.06);
	}

	//5. 刷新轨迹时间戳
	last_track_time_ = now;
}

geometry_msgs::msg::Point TargetFusionProcessor::predict(const rclcpp::Time &now) const
{
	//1. 未初始化则返回零向量，避免使用未定义轨迹状态
	if (!track_initialized_) return geometry_msgs::msg::Point();

	//2. 匀速模型前向预测当前位置
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
	//1. 填充时间戳、ID与基础语义字段
	msg::TargetObservation obs;
	obs.header.stamp = stamp;
	obs.header.frame_id = "base_link";
	obs.t_usec = static_cast<uint64_t>(stamp.nanoseconds() / 1000ULL);
	obs.obs_id = ++obs_id_;
	obs.track_id = track_id_;
	obs.target_id = target_id;
	obs.frame = msg::TargetObservation::FRAME_BODY_DRONE;
	obs.target_type = target_type;

	//2. 由三维位置解算距离与方位角信息
	obs.position = position;
	obs.range = static_cast<float>(std::sqrt(position.x * position.x + position.y * position.y + position.z * position.z));
	obs.yaw = static_cast<float>(std::atan2(position.y, position.x));
	obs.pitch = static_cast<float>(std::atan2(position.z, std::sqrt(position.x * position.x + position.y * position.y)));

	//3. 融合置信度并根据阈值给出观测状态
	obs.confidence = computeConfidence(detection_conf, cluster_quality, stability_, lost);
	obs.source = source;
	obs.status = lost ? msg::TargetObservation::STATUS_LOST :
		(obs.confidence >= config_.low_conf_threshold ? msg::TargetObservation::STATUS_VALID : msg::TargetObservation::STATUS_LOW_CONF);
	return obs;
}

void TargetFusionProcessor::parseDetectionMeta(
	const std::vector<float> &data, uint32_t &target_id, float &detection_conf) const
{
	if (data.size() >= 5) detection_conf = static_cast<float>(std::clamp<double>(data[4], 0.0, 1.0));
	if (data.size() >= 6) target_id = static_cast<uint32_t>(std::max(0.0f, data[5]));
}

msg::GimbalVisualCommand TargetFusionProcessor::buildVisualCommand(const msg::TargetObservation &obs) const
{
	msg::GimbalVisualCommand cmd;
	cmd.header = obs.header;
	const std::array<double, 3> body_point{
		static_cast<double>(obs.position.x),
		static_cast<double>(obs.position.y),
		static_cast<double>(obs.position.z)};
	const auto gimbal_angles = pose_estimator_.bodyPointToGimbalYawPitch(body_point);
	if (gimbal_angles.has_value())
	{
		cmd.yaw = static_cast<float>(config_.visual_cmd_gain * gimbal_angles->first);
		cmd.pitch = static_cast<float>(config_.visual_cmd_gain * gimbal_angles->second);
	}
	else
	{
		cmd.yaw = static_cast<float>(config_.visual_cmd_gain * obs.yaw);
		cmd.pitch = static_cast<float>(config_.visual_cmd_gain * obs.pitch);
	}
	cmd.confidence = obs.confidence;
	return cmd;
}

std::optional<TargetFusionProcessor::Result> TargetFusionProcessor::buildLostResult(
	const rclcpp::Time &now,
	const msg::TcDetection &detection_data,
	uint8_t target_type,
	uint8_t source)
{
	//1. 从最近一次检测提取目标元信息（ID/检测置信度）
	uint32_t target_id = 0;
	float detection_conf = static_cast<float>(config_.default_confidence);
	parseDetectionMeta(detection_data.bbox.data, target_id, detection_conf);

	//2. 丢失时使用轨迹预测位置，并以“无观测”模式更新跟踪器
	const geometry_msgs::msg::Point predicted = track_initialized_ ? predict(now) : geometry_msgs::msg::Point();
	updateTrack(predicted, now, false);

	//3. 构造LOST状态观测（仅保留观测，不下发云台视觉指令）
	Result result;
	result.observation = buildObservation(now, target_id, target_type, detection_conf, predicted, 0.0, source, true);
	result.has_observation = true;
	result.lost = true;
	return result;
}

double TargetFusionProcessor::computeConfidence(float detection_conf, double cluster_quality, double stability, bool lost) const
{
	//1. 丢失状态直接置零置信度，避免误导下游决策
	if (lost) return 0.0;

	//2. 按配置权重融合检测置信度、点云质量与轨迹稳定度
	const double fused = config_.confidence_weight_detection * std::clamp<double>(detection_conf, 0.0, 1.0) +
		config_.confidence_weight_cluster * std::clamp(cluster_quality, 0.0, 1.0) +
		config_.confidence_weight_stability * std::clamp(stability, 0.0, 1.0);

	//3. 归一到[0, 1]
	return std::clamp(fused, 0.0, 1.0);
}

} // namespace c3_drone_driver
