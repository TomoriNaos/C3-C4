#include "c3_drone_driver/target_fusion_processor.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace c3_drone_driver
{

TargetFusionProcessor::TargetFusionProcessor(const Config &config)
: config_(config)
{
}

std::optional<TargetFusionProcessor::Result> TargetFusionProcessor::process(const rclcpp::Time &now)
{
	//1. 清理过期数据
	pruneBuffers(now);

	//2. 更新数据
	const auto *tc_detection = latestTcDetection();
	if (!tc_detection || !tc_detection->data) return std::nullopt;
	const auto &bbox_data = tc_detection->data->bbox.data;
	const auto &tc_cloud = tc_detection->data->cloud;
	if (bbox_data.empty() || tc_cloud.width == 0U || tc_cloud.height == 0U) return std::nullopt;
	rclcpp::Time gc_stamp(0, 0, RCL_ROS_TIME);
	const auto gc_cloud = findNearestGcCloud(tc_detection->stamp, &gc_stamp);
	const bool gc_ready = gc_cloud && withinSyncThreshold(gc_stamp, tc_detection->stamp);

	if (!gc_ready)
	{
		if ((now - tc_detection->stamp).seconds() < config_.pending_wait_s) return std::nullopt;
		return buildLostResult(now, *tc_detection->data);
	}

	//3. 解析bbox信息（ROS2 data）
	uint32_t target_id = 0;
	float detection_conf = static_cast<float>(config_.default_confidence);
	uint8_t target_type = config_.target_type_default;
	parseDetectionMeta(bbox_data, target_id, detection_conf, target_type);

	//4. 构建ROI并提取质心
	const auto roi = buildRoi(bbox_data);

	// 若ROI无效，直接返回丢失结果（但仍携带检测信息），避免后续处理出错
	if (!roi) return buildLostResult(now, *tc_detection->data);

	const auto tc_result = extractRoiCentroid(tc_cloud, *roi);
	const auto gc_result = gc_ready ? extractRoiCentroid(**gc_cloud, *roi) : std::nullopt;

	if (!tc_result && !gc_result) return buildLostResult(now, *tc_detection->data);

	//5. 融合结果并更新跟踪
	geometry_msgs::msg::Point fused_center;
	uint8_t source = msg::TargetObservation::SOURCE_FUSED;
	double cluster_quality = 0.0;
	if (tc_result && gc_result)
	{
		// 根据质心质量（即ROI内点的数量和分布情况）进行加权融合，质量更高的结果权重更大
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

	//6.更新跟踪状态
	updateTrack(fused_center, now, true);

	//7. 构建结果
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

void TargetFusionProcessor::updateGcPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
{
	if (!msg) return;
	const rclcpp::Time stamp(msg->header.stamp);
	gc_pc_buffer_.push_back({stamp, msg});
	pruneBuffers(stamp);
}

const TargetFusionProcessor::TimestampedTcDetection *TargetFusionProcessor::latestTcDetection() const
{
	return tc_detection_buffer_.empty() ? nullptr : &tc_detection_buffer_.back();
}

std::optional<sensor_msgs::msg::PointCloud2::SharedPtr> TargetFusionProcessor::findNearestGcCloud(
	const rclcpp::Time &stamp, rclcpp::Time *matched_stamp) const
{
	if (gc_pc_buffer_.empty()) return std::nullopt;
	const auto nearest_one = std::min_element(gc_pc_buffer_.begin(), gc_pc_buffer_.end(), [&](const auto &a, const auto &b) {
		return std::abs((a.first - stamp).seconds()) < std::abs((b.first - stamp).seconds());
	});
	if (matched_stamp) *matched_stamp = nearest_one->first;
	return nearest_one->second;
}

void TargetFusionProcessor::pruneBuffers(const rclcpp::Time &now)
{
	// 当前时间与缓冲区中最旧数据的时间差超过buffer_keep_s时，丢弃最旧数据，直到缓冲区内数据的时间范围在buffer_keep_s内
	while (!tc_detection_buffer_.empty() && (now - tc_detection_buffer_.front().stamp).seconds() > config_.buffer_keep_s)
	{
		tc_detection_buffer_.pop_front();
	}
	while (!gc_pc_buffer_.empty() && (now - gc_pc_buffer_.front().first).seconds() > config_.buffer_keep_s)
	{
		gc_pc_buffer_.pop_front();
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
	const std::vector<float> &data, uint32_t &target_id, float &detection_conf, uint8_t &target_type) const
{
	if (data.size() >= 5) detection_conf = static_cast<float>(std::clamp<double>(data[4], 0.0, 1.0));
	if (data.size() >= 6) target_id = static_cast<uint32_t>(std::max(0.0f, data[5]));
	if (data.size() >= 7) target_type = static_cast<uint8_t>(std::max(0.0f, data[6]));
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

std::optional<TargetFusionProcessor::Result> TargetFusionProcessor::buildLostResult(const rclcpp::Time &now, const msg::TcDetection &tc_data)
{
	//1. 从最近一次检测提取目标元信息（ID/类别/检测置信度）
	uint32_t target_id = 0;
	float detection_conf = static_cast<float>(config_.default_confidence);
	uint8_t target_type = config_.target_type_default;
	parseDetectionMeta(tc_data.bbox.data, target_id, detection_conf, target_type);

	//2. 丢失时使用轨迹预测位置，并以“无观测”模式更新跟踪器
	const geometry_msgs::msg::Point predicted = track_initialized_ ? predict(now) : geometry_msgs::msg::Point();
	updateTrack(predicted, now, false);

	//3. 构造LOST状态观测（仅保留观测，不下发云台视觉指令）
	Result result;
	result.observation = buildObservation(now, target_id, target_type, detection_conf, predicted, 0.0, 0, true);
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
