#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "c3_drone_driver/msg/gimbal_visual_command.hpp"
#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/tc_detection.hpp"
#include "c3_drone_driver/msg/target_observation.hpp"
#include "c3_drone_driver/pose_estimator.h"

namespace c3_drone_driver
{

class TargetFusionProcessor
{
public:
    /**
	 * @brief 自定义target_fusion_processor配置结构体
	 * 详见config/target_fusion_default.yaml
	 */
	struct Config
	{
		double time_sync_threshold_s{0.1};
		double pending_wait_s{0.06};
		double buffer_keep_s{0.2};
		double default_confidence{0.6};
		double visual_cmd_gain{1.0};
		int roi_margin_px{16};
		int image_width{1280};
		int image_height{720};
		double body_to_gimbal_x{0.15};
		double body_to_gimbal_y{0.0};
		double body_to_gimbal_z{-0.05};
		double tc_to_gimbal_x{0.05};
		double tc_to_gimbal_y{0.02};
		double tc_to_gimbal_z{0.0};
		double gc_to_gimbal_x{0.05};
		double gc_to_gimbal_y{-0.02};
		double gc_to_gimbal_z{0.0};
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

	/**
	 * @brief 自定义target_fusion_processor观测结果
	 * @details 包含
	 * - TargetObservation：融合后的目标观测结果
	 * - GimbalVisualCommand：基于观测结果生成的云台控制指令
	 * - has_observation：是否有有效观测结果
	 * - has_visual_command：是否生成了有效的云台控制指令
	 * - lost：目标是否处于丢失状态（即当前无有效观测且跟踪丢失超过阈值）
	 */
	struct Result
	{
		msg::TargetObservation observation;
		msg::GimbalVisualCommand gimbal_command;
		bool has_observation{false};
		bool has_visual_command{false};
		bool lost{false};
	};

	/// 构造函数 
	explicit TargetFusionProcessor(const Config &config);

	void updateTcDetection(const msg::TcDetection::SharedPtr &msg);
	void updateGcDetection(const msg::TcDetection::SharedPtr &msg);
	void updateGimbalState(const msg::GimbalState::SharedPtr &msg);
	std::optional<Result> process(const rclcpp::Time &now);

private:
	static constexpr uint8_t kTargetTypeGcDetection = 0U;
	static constexpr uint8_t kTargetTypeTcDetection = 1U;

	struct TimestampedDetection
	{
		rclcpp::Time stamp;
		msg::TcDetection::SharedPtr data;
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

	/**
	 * @brief 获取最新的TC联合检测（bbox + 点云）
	 * @return 指向最新检测的指针，若无则返回nullptr
	 */
	const TimestampedDetection *latestTcDetection() const;

	/**
	 * @brief 获取最新的GC联合检测（bbox + 点云）
	 * @return 指向最新检测的指针，若无则返回nullptr
	 */
	const TimestampedDetection *latestGcDetection() const;

	/**
	 * @brief 在给定时间戳附近查找时间最接近的TC检测
	 * @param stamp 目标时间戳
	 * @return 若找到则返回检测指针，否则返回std::nullopt
	 */
	std::optional<msg::TcDetection::SharedPtr> findNearestTcDetection(
		const rclcpp::Time &stamp,
		rclcpp::Time *matched_stamp = nullptr) const;

	/**
	 * @brief 在给定时间戳附近查找时间最接近的GC检测
	 * @param stamp 目标时间戳
	 * @return 若找到则返回检测指针，否则返回std::nullopt
	 */
	std::optional<msg::TcDetection::SharedPtr> findNearestGcDetection(
		const rclcpp::Time &stamp,
		rclcpp::Time *matched_stamp = nullptr) const;
	
	/**
	 * @brief 清理过期数据，保持缓冲区内数据的时间范围在当前时间的buffer_keep_s内
	 * @param now 当前时间
	 */
	void pruneBuffers(const rclcpp::Time &now);

	/**
	 * @brief 判断两时间是否同步
	 * 若时间差小于config_.time_sync_threshold_s，则认为同步
	 * @param lhs 时间1
	 * @param rhs 时间2
	 * @return 是否同步
	 */
	bool withinSyncThreshold(const rclcpp::Time &lhs, const rclcpp::Time &rhs) const;
	
	/**
	 * @brief 解析bbox信息，提取目标ID和检测置信度
	 * @param data bbox数据
	 * @param target_id 输出参数，提取的目标ID
	 * @param detection_conf 输出参数，提取的检测置信度
	 */
	void parseDetectionMeta(
		const std::vector<float> &data,
		uint32_t &target_id,
		float &detection_conf) const;
	
	/**
	 * @brief 构建ROI边界
	 * 根据bbox中心和尺寸，以及图像尺寸和预设的margin，计算ROI在图像中的边界
	 * @param data bbox数据，包含中心坐标和尺寸信息
	 * @return ROI边界，若数据无效则返回std::nullopt
	 */
	std::optional<RoiBounds> buildRoi(const std::vector<float> &data) const;

	/**
	 * @brief 从点云中提取ROI内点的质心
	 * 根据给定的ROI边界，从点云中筛选出位于ROI内的点，并计算这些点的质心坐标
	 * @param cloud 输入点云数据
	 * @param roi ROI边界
	 * @return 质心坐标和相关信息，若ROI内无有效点则返回std::nullopt
	 */
	std::optional<RoiResult> extractRoiCentroid(const sensor_msgs::msg::PointCloud2 &cloud, const RoiBounds &roi) const;
	std::optional<RoiResult> extractRoiCentroidInBody(
		const sensor_msgs::msg::PointCloud2 &cloud,
		const RoiBounds &roi,
		double camera_to_gimbal_x,
		double camera_to_gimbal_y,
		double camera_to_gimbal_z) const;

	/**
	 * @brief 更新跟踪状态
	 * 根据当前观测结果更新跟踪状态，包括位置、速度和稳定性等
	 * @param observed 当前观测到的目标位置
	 * @param now 当前时间
	 * @param has_observation 是否有有效观测结果
	 */
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
	
	/**
	 * @brief 构建丢失结果
	 * @details 当目标丢失时，构建一个包含丢失状态的Result对象，方便上层处理
	 * @param now 当前时间
	 * @param detection_data 最近的检测数据，用于提取目标ID和检测置信度
	 * @param target_type 丢失观测仍保留的数据来源类型（TC=1, GC=0）
	 * @return 包含丢失状态的Result对象
	 */
	std::optional<Result> buildLostResult(
		const rclcpp::Time &now,
		const msg::TcDetection &detection_data,
		uint8_t target_type,
		uint8_t source);
	double computeConfidence(float detection_conf, double cluster_quality, double stability, bool lost) const;
	
	Config config_;
	PoseEstimator pose_estimator_;
	std::deque<TimestampedDetection> tc_detection_buffer_;
	std::deque<TimestampedDetection> gc_detection_buffer_;

	bool track_initialized_{false};
	geometry_msgs::msg::Point track_position_{};
	geometry_msgs::msg::Point track_velocity_{};
	double stability_{0.5};
	rclcpp::Time last_track_time_{0, 0, RCL_ROS_TIME};

	uint32_t obs_id_{0};
	uint32_t track_id_{1};
};

} // namespace c3_drone_driver
