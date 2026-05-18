#pragma once

#include <array>
#include <optional>
#include <utility>

#include <Eigen/Core>

#include "c3_drone_driver/msg/gimbal_state.hpp"
#include "c3_drone_driver/msg/target_observation.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"

namespace c3_drone_driver
{

	/**
	 * @brief 负责将目标观测在云台/机体/NED坐标系之间进行位姿解算与转换
	 */
	class PoseEstimator
	{
	public:
		/**
		 * @brief 自定义pose_estimator配置结构体
		 * @details 详见config/pose_estimator_default.yaml
		 */
		struct Config
		{
			double body_to_gimbal_x{0.0};
			double body_to_gimbal_y{0.0};
			double body_to_gimbal_z{0.0};
		};

		/**
		 * @brief 自定义pose_estimator解算结果
		 * @details 包含
		 * - position_gimbal：目标在云台坐标系下的位置
		 * - position_body：目标在机体坐标系下的位置
		 * - position_ned：目标在NED坐标系下的位置
		 * - range：目标相对距离
		 * - yaw_body：目标在机体系下的方位角
		 * - pitch_body：目标在机体系下的俯仰角
		 */
		struct Result
		{
			std::array<double, 3> position_gimbal{};
			std::array<double, 3> position_body{};
			std::array<double, 3> position_ned{};
			double range{0.0};
			double yaw_body{0.0};
			double pitch_body{0.0};
		};

		/// 构造函数
		explicit PoseEstimator(const Config &config);

		/**
		 * @brief 更新飞行器位姿（NED<-Body）
		 * @param pose_msg 飞行器位姿消息
		 */
		void updateVehiclePose(const geometry_msgs::msg::PoseStamped &pose_msg);

		/**
		 * @brief 更新云台状态（Yaw/Pitch）
		 * @param gimbal_msg 云台状态消息
		 */
		void updateGimbalState(const msg::GimbalState &gimbal_msg);

		/**
		 * @brief 将目标观测从云台系解算到机体系与NED系
		 * @param obs 目标观测消息（位置默认在云台系）
		 * @return 成功返回解算结果；当位姿或云台状态未就绪时返回std::nullopt
		 */
		std::optional<Result> transformObservation(const msg::TargetObservation &obs) const;

		/**
		 * @brief 将机体系目标点反解为云台期望Yaw/Pitch
		 * @param point_body 目标点（机体系）
		 * @return 成功返回<yaw, pitch>；无效输入或不可解时返回std::nullopt
		 */
		std::optional<std::pair<double, double>> bodyPointToGimbalYawPitch(
			const std::array<double, 3> &point_body) const;

	private:
		using Mat3 = Eigen::Matrix3d;
		using Vec3 = Eigen::Vector3d;

		static Mat3 quatToRotNedBody(
			double x, double y, double z, double w);

		static std::pair<double, double> bearing(const Vec3 &v);

		Mat3 buildRBodyGimbal(double yaw, double pitch) const;
		static std::array<double, 3> toArray3(const Vec3 &v);
		static Vec3 fromArray3(const std::array<double, 3> &v);

		Config config_{};
		bool has_vehicle_pose_{false};
		bool has_gimbal_state_{false};
		Mat3 r_ned_body_{Mat3::Identity()};
		Vec3 t_ned_body_{Vec3::Zero()};
		double gimbal_yaw_{0.0};
		double gimbal_pitch_{0.0};
		rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
		rclcpp::Time last_gimbal_stamp_{0, 0, RCL_ROS_TIME};
	};

} // namespace c3_drone_driver
