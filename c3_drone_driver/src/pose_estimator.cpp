#include "c3_drone_driver/pose_estimator.h"

#include <cmath>

#include <Eigen/Geometry>

namespace c3_drone_driver
{

	PoseEstimator::PoseEstimator(const Config &config)
		: config_(config)
	{
	}

	void PoseEstimator::updateVehiclePose(const geometry_msgs::msg::PoseStamped &pose_msg)
	{
		const auto &p = pose_msg.pose.position;
		const auto &q = pose_msg.pose.orientation;
		t_ned_body_ = Vec3(p.x, p.y, p.z);
		r_ned_body_ = quatToRotNedBody(q.x, q.y, q.z, q.w);
		last_pose_stamp_ = rclcpp::Time(pose_msg.header.stamp);
		has_vehicle_pose_ = true;
	}

	void PoseEstimator::updateGimbalState(const msg::GimbalState &gimbal_msg)
	{
		gimbal_yaw_ = static_cast<double>(gimbal_msg.yaw);
		gimbal_pitch_ = static_cast<double>(gimbal_msg.pitch);
		last_gimbal_stamp_ = rclcpp::Time(gimbal_msg.header.stamp);
		has_gimbal_state_ = true;
	}

	bool PoseEstimator::hasGimbalState() const
	{
		return has_gimbal_state_;
	}

	std::optional<PoseEstimator::Result> PoseEstimator::transformObservation(
		const msg::TargetObservation &obs) const
	{
		if (!has_vehicle_pose_ || !has_gimbal_state_)
		{
			return std::nullopt;
		}

		const Vec3 p_in(obs.position.x, obs.position.y, obs.position.z);
		const Mat3 r_body_gimbal = buildRBodyGimbal(gimbal_yaw_, gimbal_pitch_);
		const Vec3 t_body_gimbal(
			config_.body_to_gimbal_x, config_.body_to_gimbal_y, config_.body_to_gimbal_z);

		Vec3 p_body = p_in;
		if (obs.frame == msg::TargetObservation::FRAME_BODY_DRONE)
		{
			p_body = p_in;
		}
		else
		{
			p_body = r_body_gimbal * p_in + t_body_gimbal;
		}

		const Vec3 p_ned = t_ned_body_ + r_ned_body_ * p_body;

		const auto [yaw_body, pitch_body] = bearing(p_body);
		Result out;
		out.position_gimbal = toArray3(p_in);
		out.position_body = toArray3(p_body);
		out.position_ned = toArray3(p_ned);
		out.range = p_body.norm();
		out.yaw_body = yaw_body;
		out.pitch_body = pitch_body;
		return out;
	}

	std::optional<std::array<double, 3>> PoseEstimator::cameraOpticalPointToBody(
		const std::array<double, 3> &point_camera_optical,
		double camera_to_gimbal_x,
		double camera_to_gimbal_y,
		double camera_to_gimbal_z) const
	{
		if (!has_gimbal_state_)
		{
			return std::nullopt;
		}

		const Mat3 r_body_gimbal = buildRBodyGimbal(gimbal_yaw_, gimbal_pitch_);
		const Mat3 r_gimbal_camera = (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())).toRotationMatrix();
		const Vec3 t_body_gimbal(
			config_.body_to_gimbal_x, config_.body_to_gimbal_y, config_.body_to_gimbal_z);
		const Vec3 t_gimbal_camera(camera_to_gimbal_x, camera_to_gimbal_y, camera_to_gimbal_z);
		const Vec3 p_camera(point_camera_optical[0], point_camera_optical[1], point_camera_optical[2]);
		const Vec3 p_gimbal = t_gimbal_camera + r_gimbal_camera * p_camera;
		const Vec3 p_body = r_body_gimbal * p_gimbal + t_body_gimbal;
		return toArray3(p_body);
	}

	std::optional<std::pair<double, double>> PoseEstimator::bodyPointToGimbalYawPitch(
		const std::array<double, 3> &point_body) const
	{
		if (!has_gimbal_state_)
		{
			return std::nullopt;
		}
		const Mat3 r_body_gimbal = buildRBodyGimbal(gimbal_yaw_, gimbal_pitch_);
		const Mat3 r_gimbal_body = r_body_gimbal.transpose();
		const Vec3 t_body_gimbal(
			config_.body_to_gimbal_x, config_.body_to_gimbal_y, config_.body_to_gimbal_z);
		const Vec3 rel = fromArray3(point_body) - t_body_gimbal;
		const Vec3 p_gimbal = r_gimbal_body * rel;
		return bearing(p_gimbal);
	}

	PoseEstimator::Mat3 PoseEstimator::buildRBodyGimbal(double yaw, double pitch) const
	{
		const Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());
		const Eigen::AngleAxisd pitch_rot(pitch, Eigen::Vector3d::UnitY());
		return (yaw_rot * pitch_rot).toRotationMatrix();
	}

	PoseEstimator::Mat3 PoseEstimator::quatToRotNedBody(double x, double y, double z, double w)
	{
		const double n = std::sqrt(x * x + y * y + z * z + w * w);
		if (n < 1e-9)
		{
			return Mat3::Identity();
		}
		x /= n;
		y /= n;
		z /= n;
		w /= n;
		const Eigen::Quaterniond q(w, x, y, z);
		return q.normalized().toRotationMatrix();
	}

	std::pair<double, double> PoseEstimator::bearing(const Vec3 &v)
	{
		const double yaw = std::atan2(v.y(), v.x());
		const double pitch = std::atan2(v.z(), std::hypot(v.x(), v.y()));
		return {yaw, pitch};
	}

	std::array<double, 3> PoseEstimator::toArray3(const Vec3 &v)
	{
		return {v.x(), v.y(), v.z()};
	}

	PoseEstimator::Vec3 PoseEstimator::fromArray3(const std::array<double, 3> &v)
	{
		return Vec3(v[0], v[1], v[2]);
	}

	std::array<double, 3> PoseEstimator::shipRelativePointToNed(
		const geometry_msgs::msg::PoseStamped &ship_pose_world,
		const std::array<double, 3> &target_rel_ship)
	{
		const auto &sp = ship_pose_world.pose.position;
		const auto &sq = ship_pose_world.pose.orientation;
		const double ship_yaw = quatYaw(sq.x, sq.y, sq.z, sq.w);
		const double c = std::cos(ship_yaw);
		const double s = std::sin(ship_yaw);

		const double rx = target_rel_ship[0];
		const double ry = target_rel_ship[1];
		const double rz = target_rel_ship[2];
		return {
			sp.x + c * rx - s * ry,
			sp.y + s * rx + c * ry,
			sp.z + rz};
	}

	double PoseEstimator::quatYaw(double x, double y, double z, double w)
	{
		const double siny_cosp = 2.0 * (w * z + x * y);
		const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
		return std::atan2(siny_cosp, cosy_cosp);
	}

} // namespace c3_drone_driver
