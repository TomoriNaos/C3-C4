#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include <Eigen/Core>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "c3_drone_driver/msg/mission_command.hpp"

namespace c3_drone_driver
{

    class MotionControllerNode : public rclcpp::Node
    {
    public:
        MotionControllerNode()
            : Node("motion_controller_node")
        {
            // config/motion_controller_default.yaml
            control_hz_ = declare_parameter<double>("control_hz", 50.0);
            cruise_speed_mps_ = declare_parameter<double>("cruise_speed_mps", 3.0);
            max_accel_mps2_ = declare_parameter<double>("max_accel_mps2", 1.5);
            arrive_radius_m_ = declare_parameter<double>("arrive_radius_m", 2.0);
            hover_altitude_m_ = declare_parameter<double>("hover_altitude_m", 20.0);
            home_x_ = declare_parameter<double>("home_x", 0.0);
            home_y_ = declare_parameter<double>("home_y", 0.0);
            home_z_ = declare_parameter<double>("home_z", 0.0);

            current_position_ = {home_x_, home_y_, home_z_};
            current_velocity_ = {0.0, 0.0, 0.0};
            target_position_ = current_position_;
            mode_ = Mode::HOLD;

            // 订阅来自主控的消息
            // 目标位置由主控发布的/mission/goal提供，类型为geometry_msgs::msg::PoseStamped
            mission_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                "/mission/goal", 10, std::bind(&MotionControllerNode::onMissionGoal, this, std::placeholders::_1));
        
            // 任务命令由主控发布的/mission/cmd提供，类型为c3_drone_driver::msg::MissionCommand
            mission_cmd_sub_ = create_subscription<msg::MissionCommand>(
                "/mission/cmd", 10, std::bind(&MotionControllerNode::onMissionCommand, this, std::placeholders::_1));

            // 无人机位姿发布器
            vehicle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/px4/vehicle_pose", 10);
            
            // PX4 offboard 离线控制目标发布器
            offboard_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/px4/offboard_goal", 10);

            // 定时器,根据实时更新的内部状态信息，结合PX4 offboard控制接口要求，发布无人机位姿和离线控制目标
            const auto period = std::chrono::duration<double>(1.0 / std::max(control_hz_, 5.0));
            timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&MotionControllerNode::onTick, this));
            last_tick_time_ = now();

            RCLCPP_INFO(get_logger(), "motion_controller_node started");
        }

    private:
        /**
         * @brief 无人机状态枚举
         * @details
         *  -HOLD：保持当前位置
         *  -TRANSIT：前往目标位置
         *  -RETURN：返回起飞点
         */
        enum class Mode : uint8_t
        {
            HOLD = 0,
            TRANSIT = 1,
            RETURN = 2
        };

        /**
         * @brief 目标位置接收器回调函数
         * @param msg 目标位置消息，类型为geometry_msgs::msg::PoseStamped
         * @details
         *  -接收主控发布的目标粗位置，并更新内部目标粗位置
         */
        void onMissionGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            // 主控已完成坐标转换，这里只接收世界系目标点
            target_position_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
            // 如果目标位置的z坐标过小，认为是未设置高度，使用默认悬停高度
            if (std::abs(target_position_[2]) < 1e-6)
            {
                target_position_[2] = hover_altitude_m_;
            }
            mode_ = Mode::TRANSIT;
        }

        /**
         * @brief 任务命令接收器回调函数
         * @param msg 任务命令消息，类型为c3_drone_driver::msg::MissionCommand
         * @details
         * -接收主控发布的任务命令，根据命令类型修改自身状态
         */
        void onMissionCommand(const msg::MissionCommand::SharedPtr msg)
        {
            // 若为CMD_BACK命令，进入RETURN模式（返航目标由主控发布）
            if (msg->command == msg::MissionCommand::CMD_BACK)
            {
                mode_ = Mode::RETURN;
                return;
            }

            // 若为CMD_HOLD命令，设置目标位置为当前位置，速度变为0，切换到HOLD模式
            if (msg->command == msg::MissionCommand::CMD_HOLD)
            {
                target_position_ = current_position_;
                current_velocity_ = {0.0, 0.0, 0.0};
                mode_ = Mode::HOLD;
                return;
            }
        }

        /**
         * @brief 定时器回调函数，执行无人机运动控制逻辑
         * 根据当前模式和状态计算无人机的下一个位置和速度，并发布无人机位姿和离线控制目标 
         */
        void onTick()
        {
            const auto now_time = now();
            const double dt = std::clamp((now_time - last_tick_time_).seconds(), 0.001, 0.2);
            last_tick_time_ = now_time;

            const Eigen::Vector3d current(current_position_[0], current_position_[1], current_position_[2]);
            const Eigen::Vector3d target(target_position_[0], target_position_[1], target_position_[2]);
            const Eigen::Vector3d velocity(current_velocity_[0], current_velocity_[1], current_velocity_[2]);

            const Eigen::Vector3d delta = target - current;
            const double distance = delta.norm();

            Eigen::Vector3d desired_velocity = Eigen::Vector3d::Zero();
            if (distance > arrive_radius_m_)
            {
                desired_velocity = (delta / std::max(distance, 1e-6)) * cruise_speed_mps_;
            }
            else
            {
                mode_ = Mode::HOLD;
            }

            const Eigen::Vector3d vel_error = desired_velocity - velocity;
            const double vel_error_norm = vel_error.norm();
            const double max_dv = max_accel_mps2_ * dt;
            Eigen::Vector3d dv = vel_error;
            if (vel_error_norm > max_dv && vel_error_norm > 1e-9)
            {
                dv = vel_error * (max_dv / vel_error_norm);
            }

            const Eigen::Vector3d next_velocity = velocity + dv;
            const Eigen::Vector3d next_position = current + next_velocity * dt;

            current_velocity_ = {next_velocity.x(), next_velocity.y(), next_velocity.z()};
            current_position_ = {next_position.x(), next_position.y(), next_position.z()};

            publishVehiclePose(now_time);
            publishOffboardGoal(now_time);
        }

        void publishVehiclePose(const rclcpp::Time &stamp)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = stamp;
            pose.header.frame_id = "ned";
            pose.pose.position.x = current_position_[0];
            pose.pose.position.y = current_position_[1];
            pose.pose.position.z = current_position_[2];
            pose.pose.orientation.w = 1.0;
            vehicle_pose_pub_->publish(pose);
        }

        void publishOffboardGoal(const rclcpp::Time &stamp)
        {
            geometry_msgs::msg::PoseStamped goal;
            goal.header.stamp = stamp;
            goal.header.frame_id = "ned";
            goal.pose.position.x = target_position_[0];
            goal.pose.position.y = target_position_[1];
            goal.pose.position.z = target_position_[2];
            goal.pose.orientation.w = 1.0;
            offboard_goal_pub_->publish(goal);
        }

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mission_goal_sub_;
        rclcpp::Subscription<msg::MissionCommand>::SharedPtr mission_cmd_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr offboard_goal_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        double control_hz_{50.0};
        double cruise_speed_mps_{3.0};
        double max_accel_mps2_{1.5};
        double arrive_radius_m_{2.0};
        double hover_altitude_m_{20.0};
        double home_x_{0.0};
        double home_y_{0.0};
        double home_z_{0.0};

        Mode mode_{Mode::HOLD};
        std::array<double, 3> current_position_{0.0, 0.0, 0.0};
        std::array<double, 3> current_velocity_{0.0, 0.0, 0.0};
        std::array<double, 3> target_position_{0.0, 0.0, 0.0};
        rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
    };

} // namespace c3_drone_driver

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<c3_drone_driver::MotionControllerNode>());
    rclcpp::shutdown();
    return 0;
}
