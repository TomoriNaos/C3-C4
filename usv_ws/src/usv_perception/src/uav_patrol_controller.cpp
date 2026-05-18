#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "usv_perception/common.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace usv_perception
{

class UavPatrolController : public rclcpp::Node
{
public:
  UavPatrolController()
  : Node("uav_patrol_controller"), start_time_(std::chrono::steady_clock::now())
  {
    model_name_ = declare_parameter<std::string>("model_name", "als_uav");
    const double update_rate = declare_parameter<double>("update_rate", 8.0);
    center_x_ = declare_parameter<double>("center_x", 16.0);
    center_y_ = declare_parameter<double>("center_y", 0.0);
    altitude_ = declare_parameter<double>("altitude", 26.0);
    radius_x_ = declare_parameter<double>("radius_x", 22.0);
    radius_y_ = declare_parameter<double>("radius_y", 16.0);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.045);
    camera_pitch_ = declare_parameter<double>("camera_pitch", 0.30);

    client_ = create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("uav/als/status_marker", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(update_rate, 0.1)));
    timer_ = create_wall_timer(period, std::bind(&UavPatrolController::on_timer, this));
  }

private:
  double elapsed_seconds() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  }

  void on_timer()
  {
    const auto state = make_state(elapsed_seconds());
    publish_tf(state);
    publish_marker(state);

    if (!client_->service_is_ready()) {
      if (!warned_waiting_) {
        RCLCPP_INFO(get_logger(), "Waiting for /set_entity_state to move ALS UAV");
        warned_waiting_ = true;
      }
      return;
    }

    auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    req->state = state;
    client_->async_send_request(req);
  }

  gazebo_msgs::msg::EntityState make_state(double t) const
  {
    const double phase = angular_speed_ * t;
    const double x = center_x_ + radius_x_ * std::cos(phase);
    const double y = center_y_ + radius_y_ * std::sin(phase);
    const double z = altitude_ + 1.5 * std::sin(0.31 * phase);
    const double vx = -radius_x_ * angular_speed_ * std::sin(phase);
    const double vy = radius_y_ * angular_speed_ * std::cos(phase);
    const double yaw = std::atan2(center_y_ - y, center_x_ - x);

    gazebo_msgs::msg::EntityState state;
    state.name = model_name_;
    state.reference_frame = "world";
    state.pose.position.x = x;
    state.pose.position.y = y;
    state.pose.position.z = z;
    state.pose.orientation = quaternion_from_euler(0.0, camera_pitch_, yaw);
    state.twist.linear.x = vx;
    state.twist.linear.y = vy;
    state.twist.linear.z = 0.0;
    return state;
  }

  void publish_tf(const gazebo_msgs::msg::EntityState & state)
  {
    const builtin_interfaces::msg::Time now = get_clock()->now();

    geometry_msgs::msg::TransformStamped uav_tf;
    uav_tf.header.stamp = now;
    uav_tf.header.frame_id = "world";
    uav_tf.child_frame_id = "als_uav/base_link";
    uav_tf.transform.translation.x = state.pose.position.x;
    uav_tf.transform.translation.y = state.pose.position.y;
    uav_tf.transform.translation.z = state.pose.position.z;
    uav_tf.transform.rotation = state.pose.orientation;

    geometry_msgs::msg::TransformStamped camera_tf;
    camera_tf.header.stamp = now;
    camera_tf.header.frame_id = "als_uav/base_link";
    camera_tf.child_frame_id = "uav_front_camera_link";
    camera_tf.transform.translation.x = 0.45;
    camera_tf.transform.translation.y = 0.0;
    camera_tf.transform.translation.z = -0.10;
    camera_tf.transform.rotation = quaternion_from_euler(0.0, 0.22, 0.0);

    geometry_msgs::msg::TransformStamped als_tf;
    als_tf.header.stamp = now;
    als_tf.header.frame_id = "als_uav/base_link";
    als_tf.child_frame_id = "uav_als_link";
    als_tf.transform.translation.x = 0.12;
    als_tf.transform.translation.y = 0.0;
    als_tf.transform.translation.z = -0.22;
    als_tf.transform.rotation = quaternion_from_euler(0.0, 0.58, 0.0);

    std::vector<geometry_msgs::msg::TransformStamped> transforms{uav_tf, camera_tf, als_tf};
    tf_broadcaster_->sendTransform(transforms);
  }

  void publish_marker(const gazebo_msgs::msg::EntityState & state)
  {
    visualization_msgs::msg::MarkerArray markers;
    const builtin_interfaces::msg::Time now = get_clock()->now();

    visualization_msgs::msg::Marker uav;
    uav.header.frame_id = "world";
    uav.header.stamp = now;
    uav.ns = "als_uav";
    uav.id = 1;
    uav.type = visualization_msgs::msg::Marker::CUBE;
    uav.action = visualization_msgs::msg::Marker::ADD;
    uav.pose = state.pose;
    uav.scale.x = 1.2;
    uav.scale.y = 0.45;
    uav.scale.z = 0.18;
    uav.color.r = 0.05;
    uav.color.g = 0.28;
    uav.color.b = 0.90;
    uav.color.a = 0.92;
    uav.lifetime.sec = 1;
    markers.markers.push_back(uav);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = "world";
    text.header.stamp = now;
    text.ns = "als_uav";
    text.id = 2;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = state.pose.position.x;
    text.pose.position.y = state.pose.position.y;
    text.pose.position.z = state.pose.position.z + 1.6;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.8;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    text.text = "ALS UAV";
    text.lifetime.sec = 1;
    markers.markers.push_back(text);

    marker_pub_->publish(markers);
  }

  std::string model_name_{"als_uav"};
  double center_x_{16.0};
  double center_y_{0.0};
  double altitude_{26.0};
  double radius_x_{22.0};
  double radius_y_{16.0};
  double angular_speed_{0.045};
  double camera_pitch_{0.30};
  bool warned_waiting_{false};
  std::chrono::steady_clock::time_point start_time_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::UavPatrolController>());
  rclcpp::shutdown();
  return 0;
}
