#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace usv_perception
{

class AisSimulator : public rclcpp::Node
{
public:
  AisSimulator()
  : Node("ais_simulator")
  {
    model_states_topic_ = declare_parameter<std::string>("model_states_topic", "/model_states");
    output_topic_ = declare_parameter<std::string>("output_topic", "/ais/targets");
    usv_model_name_ = declare_parameter<std::string>("usv_model_name", "wamv");
    target_model_names_ = declare_parameter<std::vector<std::string>>(
      "target_model_names",
      std::vector<std::string>{
        "moving_vessel",
        "small_fishing_boat",
        "survey_boat",
        "service_boat",
        "cargo_ship_far",
        "anchored_tanker"});
    mmsi_start_ = declare_parameter<int>("mmsi_start", 413000100);
    publish_rate_ = declare_parameter<double>("publish_rate", 2.0);
    confidence_ = declare_parameter<double>("confidence", 0.90);
    max_range_ = declare_parameter<double>("max_range", 240.0);
    position_noise_stddev_ = declare_parameter<double>("position_noise_stddev", 2.5);
    velocity_noise_stddev_ = declare_parameter<double>("velocity_noise_stddev", 0.35);
    heading_noise_stddev_ = declare_parameter<double>("heading_noise_stddev", 0.10);
    dropout_probability_ = declare_parameter<double>("dropout_probability", 0.12);
    confidence_jitter_ = declare_parameter<double>("confidence_jitter", 0.10);
    random_seed_ = declare_parameter<int>("random_seed", 20260714);
    rng_.seed(static_cast<std::mt19937::result_type>(random_seed_));

    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic_, 10, std::bind(&AisSimulator::on_model_states, this, std::placeholders::_1));
    ais_pub_ = create_publisher<std_msgs::msg::String>(output_topic_, 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(publish_rate_, 0.1)));
    timer_ = create_wall_timer(period, std::bind(&AisSimulator::on_timer, this));
  }

private:
  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    last_model_states_ = *msg;
    has_model_states_ = true;
  }

  void on_timer()
  {
    if (!has_model_states_) {
      return;
    }

    const int usv_index = find_model(last_model_states_, usv_model_name_);
    if (usv_index < 0) {
      return;
    }

    const auto & usv_pose = last_model_states_.pose[static_cast<std::size_t>(usv_index)];
    const auto & usv_twist = last_model_states_.twist[static_cast<std::size_t>(usv_index)];
    const double usv_yaw = yaw_from_quaternion(usv_pose);
    const double cos_yaw = std::cos(usv_yaw);
    const double sin_yaw = std::sin(usv_yaw);

    std::ostringstream status;
    status.setf(std::ios::fixed, std::ios::floatfield);
    status << std::setprecision(3) << "[";
    int published = 0;
    for (std::size_t i = 0; i < last_model_states_.name.size() && i < last_model_states_.pose.size() &&
      i < last_model_states_.twist.size(); ++i)
    {
      const std::string & name = last_model_states_.name[i];
      if (std::find(target_model_names_.begin(), target_model_names_.end(), name) == target_model_names_.end()) {
        continue;
      }

      const double dx = last_model_states_.pose[i].position.x - usv_pose.position.x;
      const double dy = last_model_states_.pose[i].position.y - usv_pose.position.y;
      const double rel_x = cos_yaw * dx + sin_yaw * dy;
      const double rel_y = -sin_yaw * dx + cos_yaw * dy;
      const double range = std::hypot(rel_x, rel_y);
      if (!std::isfinite(range) || range > max_range_) {
        continue;
      }
      if (uniform_(rng_) < dropout_probability_) {
        continue;
      }

      const double dvx = last_model_states_.twist[i].linear.x - usv_twist.linear.x;
      const double dvy = last_model_states_.twist[i].linear.y - usv_twist.linear.y;
      const double rel_vx = cos_yaw * dvx + sin_yaw * dvy;
      const double rel_vy = -sin_yaw * dvx + cos_yaw * dvy;
      const double sog = std::hypot(last_model_states_.twist[i].linear.x, last_model_states_.twist[i].linear.y);
      const double cog = std::atan2(last_model_states_.twist[i].linear.y, last_model_states_.twist[i].linear.x);
      const double heading = yaw_from_quaternion(last_model_states_.pose[i]);
      const double class_id = name == "small_fishing_boat" ? 2.0 : 5.0;
      const double noisy_x = rel_x + sample_normal(position_noise_stddev_);
      const double noisy_y = rel_y + sample_normal(position_noise_stddev_);
      const double noisy_vx = rel_vx + sample_normal(velocity_noise_stddev_);
      const double noisy_vy = rel_vy + sample_normal(velocity_noise_stddev_);
      const double noisy_heading = heading + sample_normal(heading_noise_stddev_);
      const double noisy_confidence = std::clamp(
        confidence_ + sample_normal(confidence_jitter_), 0.45, 0.98);

      if (published++ > 0) {
        status << ",";
      }
      status << "{\"mmsi\":" << (mmsi_start_ + static_cast<int>(i))
             << ",\"name\":\"" << name << "\""
             << ",\"x\":" << noisy_x
             << ",\"y\":" << noisy_y
             << ",\"vx\":" << noisy_vx
             << ",\"vy\":" << noisy_vy
             << ",\"sog\":" << sog
             << ",\"cog\":" << cog
             << ",\"heading\":" << noisy_heading
             << ",\"confidence\":" << noisy_confidence
             << ",\"class_id\":" << class_id
             << ",\"last_source\":\"ais\"}";
    }
    status << "]";

    std_msgs::msg::String msg;
    msg.data = status.str();
    ais_pub_->publish(msg);
  }

  static int find_model(const gazebo_msgs::msg::ModelStates & msg, const std::string & name)
  {
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      if (msg.name[i] == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  double sample_normal(double stddev)
  {
    if (stddev <= 0.0) {
      return 0.0;
    }
    return std::normal_distribution<double>(0.0, stddev)(rng_);
  }

  std::string model_states_topic_{"/model_states"};
  std::string output_topic_{"/ais/targets"};
  std::string usv_model_name_{"wamv"};
  std::vector<std::string> target_model_names_;
  int mmsi_start_{413000100};
  double publish_rate_{2.0};
  double confidence_{0.90};
  double max_range_{240.0};
  double position_noise_stddev_{2.5};
  double velocity_noise_stddev_{0.35};
  double heading_noise_stddev_{0.10};
  double dropout_probability_{0.12};
  double confidence_jitter_{0.10};
  int random_seed_{20260714};
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  bool has_model_states_{false};
  gazebo_msgs::msg::ModelStates last_model_states_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ais_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::AisSimulator>());
  rclcpp::shutdown();
  return 0;
}
