#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "usv_perception/common.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace usv_perception
{

class DynamicTargetController : public rclcpp::Node
{
public:
  DynamicTargetController()
  : Node("dynamic_target_controller"), start_time_(std::chrono::steady_clock::now())
  {
    start_ros_seconds_ = get_clock()->now().seconds();
    const double update_rate = declare_parameter<double>("update_rate", 15.0);
    wave_amplitude_ = declare_parameter<double>("wave_amplitude", 0.10);
    motion_time_scale_ = declare_parameter<double>("motion_time_scale", 3.0);
    vessel_name_ = declare_parameter<std::string>("vessel_name", "moving_vessel");
    fishing_boat_name_ = declare_parameter<std::string>("fishing_boat_name", "small_fishing_boat");
    fishnet_buoy_name_ = declare_parameter<std::string>("fishnet_buoy_name", "fishnet_buoy");
    obstacle_name_ = declare_parameter<std::string>("obstacle_name", "floating_obstacle");
    debris_name_ = declare_parameter<std::string>("debris_name", "drift_debris");
    survey_boat_name_ = declare_parameter<std::string>("survey_boat_name", "survey_boat");
    service_boat_name_ = declare_parameter<std::string>("service_boat_name", "service_boat");
    container_name_ = declare_parameter<std::string>("container_name", "floating_container");
    platform_name_ = declare_parameter<std::string>("platform_name", "research_platform");
    tracked_target_name_ = declare_parameter<std::string>("tracked_target_name", vessel_name_);
    annotation_mode_ = declare_parameter<bool>("annotation_mode", false);
    annotation_occlusion_mode_ = declare_parameter<bool>("annotation_occlusion_mode", false);
    annotation_scene_duration_ = declare_parameter<double>("annotation_scene_duration", 2.4);

    client_ = create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("simulated_targets", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(update_rate, 0.1)));
    timer_ = create_wall_timer(period, std::bind(&DynamicTargetController::on_timer, this));
  }

private:
  struct Waypoint
  {
    double x;
    double y;
  };

  struct AnnotationObject
  {
    std::string name;
    double base_z;
    double base_x;
    double base_y;
  };

  double elapsed_seconds()
  {
    const double ros_now = get_clock()->now().seconds();
    if (std::isfinite(ros_now) && ros_now >= start_ros_seconds_) {
      return ros_now - start_ros_seconds_;
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  }

  void on_timer()
  {
    if (!client_->service_is_ready()) {
      if (!warned_waiting_) {
        RCLCPP_INFO(get_logger(), "Waiting for /set_entity_state to move simulated targets");
        warned_waiting_ = true;
      }
      return;
    }

    const double t = elapsed_seconds() * motion_time_scale_;
    if (annotation_mode_) {
      if (annotation_occlusion_mode_) {
        publish_occlusion_annotation_targets(t);
        return;
      }
      publish_annotation_targets(t);
      return;
    }

    std::vector<gazebo_msgs::msg::EntityState> targets;
    targets.reserve(7);
    targets.push_back(path_state(
      vessel_name_,
      {{18.0, -18.0}, {42.0, -8.0}, {66.0, 11.0}, {82.0, 28.0}, {58.0, 47.0},
        {30.0, 31.0}, {15.0, 8.0}},
      0.42, 0.30, t, 0.0));
    targets.push_back(path_state(
      fishing_boat_name_,
      {{-18.0, 34.0}, {8.0, 42.0}, {36.0, 37.0}, {62.0, 21.0}, {78.0, -4.0}},
      0.34, 0.17, t, 35.0));
    targets.push_back(path_state(
      fishnet_buoy_name_,
      {{24.0, -32.0}, {28.0, -18.0}, {34.0, -4.0}, {42.0, 10.0}, {48.0, 26.0}},
      0.30, 0.045, t, 90.0));
    targets.push_back(path_state(
      obstacle_name_,
      {{52.0, -46.0}, {57.0, -28.0}, {62.0, -10.0}, {70.0, 8.0}, {76.0, 24.0}},
      0.24, 0.060, t, 150.0));
    targets.push_back(path_state(
      debris_name_,
      {{8.0, 48.0}, {20.0, 52.0}, {37.0, 48.0}, {53.0, 42.0}, {71.0, 34.0}},
      0.16, 0.035, t, 210.0));
    targets.push_back(path_state(
      survey_boat_name_,
      {{-35.0, -24.0}, {-10.0, -28.0}, {18.0, -25.0}, {45.0, -18.0}, {72.0, -6.0}},
      0.36, 0.20, t, 20.0));
    targets.push_back(path_state(
      service_boat_name_,
      {{95.0, 36.0}, {74.0, 22.0}, {53.0, 12.0}, {31.0, 5.0}, {12.0, -2.0}},
      0.38, 0.15, t, 75.0));

    for (const auto & target : targets) {
      set_target_state(target);
    }
    publish_markers(targets);
  }

  void publish_occlusion_annotation_targets(double t)
  {
    const std::vector<AnnotationObject> subjects{
      {vessel_name_, 0.42, 0.0, 0.0},
      {fishing_boat_name_, 0.34, 0.0, 0.0},
      {survey_boat_name_, 0.36, 0.0, 0.0},
      {service_boat_name_, 0.38, 0.0, 0.0},
      {fishnet_buoy_name_, 0.30, 0.0, 0.0},
      {obstacle_name_, 0.24, 0.0, 0.0},
      {debris_name_, 0.16, 0.0, 0.0},
      {container_name_, 0.20, 0.0, 0.0},
      {platform_name_, 0.45, 0.0, 0.0}};

    std::vector<gazebo_msgs::msg::EntityState> targets;
    targets.reserve(subjects.size());
    for (const auto & subject : subjects) {
      targets.push_back(hidden_state(subject.name, subject.base_z));
    }

    const double scene_duration = std::max(0.8, annotation_scene_duration_);
    const int scene_index = static_cast<int>(std::floor(t / scene_duration));
    const double local_t = std::fmod(t, scene_duration);
    constexpr double two_pi = 6.28318530717958647692;
    constexpr int scene_count = 18;
    const int scene = scene_index % scene_count;
    const int cycle = scene_index / scene_count;
    const double phase = std::clamp(local_t / scene_duration, 0.0, 1.0);
    const double centered_phase = phase - 0.5;
    const double fast = std::sin(2.0 * two_pi * phase);
    const double cycle_lateral_shift = 0.22 * std::sin(0.73 * static_cast<double>(cycle) + 0.41 * scene);
    const double cycle_range_shift = 0.30 * std::cos(0.37 * static_cast<double>(cycle) + 0.29 * scene);

    auto set_named = [&](
        const std::string & name, double x, double y, double z, double yaw,
        double along_motion = 0.0, double lateral_motion = 0.0, double yaw_motion = 0.0) {
      const double animated_x =
        x + cycle_range_shift + along_motion * centered_phase + 0.07 * std::sin(two_pi * phase + x);
      const double animated_y =
        y + cycle_lateral_shift + lateral_motion * centered_phase + 0.06 * std::cos(two_pi * phase + y);
      const double animated_yaw = yaw + yaw_motion * centered_phase + 0.04 * fast;
      const double vx = along_motion / scene_duration;
      const double vy = lateral_motion / scene_duration;
      for (auto & state : targets) {
        if (state.name == name) {
          state = make_state(
            name, animated_x, animated_y,
            z + wave_height(animated_x, animated_y, t, wave_amplitude_), animated_yaw, vx, vy);
          return;
        }
      }
    };

    switch (scene) {
      case 0:
        set_named(vessel_name_, 12.8, 0.35, 0.42, 0.12, 0.70, -0.28, 0.25);
        set_named(survey_boat_name_, 9.8, 0.08, 0.36, -0.08, 0.36, 0.18, 0.18);
        set_named(obstacle_name_, 7.2, -0.34, 0.24, 0.0, 0.02, 0.10, 0.0);
        set_named(debris_name_, 7.8, -2.10, 0.16, 0.16, 0.14, 0.36, 0.15);
        set_named(fishnet_buoy_name_, 15.4, 1.32, 0.30, 0.0, -0.05, -0.08, 0.0);
        break;
      case 1:
        set_named(container_name_, 7.0, 1.55, 0.20, -0.18, -0.08, -0.34, 0.18);
        set_named(fishing_boat_name_, 10.6, 0.70, 0.34, 0.28, 0.52, -0.18, 0.35);
        set_named(vessel_name_, 14.5, 0.25, 0.42, 0.02, 0.28, 0.08, 0.12);
        set_named(fishnet_buoy_name_, 12.6, -1.42, 0.30, 0.0, 0.05, 0.08, 0.0);
        set_named(platform_name_, 17.8, -0.72, 0.45, 0.0, -0.04, 0.06, 0.0);
        break;
      case 2:
        set_named(service_boat_name_, 9.2, -0.68, 0.38, 0.16, 0.72, 0.16, 0.36);
        set_named(survey_boat_name_, 11.4, -0.92, 0.36, -0.20, -0.20, 0.12, 0.20);
        set_named(obstacle_name_, 8.1, -0.70, 0.24, 0.0, 0.00, -0.04, 0.0);
        set_named(container_name_, 15.4, 1.10, 0.20, 0.20, -0.08, -0.14, 0.12);
        set_named(fishnet_buoy_name_, 17.2, -1.78, 0.30, 0.0, 0.00, 0.12, 0.0);
        break;
      case 3:
        set_named(platform_name_, 11.2, 0.16, 0.45, 0.0, 0.05, -0.06, 0.0);
        set_named(vessel_name_, 13.4, 0.55, 0.42, 0.10, 0.34, -0.10, 0.16);
        set_named(obstacle_name_, 8.4, -0.02, 0.24, 0.0, 0.02, 0.04, 0.0);
        set_named(debris_name_, 7.2, 1.85, 0.16, -0.25, 0.24, -0.22, 0.22);
        set_named(service_boat_name_, 16.2, -1.20, 0.38, -0.10, -0.15, 0.15, 0.12);
        break;
      case 4:
        set_named(survey_boat_name_, 9.0, -1.10, 0.36, 0.35, 0.48, 0.22, 0.38);
        set_named(fishnet_buoy_name_, 7.0, -0.92, 0.30, 0.0, 0.00, 0.10, 0.0);
        set_named(container_name_, 12.0, -1.38, 0.20, -0.15, 0.18, 0.02, 0.16);
        set_named(vessel_name_, 17.2, 1.55, 0.42, -0.05, -0.18, -0.10, 0.08);
        set_named(debris_name_, 8.2, 1.95, 0.16, 0.10, 0.20, -0.18, 0.12);
        break;
      case 5:
        set_named(container_name_, 8.2, 0.88, 0.20, -0.15, 0.24, 0.14, 0.45);
        set_named(vessel_name_, 12.8, 1.28, 0.42, 0.18, 0.40, -0.12, 0.28);
        set_named(fishnet_buoy_name_, 6.8, -1.62, 0.30, 0.0, 0.00, 0.08, 0.0);
        set_named(service_boat_name_, 16.4, -0.45, 0.38, 0.18, -0.18, 0.20, 0.18);
        set_named(obstacle_name_, 11.0, 1.65, 0.24, 0.0, 0.02, -0.12, 0.0);
        break;
      case 6:
        set_named(debris_name_, 6.6, -0.72, 0.16, 0.15, 0.32, 0.12, 0.30);
        set_named(fishing_boat_name_, 10.1, -1.10, 0.34, -0.12, 0.50, 0.16, 0.36);
        set_named(obstacle_name_, 13.2, 1.16, 0.24, 0.0, -0.04, -0.08, 0.0);
        set_named(platform_name_, 18.0, 1.82, 0.45, 0.0, 0.00, -0.08, 0.0);
        set_named(vessel_name_, 15.5, -1.50, 0.42, 0.08, -0.18, 0.10, 0.10);
        break;
      case 7:
        set_named(obstacle_name_, 7.4, 1.10, 0.24, 0.0, 0.02, -0.10, 0.0);
        set_named(service_boat_name_, 10.8, 1.48, 0.38, 0.18, 0.58, -0.16, 0.34);
        set_named(survey_boat_name_, 14.0, -0.88, 0.36, -0.12, -0.12, 0.12, 0.12);
        set_named(debris_name_, 6.8, -2.20, 0.16, 0.0, 0.22, 0.20, 0.16);
        set_named(container_name_, 17.4, 0.55, 0.20, 0.25, -0.10, -0.10, 0.08);
        break;
      case 8:
        set_named(vessel_name_, 11.8, -1.45, 0.42, 0.22, 0.44, 0.20, 0.26);
        set_named(platform_name_, 9.2, -1.02, 0.45, 0.0, 0.02, 0.08, 0.0);
        set_named(fishnet_buoy_name_, 7.4, 1.62, 0.30, 0.0, 0.00, -0.08, 0.0);
        set_named(container_name_, 15.2, 0.10, 0.20, 0.20, -0.12, -0.06, 0.12);
        set_named(fishing_boat_name_, 18.4, -1.85, 0.34, -0.16, -0.18, 0.08, 0.10);
        break;
      case 9:
        set_named(fishing_boat_name_, 9.0, 0.20, 0.34, -0.22, 0.52, -0.22, 0.42);
        set_named(debris_name_, 6.8, 0.06, 0.16, 0.20, 0.22, 0.10, 0.18);
        set_named(obstacle_name_, 12.5, -1.42, 0.24, 0.0, -0.04, 0.12, 0.0);
        set_named(vessel_name_, 16.8, 1.12, 0.42, -0.10, -0.16, -0.12, 0.08);
        set_named(service_boat_name_, 14.4, 0.40, 0.38, 0.18, 0.18, -0.16, 0.16);
        break;
      case 10:
        set_named(service_boat_name_, 11.7, 0.50, 0.38, -0.20, 0.42, -0.16, 0.32);
        set_named(fishnet_buoy_name_, 8.0, 0.42, 0.30, 0.0, 0.00, -0.08, 0.0);
        set_named(container_name_, 7.4, -1.72, 0.20, 0.35, 0.18, 0.20, 0.20);
        set_named(platform_name_, 17.4, 1.78, 0.45, 0.0, -0.02, -0.10, 0.0);
        set_named(survey_boat_name_, 15.6, 0.10, 0.36, -0.10, -0.12, 0.08, 0.10);
        break;
      case 11:
        set_named(survey_boat_name_, 9.8, -0.35, 0.36, 0.10, 0.46, 0.14, 0.40);
        set_named(vessel_name_, 13.4, -0.70, 0.42, -0.10, 0.18, 0.10, 0.12);
        set_named(debris_name_, 6.4, 1.52, 0.16, -0.15, 0.20, -0.20, 0.18);
        set_named(obstacle_name_, 7.8, -1.90, 0.24, 0.0, 0.00, 0.12, 0.0);
        set_named(fishnet_buoy_name_, 16.5, 1.05, 0.30, 0.0, -0.04, -0.08, 0.0);
        break;
      case 12:
        set_named(vessel_name_, 10.8, 0.88, 0.42, 0.02, 0.52, -0.18, 0.26);
        set_named(container_name_, 8.6, 0.58, 0.20, 0.16, 0.16, -0.04, 0.18);
        set_named(service_boat_name_, 14.8, 0.15, 0.38, -0.14, -0.16, 0.10, 0.12);
        set_named(debris_name_, 7.2, -1.92, 0.16, -0.24, 0.16, 0.20, 0.20);
        set_named(obstacle_name_, 18.2, 1.65, 0.24, 0.0, -0.02, -0.12, 0.0);
        break;
      case 13:
        set_named(platform_name_, 12.4, -0.48, 0.45, 0.0, 0.00, 0.06, 0.0);
        set_named(fishing_boat_name_, 10.4, -0.72, 0.34, 0.24, 0.50, 0.16, 0.36);
        set_named(vessel_name_, 16.0, -0.10, 0.42, -0.06, -0.10, 0.06, 0.08);
        set_named(fishnet_buoy_name_, 8.0, 1.36, 0.30, 0.0, 0.00, -0.10, 0.0);
        set_named(container_name_, 17.6, -1.82, 0.20, 0.24, -0.10, 0.12, 0.10);
        break;
      case 14:
        set_named(service_boat_name_, 8.8, -1.35, 0.38, 0.32, 0.64, 0.28, 0.42);
        set_named(obstacle_name_, 7.6, -1.22, 0.24, 0.0, 0.02, 0.10, 0.0);
        set_named(survey_boat_name_, 12.8, -0.85, 0.36, -0.08, 0.10, 0.04, 0.12);
        set_named(debris_name_, 8.8, 1.68, 0.16, 0.18, 0.24, -0.16, 0.22);
        set_named(vessel_name_, 17.0, 0.98, 0.42, -0.12, -0.18, -0.08, 0.10);
        break;
      case 15:
        set_named(fishing_boat_name_, 9.4, 1.12, 0.34, -0.16, 0.56, -0.24, 0.34);
        set_named(fishnet_buoy_name_, 7.2, 0.98, 0.30, 0.0, 0.00, -0.08, 0.0);
        set_named(vessel_name_, 13.8, 1.42, 0.42, 0.08, 0.26, -0.12, 0.14);
        set_named(platform_name_, 17.8, -1.35, 0.45, 0.0, -0.04, 0.10, 0.0);
        set_named(container_name_, 7.6, -2.05, 0.20, -0.22, 0.16, 0.18, 0.16);
        break;
      case 16:
        set_named(survey_boat_name_, 10.8, 0.08, 0.36, 0.18, 0.48, -0.10, 0.32);
        set_named(container_name_, 6.9, -0.18, 0.20, 0.28, 0.18, 0.10, 0.20);
        set_named(vessel_name_, 15.2, -0.42, 0.42, -0.08, -0.14, 0.06, 0.08);
        set_named(fishnet_buoy_name_, 13.0, 1.55, 0.30, 0.0, 0.02, -0.10, 0.0);
        set_named(debris_name_, 9.2, -2.00, 0.16, -0.20, 0.14, 0.18, 0.18);
        break;
      default:
        set_named(platform_name_, 10.4, 0.72, 0.45, 0.0, 0.00, -0.06, 0.0);
        set_named(service_boat_name_, 12.2, 0.40, 0.38, -0.18, 0.44, -0.12, 0.28);
        set_named(obstacle_name_, 7.5, 0.62, 0.24, 0.0, 0.00, 0.08, 0.0);
        set_named(fishing_boat_name_, 16.4, -1.20, 0.34, 0.18, -0.16, 0.14, 0.10);
        set_named(debris_name_, 7.0, -1.78, 0.16, 0.14, 0.22, 0.20, 0.20);
        break;
    }

    for (const auto & target : targets) {
      set_target_state(target);
    }
    publish_markers(targets);
  }

  void publish_annotation_targets(double t)
  {
    const std::vector<AnnotationObject> subjects{
      {vessel_name_, 0.42, 9.0, 0.0},
      {fishing_boat_name_, 0.34, 7.4, 0.0},
      {survey_boat_name_, 0.36, 8.3, 0.0},
      {service_boat_name_, 0.38, 8.6, 0.0},
      {fishnet_buoy_name_, 0.30, 5.3, 0.0},
      {obstacle_name_, 0.24, 5.7, 0.0},
      {debris_name_, 0.16, 6.2, 0.0},
      {container_name_, 0.20, 6.8, 0.0},
      {platform_name_, 0.45, 12.0, 0.0}};

    std::vector<gazebo_msgs::msg::EntityState> targets;
    targets.reserve(subjects.size());

    const double scene_duration = std::max(0.8, annotation_scene_duration_);
    const double local_t = std::fmod(t, scene_duration);
    constexpr double two_pi = 6.28318530717958647692;
    const int scene = static_cast<int>(std::floor(t / scene_duration)) %
      (static_cast<int>(subjects.size()) + 3);
    const double phase = std::clamp(local_t / scene_duration, 0.0, 1.0);

    for (const auto & subject : subjects) {
      targets.push_back(hidden_state(subject.name, subject.base_z));
    }

    auto set_by_name = [&](const AnnotationObject & subject, double yaw, double x_offset, double y_offset) {
      for (auto & target : targets) {
        if (target.name != subject.name) {
          continue;
        }
        const double x = subject.base_x + x_offset + 0.10 * std::sin(two_pi * phase);
        const double y = subject.base_y + y_offset + 0.10 * std::cos(two_pi * phase);
        target = make_state(
          subject.name, x, y, subject.base_z + wave_height(x, y, t, wave_amplitude_),
          yaw, 0.0, 0.0);
        return;
      }
    };

    if (scene < static_cast<int>(subjects.size())) {
      const auto & subject = subjects[static_cast<std::size_t>(scene)];
      const double distance_offset = (scene % 2 == 0) ? 0.0 : 2.4;
      const double lateral_offset = static_cast<double>((scene % 3) - 1) * 0.55;
      set_by_name(subject, two_pi * phase, distance_offset, lateral_offset);
    } else {
      const int combo_group = scene - static_cast<int>(subjects.size());
      if (combo_group == 0) {
        set_by_name(subjects[0], two_pi * phase, 1.0, -1.25);
        set_by_name(subjects[4], two_pi * phase + 1.8, -1.0, 1.45);
      } else if (combo_group == 1) {
        set_by_name(subjects[1], two_pi * phase, 1.6, 1.05);
        set_by_name(subjects[5], two_pi * phase + 1.3, -1.0, -1.35);
        set_by_name(subjects[6], two_pi * phase + 3.1, 3.0, -0.2);
      } else {
        set_by_name(subjects[8], two_pi * phase, 3.0, -0.85);
        set_by_name(subjects[3], two_pi * phase + 2.4, -1.0, 1.45);
      }
    }

    for (const auto & target : targets) {
      set_target_state(target);
    }
    publish_markers(targets);
  }

  gazebo_msgs::msg::EntityState path_state(
    const std::string & name,
    const std::vector<Waypoint> & path,
    double base_z,
    double speed,
    double t,
    double phase_seconds) const
  {
    if (path.size() < 2) {
      return make_state(name, 0.0, 0.0, base_z, 0.0, 0.0, 0.0);
    }

    double total_length = 0.0;
    std::vector<double> segment_lengths;
    segment_lengths.reserve(path.size() - 1);
    for (std::size_t index = 1; index < path.size(); ++index) {
      const double length = std::hypot(path[index].x - path[index - 1].x, path[index].y - path[index - 1].y);
      segment_lengths.push_back(length);
      total_length += length;
    }

    const double cycle_length = std::max(0.1, 2.0 * total_length);
    double distance = std::fmod(std::max(0.0, t + phase_seconds) * speed, cycle_length);
    bool reverse = false;
    if (distance > total_length) {
      distance = cycle_length - distance;
      reverse = true;
    }

    std::size_t segment_index = 0;
    while (segment_index < segment_lengths.size() && distance > segment_lengths[segment_index]) {
      distance -= segment_lengths[segment_index];
      ++segment_index;
    }
    segment_index = std::min(segment_index, segment_lengths.size() - 1);

    const auto & a = path[segment_index];
    const auto & b = path[segment_index + 1];
    const double length = std::max(0.1, segment_lengths[segment_index]);
    double ratio = std::clamp(distance / length, 0.0, 1.0);
    if (reverse) {
      ratio = 1.0 - ratio;
    }

    const double x = a.x + (b.x - a.x) * ratio;
    const double y = a.y + (b.y - a.y) * ratio;
    const double direction_sign = reverse ? -1.0 : 1.0;
    const double heading_x = direction_sign * (b.x - a.x);
    const double heading_y = direction_sign * (b.y - a.y);
    const double yaw = std::atan2(heading_y, heading_x);
    const double vx = speed * std::cos(yaw);
    const double vy = speed * std::sin(yaw);
    const double z = base_z + wave_height(x, y, t, wave_amplitude_);
    return make_state(name, x, y, z, yaw, vx, vy);
  }

  static gazebo_msgs::msg::EntityState make_state(
    const std::string & name, double x, double y, double z, double yaw, double vx, double vy)
  {
    gazebo_msgs::msg::EntityState state;
    state.name = name;
    state.reference_frame = "world";
    state.pose.position.x = x;
    state.pose.position.y = y;
    state.pose.position.z = z;
    state.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw);
    state.twist.linear.x = vx;
    state.twist.linear.y = vy;
    state.twist.angular.z = 0.0;
    return state;
  }

  static gazebo_msgs::msg::EntityState hidden_state(const std::string & name, double z)
  {
    return make_state(name, -80.0, 0.0, z, 0.0, 0.0, 0.0);
  }

  void set_target_state(const gazebo_msgs::msg::EntityState & state)
  {
    auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    req->state = state;
    client_->async_send_request(req);
  }

  void publish_markers(const std::vector<gazebo_msgs::msg::EntityState> & states)
  {
    visualization_msgs::msg::MarkerArray markers;
    const builtin_interfaces::msg::Time now = get_clock()->now();
    for (std::size_t index = 0; index < states.size(); ++index) {
      const auto & state = states[index];
      const bool is_primary_target = state.name == tracked_target_name_;
      const bool is_vessel =
        state.name == vessel_name_ || state.name == fishing_boat_name_ ||
        state.name == survey_boat_name_ || state.name == service_boat_name_;
      const bool is_buoy = state.name == fishnet_buoy_name_;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = now;
      marker.ns = "simulated_targets";
      marker.id = static_cast<int>(index);
      marker.type = is_buoy ? visualization_msgs::msg::Marker::CYLINDER :
        visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = state.pose;
      marker.scale.x = state.name == vessel_name_ ? 2.2 : (is_vessel ? 1.35 : 0.55);
      marker.scale.y = state.name == vessel_name_ ? 0.8 : (is_vessel ? 0.50 : 0.55);
      marker.scale.z = is_vessel ? 0.55 : 0.42;
      marker.color.r = is_primary_target ? 0.05 : (is_vessel ? 1.0 : 0.9);
      marker.color.g = is_primary_target ? 1.0 : (state.name == vessel_name_ ? 0.85 : (is_vessel ? 0.35 : 0.2));
      marker.color.b = is_primary_target ? 0.25 : (is_vessel ? 0.1 : 0.05);
      marker.color.a = 0.9;
      marker.lifetime.sec = 1;
      markers.markers.push_back(marker);

      visualization_msgs::msg::Marker text;
      text.header.frame_id = "world";
      text.header.stamp = now;
      text.ns = "simulated_target_labels";
      text.id = 100 + static_cast<int>(index);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = state.pose.position.x;
      text.pose.position.y = state.pose.position.y;
      text.pose.position.z = state.pose.position.z + 1.0;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.45;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      text.text = is_primary_target ? "TRACK TARGET: " + state.name : state.name;
      text.lifetime.sec = 1;
      markers.markers.push_back(text);
    }
    marker_pub_->publish(markers);
  }

  double wave_amplitude_{0.10};
  double motion_time_scale_{3.0};
  std::string vessel_name_;
  std::string fishing_boat_name_;
  std::string fishnet_buoy_name_;
  std::string obstacle_name_;
  std::string debris_name_;
  std::string survey_boat_name_;
  std::string service_boat_name_;
  std::string container_name_;
  std::string platform_name_;
  std::string tracked_target_name_;
  bool annotation_mode_{false};
  bool annotation_occlusion_mode_{false};
  double annotation_scene_duration_{2.4};
  bool warned_waiting_{false};
  std::chrono::steady_clock::time_point start_time_;
  double start_ros_seconds_{0.0};
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usv_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_perception::DynamicTargetController>());
  rclcpp::shutdown();
  return 0;
}
