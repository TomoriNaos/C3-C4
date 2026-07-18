#!/usr/bin/env python3
"""Kinematic flight-dynamics layer for the Gazebo Classic demonstration UAV."""

import json
import math
import random
from collections import deque

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def euler_from_quaternion(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi * 0.5, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class UavFlightSimulator(Node):
    """Applies command delay, acceleration limits, wind and navigation noise."""

    def __init__(self):
        super().__init__('uav_flight_simulator')
        defaults = {
            'model_name': 'scout_uav', 'model_states_topic': '/model_states',
            'command_topic': '/uav/sim/command', 'estimated_pose_topic': '/uav/sim/estimated_pose',
            'estimated_twist_topic': '/uav/sim/estimated_twist', 'status_topic': '/uav/sim/status',
            'update_rate_hz': 30.0, 'command_delay_s': 0.22, 'command_timeout_s': 1.5,
            'max_speed_mps': 5.0, 'position_speed_gain': 0.72, 'max_climb_rate_mps': 2.0, 'max_accel_mps2': 2.4,
            'velocity_time_constant_s': 0.65, 'max_yaw_rate_rps': 1.25,
            'yaw_time_constant_s': 0.45, 'max_bank_rad': 0.28, 'wind_speed_mps': 0.65,
            'wind_direction_rad': 0.35, 'wind_gust_mps': 0.28, 'wind_gust_hz': 0.16,
            'position_noise_stddev_m': 0.18, 'altitude_noise_stddev_m': 0.10,
            'yaw_noise_stddev_rad': 0.025, 'velocity_noise_stddev_mps': 0.06,
            'random_seed': 17,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.model_name = self.get_parameter('model_name').value
        self.command_queue = deque()
        self.active_command = None
        self.last_command_time = None
        self.last_tick_time = None
        self.initialized = False
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.target_pitch = 0.0
        self.random = random.Random(int(self.get_parameter('random_seed').value))

        self.create_subscription(PoseStamped, self.get_parameter('command_topic').value, self.command_callback, 10)
        self.create_subscription(ModelStates, self.get_parameter('model_states_topic').value, self.model_states_callback, 10)
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter('estimated_pose_topic').value, 10)
        self.twist_pub = self.create_publisher(TwistStamped, self.get_parameter('estimated_twist_topic').value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        rate = max(1.0, float(self.get_parameter('update_rate_hz').value))
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f'Flight simulation consumes {self.get_parameter("command_topic").value}')

    def command_callback(self, message):
        due = self.time_seconds() + float(self.get_parameter('command_delay_s').value)
        self.command_queue.append((due, message))

    def model_states_callback(self, message):
        if self.initialized or self.model_name not in message.name:
            return
        index = message.name.index(self.model_name)
        pose = message.pose[index]
        self.position = [pose.position.x, pose.position.y, pose.position.z]
        self.yaw = euler_from_quaternion(pose.orientation)[2]
        if index < len(message.twist):
            twist = message.twist[index]
            self.velocity = [twist.linear.x, twist.linear.y, twist.linear.z]
        self.initialized = True
        self.get_logger().info(f'Initialized {self.model_name} flight state from Gazebo')

    def on_timer(self):
        now = self.time_seconds()
        if self.last_tick_time is None:
            self.last_tick_time = now
            return
        dt = clamp(now - self.last_tick_time, 0.001, 0.15)
        self.last_tick_time = now
        while self.command_queue and self.command_queue[0][0] <= now:
            _, self.active_command = self.command_queue.popleft()
            self.last_command_time = now
        if not self.initialized or self.active_command is None:
            return

        fresh = now - self.last_command_time <= float(self.get_parameter('command_timeout_s').value)
        target_velocity, target_yaw = self.command_velocity(fresh)
        tau = max(0.05, float(self.get_parameter('velocity_time_constant_s').value))
        desired_accel = [(target_velocity[i] - self.velocity[i]) / tau for i in range(3)]
        accel_norm = math.sqrt(sum(value * value for value in desired_accel))
        max_accel = float(self.get_parameter('max_accel_mps2').value)
        if accel_norm > max_accel:
            desired_accel = [value * max_accel / accel_norm for value in desired_accel]
        for index in range(3):
            self.velocity[index] += desired_accel[index] * dt
        wind_x, wind_y = self.wind_velocity(now)
        self.position[0] += (self.velocity[0] + wind_x) * dt
        self.position[1] += (self.velocity[1] + wind_y) * dt
        self.position[2] += self.velocity[2] * dt

        yaw_tau = max(0.05, float(self.get_parameter('yaw_time_constant_s').value))
        max_yaw_rate = float(self.get_parameter('max_yaw_rate_rps').value)
        desired_yaw_rate = clamp(wrap_angle(target_yaw - self.yaw) / yaw_tau, -max_yaw_rate, max_yaw_rate)
        self.yaw_rate += clamp((desired_yaw_rate - self.yaw_rate) * 3.0, -4.0, 4.0) * dt
        self.yaw_rate = clamp(self.yaw_rate, -max_yaw_rate, max_yaw_rate)
        self.yaw = wrap_angle(self.yaw + self.yaw_rate * dt)
        lateral_accel = -math.sin(self.yaw) * desired_accel[0] + math.cos(self.yaw) * desired_accel[1]
        roll = clamp(-math.atan2(lateral_accel, 9.81), -float(self.get_parameter('max_bank_rad').value), float(self.get_parameter('max_bank_rad').value))
        self.write_gazebo_state(roll, clamp(self.target_pitch, -0.45, 0.45), wind_x, wind_y)
        self.publish_estimate(now, roll, wind_x, wind_y, fresh)

    def command_velocity(self, fresh):
        if not fresh:
            return [0.0, 0.0, 0.0], self.yaw
        target = self.active_command.pose.position
        dx, dy, dz = target.x - self.position[0], target.y - self.position[1], target.z - self.position[2]
        horizontal_distance = math.hypot(dx, dy)
        velocity = [0.0, 0.0, clamp(0.9 * dz, -float(self.get_parameter('max_climb_rate_mps').value), float(self.get_parameter('max_climb_rate_mps').value))]
        if horizontal_distance > 1e-4:
            speed = min(
                float(self.get_parameter('max_speed_mps').value),
                float(self.get_parameter('position_speed_gain').value) * horizontal_distance)
            velocity[0], velocity[1] = speed * dx / horizontal_distance, speed * dy / horizontal_distance
        _, self.target_pitch, yaw = euler_from_quaternion(self.active_command.pose.orientation)
        return velocity, yaw

    def wind_velocity(self, now):
        direction = float(self.get_parameter('wind_direction_rad').value)
        speed = float(self.get_parameter('wind_speed_mps').value)
        speed += float(self.get_parameter('wind_gust_mps').value) * math.sin(2.0 * math.pi * float(self.get_parameter('wind_gust_hz').value) * now + 0.9)
        return speed * math.cos(direction), speed * math.sin(direction)

    def write_gazebo_state(self, roll, pitch, wind_x, wind_y):
        if not self.set_state_client.service_is_ready():
            return
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, self.yaw)
        state = EntityState()
        state.name, state.reference_frame = self.model_name, 'world'
        state.pose.position.x, state.pose.position.y, state.pose.position.z = self.position
        state.pose.orientation.x, state.pose.orientation.y = qx, qy
        state.pose.orientation.z, state.pose.orientation.w = qz, qw
        state.twist.linear.x, state.twist.linear.y, state.twist.linear.z = self.velocity[0] + wind_x, self.velocity[1] + wind_y, self.velocity[2]
        state.twist.angular.z = self.yaw_rate
        request = SetEntityState.Request()
        request.state = state
        self.set_state_client.call_async(request)

    def publish_estimate(self, now, roll, wind_x, wind_y, fresh):
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = self.get_clock().now().to_msg(), 'world'
        pos_noise = float(self.get_parameter('position_noise_stddev_m').value)
        pose.pose.position.x = self.position[0] + self.random.gauss(0.0, pos_noise)
        pose.pose.position.y = self.position[1] + self.random.gauss(0.0, pos_noise)
        pose.pose.position.z = self.position[2] + self.random.gauss(0.0, float(self.get_parameter('altitude_noise_stddev_m').value))
        qx, qy, qz, qw = quaternion_from_euler(roll, self.target_pitch, self.yaw + self.random.gauss(0.0, float(self.get_parameter('yaw_noise_stddev_rad').value)))
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        self.pose_pub.publish(pose)
        twist = TwistStamped()
        twist.header = pose.header
        noise = float(self.get_parameter('velocity_noise_stddev_mps').value)
        twist.twist.linear.x = self.velocity[0] + wind_x + self.random.gauss(0.0, noise)
        twist.twist.linear.y = self.velocity[1] + wind_y + self.random.gauss(0.0, noise)
        twist.twist.linear.z = self.velocity[2] + self.random.gauss(0.0, noise)
        twist.twist.angular.z = self.yaw_rate
        self.twist_pub.publish(twist)
        status = String()
        status.data = json.dumps({'sim_time_s': round(now, 3), 'command_fresh': fresh, 'position_m': [round(value, 2) for value in self.position], 'queued_commands': len(self.command_queue)})
        self.status_pub.publish(status)

    def time_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = UavFlightSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
