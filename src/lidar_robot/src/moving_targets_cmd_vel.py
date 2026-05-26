#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


class MovingTargetsCmdVel(Node):
    def __init__(self):
        super().__init__('moving_targets_cmd_vel')

        self.declare_parameter('update_rate_hz', 20.0)

        self.targets = {
            'navigation_marker_port': {
                'topic': '/navigation_marker_port/cmd_vel',
                'x': 18.0,
                'y': 6.0,
                'z': 1.48,
                'yaw': 0.0,
                'twist': Twist(),
            },
            'navigation_marker_starboard': {
                'topic': '/navigation_marker_starboard/cmd_vel',
                'x': 24.0,
                'y': -7.0,
                'z': 1.48,
                'yaw': 0.0,
                'twist': Twist(),
            },
            'target_200m_box': {
                'topic': '/target_200m_box/cmd_vel',
                'x': 173.205,
                'y': 100.0,
                'z': 1.5,
                'yaw': 0.0,
                'twist': Twist(),
            },
            'target_250m_box': {
                'topic': '/target_250m_box/cmd_vel',
                'x': 216.506,
                'y': -125.0,
                'z': 1.0,
                'yaw': 0.0,
                'twist': Twist(),
            },
            'target_300m_box': {
                'topic': '/target_300m_box/cmd_vel',
                'x': 300.0,
                'y': 40.0,
                'z': 1.98,
                'yaw': 0.0,
                'twist': Twist(),
            },
            'target_500m_cylinder': {
                'topic': '/target_500m_cylinder/cmd_vel',
                'x': 500.0,
                'y': -50.0,
                'z': 7.5,
                'yaw': 0.0,
                'twist': Twist(),
            },
        }

        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.client.wait_for_service()

        for name, target in self.targets.items():
            self.create_subscription(
                Twist,
                target['topic'],
                self.make_cmd_callback(name),
                10,
            )

        self.last_time = self.get_clock().now()
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(update_rate_hz, 1.0), self.update)

    def make_cmd_callback(self, name):
        def callback(msg):
            self.targets[name]['twist'] = msg
        return callback

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        for name, target in self.targets.items():
            twist = target['twist']

            yaw = target['yaw']
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            # 和 /wamv/cmd_vel 类似：linear.x 是自身前向速度，linear.y 是自身横向速度。
            vx_world = cos_yaw * twist.linear.x - sin_yaw * twist.linear.y
            vy_world = sin_yaw * twist.linear.x + cos_yaw * twist.linear.y

            target['x'] += vx_world * dt
            target['y'] += vy_world * dt
            target['z'] += twist.linear.z * dt
            target['yaw'] += twist.angular.z * dt

            self.publish_state(name, target)

    def publish_state(self, name, target):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.reference_frame = 'world'

        req.state.pose.position.x = target['x']
        req.state.pose.position.y = target['y']
        req.state.pose.position.z = target['z']

        half_yaw = 0.5 * target['yaw']
        req.state.pose.orientation.z = math.sin(half_yaw)
        req.state.pose.orientation.w = math.cos(half_yaw)

        req.state.twist = target['twist']

        self.client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = MovingTargetsCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
