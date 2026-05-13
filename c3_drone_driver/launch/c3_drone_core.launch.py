from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('c3_drone_driver')

    return LaunchDescription([
        Node(
            package='c3_drone_driver',
            executable='gimbal_controller_node',
            name='gimbal_controller_node',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'gimbal.yaml')],
        ),
        Node(
            package='c3_drone_driver',
            executable='target_processor_node',
            name='target_processor_node',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'target_processor.yaml')],
        ),
        Node(
            package='c3_drone_driver',
            executable='mavlink_bridge_node',
            name='mavlink_bridge_node',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'mavlink_bridge.yaml')],
        ),
    ])
