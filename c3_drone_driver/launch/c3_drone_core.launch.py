from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('c3_drone_driver')

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', '/tmp/ros_logs'),
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
        ),
        Node(
            package='c3_drone_driver',
            executable='mavlink_bridge_node',
            name='mavlink_bridge_node',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'mavlink_bridge.yaml')],
        ),
        Node(
            package='c3_drone_driver',
            executable='drone_main_controller_node',
            name='drone_main_controller_node',
            output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'drone_main_controller.yaml'),
                {
                    'pose_config_file': os.path.join(pkg, 'config', 'pose_estimator_default.yaml'),
                },
            ],
        ),
        Node(
            package='c3_drone_driver',
            executable='motion_controller_node',
            name='motion_controller_node',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'motion_controller.yaml')],
        ),
        Node(
            package='c3_drone_driver',
            executable='px4_pose_bridge_node',
            name='px4_pose_bridge_node',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'px4_pose.yaml')],
        ),
    ])
