from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('usv_bringup'), 'config', 'perception.yaml')
    auto_arm = LaunchConfiguration('auto_arm')
    auto_enter_offboard = LaunchConfiguration('auto_enter_offboard')

    return LaunchDescription([
        DeclareLaunchArgument('auto_arm', default_value='false'),
        DeclareLaunchArgument('auto_enter_offboard', default_value='false'),
        Node(
            package='usv_perception',
            executable='uav_patrol_controller',
            name='uav_patrol_controller',
            output='screen',
            parameters=[config, {
                'use_sim_time': False,
                'control_backend': 'px4',
            }],
        ),
        Node(
            package='usv_perception',
            executable='px4_offboard_bridge',
            name='px4_offboard_bridge',
            output='screen',
            parameters=[config, {
                'auto_arm': auto_arm,
                'auto_enter_offboard': auto_enter_offboard,
            }],
        ),
    ])
