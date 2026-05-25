from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('c3_sonar_driver'),
        'urdf',
        'sonar.urdf.xacro'
    ])

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
            output='screen',
        ),
        Node(
            package='c3_sonar_driver',
            executable='sonar_main_controller_node',
            name='sonar_main_controller_node',
            output='screen',
        ),
        Node(
            package='c3_sonar_driver',
            executable='communicate_node',
            name='communicate_node',
            output='screen',
        ),
    ])
