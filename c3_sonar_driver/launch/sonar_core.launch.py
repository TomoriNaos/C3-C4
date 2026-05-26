from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_dave_adapter = LaunchConfiguration('enable_dave_adapter')
    enable_dave_plugin = LaunchConfiguration('enable_dave_plugin')
    dave_pointcloud_topic = LaunchConfiguration('dave_pointcloud_topic')
    dave_image_topic = LaunchConfiguration('dave_image_topic')
    dave_image_raw_topic = LaunchConfiguration('dave_image_raw_topic')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('c3_sonar_driver'),
        'urdf',
        'sonar.urdf.xacro'
    ])

    robot_description = Command([
        'xacro ', xacro_file,
        ' enable_dave_plugin:=', enable_dave_plugin,
        ' dave_pointcloud_topic:=', dave_pointcloud_topic,
        ' dave_image_topic:=', dave_image_topic,
        ' dave_image_raw_topic:=', dave_image_raw_topic,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_dave_adapter', default_value='true'),
        DeclareLaunchArgument('enable_dave_plugin', default_value='false'),
        DeclareLaunchArgument('dave_pointcloud_topic', default_value='/multibeam_sonar_point_cloud'),
        DeclareLaunchArgument('dave_image_topic', default_value='/sonar_image'),
        DeclareLaunchArgument('dave_image_raw_topic', default_value='/sonar_image_raw'),
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
            parameters=[PathJoinSubstitution([FindPackageShare('c3_sonar_driver'), 'config', 'main_controller.yaml'])],
            output='screen',
        ),
        Node(
            package='c3_sonar_driver',
            executable='communicate_node',
            name='communicate_node',
            parameters=[PathJoinSubstitution([FindPackageShare('c3_sonar_driver'), 'config', 'communicate.yaml'])],
            output='screen',
        ),
        Node(
            package='c3_sonar_driver',
            executable='dave_sonar_adapter_node',
            name='dave_sonar_adapter_node',
            parameters=[PathJoinSubstitution([FindPackageShare('c3_sonar_driver'), 'config', 'dave_adapter.yaml'])],
            condition=IfCondition(enable_dave_adapter),
            output='screen',
        ),
    ])
