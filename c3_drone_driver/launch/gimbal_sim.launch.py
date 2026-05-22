from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')
    use_controller = LaunchConfiguration('use_controller')
    tc_camera_xyz = LaunchConfiguration('tc_camera_xyz')
    tc_camera_rpy = LaunchConfiguration('tc_camera_rpy')
    gated_camera_xyz = LaunchConfiguration('gated_camera_xyz')
    gated_camera_rpy = LaunchConfiguration('gated_camera_rpy')

    model_file = PathJoinSubstitution([
        FindPackageShare('c3_drone_driver'),
        'urdf',
        'c3_drone_with_gimbal.urdf.xacro'
    ])

    robot_description = {
        'robot_description': Command([
            'xacro ',
            model_file,
            ' tc_camera_xyz:=',
            tc_camera_xyz,
            ' tc_camera_rpy:=',
            tc_camera_rpy,
            ' gated_camera_xyz:=',
            gated_camera_xyz,
            ' gated_camera_rpy:=',
            gated_camera_rpy,
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui for manual joint control'
        ),
        DeclareLaunchArgument(
            'use_controller',
            default_value='false',
            description='Run gimbal controller + bridge to drive joints from /gimbal/state'
        ),
        DeclareLaunchArgument('tc_camera_xyz', default_value='0.050 0.020 0.0'),
        DeclareLaunchArgument('tc_camera_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('gated_camera_xyz', default_value='0.050 -0.020 0.0'),
        DeclareLaunchArgument('gated_camera_rpy', default_value='0 0 0'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(use_gui)
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(use_gui)
        ),
        Node(
            package='c3_drone_driver',
            executable='gimbal_controller_node',
            name='gimbal_controller_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('c3_drone_driver'),
                'config',
                'gimbal.yaml'
            ])],
            condition=IfCondition(use_controller)
        ),
        Node(
            package='c3_drone_driver',
            executable='gimbal_joint_state_bridge_node',
            name='gimbal_joint_state_bridge_node',
            output='screen',
            condition=IfCondition(use_controller)
        )
    ])
