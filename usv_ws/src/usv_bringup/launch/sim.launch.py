import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('usv_bringup')
    pkg_description = get_package_share_directory('usv_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_bringup, 'worlds', 'ocean_fog.world')
    xacro_file = os.path.join(pkg_description, 'urdf', 'wamv_base.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'default.rviz')
    perception_config = os.path.join(pkg_bringup, 'config', 'perception.yaml')
    default_yolo_model = os.path.join(pkg_bringup, 'models', 'best.onnx')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo client'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2'
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Run gzserver with verbose logging'
    )
    perception_arg = DeclareLaunchArgument(
        'perception',
        default_value='true',
        description='Start multimodal perception and tracking nodes'
    )
    dynamic_targets_arg = DeclareLaunchArgument(
        'dynamic_targets',
        default_value='true',
        description='Move the simulated vessel and floating obstacle'
    )
    uav_arg = DeclareLaunchArgument(
        'uav',
        default_value='true',
        description='Start the simulated ALS UAV and long-range recognizer'
    )
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model_path',
        default_value=default_yolo_model,
        description='Path to the YOLO ONNX model used by camera recognizers'
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_gazebo_ros, '/launch/gzserver.launch.py']),
        launch_arguments={
            'world': world_path,
            'verbose': LaunchConfiguration('verbose')
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_gazebo_ros, '/launch/gzclient.launch.py']),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_file]),
            'use_sim_time': True
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'wamv', 
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.32',
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )

    world_to_usv_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_usv_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.32',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'base_footprint'
        ],
        parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    wave_buoyancy_node = Node(
        package='usv_perception',
        executable='wave_buoyancy_node',
        name='wave_buoyancy_node',
        output='screen',
        parameters=[perception_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('perception'))
    )

    dynamic_target_controller = Node(
        package='usv_perception',
        executable='dynamic_target_controller',
        name='dynamic_target_controller',
        output='screen',
        parameters=[perception_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('dynamic_targets'))
    )

    radar_sonar_tracker = Node(
        package='usv_perception',
        executable='radar_sonar_tracker',
        name='radar_sonar_tracker',
        output='screen',
        parameters=[perception_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('perception'))
    )

    gated_camera_recognizer = Node(
        package='usv_perception',
        executable='gated_camera_recognizer',
        name='gated_camera_recognizer',
        output='screen',
        parameters=[
            perception_config,
            {
                'use_sim_time': True,
                'yolo_model_path': LaunchConfiguration('yolo_model_path')
            }
        ],
        condition=IfCondition(LaunchConfiguration('perception'))
    )

    uav_patrol_controller = Node(
        package='usv_perception',
        executable='uav_patrol_controller',
        name='uav_patrol_controller',
        output='screen',
        parameters=[perception_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('uav'))
    )

    uav_long_range_recognizer = Node(
        package='usv_perception',
        executable='gated_camera_recognizer',
        name='uav_long_range_recognizer',
        output='screen',
        parameters=[
            perception_config,
            {
                'use_sim_time': True,
                'yolo_model_path': LaunchConfiguration('yolo_model_path')
            }
        ],
        condition=IfCondition(LaunchConfiguration('uav'))
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        verbose_arg,
        perception_arg,
        dynamic_targets_arg,
        uav_arg,
        yolo_model_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        world_to_usv_tf,
        wave_buoyancy_node,
        dynamic_target_controller,
        radar_sonar_tracker,
        gated_camera_recognizer,
        uav_patrol_controller,
        uav_long_range_recognizer,
        rviz_node
    ])
