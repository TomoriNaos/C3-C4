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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    mmwave_converter_10m = Node(
        package='lidar_robot',
        executable='mmwave_scan_converter.py',
        name='mmwave_scan_converter_10m',
        output='screen',
        parameters=[{
            'input_topic': '/radar_10m/raw_scan',
            'output_topic': '/mmwave_10m/detections',
            'frame_id': 'radar_10m_link',
            'max_range': 800.0,
            'min_range': 2.0,
            'horizontal_fov_deg': 120.0,
            'angular_resolution_deg': 1.5,
            'subbeam_resolution_deg': 0.3,
            'radar_height_m': 10.0,
            'intensity_threshold': 0.10,
            'target_base_rcs': 1.0,
            'target_material_reflectivity': 0.85,
            'target_random_stddev': 0.04,
            'radial_velocity_noise_stddev': 0.05,
            'range_attenuation_alpha': 0.0035,
            'point_target_angle_noise_deg': 0.25,
            'extended_target_angle_noise_deg': 0.10,
            'center_weight_enabled': True,
            'center_weight_sigma': 1.0,
            'center_weight_min': 0.7,
            'sea_clutter_enabled': True,
            'sea_state': 0.65,
            'sea_clutter_range_min': 5.0,
            'sea_clutter_range_max': 800.0,
            'sea_clutter_height_scale': 0.25,
            'sea_clutter_height_max': 2.0,
            'sea_clutter_compete_min_height': 1.0,
            'sea_clutter_random_stddev': 0.03,
            'max_detections': 80,
        }]
    )

    mmwave_converter_4m = Node(
        package='lidar_robot',
        executable='mmwave_scan_converter.py',
        name='mmwave_scan_converter_4m',
        output='screen',
        parameters=[{
            'input_topic': '/radar_4m/raw_scan',
            'output_topic': '/mmwave_4m/detections',
            'frame_id': 'radar_4m_link',
            'max_range': 800.0,
            'min_range': 2.0,
            'horizontal_fov_deg': 120.0,
            'angular_resolution_deg': 1.5,
            'subbeam_resolution_deg': 0.3,
            'radar_height_m': 4.0,
            'intensity_threshold': 0.10,
            'target_base_rcs': 1.0,
            'target_material_reflectivity': 0.85,
            'target_random_stddev': 0.04,
            'radial_velocity_noise_stddev': 0.05,
            'range_attenuation_alpha': 0.0035,
            'point_target_angle_noise_deg': 0.25,
            'extended_target_angle_noise_deg': 0.10,
            'center_weight_enabled': True,
            'center_weight_sigma': 1.0,
            'center_weight_min': 0.7,
            'sea_clutter_enabled': True,
            'sea_state': 0.65,
            'sea_clutter_range_min': 5.0,
            'sea_clutter_range_max': 800.0,
            'sea_clutter_height_scale': 0.35,
            'sea_clutter_height_max': 2.0,
            'sea_clutter_compete_min_height': 0.7,
            'sea_clutter_random_stddev': 0.04,
            'max_detections': 80,
        }]
    )

    mmwave_converter_1p9m = Node(
        package='lidar_robot',
        executable='mmwave_scan_converter.py',
        name='mmwave_scan_converter_1p9m',
        output='screen',
        parameters=[{
            'input_topic': '/radar_1p9m/raw_scan',
            'output_topic': '/mmwave_1p9m/detections',
            'frame_id': 'radar_1p9m_link',
            'max_range': 800.0,
            'min_range': 2.0,
            'horizontal_fov_deg': 120.0,
            'angular_resolution_deg': 1.5,
            'subbeam_resolution_deg': 0.3,
            'radar_height_m': 1.9,
            'intensity_threshold': 0.10,
            'target_base_rcs': 1.0,
            'target_material_reflectivity': 0.85,
            'target_random_stddev': 0.04,
            'radial_velocity_noise_stddev': 0.05,
            'range_attenuation_alpha': 0.0035,
            'point_target_angle_noise_deg': 0.25,
            'extended_target_angle_noise_deg': 0.10,
            'center_weight_enabled': True,
            'center_weight_sigma': 1.0,
            'center_weight_min': 0.7,
            'sea_clutter_enabled': True,
            'sea_state': 0.65,
            'sea_clutter_range_min': 5.0,
            'sea_clutter_range_max': 800.0,
            'sea_clutter_height_scale': 0.50,
            'sea_clutter_height_max': 2.0,
            'sea_clutter_compete_min_height': 0.35,
            'sea_clutter_random_stddev': 0.05,
            'max_detections': 80,
        }]
    )

    mmwave_converter_1p5m = Node(
        package='lidar_robot',
        executable='mmwave_scan_converter.py',
        name='mmwave_scan_converter_1p5m',
        output='screen',
        parameters=[{
            'input_topic': '/radar_1p5m/raw_scan',
            'output_topic': '/mmwave_1p5m/detections',
            'frame_id': 'radar_1p5m_link',
            'max_range': 800.0,
            'min_range': 2.0,
            'horizontal_fov_deg': 120.0,
            'angular_resolution_deg': 1.5,
            'subbeam_resolution_deg': 0.3,
            'radar_height_m': 1.5,
            'intensity_threshold': 0.10,
            'target_base_rcs': 1.0,
            'target_material_reflectivity': 0.85,
            'target_random_stddev': 0.04,
            'radial_velocity_noise_stddev': 0.05,
            'range_attenuation_alpha': 0.0035,
            'point_target_angle_noise_deg': 0.25,
            'extended_target_angle_noise_deg': 0.10,
            'center_weight_enabled': True,
            'center_weight_sigma': 1.0,
            'center_weight_min': 0.7,
            'sea_clutter_enabled': True,
            'sea_state': 0.65,
            'sea_clutter_range_min': 5.0,
            'sea_clutter_range_max': 800.0,
            'sea_clutter_height_scale': 0.60,
            'sea_clutter_height_max': 2.0,
            'sea_clutter_compete_min_height': 0.25,
            'sea_clutter_random_stddev': 0.06,
            'max_detections': 80,
        }]
    )

    mmwave_converter_1m = Node(
        package='lidar_robot',
        executable='mmwave_scan_converter.py',
        name='mmwave_scan_converter_1m',
        output='screen',
        parameters=[{
            'input_topic': '/radar_1m/raw_scan',
            'output_topic': '/mmwave_1m/detections',
            'frame_id': 'radar_1m_link',
            'max_range': 800.0,
            'min_range': 2.0,
            'horizontal_fov_deg': 120.0,
            'angular_resolution_deg': 1.5,
            'subbeam_resolution_deg': 0.3,
            'radar_height_m': 1.0,
            'intensity_threshold': 0.10,
            'target_base_rcs': 1.0,
            'target_material_reflectivity': 0.85,
            'target_random_stddev': 0.04,
            'radial_velocity_noise_stddev': 0.05,
            'range_attenuation_alpha': 0.0035,
            'point_target_angle_noise_deg': 0.25,
            'extended_target_angle_noise_deg': 0.10,
            'center_weight_enabled': True,
            'center_weight_sigma': 1.0,
            'center_weight_min': 0.7,
            'sea_clutter_enabled': True,
            'sea_state': 0.65,
            'sea_clutter_range_min': 5.0,
            'sea_clutter_range_max': 800.0,
            'sea_clutter_height_scale': 0.70,
            'sea_clutter_height_max': 2.0,
            'sea_clutter_compete_min_height': 0.20,
            'sea_clutter_random_stddev': 0.07,
            'max_detections': 80,
        }]
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        verbose_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        rviz_node,
        mmwave_converter_10m,
        mmwave_converter_4m,
        mmwave_converter_1p9m,
        mmwave_converter_1p5m,
        mmwave_converter_1m,
    ])
