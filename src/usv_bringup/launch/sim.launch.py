import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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

    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
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

    def make_mmwave_converter(name, input_topic, output_topic, source_id, yaw_deg):
        return Node(
            package='lidar_robot',
            executable='mmwave_scan_converter.py',
            name=name,
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,

                # All converted detections are expressed in base_link.
                'frame_id': 'base_link',

                # Mounting pose of each radar in base_link.
                # Must match the x/y/z/yaw in wamv_base.urdf.xacro.
                'radar_x_in_base': -0.35,
                'radar_y_in_base': 0.0,
                'radar_z_in_base': 2.5,
                'radar_yaw_in_base_deg': yaw_deg,
                'source_id': float(source_id),

                'max_range': 800.0,
                'min_range': 2.0,
                'horizontal_fov_deg': 120.0,
                'angular_resolution_deg': 1.5,
                'subbeam_resolution_deg': 0.15,
                'radar_height_m': 2.5,
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
                'sea_clutter_height_scale': 0.42,
                'sea_clutter_height_max': 2.0,
                'sea_clutter_compete_min_height': 0.45,
                'sea_clutter_random_stddev': 0.05,

                # Each radar publishes to its own topic now.
                # The merger node combines the four topics into one fused cloud.
                'max_detections': 80,
            }]
        )

    mmwave_converter_front = make_mmwave_converter(
        'mmwave_scan_converter_front',
        '/radar/front/raw_scan',
        '/mmwave/detections/front',
        1,
        0.0,
    )

    mmwave_converter_left = make_mmwave_converter(
        'mmwave_scan_converter_left',
        '/radar/left/raw_scan',
        '/mmwave/detections/left',
        2,
        90.0,
    )

    mmwave_converter_right = make_mmwave_converter(
        'mmwave_scan_converter_right',
        '/radar/right/raw_scan',
        '/mmwave/detections/right',
        3,
        -90.0,
    )

    mmwave_converter_rear = make_mmwave_converter(
        'mmwave_scan_converter_rear',
        '/radar/rear/raw_scan',
        '/mmwave/detections/rear',
        4,
        180.0,
    )

    mmwave_detections_merger = Node(
        package='lidar_robot',
        executable='mmwave_detections_merger.py',
        name='mmwave_detections_merger',
        output='screen',
        parameters=[{
            'front_topic': '/mmwave/detections/front',
            'left_topic': '/mmwave/detections/left',
            'right_topic': '/mmwave/detections/right',
            'rear_topic': '/mmwave/detections/rear',
            'output_topic': '/mmwave/detections_fused',
            'frame_id': 'base_link',
            'publish_rate_hz': 12.5,
            'max_cloud_age_sec': 0.15,
        }]
    )

    mmwave_filter = Node(
        package='lidar_robot',
        executable='mmwave_radar_node',
        name='mmwave_radar_filter_node',
        output='screen',
        parameters=[{
            # IMPORTANT:
            # The tracker must receive one fused observation frame,
            # not four single-radar frames arriving one after another.
            'input_topic': '/mmwave/detections_fused',
            'output_topic': '/mmwave/filtered_detections',
        }]
    )

    moving_targets_cmd_vel = Node(
        package='lidar_robot',
        executable='moving_targets_cmd_vel.py',
        name='moving_targets_cmd_vel',
        output='screen',
        parameters=[{
            'update_rate_hz': 20.0,
        }]
    )

    mmwave_odom_imu_fusion = Node(
        package='lidar_robot',
        executable='mmwave_odom_imu_fusion.py',
        name='mmwave_odom_imu_fusion',
        output='screen',
        parameters=[{
            # This node should run after filtering.
            # It converts filtered base_link detections to odom/global coordinates.
            'input_cloud_topic': '/mmwave/filtered_detections',
            'output_cloud_topic': '/mmwave/global_detections',
            'output_pose_topic': '/mmwave/global_targets',
            'odom_topic': '/wamv/odom',
            'imu_topic': '/wamv/imu/data',
            'global_frame_id': 'odom',

            # The filtered detections are already in base_link,
            # so keep this transform zero here to avoid applying radar offset twice.
            'radar_x_in_base': 0.0,
            'radar_y_in_base': 0.0,
            'radar_z_in_base': 0.0,
            'radar_yaw_in_base_deg': 0.0,

            'use_imu_yaw': True,
            'publish_only_target_source': False,
            'max_state_age_sec': 1.0,
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

        mmwave_converter_front,
        mmwave_converter_left,
        mmwave_converter_right,
        mmwave_converter_rear,
        mmwave_detections_merger,
        mmwave_filter,
        mmwave_odom_imu_fusion,
        moving_targets_cmd_vel,
    ])
