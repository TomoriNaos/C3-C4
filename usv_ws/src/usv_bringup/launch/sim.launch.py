import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    usv_follow_arg = DeclareLaunchArgument(
        'usv_follow',
        default_value='true',
        description='Move the USV slowly toward the fused tracked target'
    )
    uav_arg = DeclareLaunchArgument(
        'uav',
        default_value='true',
        description='Start the simulated ALS UAV and gated-camera recognizer'
    )
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model_path',
        default_value=default_yolo_model,
        description='Path to the YOLO ONNX model used by camera recognizers'
    )
    c3_mmwave_arg = DeclareLaunchArgument(
        'c3_mmwave',
        default_value='true',
        description='Enable the C3 multi-height mmWave raw sensors and converters'
    )
    c3_mmwave_debug_arg = DeclareLaunchArgument(
        'c3_mmwave_debug',
        default_value='false',
        description='Print decoded C3 mmWave detections'
    )
    rgbd_dehaze_arg = DeclareLaunchArgument(
        'rgbd_dehaze',
        default_value='true',
        description='Start the C3 RGB-D dehaze pointcloud node for the USV depth camera'
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
            'robot_description': Command([
                'xacro',
                ' ',
                xacro_file,
                ' ',
                'enable_c3_mmwave:=',
                LaunchConfiguration('c3_mmwave')
            ]),
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
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(LaunchConfiguration('usv_follow'))
    )

    usv_target_follower = Node(
        package='usv_perception',
        executable='usv_target_follower',
        name='usv_target_follower',
        output='screen',
        parameters=[perception_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('usv_follow'))
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

    uav_gated_camera_recognizer = Node(
        package='usv_perception',
        executable='gated_camera_recognizer',
        name='uav_gated_camera_recognizer',
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

    def make_c3_mmwave_converter(
        radar_id,
        radar_height,
        sea_clutter_height_scale,
        sea_clutter_compete_min_height,
        sea_clutter_random_stddev
    ):
        return Node(
            package='lidar_robot',
            executable='mmwave_scan_converter.py',
            name=f'mmwave_scan_converter_{radar_id}',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_topic': f'/radar_{radar_id}/raw_scan',
                'output_topic': f'/mmwave_{radar_id}/detections',
                'frame_id': f'radar_{radar_id}_link',
                'max_range': 800.0,
                'min_range': 2.0,
                'horizontal_fov_deg': 120.0,
                'angular_resolution_deg': 1.5,
                'subbeam_resolution_deg': 0.3,
                'radar_height_m': radar_height,
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
                'sea_clutter_height_scale': sea_clutter_height_scale,
                'sea_clutter_height_max': 2.0,
                'sea_clutter_compete_min_height': sea_clutter_compete_min_height,
                'sea_clutter_random_stddev': sea_clutter_random_stddev,
                'max_detections': 80,
            }],
            condition=IfCondition(LaunchConfiguration('c3_mmwave'))
        )

    mmwave_converter_10m = make_c3_mmwave_converter('10m', 10.0, 0.25, 1.0, 0.03)
    mmwave_converter_4m = make_c3_mmwave_converter('4m', 4.0, 0.35, 0.7, 0.04)
    mmwave_converter_1p9m = make_c3_mmwave_converter('1p9m', 1.9, 0.50, 0.35, 0.05)
    mmwave_converter_1p5m = make_c3_mmwave_converter('1p5m', 1.5, 0.60, 0.25, 0.06)
    mmwave_converter_1m = make_c3_mmwave_converter('1m', 1.0, 0.70, 0.20, 0.07)

    mmwave_debug_node = Node(
        package='lidar_robot',
        executable='mmwave_detection_debug.py',
        name='mmwave_detection_debug',
        output='screen',
        condition=IfCondition(LaunchConfiguration('c3_mmwave_debug'))
    )

    rgbd_dehaze_pointcloud = Node(
        package='depth_image_to_pointcloud2',
        executable='depth_image_to_pointcloud2',
        name='depth_image_to_pointcloud2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'rgb_topic': '/depth_camera/image_raw',
            'depth_topic': '/depth_camera/depth/image_raw',
            'camera_info_topic': '/depth_camera/camera_info',
            'pointcloud_topic': '/depth_camera/dehazed_points',
            'frame_stride': 3,
            'depth_scale': 0.001,
            'max_valid_depth': 60.0,
            'show_images': False,
            'dark_channel_radius': 7,
            'omega': 0.92,
            'min_transmission': 0.12,
            'atmospheric_light_percent': 0.001,
            'depth_compensation_strength': 0.25,
            'max_depth_scale': 1.35,
        }],
        condition=IfCondition(LaunchConfiguration('rgbd_dehaze'))
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        verbose_arg,
        perception_arg,
        dynamic_targets_arg,
        usv_follow_arg,
        uav_arg,
        yolo_model_arg,
        c3_mmwave_arg,
        c3_mmwave_debug_arg,
        rgbd_dehaze_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        world_to_usv_tf,
        usv_target_follower,
        wave_buoyancy_node,
        dynamic_target_controller,
        radar_sonar_tracker,
        gated_camera_recognizer,
        uav_patrol_controller,
        uav_gated_camera_recognizer,
        mmwave_converter_10m,
        mmwave_converter_4m,
        mmwave_converter_1p9m,
        mmwave_converter_1p5m,
        mmwave_converter_1m,
        mmwave_debug_node,
        rgbd_dehaze_pointcloud,
        rviz_node
    ])
