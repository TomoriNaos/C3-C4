import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
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
    workspace_root = os.path.abspath(os.path.join(pkg_bringup, '..', '..', '..', '..'))
    source_models = os.path.join(workspace_root, 'src', 'usv_bringup', 'models')
    models_dir = source_models if os.path.isdir(source_models) else os.path.join(pkg_bringup, 'models')
    default_camera_model = os.path.join(models_dir, 'camera.onnx')
    default_gated_camera_model = os.path.join(models_dir, 'gated_camera.onnx')
    default_plane_model = os.path.join(models_dir, 'plane.onnx')
    default_plane_gated_model = os.path.join(models_dir, 'plane_gated.onnx')

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
    world_path_arg = DeclareLaunchArgument(
        'world_path',
        default_value=world_path,
        description='Gazebo world file to load'
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
    target_model_arg = DeclareLaunchArgument(
        'target_model',
        default_value='moving_vessel',
        description='Gazebo model name used as the primary tracking target'
    )
    usv_follow_arg = DeclareLaunchArgument(
        'usv_follow',
        default_value='true',
        description='Move the USV slowly toward the fused tracked target'
    )
    evaluation_arg = DeclareLaunchArgument(
        'evaluation',
        default_value='true',
        description='Publish tracking and collision-risk evaluation metrics'
    )
    ais_arg = DeclareLaunchArgument(
        'ais',
        default_value='true',
        description='Publish simulated AIS targets and fuse them in the tracker'
    )
    uav_arg = DeclareLaunchArgument(
        'uav',
        default_value='true',
        description='Start the simulated scout UAV and gated-camera recognizer'
    )
    sonar_simulation_arg = DeclareLaunchArgument(
        'sonar_simulation',
        default_value='true',
        description='Convert ideal Gazebo sonar rays to delayed/noisy C3 sonar detections'
    )
    camera_model_arg = DeclareLaunchArgument(
        'camera_model_path',
        default_value=default_camera_model,
        description='Path to the ship normal/depth camera YOLO ONNX model'
    )
    gated_camera_model_arg = DeclareLaunchArgument(
        'gated_camera_model_path',
        default_value=default_gated_camera_model,
        description='Path to the ship gated-camera YOLO ONNX model'
    )
    plane_model_arg = DeclareLaunchArgument(
        'plane_model_path',
        default_value=default_plane_model,
        description='Path to the UAV/plane normal camera YOLO ONNX model'
    )
    plane_gated_model_arg = DeclareLaunchArgument(
        'plane_gated_model_path',
        default_value=default_plane_gated_model,
        description='Path to the UAV/plane gated-camera YOLO ONNX model'
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
    stf_gated_fusion_arg = DeclareLaunchArgument(
        'stf_gated_fusion',
        default_value='true',
        description='Start the additional STF-style three-slice gated recognizer'
    )
    gated_bev_detection_arg = DeclareLaunchArgument(
        'gated_bev_detection',
        default_value='true',
        description='Start the additional depth-to-BEV gated camera detector'
    )
    pseudocolor_gated_yolo_arg = DeclareLaunchArgument(
        'pseudocolor_gated_yolo',
        default_value='false',
        description='Start the legacy extra pseudo-color range-view recognizer'
    )
    c3_multimodal_fusion_arg = DeclareLaunchArgument(
        'c3_multimodal_fusion',
        default_value='true',
        description='Start the C3 multimodal pointcloud buffer, heatmap, and detected-object node'
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_gazebo_ros, '/launch/gzserver.launch.py']),
        launch_arguments={
            'world': LaunchConfiguration('world_path'),
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
        parameters=[
            perception_config,
            {
                'use_sim_time': True,
            }
        ],
        condition=IfCondition(LaunchConfiguration('usv_follow'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        additional_env={
            'QT_QPA_PLATFORM': 'xcb',
            'LIBGL_ALWAYS_SOFTWARE': '1',
        },
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
        parameters=[
            perception_config,
            {
                'use_sim_time': True,
                'tracked_target_name': LaunchConfiguration('target_model')
            }
        ],
        condition=IfCondition(LaunchConfiguration('dynamic_targets'))
    )

    ais_simulator = Node(
        package='usv_perception',
        executable='ais_simulator',
        name='ais_simulator',
        output='screen',
        parameters=[perception_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('ais'))
    )

    radar_sonar_tracker = Node(
        package='usv_perception',
        executable='radar_sonar_tracker',
        name='radar_sonar_tracker',
        output='screen',
        parameters=[perception_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('perception'))
    )

    tracking_evaluator = Node(
        package='usv_perception',
        executable='tracking_evaluator',
        name='tracking_evaluator',
        output='screen',
        parameters=[perception_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('evaluation'))
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
                'yolo_model_path': LaunchConfiguration('gated_camera_model_path')
            }
        ],
        condition=IfCondition(LaunchConfiguration('perception'))
    )

    pseudocolor_gated_camera_recognizer = Node(
        package='usv_perception',
        executable='gated_camera_recognizer',
        name='pseudocolor_gated_camera_recognizer',
        output='screen',
        parameters=[
            perception_config,
            {
                'use_sim_time': True,
                'yolo_model_path': LaunchConfiguration('gated_camera_model_path')
            }
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('perception'), "' == 'true' and '",
            LaunchConfiguration('pseudocolor_gated_yolo'), "' == 'true'"
        ]))
    )

    depth_camera_recognizer = Node(
        package='usv_perception',
        executable='gated_camera_recognizer',
        name='depth_camera_recognizer',
        output='screen',
        parameters=[
            perception_config,
            {
                'use_sim_time': True,
                'yolo_model_path': LaunchConfiguration('camera_model_path')
            }
        ],
        condition=IfCondition(LaunchConfiguration('perception'))
    )

    gated_slice_fusion_recognizer = Node(
        package='usv_perception',
        executable='gated_slice_fusion_recognizer',
        name='gated_slice_fusion_recognizer',
        output='screen',
        parameters=[perception_config, {'use_sim_time': True}],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('perception'), "' == 'true' and '",
            LaunchConfiguration('stf_gated_fusion'), "' == 'true'"
        ]))
    )

    gated_bev_detector = Node(
        package='usv_perception',
        executable='gated_bev_detector',
        name='gated_bev_detector',
        output='screen',
        parameters=[perception_config, {'use_sim_time': True}],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('perception'), "' == 'true' and '",
            LaunchConfiguration('gated_bev_detection'), "' == 'true'"
        ]))
    )

    c3_multimodal_buffer_fusion = Node(
        package='usv_perception',
        executable='c3_multimodal_buffer_fusion',
        name='c3_multimodal_buffer_fusion',
        output='screen',
        parameters=[
            perception_config,
            {
                'use_sim_time': True,
                'evaluation_target_model_name': LaunchConfiguration('target_model')
            }
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('perception'), "' == 'true' and '",
            LaunchConfiguration('c3_multimodal_fusion'), "' == 'true'"
        ]))
    )

    uav_patrol_controller = Node(
        package='usv_perception',
        executable='uav_patrol_controller',
        name='uav_patrol_controller',
        output='screen',
        parameters=[
            perception_config,
            {
                'use_sim_time': True,
                # The Classic actor follows the same goal interface through a flight-dynamics layer.
                'control_backend': 'gazebo_simulated'
            }
        ],
        condition=IfCondition(LaunchConfiguration('uav'))
    )

    uav_flight_simulator = Node(
        package='usv_perception',
        executable='uav_flight_simulator.py',
        name='uav_flight_simulator',
        output='screen',
        parameters=[perception_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('uav'))
    )

    uav_camera_recognizer = Node(
        package='usv_perception',
        executable='gated_camera_recognizer',
        name='uav_camera_recognizer',
        output='screen',
        parameters=[
            perception_config,
            {
                'use_sim_time': True,
                'yolo_model_path': LaunchConfiguration('plane_model_path')
            }
        ],
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
                'yolo_model_path': LaunchConfiguration('plane_gated_model_path')
            }
        ],
        condition=IfCondition(LaunchConfiguration('uav'))
    )

    def make_c3_mmwave_converter(
        radar_id,
        radar_height,
        sector_name,
        sector_yaw,
        sea_clutter_height_scale,
        sea_clutter_compete_min_height,
        sea_clutter_random_stddev
    ):
        return Node(
            package='usv_perception',
            executable='mmwave_scan_converter.py',
            name=f'mmwave_scan_converter_{radar_id}_{sector_name}',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_topic': f'/radar/{sector_name}/{radar_id}/raw_scan',
                'output_topic': f'/mmwave/{sector_name}/{radar_id}/detections',
                'frame_id': f'radar_{radar_id}_{sector_name}_link',
                'output_frame_id': 'base_link',
                'mount_yaw_rad': sector_yaw,
                'max_range': 800.0,
                'min_range': 2.0,
                'horizontal_fov_deg': 90.0,
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

    mmwave_heights = [
        ('h10m', 10.0, 0.25, 1.0, 0.03),
        ('h4m', 4.0, 0.35, 0.7, 0.04),
        ('h1p9m', 1.9, 0.50, 0.35, 0.05),
        ('h1p5m', 1.5, 0.60, 0.25, 0.06),
        ('h1m', 1.0, 0.70, 0.20, 0.07),
    ]
    mmwave_sectors = [
        ('front', 0.0),
        ('right', -1.57079632679),
        ('back', 3.14159265359),
        ('left', 1.57079632679),
    ]
    mmwave_converter_nodes = [
        make_c3_mmwave_converter(height_id, height, sector_name, sector_yaw, scale, compete, stddev)
        for height_id, height, scale, compete, stddev in mmwave_heights
        for sector_name, sector_yaw in mmwave_sectors
    ]

    mmwave_debug_node = Node(
        package='usv_perception',
        executable='mmwave_detection_debug.py',
        name='mmwave_detection_debug',
        output='screen',
        condition=IfCondition(LaunchConfiguration('c3_mmwave_debug'))
    )

    sonar_sectors = [
        ('front', '/sonar/scan', 0.0, 1),
        ('right', '/sonar/right_scan', -1.57079632679, 2),
        ('back', '/sonar/back_scan', 3.14159265359, 3),
        ('left', '/sonar/left_scan', 1.57079632679, 4),
    ]
    sonar_simulator_nodes = [
        Node(
            package='usv_perception',
            executable='sonar_scan_simulator.py',
            name=f'sonar_scan_simulator_{sector_name}',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_topic': input_topic,
                'output_topic': '/sonar/detect',
                'status_topic': '/sonar/status',
                'frame_id': 'base_link',
                'mount_yaw_rad': yaw,
                'sensor_x_offset_m': 0.65,
                'sensor_id': sensor_id,
            }],
            condition=IfCondition(LaunchConfiguration('sonar_simulation'))
        )
        for sector_name, input_topic, yaw, sensor_id in sonar_sectors
    ]

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
        world_path_arg,
        perception_arg,
        dynamic_targets_arg,
        target_model_arg,
        usv_follow_arg,
        evaluation_arg,
        ais_arg,
        uav_arg,
        sonar_simulation_arg,
        camera_model_arg,
        gated_camera_model_arg,
        plane_model_arg,
        plane_gated_model_arg,
        c3_mmwave_arg,
        c3_mmwave_debug_arg,
        rgbd_dehaze_arg,
        stf_gated_fusion_arg,
        gated_bev_detection_arg,
        pseudocolor_gated_yolo_arg,
        c3_multimodal_fusion_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        world_to_usv_tf,
        usv_target_follower,
        wave_buoyancy_node,
        dynamic_target_controller,
        ais_simulator,
        radar_sonar_tracker,
        tracking_evaluator,
        gated_camera_recognizer,
        pseudocolor_gated_camera_recognizer,
        depth_camera_recognizer,
        gated_slice_fusion_recognizer,
        gated_bev_detector,
        c3_multimodal_buffer_fusion,
        uav_patrol_controller,
        uav_flight_simulator,
        uav_camera_recognizer,
        uav_gated_camera_recognizer,
        *mmwave_converter_nodes,
        mmwave_debug_node,
        *sonar_simulator_nodes,
        rgbd_dehaze_pointcloud,
        rviz_node
    ])
