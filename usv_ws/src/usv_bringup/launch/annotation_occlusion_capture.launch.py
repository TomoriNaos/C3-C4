import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory('usv_bringup')
    annotation_world = os.path.join(pkg_bringup, 'worlds', 'annotation_targets.world')
    sim_launch = os.path.join(pkg_bringup, 'launch', 'sim.launch.py')
    perception_config = os.path.join(pkg_bringup, 'config', 'perception.yaml')

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/hu/usv_captures/annotation_occlusion',
        description='Directory for occlusion annotation images'
    )
    max_images_arg = DeclareLaunchArgument(
        'max_images',
        default_value='96',
        description='Stop after this many images'
    )
    every_n_arg = DeclareLaunchArgument(
        'every_n',
        default_value='5',
        description='Save every Nth frame'
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Show Gazebo client during capture'
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'world_path': annotation_world,
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',
            'perception': 'false',
            'dynamic_targets': 'false',
            'usv_follow': 'false',
            'evaluation': 'false',
            'ais': 'false',
            'uav': 'false',
            'c3_mmwave': 'false',
            'rgbd_dehaze': 'false',
        }.items()
    )

    dynamic_targets = Node(
        package='usv_perception',
        executable='dynamic_target_controller',
        name='dynamic_target_controller',
        output='screen',
        parameters=[
            perception_config,
            {
                'use_sim_time': True,
                'annotation_mode': True,
                'annotation_occlusion_mode': True,
                'annotation_scene_duration': 2.0,
                'update_rate': 30.0,
                'motion_time_scale': 1.0,
            }
        ]
    )

    recorder = Node(
        package='usv_perception',
        executable='ros_image_recorder',
        name='annotation_occlusion_image_recorder',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/gated_camera/image_raw',
            'output_dir': LaunchConfiguration('output_dir'),
            'prefix': 'occlusion',
            'every_n': ParameterValue(LaunchConfiguration('every_n'), value_type=int),
            'max_images': ParameterValue(LaunchConfiguration('max_images'), value_type=int),
            'extension': 'jpg',
            'crop_x': 40,
            'crop_y': 30,
            'crop_width': 560,
            'crop_height': 420,
            'resize_width': 512,
            'resize_height': 384,
        }]
    )

    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder,
            on_exit=[EmitEvent(event=Shutdown(reason='occlusion annotation image capture finished'))],
        )
    )

    return LaunchDescription([
        output_dir_arg,
        max_images_arg,
        every_n_arg,
        gui_arg,
        sim,
        dynamic_targets,
        recorder,
        shutdown_when_done,
    ])
