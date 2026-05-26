from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_robot')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf')

    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', urdf_file],
        output='screen'
    )

    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', urdf_file],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}],
        output='screen'
    )

    mmwave_radar_node = Node(
        package='lidar_robot',
        executable='mmwave_radar_node',
        output='screen'
    )

    mmwave_odom_imu_fusion_node = Node(
        package='lidar_robot',
        executable='mmwave_odom_imu_fusion.py',
        name='mmwave_odom_imu_fusion',
        output='screen',
        parameters=[{
            'input_cloud_topic': '/mmwave/filtered_detections',
            'output_cloud_topic': '/mmwave/global_detections',
            'output_pose_topic': '/mmwave/global_targets',
            'odom_topic': '/wamv/odom',
            'imu_topic': '/wamv/imu/data',
            'global_frame_id': 'odom',

            # 如果你的雷达安装在 base_link 前方 2.5m、高 1.86m，可以这样写：
            # 但如果你不确定，先保持 0，避免重复计算安装偏移。
            'radar_x_in_base': 0.0,
            'radar_y_in_base': 0.0,
            'radar_z_in_base': 0.0,
            'radar_yaw_in_base_deg': 0.0,

            'use_imu_yaw': True,
            'publish_only_target_source': False,
        }]
    )


    return LaunchDescription([
        xacro_to_urdf,
        start_gazebo,
        spawn_robot,
        robot_state_publisher,
        mmwave_radar_node,
        mmwave_odom_imu_fusion_node,
    ])

