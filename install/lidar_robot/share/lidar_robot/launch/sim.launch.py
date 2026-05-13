from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_robot')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf')

    # 将 xacro 转为 urdf
    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', urdf_file],
        output='screen'
    )

    # 启动 Gazebo
    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 将机器人模型加载进 Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', urdf_file],
        output='screen'
    )

    # 发布静态 TF
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


        return LaunchDescription([
        xacro_to_urdf,
        start_gazebo,
        spawn_robot,
        robot_state_publisher,
        mmwave_radar_node,
    ])
