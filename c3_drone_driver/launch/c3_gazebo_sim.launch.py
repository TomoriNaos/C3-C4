import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    tc_camera_xyz = LaunchConfiguration("tc_camera_xyz")
    tc_camera_rpy = LaunchConfiguration("tc_camera_rpy")
    gated_camera_xyz = LaunchConfiguration("gated_camera_xyz")
    gated_camera_rpy = LaunchConfiguration("gated_camera_rpy")

    pkg_share = get_package_share_directory("c3_drone_driver")
    gazebo_launch_file = os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
    )
    model_file = PathJoinSubstitution(
        [FindPackageShare("c3_drone_driver"), "urdf", "c3_drone_with_gimbal.urdf.xacro"]
    )

    robot_description = {"robot_description": Command([
        "xacro ",
        model_file,
        " tc_camera_xyz:=",
        tc_camera_xyz,
        " tc_camera_rpy:=",
        tc_camera_rpy,
        " gated_camera_xyz:=",
        gated_camera_xyz,
        " gated_camera_rpy:=",
        gated_camera_rpy,
    ])}

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOG_DIR", "/tmp/ros_logs"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("tc_camera_xyz", default_value="0.050 0.020 0.0"),
            DeclareLaunchArgument("tc_camera_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("gated_camera_xyz", default_value="0.050 -0.020 0.0"),
            DeclareLaunchArgument("gated_camera_rpy", default_value="0 0 0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_file),
                launch_arguments={"verbose": "true"}.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_c3_drone",
                output="screen",
                arguments=[
                    "-entity",
                    "c3_drone",
                    "-topic",
                    "robot_description",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.5",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
