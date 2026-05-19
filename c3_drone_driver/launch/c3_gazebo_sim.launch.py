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

    pkg_share = get_package_share_directory("c3_drone_driver")
    gazebo_launch_file = os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
    )
    model_file = PathJoinSubstitution(
        [FindPackageShare("c3_drone_driver"), "urdf", "c3_drone_with_gimbal.urdf.xacro"]
    )

    robot_description = {"robot_description": Command(["xacro ", model_file])}

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOG_DIR", "/tmp/ros_logs"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
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
                package="c3_drone_driver",
                executable="sensor_mock_node",
                name="sensor_mock_node",
                output="screen",
                parameters=[os.path.join(pkg_share, "config", "sensor_mock.yaml")],
            ),
            Node(
                package="c3_drone_driver",
                executable="target_processor_node",
                name="target_processor_node",
                output="screen",
            ),
            Node(
                package="c3_drone_driver",
                executable="gimbal_controller_node",
                name="gimbal_controller_node",
                output="screen",
                parameters=[os.path.join(pkg_share, "config", "gimbal.yaml")],
            ),
            Node(
                package="c3_drone_driver",
                executable="gimbal_joint_state_bridge_node",
                name="gimbal_joint_state_bridge_node",
                output="screen",
            ),
            Node(
                package="c3_drone_driver",
                executable="drone_main_controller_node",
                name="drone_main_controller_node",
                output="screen",
                parameters=[
                    os.path.join(pkg_share, "config", "drone_main_controller.yaml"),
                    {"pose_config_file": os.path.join(pkg_share, "config", "pose_estimator_default.yaml")},
                ],
            ),
            Node(
                package="c3_drone_driver",
                executable="motion_controller_node",
                name="motion_controller_node",
                output="screen",
                parameters=[os.path.join(pkg_share, "config", "motion_controller.yaml")],
            ),
            Node(
                package="c3_drone_driver",
                executable="px4_pose_bridge_node",
                name="px4_pose_bridge_node",
                output="screen",
                parameters=[{
                    "use_odom_input": True,
                    "odom_topic": "/odom",
                    "output_topic": "/px4/vehicle_pose",
                    "output_frame_id": "ned",
                }],
            ),
            Node(
                package="c3_drone_driver",
                executable="offboard_setpoint_bridge_node",
                name="offboard_setpoint_bridge_node",
                output="screen",
                parameters=[{
                    "input_topic": "/px4/offboard_goal",
                    "output_topic": "/px4/setpoint_pose",
                    "force_frame_id": "ned",
                }],
            ),
            Node(
                package="c3_drone_driver",
                executable="mavlink_bridge_node",
                name="mavlink_bridge_node",
                output="screen",
                parameters=[os.path.join(pkg_share, "config", "mavlink_bridge.yaml")],
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
