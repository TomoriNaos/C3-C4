from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    package_share = FindPackageShare("depth_image_to_pointcloud2")
    gazebo_share = FindPackageShare("gazebo_ros")

    world_file = PathJoinSubstitution([package_share, "worlds", "fog_depth_camera.world"])
    rviz_config = PathJoinSubstitution([package_share, "rviz", "pointcloud.rviz"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world_file}.items(),
    )

    rgbd_processor = Node(
        package="depth_image_to_pointcloud2",
        executable="depth_image_to_pointcloud2",
        name="depth_image_to_pointcloud2",
        output="screen",
        parameters=[
            {
                # 需要调参：每 4 帧处理 1 帧，减轻 OpenCV 和点云发布压力。
                "frame_stride": 4,
                # 需要调参：16UC1 深度图单位换算；Gazebo 默认 32FC1 米制时不会用到。
                "depth_scale": 0.001,
                # 需要调参：超过该距离的点置为 NaN，避免 RViz 中远处噪声太多。
                "max_valid_depth": 20.0,
                "show_images": True,
                # 需要调参：去雾参数，雾更浓时可增大 omega 或 dark_channel_radius。
                "dark_channel_radius": 7,
                "omega": 0.95,
                "min_transmission": 0.10,
                "atmospheric_light_percent": 0.001,
                "depth_compensation_strength": 0.35,
                "max_depth_scale": 1.50,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([gazebo, rgbd_processor, rviz])
