#!/usr/bin/env python3
import math
import struct
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header


class MmwaveOdomImuFusion(Node):
    """
    把毫米波雷达局部坐标系下的检测点转换到 odom 全局坐标系。

    输入：
      1. /mmwave/filtered_detections 或 /mmwave/detections
         PointCloud2，字段来自你现有的 mmwave_scan_converter.py：
           x, y, z, power, radial_velocity, snr, rcs, range,
           azimuth_deg, timestamp, source_id, target_type,
           beam_count, angular_extent_deg

      2. /wamv/odom
         nav_msgs/Odometry，用于提供主船全局位置和线速度。

      3. /wamv/imu/data
         sensor_msgs/Imu，用于提供主船姿态 yaw 和 yaw_rate。

    输出：
      1. /mmwave/global_detections
         PointCloud2，保留原始毫米波字段，并新增：
           global_x, global_y, global_z,
           ship_x, ship_y, ship_yaw,
           imu_yaw_rate,
           compensated_radial_velocity

      2. /mmwave/global_targets
         PoseArray，只包含全局目标位置，方便后续无人机任务节点使用。
    """

    def __init__(self):
        super().__init__('mmwave_odom_imu_fusion')

        # -----------------------------
        # 参数
        # -----------------------------
        self.declare_parameter('input_cloud_topic', '/mmwave/filtered_detections')
        self.declare_parameter('output_cloud_topic', '/mmwave/global_detections')
        self.declare_parameter('output_pose_topic', '/mmwave/global_targets')
        self.declare_parameter('odom_topic', '/wamv/odom')
        self.declare_parameter('imu_topic', '/wamv/imu/data')
        self.declare_parameter('global_frame_id', 'odom')

        # 雷达相对 base_link 的安装位置。
        # 如果你的毫米波雷达就在 base_link 原点附近，可以先保持默认 0。
        # 如果你知道雷达安装位置，例如前方 2.5m、高 1.86m，可以改参数。
        self.declare_parameter('radar_x_in_base', 0.0)
        self.declare_parameter('radar_y_in_base', 0.0)
        self.declare_parameter('radar_z_in_base', 0.0)
        self.declare_parameter('radar_yaw_in_base_deg', 0.0)

        # 如果 True：目标全局转换使用 IMU 的 yaw。
        # 如果 False：使用 /wamv/odom.pose.pose.orientation 的 yaw。
        self.declare_parameter('use_imu_yaw', True)

        # 如果毫米波输入点是 /mmwave/detections，里面包含海杂波 source_id=2。
        # 如果你只想给无人机发真实目标，可以打开此过滤。
        self.declare_parameter('publish_only_target_source', False)
        self.declare_parameter('target_source_id', 1.0)

        # 数据超时保护，防止 odom/imu 很久没更新还继续用旧数据。
        self.declare_parameter('max_state_age_sec', 1.0)

        input_cloud_topic = self.get_parameter('input_cloud_topic').value
        output_cloud_topic = self.get_parameter('output_cloud_topic').value
        output_pose_topic = self.get_parameter('output_pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        imu_topic = self.get_parameter('imu_topic').value

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        qos_normal = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -----------------------------
        # 状态缓存
        # -----------------------------
        self.latest_odom: Optional[Odometry] = None
        self.latest_imu: Optional[Imu] = None
        self.latest_odom_time = None
        self.latest_imu_time = None

        # -----------------------------
        # 订阅与发布
        # -----------------------------
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_normal,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            qos_sensor,
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            input_cloud_topic,
            self.cloud_callback,
            qos_sensor,
        )

        self.global_cloud_pub = self.create_publisher(
            PointCloud2,
            output_cloud_topic,
            qos_sensor,
        )

        self.global_pose_pub = self.create_publisher(
            PoseArray,
            output_pose_topic,
            qos_normal,
        )

        self.get_logger().info(f'subscribed cloud: {input_cloud_topic}')
        self.get_logger().info(f'subscribed odom : {odom_topic}')
        self.get_logger().info(f'subscribed imu  : {imu_topic}')
        self.get_logger().info(f'publishing global cloud: {output_cloud_topic}')
        self.get_logger().info(f'publishing global poses: {output_pose_topic}')

    # ==========================================================
    # 回调：保存 odom 和 imu
    # ==========================================================

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        self.latest_odom_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg
        self.latest_imu_time = self.get_clock().now()

    # ==========================================================
    # 主回调：处理毫米波点云
    # ==========================================================

    def cloud_callback(self, msg: PointCloud2):
        if self.latest_odom is None:
            self.get_logger().warn('No /wamv/odom received yet, skip mmwave frame.', throttle_duration_sec=2.0)
            return

        if self.latest_imu is None:
            self.get_logger().warn('No /wamv/imu/data received yet, skip mmwave frame.', throttle_duration_sec=2.0)
            return

        if not self.state_is_fresh():
            self.get_logger().warn('Odom or IMU data is too old, skip mmwave frame.', throttle_duration_sec=2.0)
            return

        field_map = self.build_field_map(msg)

        required_fields = [
            'x', 'y', 'z',
            'power', 'radial_velocity', 'snr', 'rcs',
            'range', 'azimuth_deg', 'timestamp',
            'source_id', 'target_type', 'beam_count', 'angular_extent_deg',
        ]

        for name in required_fields:
            if name not in field_map:
                self.get_logger().error(f'Input PointCloud2 missing required field: {name}')
                return

        global_frame_id = self.get_parameter('global_frame_id').value
        use_imu_yaw = bool(self.get_parameter('use_imu_yaw').value)
        publish_only_target_source = bool(self.get_parameter('publish_only_target_source').value)
        target_source_id = float(self.get_parameter('target_source_id').value)

        # 主船全局位置，来自 /wamv/odom
        ship_x = self.latest_odom.pose.pose.position.x
        ship_y = self.latest_odom.pose.pose.position.y
        ship_z = self.latest_odom.pose.pose.position.z

        # 主船全局速度，来自 /wamv/odom
        ship_vx = self.latest_odom.twist.twist.linear.x
        ship_vy = self.latest_odom.twist.twist.linear.y

        # 主船 yaw：推荐来自 IMU
        if use_imu_yaw:
            ship_yaw = self.quaternion_to_yaw(self.latest_imu.orientation)
        else:
            ship_yaw = self.quaternion_to_yaw(self.latest_odom.pose.pose.orientation)

        # 主船 yaw_rate：来自 IMU
        imu_yaw_rate = self.latest_imu.angular_velocity.z

        radar_x = float(self.get_parameter('radar_x_in_base').value)
        radar_y = float(self.get_parameter('radar_y_in_base').value)
        radar_z = float(self.get_parameter('radar_z_in_base').value)
        radar_yaw = math.radians(float(self.get_parameter('radar_yaw_in_base_deg').value))

        output_points = []
        output_poses = []

        point_count = msg.width * msg.height

        for i in range(point_count):
            base_offset = i * msg.point_step
            point = self.read_input_point(msg, field_map, base_offset)

            source_id = point['source_id']
            if publish_only_target_source and abs(source_id - target_source_id) > 1e-3:
                continue

            # --------------------------------------------------
            # 1. 雷达局部坐标 -> base_link 坐标
            # --------------------------------------------------
            # 输入 x/y/z 是毫米波雷达坐标系下的点。
            # 先考虑雷达相对 base_link 的安装 yaw 和 xyz 偏移。
            x_radar = point['x']
            y_radar = point['y']
            z_radar = point['z']

            cos_ry = math.cos(radar_yaw)
            sin_ry = math.sin(radar_yaw)

            x_base = radar_x + cos_ry * x_radar - sin_ry * y_radar
            y_base = radar_y + sin_ry * x_radar + cos_ry * y_radar
            z_base = radar_z + z_radar

            # --------------------------------------------------
            # 2. base_link 坐标 -> odom 全局坐标
            # --------------------------------------------------
            cos_yaw = math.cos(ship_yaw)
            sin_yaw = math.sin(ship_yaw)

            global_x = ship_x + cos_yaw * x_base - sin_yaw * y_base
            global_y = ship_y + sin_yaw * x_base + cos_yaw * y_base
            global_z = ship_z + z_base

            # --------------------------------------------------
            # 3. 用 odom 速度 + imu yaw_rate 做简单运动补偿
            # --------------------------------------------------
            # 从主船指向目标的全局视线方向
            dx = global_x - ship_x
            dy = global_y - ship_y
            dist = math.hypot(dx, dy)

            if dist > 1e-6:
                los_x = dx / dist
                los_y = dy / dist
            else:
                los_x = math.cos(ship_yaw)
                los_y = math.sin(ship_yaw)

            # 主船线速度在目标视线方向上的投影。
            # 如果船正在朝目标方向运动，这个值为正。
            ship_velocity_along_los = ship_vx * los_x + ship_vy * los_y

            # 原始 radial_velocity 是雷达相对测到的径向速度。
            # 加上主船沿视线方向速度，得到更接近目标在 odom 下的径向速度。
            compensated_radial_velocity = point['radial_velocity'] + ship_velocity_along_los

            # yaw_rate 的主要作用是给方位角变化做补偿。
            # 这里把它作为输出字段保留下来，后面做目标跟踪/滤波时可以继续用。
            # 简单理解：雷达看到的方位角变化中，有一部分来自船自己转头。
            # corrected_azimuth_rate ≈ measured_azimuth_rate + imu_yaw_rate

            output_points.append((
                # 原始雷达局部字段
                point['x'],
                point['y'],
                point['z'],
                point['power'],
                point['radial_velocity'],
                point['snr'],
                point['rcs'],
                point['range'],
                point['azimuth_deg'],
                point['timestamp'],
                point['source_id'],
                point['target_type'],
                point['beam_count'],
                point['angular_extent_deg'],

                # 新增全局/补偿字段
                global_x,
                global_y,
                global_z,
                ship_x,
                ship_y,
                ship_yaw,
                imu_yaw_rate,
                compensated_radial_velocity,
            ))

            pose = Pose()
            pose.position.x = global_x
            pose.position.y = global_y
            pose.position.z = global_z
            pose.orientation.w = 1.0
            output_poses.append(pose)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = global_frame_id

        global_cloud = self.create_output_cloud(header, output_points)
        self.global_cloud_pub.publish(global_cloud)

        pose_array = PoseArray()
        pose_array.header = header
        pose_array.poses = output_poses
        self.global_pose_pub.publish(pose_array)

    # ==========================================================
    # PointCloud2 读取与写入
    # ==========================================================

    def build_field_map(self, cloud: PointCloud2) -> Dict[str, PointField]:
        return {field.name: field for field in cloud.fields}

    def read_float32(self, cloud: PointCloud2, field_map: Dict[str, PointField], base_offset: int, name: str) -> float:
        field = field_map[name]

        if field.datatype != PointField.FLOAT32:
            raise RuntimeError(f'Field {name} is not FLOAT32')

        offset = base_offset + field.offset
        return struct.unpack_from('f', cloud.data, offset)[0]

    def read_input_point(self, cloud: PointCloud2, field_map: Dict[str, PointField], base_offset: int) -> Dict[str, float]:
        return {
            'x': self.read_float32(cloud, field_map, base_offset, 'x'),
            'y': self.read_float32(cloud, field_map, base_offset, 'y'),
            'z': self.read_float32(cloud, field_map, base_offset, 'z'),
            'power': self.read_float32(cloud, field_map, base_offset, 'power'),
            'radial_velocity': self.read_float32(cloud, field_map, base_offset, 'radial_velocity'),
            'snr': self.read_float32(cloud, field_map, base_offset, 'snr'),
            'rcs': self.read_float32(cloud, field_map, base_offset, 'rcs'),
            'range': self.read_float32(cloud, field_map, base_offset, 'range'),
            'azimuth_deg': self.read_float32(cloud, field_map, base_offset, 'azimuth_deg'),
            'timestamp': self.read_float32(cloud, field_map, base_offset, 'timestamp'),
            'source_id': self.read_float32(cloud, field_map, base_offset, 'source_id'),
            'target_type': self.read_float32(cloud, field_map, base_offset, 'target_type'),
            'beam_count': self.read_float32(cloud, field_map, base_offset, 'beam_count'),
            'angular_extent_deg': self.read_float32(cloud, field_map, base_offset, 'angular_extent_deg'),
        }

    def create_output_cloud(self, header: Header, points: List[tuple]) -> PointCloud2:
        fields = [
            # 原始毫米波字段，和 mmwave_scan_converter.py 保持一致
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='power', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='radial_velocity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='snr', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='rcs', offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='range', offset=28, datatype=PointField.FLOAT32, count=1),
            PointField(name='azimuth_deg', offset=32, datatype=PointField.FLOAT32, count=1),
            PointField(name='timestamp', offset=36, datatype=PointField.FLOAT32, count=1),
            PointField(name='source_id', offset=40, datatype=PointField.FLOAT32, count=1),
            PointField(name='target_type', offset=44, datatype=PointField.FLOAT32, count=1),
            PointField(name='beam_count', offset=48, datatype=PointField.FLOAT32, count=1),
            PointField(name='angular_extent_deg', offset=52, datatype=PointField.FLOAT32, count=1),

            # 新增字段
            PointField(name='global_x', offset=56, datatype=PointField.FLOAT32, count=1),
            PointField(name='global_y', offset=60, datatype=PointField.FLOAT32, count=1),
            PointField(name='global_z', offset=64, datatype=PointField.FLOAT32, count=1),
            PointField(name='ship_x', offset=68, datatype=PointField.FLOAT32, count=1),
            PointField(name='ship_y', offset=72, datatype=PointField.FLOAT32, count=1),
            PointField(name='ship_yaw', offset=76, datatype=PointField.FLOAT32, count=1),
            PointField(name='imu_yaw_rate', offset=80, datatype=PointField.FLOAT32, count=1),
            PointField(name='compensated_radial_velocity', offset=84, datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 88
        data = bytearray()

        for p in points:
            data.extend(struct.pack('ffffffffffffffffffffff', *p))

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = point_step
        cloud.row_step = point_step * len(points)
        cloud.is_dense = True
        cloud.data = bytes(data)
        return cloud

    # ==========================================================
    # 工具函数
    # ==========================================================

    def quaternion_to_yaw(self, q) -> float:
        """
        四元数转 yaw。
        不依赖 tf_transformations，避免额外 Python 依赖。
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def state_is_fresh(self) -> bool:
        max_age = float(self.get_parameter('max_state_age_sec').value)
        now = self.get_clock().now()

        if self.latest_odom_time is None or self.latest_imu_time is None:
            return False

        odom_age = (now - self.latest_odom_time).nanoseconds * 1e-9
        imu_age = (now - self.latest_imu_time).nanoseconds * 1e-9

        return odom_age <= max_age and imu_age <= max_age


def main(args=None):
    rclpy.init(args=args)
    node = MmwaveOdomImuFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
