#!/usr/bin/env python3
import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class MmwaveDetectionDebug(Node):
    def __init__(self):
        super().__init__('mmwave_detection_debug')

        self.declare_parameter('input_topic', '/mmwave/filtered_detections')
        self.declare_parameter('radar_name', 'filtered')
        self.declare_parameter('max_print_points', 20)
        self.declare_parameter('print_global_fields', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.radar_name = self.get_parameter('radar_name').value
        self.max_print_points = int(self.get_parameter('max_print_points').value)
        self.print_global_fields = bool(self.get_parameter('print_global_fields').value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.callback,
            qos,
        )

        self.get_logger().info(f'Subscribed to {self.input_topic}')

    def callback(self, msg: PointCloud2):
        self.get_logger().info(
            f'[{self.radar_name}] Received PointCloud2: '
            f'frame_id={msg.header.frame_id}, '
            f'width={msg.width}, point_step={msg.point_step}, row_step={msg.row_step}'
        )

        field_map = {f.name: f for f in msg.fields}

        required_fields = [
            'x', 'y', 'z',
            'power', 'radial_velocity', 'snr', 'rcs',
            'range', 'azimuth_deg', 'timestamp', 'source_id',
            'target_type', 'beam_count', 'angular_extent_deg'
        ]

        for name in required_fields:
            if name not in field_map:
                self.get_logger().warn(f'[{self.radar_name}] Missing field: {name}')
                return

        has_global_fields = all(name in field_map for name in [
            'global_x',
            'global_y',
            'global_z',
            'ship_x',
            'ship_y',
            'ship_yaw',
            'imu_yaw_rate',
            'compensated_radial_velocity',
        ])

        if self.print_global_fields and has_global_fields:
            self.get_logger().info(
                f'[{self.radar_name}] Global fields detected: '
                f'global_x/global_y/global_z + ship pose + imu yaw_rate will be printed.'
            )
        elif self.print_global_fields and not has_global_fields:
            self.get_logger().info(
                f'[{self.radar_name}] No global fields detected. '
                f'Only local radar fields will be printed.'
            )

        print_count = min(msg.width, self.max_print_points)

        for i in range(print_count):
            base = i * msg.point_step

            x = self.read_float32(msg, field_map, base, 'x')
            y = self.read_float32(msg, field_map, base, 'y')
            z = self.read_float32(msg, field_map, base, 'z')
            power = self.read_float32(msg, field_map, base, 'power')
            radial_velocity = self.read_float32(msg, field_map, base, 'radial_velocity')
            snr = self.read_float32(msg, field_map, base, 'snr')
            rcs = self.read_float32(msg, field_map, base, 'rcs')
            rng = self.read_float32(msg, field_map, base, 'range')
            azimuth_deg = self.read_float32(msg, field_map, base, 'azimuth_deg')
            timestamp = self.read_float32(msg, field_map, base, 'timestamp')
            source_id = self.read_float32(msg, field_map, base, 'source_id')
            target_type = self.read_float32(msg, field_map, base, 'target_type')
            beam_count = self.read_float32(msg, field_map, base, 'beam_count')
            angular_extent_deg = self.read_float32(msg, field_map, base, 'angular_extent_deg')

            if int(target_type) == 1:
                target_type_name = 'point_target'
            elif int(target_type) == 2:
                target_type_name = 'extended_target'
            else:
                target_type_name = 'clutter_or_unknown'

            if int(source_id) == 1:
                source_name = 'real_target'
            elif int(source_id) == 2:
                source_name = 'sea_clutter'
            else:
                source_name = 'unknown'

            local_text = (
                f'[{self.radar_name}][{i:03d}] '
                f'LOCAL radar: x={x:.2f}, y={y:.2f}, z={z:.2f}, '
                f'range={rng:.2f} m, azimuth={azimuth_deg:.2f} deg, '
                f'power={power:.3f}, snr={snr:.2f}, rcs={rcs:.2f}, '
                f'raw_vel={radial_velocity:.2f} m/s, time={timestamp:.3f}, '
                f'source={int(source_id)}({source_name}), '
                f'target_type={int(target_type)}({target_type_name}), '
                f'beam_count={beam_count:.0f}, angular_extent={angular_extent_deg:.2f} deg'
            )

            self.get_logger().info(local_text)

            if self.print_global_fields and has_global_fields:
                global_x = self.read_float32(msg, field_map, base, 'global_x')
                global_y = self.read_float32(msg, field_map, base, 'global_y')
                global_z = self.read_float32(msg, field_map, base, 'global_z')
                ship_x = self.read_float32(msg, field_map, base, 'ship_x')
                ship_y = self.read_float32(msg, field_map, base, 'ship_y')
                ship_yaw = self.read_float32(msg, field_map, base, 'ship_yaw')
                imu_yaw_rate = self.read_float32(msg, field_map, base, 'imu_yaw_rate')
                compensated_radial_velocity = self.read_float32(
                    msg,
                    field_map,
                    base,
                    'compensated_radial_velocity'
                )

                global_text = (
                    f'[{self.radar_name}][{i:03d}] '
                    f'GLOBAL odom: x={global_x:.2f}, y={global_y:.2f}, z={global_z:.2f}, '
                    f'ship=({ship_x:.2f}, {ship_y:.2f}), '
                    f'ship_yaw={math.degrees(ship_yaw):.2f} deg, '
                    f'imu_yaw_rate={math.degrees(imu_yaw_rate):.2f} deg/s, '
                    f'comp_vel={compensated_radial_velocity:.2f} m/s'
                )

                self.get_logger().info(global_text)

        if msg.width > print_count:
            self.get_logger().info(
                f'[{self.radar_name}] Printed {print_count}/{msg.width} points. '
                f'Increase max_print_points to print more.'
            )

    def read_float32(self, msg: PointCloud2, field_map, base: int, field_name: str) -> float:
        field = field_map[field_name]

        if field.datatype != PointField.FLOAT32:
            self.get_logger().warn(
                f'Field {field_name} datatype is {field.datatype}, expected FLOAT32.'
            )

        return struct.unpack_from('f', msg.data, base + field.offset)[0]


def main(args=None):
    rclpy.init(args=args)
    node = MmwaveDetectionDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
