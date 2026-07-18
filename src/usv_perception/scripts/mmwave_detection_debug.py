#!/usr/bin/env python3
import struct
from functools import partial

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class MmwaveDetectionDebug(Node):
    def __init__(self):
        super().__init__('mmwave_detection_debug')

        self._subs = []
        self.topic_map = {}
        for height in ('h3m', 'h4m', 'h1p9m'):
            for sector in ('front', 'right', 'back', 'left'):
                name = f'{height}_{sector}'
                self.topic_map[name] = f'/mmwave/{sector}/{height}/detections'

        for radar_name, topic_name in self.topic_map.items():
            sub = self.create_subscription(
                PointCloud2,
                topic_name,
                partial(self.callback, radar_name),
                10,
            )
            self._subs.append(sub)
            self.get_logger().info(f'Subscribed to {topic_name}')

    def callback(self, radar_name: str, msg: PointCloud2):
        self.get_logger().info(
            f'[{radar_name}] Received PointCloud2: '
            f'width={msg.width}, point_step={msg.point_step}, row_step={msg.row_step}'
        )

        field_map = {f.name: f.offset for f in msg.fields}

        required_fields = [
            'x', 'y', 'z',
            'power', 'radial_velocity', 'snr', 'rcs',
            'range', 'azimuth_deg', 'timestamp', 'source_id',
            'target_type', 'beam_count', 'angular_extent_deg'
        ]

        for name in required_fields:
            if name not in field_map:
                self.get_logger().warn(f'[{radar_name}] Missing field: {name}')
                return

        for i in range(msg.width):
            base = i * msg.point_step

            x = struct.unpack_from('f', msg.data, base + field_map['x'])[0]
            y = struct.unpack_from('f', msg.data, base + field_map['y'])[0]
            z = struct.unpack_from('f', msg.data, base + field_map['z'])[0]
            power = struct.unpack_from('f', msg.data, base + field_map['power'])[0]
            radial_velocity = struct.unpack_from('f', msg.data, base + field_map['radial_velocity'])[0]
            snr = struct.unpack_from('f', msg.data, base + field_map['snr'])[0]
            rcs = struct.unpack_from('f', msg.data, base + field_map['rcs'])[0]
            rng = struct.unpack_from('f', msg.data, base + field_map['range'])[0]
            azimuth_deg = struct.unpack_from('f', msg.data, base + field_map['azimuth_deg'])[0]
            timestamp = struct.unpack_from('f', msg.data, base + field_map['timestamp'])[0]
            source_id = struct.unpack_from('f', msg.data, base + field_map['source_id'])[0]
            target_type = struct.unpack_from('f', msg.data, base + field_map['target_type'])[0]
            beam_count = struct.unpack_from('f', msg.data, base + field_map['beam_count'])[0]
            angular_extent_deg = struct.unpack_from('f', msg.data, base + field_map['angular_extent_deg'])[0]

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

            self.get_logger().info(
                f'[{radar_name}][{i:03d}] '
                f'x={x:.2f}, y={y:.2f}, z={z:.2f}, '
                f'range={rng:.2f} m, azimuth={azimuth_deg:.2f} deg, '
                f'power={power:.3f}, snr={snr:.2f}, rcs={rcs:.2f}, '
                f'vel={radial_velocity:.2f} m/s, time={timestamp:.3f}, '
                f'source={int(source_id)}({source_name}), '
                f'target_type={int(target_type)}({target_type_name}), '
                f'beam_count={beam_count:.0f}, angular_extent={angular_extent_deg:.2f} deg'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MmwaveDetectionDebug()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
