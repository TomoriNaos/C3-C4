#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class MmwaveDetectionsMerger(Node):
    def __init__(self):
        super().__init__('mmwave_detections_merger')

        self.declare_parameter('front_topic', '/mmwave/detections/front')
        self.declare_parameter('left_topic', '/mmwave/detections/left')
        self.declare_parameter('right_topic', '/mmwave/detections/right')
        self.declare_parameter('rear_topic', '/mmwave/detections/rear')
        self.declare_parameter('output_topic', '/mmwave/detections_fused')
        self.declare_parameter('publish_rate_hz', 12.5)
        self.declare_parameter('max_cloud_age_sec', 0.15)
        self.declare_parameter('frame_id', 'base_link')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.clouds = {
            'front': None,
            'left': None,
            'right': None,
            'rear': None,
        }
        self.cloud_times = {
            'front': None,
            'left': None,
            'right': None,
            'rear': None,
        }

        self.create_subscription(PointCloud2, self.get_parameter('front_topic').value,
                                 lambda msg: self.cloud_callback('front', msg), qos)
        self.create_subscription(PointCloud2, self.get_parameter('left_topic').value,
                                 lambda msg: self.cloud_callback('left', msg), qos)
        self.create_subscription(PointCloud2, self.get_parameter('right_topic').value,
                                 lambda msg: self.cloud_callback('right', msg), qos)
        self.create_subscription(PointCloud2, self.get_parameter('rear_topic').value,
                                 lambda msg: self.cloud_callback('rear', msg), qos)

        self.pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('output_topic').value,
            qos,
        )

        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_merged_cloud)

    def cloud_callback(self, name, msg):
        self.clouds[name] = msg
        self.cloud_times[name] = self.get_clock().now()

    def publish_merged_cloud(self):
        valid_clouds = self.get_fresh_clouds()
        if not valid_clouds:
            return

        base = valid_clouds[0]

        # 要求四路 converter 输出字段完全一样：fields、point_step、is_bigendian 等一致。
        for cloud in valid_clouds[1:]:
            if cloud.point_step != base.point_step:
                self.get_logger().warn('PointCloud2 point_step mismatch, skip merge.', throttle_duration_sec=2.0)
                return
            if len(cloud.fields) != len(base.fields):
                self.get_logger().warn('PointCloud2 fields mismatch, skip merge.', throttle_duration_sec=2.0)
                return

        merged = PointCloud2()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = self.get_parameter('frame_id').value

        merged.height = 1
        merged.fields = base.fields
        merged.is_bigendian = base.is_bigendian
        merged.point_step = base.point_step
        merged.is_dense = True

        merged_data = bytearray()
        total_points = 0

        for cloud in valid_clouds:
            point_count = cloud.width * cloud.height
            expected_size = point_count * cloud.point_step
            merged_data.extend(cloud.data[:expected_size])
            total_points += point_count

        merged.width = total_points
        merged.row_step = merged.point_step * merged.width
        merged.data = bytes(merged_data)

        self.pub.publish(merged)

    def get_fresh_clouds(self):
        now = self.get_clock().now()
        max_age = float(self.get_parameter('max_cloud_age_sec').value)

        fresh = []
        for name in ['front', 'left', 'right', 'rear']:
            cloud = self.clouds[name]
            stamp = self.cloud_times[name]
            if cloud is None or stamp is None:
                continue

            age = (now - stamp).nanoseconds * 1e-9
            if age <= max_age:
                fresh.append(cloud)

        return fresh


def main(args=None):
    rclpy.init(args=args)
    node = MmwaveDetectionsMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
