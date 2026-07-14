#!/usr/bin/env python3
"""Convert ideal Gazebo sonar rays into delayed, noisy C3 sonar detections."""

import math
import random
import struct
from collections import deque

import rclpy
from c3_sonar_driver.msg import SonarDetect, SonarStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class SonarScanSimulator(Node):
    """Models finite resolution, missed detections, multipath and false alarms."""

    def __init__(self):
        super().__init__('sonar_scan_simulator')
        defaults = {
            'input_topic': '/sonar/scan', 'output_topic': '/sonar/detect', 'status_topic': '/sonar/status',
            'frame_id': 'base_link', 'mount_yaw_rad': 0.0, 'sensor_x_offset_m': 0.65, 'sensor_id': 1,
            'min_range_m': 4.0, 'max_range_m': 60.0, 'min_cluster_beams': 2,
            'range_noise_stddev_m': 0.22, 'bearing_noise_stddev_rad': 0.018,
            'range_resolution_m': 0.15, 'bearing_resolution_rad': 0.026,
            'near_detection_probability': 0.96, 'far_detection_probability': 0.52,
            'false_alarm_rate_per_scan': 0.18, 'multipath_probability': 0.10,
            'multipath_range_bias_m': 1.8, 'latency_mean_ms': 95.0, 'latency_jitter_ms': 35.0,
            'sound_speed_mps': 1482.0, 'random_seed': 29,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.mount_yaw = float(self.get_parameter('mount_yaw_rad').value)
        self.sensor_offset = float(self.get_parameter('sensor_x_offset_m').value)
        self.sensor_id = int(self.get_parameter('sensor_id').value)
        self.random = random.Random(int(self.get_parameter('random_seed').value) + self.sensor_id)
        self.detect_id = self.sensor_id * 1_000_000
        self.pending = deque()
        self.create_subscription(LaserScan, self.get_parameter('input_topic').value, self.scan_callback, 10)
        self.detect_pub = self.create_publisher(SonarDetect, self.get_parameter('output_topic').value, 20)
        self.status_pub = self.create_publisher(SonarStatus, self.get_parameter('status_topic').value, 10)
        self.create_timer(0.02, self.flush_pending)
        self.get_logger().info(f'Sonar simulation sensor={self.sensor_id} input={self.get_parameter("input_topic").value}')

    def scan_callback(self, scan):
        for cluster in self.extract_clusters(scan):
            detection = self.make_target_detection(scan, cluster)
            if detection is not None:
                self.schedule(detection)
        if self.random.random() < float(self.get_parameter('false_alarm_rate_per_scan').value):
            self.schedule(self.make_false_alarm(scan))
        self.publish_status()

    def extract_clusters(self, scan):
        minimum = max(float(scan.range_min), float(self.get_parameter('min_range_m').value))
        maximum = min(float(scan.range_max), float(self.get_parameter('max_range_m').value))
        clusters, current, previous = [], [], None
        for index, value in enumerate(scan.ranges):
            valid = math.isfinite(value) and minimum <= value <= maximum
            if not valid:
                if current:
                    clusters.append(current)
                    current = []
                previous = None
                continue
            if previous is not None and abs(value - previous) > max(0.7, 0.08 * maximum):
                clusters.append(current)
                current = []
            current.append((index, value))
            previous = value
        if current:
            clusters.append(current)
        return [cluster for cluster in clusters if len(cluster) >= int(self.get_parameter('min_cluster_beams').value)]

    def make_target_detection(self, scan, cluster):
        raw_range = sum(value for _, value in cluster) / len(cluster)
        center_index = sum(index for index, _ in cluster) / len(cluster)
        raw_bearing = scan.angle_min + center_index * scan.angle_increment + self.mount_yaw
        maximum = max(1.0, float(self.get_parameter('max_range_m').value))
        near = float(self.get_parameter('near_detection_probability').value)
        far = float(self.get_parameter('far_detection_probability').value)
        probability = near + (far - near) * (raw_range / maximum) ** 1.4
        probability *= min(1.0, 0.55 + 0.18 * len(cluster))
        if self.random.random() > clamp(probability, 0.0, 1.0):
            return None
        measured_range = self.quantize(raw_range + self.random.gauss(0.0, float(self.get_parameter('range_noise_stddev_m').value)), float(self.get_parameter('range_resolution_m').value))
        measured_bearing = self.quantize(raw_bearing + self.random.gauss(0.0, float(self.get_parameter('bearing_noise_stddev_rad').value)), float(self.get_parameter('bearing_resolution_rad').value))
        confidence = clamp(0.92 - 0.42 * raw_range / maximum + 0.025 * len(cluster), 0.16, 0.97)
        return self.build_detection(scan, measured_range, measured_bearing, confidence, len(cluster))

    def make_false_alarm(self, scan):
        minimum = max(float(scan.range_min), float(self.get_parameter('min_range_m').value))
        maximum = min(float(scan.range_max), float(self.get_parameter('max_range_m').value))
        return self.build_detection(scan, self.random.uniform(minimum, maximum), self.mount_yaw + self.random.uniform(scan.angle_min, scan.angle_max), self.random.uniform(0.08, 0.30), 0)

    def build_detection(self, scan, measured_range, bearing, confidence, cluster_size):
        if self.random.random() < float(self.get_parameter('multipath_probability').value):
            measured_range += abs(self.random.gauss(float(self.get_parameter('multipath_range_bias_m').value), 0.45))
            confidence *= 0.70
        self.detect_id += 1
        detection = SonarDetect()
        detection.header = scan.header
        detection.header.frame_id = self.get_parameter('frame_id').value
        detection.detect_id, detection.range_m, detection.bearing_rad = self.detect_id, float(measured_range), float(bearing)
        detection.confidence = float(clamp(confidence, 0.03, 0.98))
        detection.position.x = self.sensor_offset + measured_range * math.cos(bearing)
        detection.position.y, detection.position.z = measured_range * math.sin(bearing), 0.0
        detection.cloud = self.make_cloud(detection.header, measured_range, bearing, cluster_size)
        return detection

    def make_cloud(self, header, measured_range, bearing, cluster_size):
        count = max(3, min(12, cluster_size if cluster_size else 3))
        points = []
        for index in range(count):
            spread = (index - 0.5 * (count - 1)) * 0.10
            point_range = measured_range + self.random.gauss(0.0, 0.08)
            point_bearing = bearing + spread + self.random.gauss(0.0, 0.006)
            points.append((self.sensor_offset + point_range * math.cos(point_bearing), point_range * math.sin(point_bearing), 0.0, max(0.05, 1.0 - abs(spread) * 3.0)))
        cloud = PointCloud2()
        cloud.header, cloud.height, cloud.width = header, 1, len(points)
        cloud.is_bigendian, cloud.is_dense, cloud.point_step = False, True, 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.data = b''.join(struct.pack('<ffff', *point) for point in points)
        return cloud

    def schedule(self, detection):
        latency = float(self.get_parameter('latency_mean_ms').value) + self.random.gauss(0.0, float(self.get_parameter('latency_jitter_ms').value))
        self.pending.append((self.time_seconds() + max(0.0, latency) * 1e-3, detection))

    def flush_pending(self):
        now = self.time_seconds()
        while self.pending and self.pending[0][0] <= now:
            _, detection = self.pending.popleft()
            detection.header.stamp = self.get_clock().now().to_msg()
            detection.cloud.header = detection.header
            self.detect_pub.publish(detection)

    def publish_status(self):
        status = SonarStatus()
        status.header.stamp, status.header.frame_id = self.get_clock().now().to_msg(), self.get_parameter('frame_id').value
        status.t_usec = status.header.stamp.sec * 1_000_000 + status.header.stamp.nanosec // 1_000
        status.sonar_active = True
        status.estimated_sound_speed_mps = float(self.get_parameter('sound_speed_mps').value)
        status.estimated_latency_ms = float(self.get_parameter('latency_mean_ms').value)
        self.status_pub.publish(status)

    def time_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def quantize(value, resolution):
        return round(value / resolution) * resolution if resolution > 0.0 else value


def main(args=None):
    rclpy.init(args=args)
    node = SonarScanSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
