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
            'cloud_min_points': 4, 'cloud_max_points': 10,
            'cloud_range_spread_m': 0.35, 'cloud_bearing_spread_rad': 0.012,
            'sound_speed_mps': 1482.0, 'random_seed': 29,
            'absorption_db_per_m': 0.012, 'bottom_reverb_rate_per_scan': 0.10,
            'surface_reverb_rate_per_scan': 0.08, 'shadow_probability': 0.10,
            'beam_pattern_sigma_rad': 0.42, 'reverberation_noise': 0.18,
            'sound_speed_stddev_mps': 4.0, 'latency_range_coupling': True,
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
            self.schedule(self.make_false_alarm(scan, 'water_column'))
        if self.random.random() < float(self.get_parameter('bottom_reverb_rate_per_scan').value):
            self.schedule(self.make_false_alarm(scan, 'bottom'))
        if self.random.random() < float(self.get_parameter('surface_reverb_rate_per_scan').value):
            self.schedule(self.make_false_alarm(scan, 'surface'))
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
            if previous is not None and abs(value - previous) > max(0.45, 0.035 * value):
                clusters.append(current)
                current = []
            current.append((index, value))
            previous = value
        if current:
            clusters.append(current)
        return [cluster for cluster in clusters if len(cluster) >= int(self.get_parameter('min_cluster_beams').value)]

    def make_target_detection(self, scan, cluster):
        raw_range = self.weighted_cluster_range(cluster)
        center_index = self.weighted_cluster_index(cluster)
        raw_bearing = scan.angle_min + center_index * scan.angle_increment + self.mount_yaw
        maximum = max(1.0, float(self.get_parameter('max_range_m').value))

        beam_gain = self.beam_gain(raw_bearing - self.mount_yaw)
        spreading_loss = 1.0 / max(raw_range * raw_range, 1.0)
        absorption = 10.0 ** (-float(self.get_parameter('absorption_db_per_m').value) * raw_range / 20.0)
        echo_strength = beam_gain * spreading_loss * absorption * (0.8 + 0.25 * len(cluster))
        echo_strength *= self.random.gammavariate(1.8, 1.0 / 1.8)

        near = float(self.get_parameter('near_detection_probability').value)
        far = float(self.get_parameter('far_detection_probability').value)
        range_factor = clamp(raw_range / maximum, 0.0, 1.0)
        probability = near + (far - near) * (range_factor ** 1.7)
        probability *= clamp(0.55 + 0.12 * len(cluster), 0.0, 1.0)
        probability *= clamp(0.65 + 7.0 * echo_strength, 0.0, 1.0)

        if self.random.random() < float(self.get_parameter('shadow_probability').value) * range_factor:
            probability *= 0.35

        if self.random.random() > clamp(probability, 0.0, 1.0):
            return None

        snr_like = max(0.05, echo_strength / max(float(self.get_parameter('reverberation_noise').value), 1e-3))
        range_std = float(self.get_parameter('range_noise_stddev_m').value) + 0.45 / math.sqrt(snr_like + 1.0)
        bearing_std = float(self.get_parameter('bearing_noise_stddev_rad').value) + 0.025 / math.sqrt(snr_like + 1.0)

        measured_range = self.quantize(raw_range + self.random.gauss(0.0, range_std), float(self.get_parameter('range_resolution_m').value))
        measured_bearing = self.quantize(raw_bearing + self.random.gauss(0.0, bearing_std), float(self.get_parameter('bearing_resolution_rad').value))
        confidence = clamp(0.15 + 0.78 * probability + 0.05 * math.log1p(len(cluster)), 0.05, 0.98)
        return self.build_detection(scan, measured_range, measured_bearing, confidence, len(cluster), snr_like)

    def weighted_cluster_range(self, cluster):
        weights = [1.0 / max(value, 1.0) for _, value in cluster]
        return sum(value * weight for (_, value), weight in zip(cluster, weights)) / max(sum(weights), 1e-6)

    def weighted_cluster_index(self, cluster):
        weights = [1.0 / max(value, 1.0) for _, value in cluster]
        return sum(index * weight for (index, _), weight in zip(cluster, weights)) / max(sum(weights), 1e-6)

    def make_false_alarm(self, scan, kind):
        minimum = max(float(scan.range_min), float(self.get_parameter('min_range_m').value))
        maximum = min(float(scan.range_max), float(self.get_parameter('max_range_m').value))
        u = self.random.random()

        if kind == 'bottom':
            measured_range = minimum + (maximum - minimum) * (0.55 + 0.45 * u)
            confidence = self.random.uniform(0.10, 0.34)
        elif kind == 'surface':
            measured_range = minimum + (maximum - minimum) * (u ** 2.2)
            confidence = self.random.uniform(0.08, 0.26)
        else:
            measured_range = minimum + (maximum - minimum) * u
            confidence = self.random.uniform(0.06, 0.28)

        bearing = self.mount_yaw + self.random.uniform(scan.angle_min, scan.angle_max)
        bearing += self.random.gauss(0.0, 0.5 * float(self.get_parameter('bearing_resolution_rad').value))
        return self.build_detection(scan, measured_range, bearing, confidence, 0, 0.15)

    def build_detection(self, scan, measured_range, bearing, confidence, cluster_size, snr_like):
        if self.random.random() < float(self.get_parameter('multipath_probability').value):
            measured_range += abs(self.random.gauss(float(self.get_parameter('multipath_range_bias_m').value), 0.55))
            bearing += self.random.gauss(0.0, 0.5 * float(self.get_parameter('bearing_resolution_rad').value))
            confidence *= 0.68
        measured_range = max(0.0, measured_range)
        self.detect_id += 1
        detection = SonarDetect()
        detection.header = scan.header
        detection.header.frame_id = self.get_parameter('frame_id').value
        detection.detect_id, detection.range_m, detection.bearing_rad = self.detect_id, float(measured_range), float(bearing)
        detection.confidence = float(clamp(confidence, 0.03, 0.98))
        detection.position.x = self.sensor_offset + measured_range * math.cos(bearing)
        detection.position.y, detection.position.z = measured_range * math.sin(bearing), 0.0
        detection.velocity.x = 0.0
        detection.velocity.y = 0.0
        detection.velocity.z = 0.0
        detection.cloud = self.make_cloud(detection.header, measured_range, bearing, cluster_size, snr_like)
        return detection

    def make_cloud(self, header, measured_range, bearing, cluster_size, snr_like):
        min_points = int(self.get_parameter('cloud_min_points').value)
        max_points = int(self.get_parameter('cloud_max_points').value)
        count = max(min_points, min(max_points, cluster_size + 3 if cluster_size else min_points))
        range_spread = float(self.get_parameter('cloud_range_spread_m').value) * (1.0 + 0.5 / math.sqrt(snr_like + 1.0))
        bearing_spread = float(self.get_parameter('cloud_bearing_spread_rad').value) * (1.0 + 0.7 / math.sqrt(snr_like + 1.0))
        points = []
        for _ in range(count):
            point_range = max(0.0, measured_range + self.random.gauss(0.0, range_spread))
            point_bearing = bearing + self.random.gauss(0.0, bearing_spread)
            radial_error = abs(point_range - measured_range)
            bearing_error = abs(point_bearing - bearing)
            intensity = clamp(math.exp(-1.8 * radial_error - 55.0 * bearing_error * bearing_error), 0.05, 1.0)
            intensity *= clamp(0.55 + 0.35 * snr_like, 0.08, 1.0)
            points.append((
                self.sensor_offset + point_range * math.cos(point_bearing),
                point_range * math.sin(point_bearing),
                0.0,
                intensity,
            ))
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
        if bool(self.get_parameter('latency_range_coupling').value):
            latency += 1000.0 * 2.0 * detection.range_m / max(1000.0, self.estimated_sound_speed())
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
        status.estimated_sound_speed_mps = float(self.estimated_sound_speed())
        status.estimated_latency_ms = float(self.get_parameter('latency_mean_ms').value)
        self.status_pub.publish(status)

    def estimated_sound_speed(self):
        base = float(self.get_parameter('sound_speed_mps').value)
        stddev = float(self.get_parameter('sound_speed_stddev_mps').value)
        return base + self.random.gauss(0.0, stddev)

    def beam_gain(self, local_bearing):
        sigma = max(1e-3, float(self.get_parameter('beam_pattern_sigma_rad').value))
        return clamp(math.exp(-(local_bearing * local_bearing) / (2.0 * sigma * sigma)), 0.05, 1.0)

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
