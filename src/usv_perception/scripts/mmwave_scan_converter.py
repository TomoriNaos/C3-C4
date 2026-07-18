#!/usr/bin/env python3
import math
import random
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header


class MmwaveScanConverter(Node):
    def __init__(self):
        super().__init__('mmwave_scan_converter')

        self.declare_parameter('input_topic', '/radar/raw_scan')
        self.declare_parameter('output_topic', '/mmwave/detections')
        self.declare_parameter('frame_id', 'radar_link')
        self.declare_parameter('output_frame_id', 'base_link')
        self.declare_parameter('mount_yaw_rad', 0.0)

        # 粗检测层参数
        self.declare_parameter('max_range', 800.0)
        self.declare_parameter('min_range', 2.0)
        self.declare_parameter('horizontal_fov_deg', 120.0)
        self.declare_parameter('angular_resolution_deg', 1.5)   # 最终检测单元宽度

        # 细采样层参数
        self.declare_parameter('subbeam_resolution_deg', 0.3)   # Gazebo 原始扫描分辨率

        self.declare_parameter('radar_height_m', 1.86)

        # 真实目标建模参数
        self.declare_parameter('intensity_threshold', 0.10)
        self.declare_parameter('target_base_rcs', 1.0)
        self.declare_parameter('target_material_reflectivity', 0.85)
        self.declare_parameter('target_random_stddev', 0.04)
        self.declare_parameter('radial_velocity_noise_stddev', 0.05)
        self.declare_parameter('range_attenuation_alpha', 0.0035)

        # 测角噪声
        self.declare_parameter('point_target_angle_noise_deg', 0.25)
        self.declare_parameter('extended_target_angle_noise_deg', 0.10)

        # 组内中心方向软加权
        self.declare_parameter('center_weight_enabled', True)
        self.declare_parameter('center_weight_sigma', 1.0)
        self.declare_parameter('center_weight_min', 0.7)

        # 海杂波建模参数
        self.declare_parameter('sea_clutter_enabled', True)
        self.declare_parameter('sea_state', 0.65)
        self.declare_parameter('sea_clutter_range_min', 5.0)
        self.declare_parameter('sea_clutter_range_max', 800.0)
        self.declare_parameter('sea_clutter_height_scale', 0.55)
        self.declare_parameter('sea_clutter_height_max', 2.0)
        self.declare_parameter('sea_clutter_compete_min_height', 0.30)
        self.declare_parameter('sea_clutter_random_stddev', 0.06)
        self.declare_parameter('sea_clutter_probability_per_bin', 1.0)
        # Sea returns immediately around the hull are dominated by multipath and
        # wake effects. Keep them in the simulation, but reduce their strength.
        self.declare_parameter('sea_clutter_near_field_radius_m', 35.0)
        self.declare_parameter('sea_clutter_near_field_attenuation', 0.12)
        # A one-frame sea return is not a physical object. Clutter must be
        # spatially and kinematically persistent before leaving this simulator.
        self.declare_parameter('clutter_confirmation_frames', 3)
        self.declare_parameter('clutter_range_gate_m', 1.5)
        self.declare_parameter('clutter_velocity_gate_mps', 0.30)
        self._clutter_history = {}

        # 输出限制
        self.declare_parameter('max_detections', 80)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10
        )
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(f'mmWave converter subscribed to {input_topic}')
        self.get_logger().info(f'mmWave detections publishing to {output_topic}')

    def scan_callback(self, msg: LaserScan):
        min_range = float(self.get_parameter('min_range').value)
        max_range = float(self.get_parameter('max_range').value)
        horizontal_fov_deg = float(self.get_parameter('horizontal_fov_deg').value)
        angular_resolution_deg = float(self.get_parameter('angular_resolution_deg').value)
        subbeam_resolution_deg = float(self.get_parameter('subbeam_resolution_deg').value)
        intensity_threshold = float(self.get_parameter('intensity_threshold').value)
        max_detections = int(self.get_parameter('max_detections').value)

        fov_half_rad = math.radians(horizontal_fov_deg * 0.5)
        fov_epsilon = 0.5 * abs(msg.angle_increment)

        # 1.5° / 0.3° = 5 个子 beam 一组
        subbeams_per_bin = max(1, int(round(angular_resolution_deg / subbeam_resolution_deg)))
        coarse_bin_count = max(1, int(round(horizontal_fov_deg / angular_resolution_deg)))

        # 先收集落在 FOV 内的原始子 beam
        subbeams = []
        for i, rng in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if angle < -fov_half_rad - fov_epsilon or angle > fov_half_rad + fov_epsilon:
                continue

            hit_valid = math.isfinite(rng) and (min_range <= rng <= max_range)
            subbeams.append({
                'angle': angle,
                'range': rng if hit_valid else None,
                'hit_valid': hit_valid,
            })

        expected_subbeams = subbeams_per_bin * coarse_bin_count
        if len(subbeams) + subbeams_per_bin < expected_subbeams:
            self.get_logger().warn(
                f'Collected subbeams ({len(subbeams)}) fewer than expected ({expected_subbeams})'
            )

        detections = []
        frame_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 按 1.5° 粗角单元分组，每组融合 5 个 0.3° 子 beam
        for coarse_idx in range(coarse_bin_count):
            start = coarse_idx * subbeams_per_bin
            end = start + subbeams_per_bin
            group = subbeams[start:end]
            if not group:
                continue

            target_candidate = self.build_group_target_candidate(group)
            clutter_candidate = self.build_group_clutter_candidate(group, min_range, max_range)

            winner = self.select_candidate(target_candidate, clutter_candidate)
            if winner is None or winner['snr'] < intensity_threshold:
                continue
            if winner['source'] == 2.0 and not self.accept_persistent_clutter(coarse_idx, winner):
                continue

            detections.append(self.candidate_to_point(winner, frame_timestamp))

        detections.sort(key=lambda p: p[5], reverse=True)
        detections = detections[:max_detections]

        output_header = Header()
        output_header.stamp = msg.header.stamp
        output_header.frame_id = self.get_parameter('output_frame_id').value
        cloud = self.create_cloud(output_header, detections)
        self.pub.publish(cloud)

    def accept_persistent_clutter(self, coarse_idx, candidate):
        """Reject random sea peaks while retaining a deliberately persistent return."""
        previous = self._clutter_history.get(coarse_idx)
        range_gate = float(self.get_parameter('clutter_range_gate_m').value)
        velocity_gate = float(self.get_parameter('clutter_velocity_gate_mps').value)
        if previous is not None and \
                abs(previous['range'] - candidate['range']) <= range_gate and \
                abs(previous['radial_velocity'] - candidate['radial_velocity']) <= velocity_gate:
            hits = previous['hits'] + 1
        else:
            hits = 1
        self._clutter_history[coarse_idx] = {
            'range': candidate['range'],
            'radial_velocity': candidate['radial_velocity'],
            'hits': hits,
        }
        return hits >= max(1, int(self.get_parameter('clutter_confirmation_frames').value))

    def build_group_target_candidate(self, group):
        hit_subbeams = []
        group_size = len(group)

        for idx, sb in enumerate(group):
            if not sb['hit_valid']:
                continue

            angle = sb['angle']
            rng = sb['range']
            power = self.compute_target_power(rng, angle)
            if power <= 0.0:
                continue

            direction_weight = self.compute_direction_weight(idx, group_size)
            fused_weight = power * direction_weight

            hit_subbeams.append({
                'angle': angle,
                'range': rng,
                'power': power,
                'direction_weight': direction_weight,
                'fused_weight': fused_weight,
            })

        if not hit_subbeams:
            return None

        hit_count = len(hit_subbeams)
        target_type = 1.0 if hit_count == 1 else 2.0   # 1=point, 2=extended

        total_power = sum(h['power'] for h in hit_subbeams)
        total_power = max(total_power, 1e-6)

        total_fused_weight = sum(h['fused_weight'] for h in hit_subbeams)
        total_fused_weight = max(total_fused_weight, 1e-6)

        weighted_angle = sum(h['angle'] * h['fused_weight'] for h in hit_subbeams) / total_fused_weight
        weighted_range = sum(h['range'] * h['fused_weight'] for h in hit_subbeams) / total_fused_weight

        min_angle = min(h['angle'] for h in hit_subbeams)
        max_angle = max(h['angle'] for h in hit_subbeams)
        span_deg = math.degrees(max_angle - min_angle) if hit_count > 1 else 0.0

        if hit_count == 1:
            noise_deg = float(self.get_parameter('point_target_angle_noise_deg').value)
        else:
            noise_deg = float(self.get_parameter('extended_target_angle_noise_deg').value)

        measured_angle = weighted_angle + math.radians(random.gauss(0.0, noise_deg))

        reflectivity = float(self.get_parameter('target_material_reflectivity').value)
        base_rcs = float(self.get_parameter('target_base_rcs').value)
        velocity_stddev = float(self.get_parameter('radial_velocity_noise_stddev').value)

        snr = self.power_to_snr(total_power)
        rcs = max(0.05, reflectivity * base_rcs * hit_count)

        return {
            'source': 1.0,
            'range': weighted_range,
            'azimuth': measured_angle,
            'elevation': 0.0,
            'radial_velocity': random.gauss(0.0, velocity_stddev),
            'rcs': rcs,
            'snr': snr,
            'power': total_power,
            'z': 0.0,
            'score': total_power,
            'target_type': target_type,
            'beam_count': float(hit_count),
            'angular_extent_deg': span_deg,
            'max_direction_weight': max(h['direction_weight'] for h in hit_subbeams),
            'weighted_beam_score': total_fused_weight,
        }

    def build_group_clutter_candidate(self, group, min_range, max_range):
        clutter_candidates = []
        for sb in group:
            clutter = self.build_sea_clutter_candidate(sb['angle'], min_range, max_range)
            if clutter is not None:
                clutter_candidates.append(clutter)

        if not clutter_candidates:
            return None

        # 第一版先取组内最强海杂波
        best = max(clutter_candidates, key=lambda c: c['score'])
        best['target_type'] = 0.0
        best['beam_count'] = 0.0
        best['angular_extent_deg'] = 0.0
        best['max_direction_weight'] = 0.0
        best['weighted_beam_score'] = best['score']
        return best

    def compute_target_power(self, rng, angle):
        reflectivity = float(self.get_parameter('target_material_reflectivity').value)
        base_rcs = float(self.get_parameter('target_base_rcs').value)
        random_stddev = float(self.get_parameter('target_random_stddev').value)

        power = base_rcs * reflectivity * self.range_decay(rng) * self.beam_gain(angle)
        power += random.gauss(0.0, random_stddev)
        return max(0.0, power)

    def compute_direction_weight(self, index_in_group, group_size):
        enabled = bool(self.get_parameter('center_weight_enabled').value)
        if not enabled or group_size <= 1:
            return 1.0

        sigma = float(self.get_parameter('center_weight_sigma').value)
        min_weight = float(self.get_parameter('center_weight_min').value)

        center = 0.5 * (group_size - 1)
        distance = index_in_group - center

        gaussian = math.exp(-(distance * distance) / (2.0 * sigma * sigma))
        return min_weight + (1.0 - min_weight) * gaussian

    def build_sea_clutter_candidate(self, bin_center, min_range, max_range):
        if not bool(self.get_parameter('sea_clutter_enabled').value):
            return None

        radar_height = float(self.get_parameter('radar_height_m').value)
        sea_state = float(self.get_parameter('sea_state').value)
        clutter_probability = float(self.get_parameter('sea_clutter_probability_per_bin').value)
        if random.random() > max(0.0, min(1.0, clutter_probability)):
            return None

        clutter_min = float(self.get_parameter('sea_clutter_range_min').value)
        clutter_max = float(self.get_parameter('sea_clutter_range_max').value)
        clutter_height_scale = float(self.get_parameter('sea_clutter_height_scale').value)
        clutter_height_max = float(self.get_parameter('sea_clutter_height_max').value)
        compete_min_height = float(self.get_parameter('sea_clutter_compete_min_height').value)
        clutter_random_stddev = float(self.get_parameter('sea_clutter_random_stddev').value)
        near_field_radius = float(self.get_parameter('sea_clutter_near_field_radius_m').value)
        near_field_attenuation = float(
            self.get_parameter('sea_clutter_near_field_attenuation').value)

        low = max(min_range, clutter_min)
        high = min(max_range, clutter_max)
        if high <= low:
            return None

        rng = random.uniform(low, high)

        u = max(1e-6, random.random())
        wave_height = min(
            clutter_height_scale * math.sqrt(-2.0 * math.log(u)),
            clutter_height_max,
        )

        qualifies = wave_height >= compete_min_height or wave_height >= radar_height
        if not qualifies:
            return None

        base = wave_height * (0.7 + 0.6 * sea_state)
        power = base * self.range_decay(rng) * random.uniform(0.85, 1.15)
        if near_field_radius > low and rng < near_field_radius:
            # Blend smoothly from the attenuated wake region to normal sea clutter
            # so a hard range boundary cannot create an artificial ring.
            ratio = (rng - low) / max(near_field_radius - low, 1e-3)
            attenuation = near_field_attenuation + (1.0 - near_field_attenuation) * ratio
            power *= max(0.0, min(1.0, attenuation))
        power += random.gauss(0.0, clutter_random_stddev)
        power = max(0.0, power)

        snr = self.power_to_snr(power)
        return {
            'source': 2.0,
            'range': rng,
            'azimuth': bin_center,
            'elevation': math.atan2(wave_height, max(rng, 1e-3)),
            'radial_velocity': random.gauss(0.0, 0.12 * max(sea_state, 0.1)),
            'rcs': max(0.05, wave_height * (0.6 + sea_state)),
            'snr': snr,
            'power': power,
            'z': wave_height,
            'score': power,
        }

    def select_candidate(self, target_candidate, clutter_candidate):
        if target_candidate is None and clutter_candidate is None:
            return None
        if target_candidate is None:
            return clutter_candidate
        if clutter_candidate is None:
            return target_candidate
        return target_candidate if target_candidate['score'] >= clutter_candidate['score'] else clutter_candidate

    def range_decay(self, rng):
        alpha = float(self.get_parameter('range_attenuation_alpha').value)
        return 1.0 / (1.0 + alpha * rng * rng)

    def beam_gain(self, angle_rad):
        return max(0.35, math.cos(angle_rad))

    def power_to_snr(self, power):
        return max(0.0, 10.0 * math.log10(1.0 + max(power, 0.0) * 25.0))

    def candidate_to_point(self, candidate, timestamp):
        rng = candidate['range']
        azimuth = candidate['azimuth']
        elevation = candidate['elevation']

        horizontal_range = rng * math.cos(elevation)
        local_x = horizontal_range * math.cos(azimuth)
        local_y = horizontal_range * math.sin(azimuth)
        mount_yaw = float(self.get_parameter('mount_yaw_rad').value)
        cos_yaw = math.cos(mount_yaw)
        sin_yaw = math.sin(mount_yaw)
        x = cos_yaw * local_x - sin_yaw * local_y
        y = sin_yaw * local_x + cos_yaw * local_y
        z = candidate['z']
        azimuth_world = math.atan2(y, x)

        return (
            x,
            y,
            z,
            candidate['power'],
            candidate['radial_velocity'],
            candidate['snr'],
            candidate['rcs'],
            candidate['range'],
            math.degrees(azimuth_world),
            timestamp,
            candidate['source'],
            candidate.get('target_type', 0.0),
            candidate.get('beam_count', 0.0),
            candidate.get('angular_extent_deg', 0.0),
        )

    def create_cloud(self, header: Header, points):
        fields = [
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
        ]

        point_step = 56
        data = bytearray()

        for p in points:
            data.extend(struct.pack('ffffffffffffff', *p))

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


def main(args=None):
    rclpy.init(args=args)
    node = MmwaveScanConverter()
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
