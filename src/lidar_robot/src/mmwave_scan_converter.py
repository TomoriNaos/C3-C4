#!/usr/bin/env python3
import math
import random
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header


class MmwaveScanConverter(Node):
    def __init__(self):
        super().__init__('mmwave_scan_converter')

        # 海杂波 range-angle 场：angle_bin x range_bin
        # 每个 cell 保存 intensity / height / velocity / phase 等状态。
        self.clutter_field = []
        self.clutter_geometry = None
        self.last_scan_time = None

        self.declare_parameter('input_topic', '/radar_2p5m/raw_scan')
        self.declare_parameter('output_topic', '/mmwave/detections')
        # Output frame of converted detections.
        # For multi-radar fusion, use base_link so all radars share one coordinate system.
        self.declare_parameter('frame_id', 'base_link')
        # Radar mounting pose in base_link.
        # The raw LaserScan is in each radar link frame; these parameters rotate/translate
        # detections into base_link before publishing.
        self.declare_parameter('radar_x_in_base', 0.0)
        self.declare_parameter('radar_y_in_base', 0.0)
        self.declare_parameter('radar_z_in_base', 0.0)
        self.declare_parameter('radar_yaw_in_base_deg', 0.0)
        # Numeric id used in PointCloud2 field source_id.
        self.declare_parameter('source_id', 0.0)

        # 粗检测层参数
        self.declare_parameter('max_range', 800.0)
        self.declare_parameter('min_range', 2.0)
        self.declare_parameter('horizontal_fov_deg', 120.0)
        self.declare_parameter('angular_resolution_deg', 1.5)   # 最终检测单元宽度

        # 细采样层参数
        self.declare_parameter('subbeam_resolution_deg', 0.15)   # Gazebo 原始扫描分辨率

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

        # 海杂波建模参数：保留原有参数名，保证 launch 兼容
        self.declare_parameter('sea_clutter_enabled', True)
        self.declare_parameter('sea_state', 0.65)
        self.declare_parameter('sea_clutter_range_min', 5.0)
        self.declare_parameter('sea_clutter_range_max', 800.0)
        self.declare_parameter('sea_clutter_height_scale', 0.55)
        self.declare_parameter('sea_clutter_height_max', 2.0)
        self.declare_parameter('sea_clutter_compete_min_height', 0.30)
        self.declare_parameter('sea_clutter_random_stddev', 0.06)

        # 新增参数：不影响已有 launch；如果 launch 不传，就使用这些默认值
        self.declare_parameter('sea_clutter_range_bin_m', 25.0)
        self.declare_parameter('sea_clutter_temporal_correlation', 0.92)
        self.declare_parameter('sea_clutter_spatial_correlation', 0.35)
        self.declare_parameter('sea_clutter_density', 0.22)
        self.declare_parameter('sea_clutter_weibull_shape', 1.35)
        self.declare_parameter('sea_clutter_weibull_scale', 1.0)
        self.declare_parameter('sea_clutter_velocity_bias', 0.0)
        self.declare_parameter('sea_clutter_patch_velocity_stddev', 0.08)
        self.declare_parameter('target_clutter_snr_k', 2.0)
        self.declare_parameter('target_min_detection_probability', 0.35)

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

        # 1.5° / 0.3° = 5 个子 beam 一组
        subbeams_per_bin = max(1, int(round(angular_resolution_deg / subbeam_resolution_deg)))
        coarse_bin_count = max(1, int(round(horizontal_fov_deg / angular_resolution_deg)))

        # 先收集落在 FOV 内的原始子 beam
        subbeams = []
        for i, rng in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if angle < -fov_half_rad or angle > fov_half_rad:
                continue

            hit_valid = math.isfinite(rng) and (min_range <= rng <= max_range)
            subbeams.append({
                'angle': angle,
                'range': rng if hit_valid else None,
                'hit_valid': hit_valid,
            })

        expected_subbeams = subbeams_per_bin * coarse_bin_count
        if len(subbeams) < expected_subbeams:
            self.get_logger().warn(
                f'Collected subbeams ({len(subbeams)}) fewer than expected ({expected_subbeams})'
            )

        detections = []
        frame_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 每帧更新一次具有时间/空间相关性的海杂波场
        self.update_clutter_field(coarse_bin_count, min_range, max_range, frame_timestamp)

        # 按 1.5° 粗角单元分组，每组融合 5 个 0.3° 子 beam
        for coarse_idx in range(coarse_bin_count):
            start = coarse_idx * subbeams_per_bin
            end = start + subbeams_per_bin
            group = subbeams[start:end]
            if not group:
                continue

            target_candidate = self.build_group_target_candidate(group)
            clutter_candidates = self.build_group_clutter_candidates(coarse_idx, group, min_range, max_range)

            # 目标不再被 clutter 直接替换；而是根据局部 clutter 水平降低检测概率/SNR
            if target_candidate is not None:
                target_candidate = self.apply_clutter_limited_target_detection(target_candidate)
                if target_candidate is not None and target_candidate['snr'] >= intensity_threshold:
                    detections.append(self.candidate_to_point(target_candidate, frame_timestamp))

            # clutter 可以和目标共存，输出为 source_id=2 的点
            for clutter_candidate in clutter_candidates:
                if clutter_candidate['snr'] >= intensity_threshold:
                    detections.append(self.candidate_to_point(clutter_candidate, frame_timestamp))

        detections.sort(key=lambda p: p[5], reverse=True)
        detections = detections[:max_detections]

        output_header = Header()
        output_header.stamp = msg.header.stamp
        output_header.frame_id = str(self.get_parameter('frame_id').value)
        cloud = self.create_cloud(output_header, detections)
        self.pub.publish(cloud)


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

    def update_clutter_field(self, angle_bin_count, min_range, max_range, timestamp):
        if not bool(self.get_parameter('sea_clutter_enabled').value):
            self.clutter_field = []
            self.clutter_geometry = None
            self.last_scan_time = timestamp
            return

        clutter_min = float(self.get_parameter('sea_clutter_range_min').value)
        clutter_max = float(self.get_parameter('sea_clutter_range_max').value)
        range_bin_m = max(1.0, float(self.get_parameter('sea_clutter_range_bin_m').value))

        low = max(min_range, clutter_min)
        high = min(max_range, clutter_max)
        if high <= low:
            self.clutter_field = []
            self.clutter_geometry = None
            self.last_scan_time = timestamp
            return

        range_bin_count = max(1, int(math.ceil((high - low) / range_bin_m)))
        geometry = (angle_bin_count, range_bin_count, round(low, 3), round(high, 3), round(range_bin_m, 3))

        if self.clutter_geometry != geometry or not self.clutter_field:
            self.clutter_geometry = geometry
            self.clutter_field = self.initialize_clutter_field(angle_bin_count, range_bin_count, low, range_bin_m)
            self.last_scan_time = timestamp
            return

        dt = 0.1
        if self.last_scan_time is not None:
            dt = max(0.0, min(1.0, timestamp - self.last_scan_time))
        self.last_scan_time = timestamp

        temporal_corr = self.clamp(
            float(self.get_parameter('sea_clutter_temporal_correlation').value),
            0.0,
            0.999,
        )
        spatial_corr = self.clamp(
            float(self.get_parameter('sea_clutter_spatial_correlation').value),
            0.0,
            0.95,
        )

        # dt 越大，相关性稍微降低；dt 正常情况下约为仿真扫描周期。
        effective_corr = temporal_corr ** max(dt / 0.1, 0.1)

        previous = self.clutter_field
        random_field = self.initialize_clutter_field(angle_bin_count, range_bin_count, low, range_bin_m)
        blended = []

        for a in range(angle_bin_count):
            row = []
            for r in range(range_bin_count):
                old = previous[a][r]
                new = random_field[a][r]
                row.append({
                    'range': old['range'],
                    'intensity': effective_corr * old['intensity'] + (1.0 - effective_corr) * new['intensity'],
                    'height': effective_corr * old['height'] + (1.0 - effective_corr) * new['height'],
                    'velocity': effective_corr * old['velocity'] + (1.0 - effective_corr) * new['velocity'],
                })
            blended.append(row)

        self.clutter_field = self.smooth_clutter_field(blended, spatial_corr)

    def initialize_clutter_field(self, angle_bin_count, range_bin_count, low, range_bin_m):
        sea_state = self.clamp(float(self.get_parameter('sea_state').value), 0.0, 1.0)
        density = self.clamp(float(self.get_parameter('sea_clutter_density').value), 0.0, 1.0)
        height_scale = float(self.get_parameter('sea_clutter_height_scale').value)
        height_max = float(self.get_parameter('sea_clutter_height_max').value)
        velocity_bias = float(self.get_parameter('sea_clutter_velocity_bias').value)
        velocity_stddev = float(self.get_parameter('sea_clutter_patch_velocity_stddev').value)

        field = []
        for a in range(angle_bin_count):
            row = []
            for r in range(range_bin_count):
                rng = low + (r + 0.5) * range_bin_m

                # 近距离海杂波密度更高，远距离逐渐稀疏
                range_factor = self.clamp(1.0 / (1.0 + 0.0015 * rng), 0.08, 1.0)
                probability = self.clamp(density * (0.35 + 1.25 * sea_state) * range_factor, 0.0, 0.95)

                if random.random() < probability:
                    intensity = self.sample_sea_clutter_intensity(rng)
                    u = max(1e-6, random.random())
                    height = min(height_scale * math.sqrt(-2.0 * math.log(u)) * (0.7 + sea_state), height_max)
                    velocity = random.gauss(velocity_bias, velocity_stddev * max(sea_state, 0.1))
                else:
                    intensity = 0.0
                    height = 0.0
                    velocity = 0.0

                row.append({
                    'range': rng,
                    'intensity': intensity,
                    'height': height,
                    'velocity': velocity,
                })
            field.append(row)

        return field

    def smooth_clutter_field(self, field, spatial_corr):
        if spatial_corr <= 1e-6:
            return field
        angle_count = len(field)
        range_count = len(field[0]) if angle_count > 0 else 0
        if angle_count == 0 or range_count == 0:
            return field

        smoothed = []
        for a in range(angle_count):
            row = []
            for r in range(range_count):
                center = field[a][r]
                neighbors = []
                for da, dr in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    aa = a + da
                    rr = r + dr
                    if 0 <= aa < angle_count and 0 <= rr < range_count:
                        neighbors.append(field[aa][rr])

                if neighbors:
                    inv_n = 1.0 / len(neighbors)
                    mean_intensity = sum(n['intensity'] for n in neighbors) * inv_n
                    mean_height = sum(n['height'] for n in neighbors) * inv_n
                    mean_velocity = sum(n['velocity'] for n in neighbors) * inv_n
                else:
                    mean_intensity = center['intensity']
                    mean_height = center['height']
                    mean_velocity = center['velocity']

                row.append({
                    'range': center['range'],
                    'intensity': (1.0 - spatial_corr) * center['intensity'] + spatial_corr * mean_intensity,
                    'height': (1.0 - spatial_corr) * center['height'] + spatial_corr * mean_height,
                    'velocity': (1.0 - spatial_corr) * center['velocity'] + spatial_corr * mean_velocity,
                })
            smoothed.append(row)

        return smoothed

    def sample_sea_clutter_intensity(self, rng):
        sea_state = self.clamp(float(self.get_parameter('sea_state').value), 0.0, 1.0)
        shape = max(0.25, float(self.get_parameter('sea_clutter_weibull_shape').value))
        scale = max(1e-3, float(self.get_parameter('sea_clutter_weibull_scale').value))
        random_stddev = float(self.get_parameter('sea_clutter_random_stddev').value)

        # Weibull 重尾：多数弱回波，少数强尖峰
        u = self.clamp(random.random(), 1e-6, 1.0 - 1e-6)
        weibull = scale * (-math.log(1.0 - u)) ** (1.0 / shape)

        # grazing angle：雷达越高、距离越近，近距离海面回波更明显
        radar_height = float(self.get_parameter('radar_height_m').value)
        grazing = math.atan2(radar_height, max(rng, 1e-3))
        grazing_factor = self.clamp(math.sin(grazing) * 16.0, 0.08, 1.0)

        base = weibull * (0.35 + 1.25 * sea_state) * grazing_factor
        power = base * self.range_decay(rng)
        power += random.gauss(0.0, random_stddev)
        return max(0.0, power)

    def build_group_clutter_candidates(self, coarse_idx, group, min_range, max_range):
        if not bool(self.get_parameter('sea_clutter_enabled').value):
            return []
        if not self.clutter_field or coarse_idx >= len(self.clutter_field):
            return []
        if not group:
            return []

        angle = sum(sb['angle'] for sb in group) / len(group)
        candidates = []
        cells = self.clutter_field[coarse_idx]

        for range_idx, cell in enumerate(cells):
            clutter = self.build_sea_clutter_candidate_from_cell(
                angle,
                range_idx,
                cell,
                min_range,
                max_range,
            )
            if clutter is not None:
                candidates.append(clutter)

        if not candidates:
            return []

        # 一个角度 bin 内可能有多个距离杂波，但为了不让输出爆炸，保留最强的 1 个。
        # 如果你希望更强海杂波场景，可改成 candidates[:2] 或 candidates[:3]。
        candidates.sort(key=lambda c: c['score'], reverse=True)
        return candidates[:1]

    def build_sea_clutter_candidate_from_cell(self, angle, range_idx, cell, min_range, max_range):
        sea_state = self.clamp(float(self.get_parameter('sea_state').value), 0.0, 1.0)
        compete_min_height = float(self.get_parameter('sea_clutter_compete_min_height').value)
        height_max = float(self.get_parameter('sea_clutter_height_max').value)
        range_bin_m = max(1.0, float(self.get_parameter('sea_clutter_range_bin_m').value))

        rng = cell['range'] + random.uniform(-0.35, 0.35) * range_bin_m
        if rng < min_range or rng > max_range:
            return None

        intensity = max(0.0, cell['intensity'])
        height = self.clamp(cell['height'], 0.0, height_max)
        if intensity <= 1e-6:
            return None

        # 海杂波不是每个 cell 都可见：弱斑块随机消失，强斑块更容易出现
        visibility = self.clamp(0.15 + 6.0 * intensity + 0.45 * sea_state, 0.0, 0.98)
        if random.random() > visibility:
            return None

        # 保留原来的“浪高竞争”语义，但不再要求特别死板
        if height < compete_min_height and random.random() > 0.25 * sea_state:
            return None

        azimuth = angle + random.gauss(0.0, math.radians(0.12 + 0.12 * sea_state))
        elevation = math.atan2(height, max(rng, 1e-3))

        # 轻微距离相位调制，制造海面斑块/条带感
        phase_mod = 0.85 + 0.15 * math.sin(0.17 * range_idx + 3.0 * sea_state)
        power = max(0.0, intensity * phase_mod)
        snr = self.power_to_snr(power)

        return {
            'source': 2.0,
            'range': rng,
            'azimuth': azimuth,
            'elevation': elevation,
            'radial_velocity': cell['velocity'] + random.gauss(0.0, 0.04 + 0.05 * sea_state),
            'rcs': max(0.05, height * (0.6 + sea_state) + power),
            'snr': snr,
            'power': power,
            'z': height,
            'score': power,
            'target_type': 0.0,
            'beam_count': 0.0,
            'angular_extent_deg': 0.0,
            'max_direction_weight': 0.0,
            'weighted_beam_score': power,
        }

    def apply_clutter_limited_target_detection(self, target):
        local_clutter_power = self.estimate_local_clutter_power(target['azimuth'], target['range'])

        # clutter 不直接替换目标，而是降低等效 SNR 和检测概率
        target_power = max(target['power'], 1e-6)
        clutter_power = max(local_clutter_power, 0.0)
        effective_power = target_power / (1.0 + 2.5 * clutter_power)

        k = max(0.1, float(self.get_parameter('target_clutter_snr_k').value))
        min_prob = self.clamp(float(self.get_parameter('target_min_detection_probability').value), 0.0, 1.0)
        ratio = target_power / (clutter_power + 1e-6)

        # ratio 越大越容易检测到；强 clutter 中目标会概率性漏检
        detection_probability = min_prob + (1.0 - min_prob) * (1.0 - math.exp(-ratio / k))
        detection_probability = self.clamp(detection_probability, min_prob, 1.0)

        if random.random() > detection_probability:
            return None

        target['power'] = max(1e-6, effective_power)
        target['snr'] = self.power_to_snr(target['power'])
        target['score'] = target['power']
        return target

    def estimate_local_clutter_power(self, azimuth, rng):
        if not self.clutter_field or self.clutter_geometry is None:
            return 0.0

        angle_count, range_count, low, high, range_bin_m = self.clutter_geometry
        horizontal_fov_deg = float(self.get_parameter('horizontal_fov_deg').value)
        angular_resolution_deg = float(self.get_parameter('angular_resolution_deg').value)
        fov_half_rad = math.radians(horizontal_fov_deg * 0.5)

        angle_idx = int(round((azimuth + fov_half_rad) / math.radians(angular_resolution_deg)))
        range_idx = int(round((rng - low) / max(range_bin_m, 1e-3)))

        powers = []
        for da in [-1, 0, 1]:
            for dr in [-1, 0, 1]:
                aa = angle_idx + da
                rr = range_idx + dr
                if 0 <= aa < angle_count and 0 <= rr < range_count:
                    powers.append(self.clutter_field[aa][rr]['intensity'])

        if not powers:
            return 0.0
        return sum(powers) / len(powers)

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

    def range_decay(self, rng):
        alpha = float(self.get_parameter('range_attenuation_alpha').value)
        return 1.0 / (1.0 + alpha * rng * rng)

    def beam_gain(self, angle_rad):
        return max(0.35, math.cos(angle_rad))

    def power_to_snr(self, power):
        return max(0.0, 10.0 * math.log10(1.0 + max(power, 0.0) * 25.0))

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def candidate_to_point(self, candidate, timestamp):
        rng = candidate['range']
        azimuth = candidate['azimuth']
        elevation = candidate['elevation']

        # Point in radar local frame.
        horizontal_range = rng * math.cos(elevation)
        x_radar = horizontal_range * math.cos(azimuth)
        y_radar = horizontal_range * math.sin(azimuth)
        z_radar = candidate['z']

        # Transform radar local frame -> base_link.
        radar_x = float(self.get_parameter('radar_x_in_base').value)
        radar_y = float(self.get_parameter('radar_y_in_base').value)
        radar_z = float(self.get_parameter('radar_z_in_base').value)
        radar_yaw = math.radians(float(self.get_parameter('radar_yaw_in_base_deg').value))

        cos_yaw = math.cos(radar_yaw)
        sin_yaw = math.sin(radar_yaw)

        x = radar_x + cos_yaw * x_radar - sin_yaw * y_radar
        y = radar_y + sin_yaw * x_radar + cos_yaw * y_radar
        z = radar_z + z_radar

        # Azimuth in base_link, not local radar frame.
        azimuth_in_base = math.atan2(y, x)

        return (
            x,
            y,
            z,
            candidate['power'],
            candidate['radial_velocity'],
            candidate['snr'],
            candidate['rcs'],
            candidate['range'],
            math.degrees(azimuth_in_base),
            timestamp,
            float(self.get_parameter('source_id').value),
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
