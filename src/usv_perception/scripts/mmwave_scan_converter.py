#!/usr/bin/env python3
import math
import random
import struct

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


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
        self.declare_parameter('min_range', 1.0)
        self.declare_parameter('horizontal_fov_deg', 120.0)
        self.declare_parameter('angular_resolution_deg', 1.5)   # 最终检测单元宽度

        # 细采样层参数
        self.declare_parameter('subbeam_resolution_deg', 0.3)   # Gazebo 原始扫描分辨率

        self.declare_parameter('radar_height_m', 1.86)

        # 真实目标建模参数
        self.declare_parameter('intensity_threshold', 0.003)
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
        self.declare_parameter('clutter_confirmation_frames', 2)
        self.declare_parameter('clutter_range_gate_m', 1.5)
        self.declare_parameter('clutter_velocity_gate_mps', 0.30)
        self._clutter_history = {}

        # 输出限制
        self.declare_parameter('max_detections', 80)

        # Enhanced physical model parameters. These keep the PointCloud2 layout unchanged.
        self.declare_parameter('range_resolution_m', 0.35)
        self.declare_parameter('doppler_resolution_mps', 0.08)
        self.declare_parameter('thermal_noise_power', 0.012)
        self.declare_parameter('range_noise_floor_m', 0.08)
        self.declare_parameter('range_noise_snr_scale_m', 0.75)
        self.declare_parameter('angle_noise_snr_scale_deg', 0.75)
        self.declare_parameter('swerling_fluctuation_shape', 1.4)
        self.declare_parameter('multipath_probability', 0.06)
        self.declare_parameter('multipath_range_bias_m', 1.2)
        self.declare_parameter('side_lobe_false_alarm_rate', 0.015)

        # Optional Gazebo truth Doppler. The value is still published through the
        # existing radial_velocity field as a noisy, quantized radar measurement.
        self.declare_parameter('use_ground_truth_doppler', True)
        self.declare_parameter('model_states_topic', '/model_states')
        self.declare_parameter('usv_model_name', 'wamv')
        self.declare_parameter('doppler_match_range_gate_m', 6.0)
        self.declare_parameter('doppler_match_angle_gate_deg', 7.5)
        self.declare_parameter('doppler_target_names', [
            'moving_vessel', 'small_fishing_boat', 'fishnet_buoy', 'floating_obstacle',
            'drift_debris', 'survey_boat', 'service_boat', 'floating_container',
            'research_platform', 'ship_far', 'obstacle',
        ])
        self.model_states = None

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.model_states_sub = self.create_subscription(
            ModelStates,
            self.get_parameter('model_states_topic').value,
            self.model_states_callback,
            10
        )
        self.pub = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)

        self.get_logger().info(f'mmWave converter subscribed to {input_topic}')
        self.get_logger().info(f'mmWave detections publishing to {output_topic}')

    def scan_callback(self, msg: LaserScan):
        min_range = float(self.get_parameter('min_range').value)
        max_range = float(self.get_parameter('max_range').value)
        horizontal_fov_deg = float(self.get_parameter('horizontal_fov_deg').value)
        angular_resolution_deg = float(self.get_parameter('angular_resolution_deg').value)
        intensity_threshold = float(self.get_parameter('intensity_threshold').value)
        max_detections = int(self.get_parameter('max_detections').value)

        fov_half_rad = math.radians(horizontal_fov_deg * 0.5)
        fov_epsilon = 0.5 * abs(msg.angle_increment)

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

        actual_subbeam_resolution_deg = math.degrees(abs(msg.angle_increment)) if msg.angle_increment else \
            float(self.get_parameter('subbeam_resolution_deg').value)
        subbeams_per_bin = max(1, int(round(angular_resolution_deg / max(actual_subbeam_resolution_deg, 1e-6))))
        coarse_bin_count = max(1, math.ceil(len(subbeams) / subbeams_per_bin))

        expected_subbeams = max(1, int(round(horizontal_fov_deg / max(actual_subbeam_resolution_deg, 1e-6))))
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
            false_alarm_candidate = self.build_side_lobe_false_alarm(group, min_range, max_range)

            for winner in self.rank_candidates(target_candidate, clutter_candidate, false_alarm_candidate):
                if winner['snr'] < intensity_threshold:
                    continue
                if winner['source'] == 2.0 and not self.accept_persistent_clutter(coarse_idx, winner):
                    continue
                detections.append(self.candidate_to_point(winner, frame_timestamp))
                break

        detections.sort(key=lambda p: p[5], reverse=True)
        detections = detections[:max_detections]

        output_header = Header()
        output_header.stamp = msg.header.stamp
        output_header.frame_id = self.get_parameter('output_frame_id').value
        cloud = self.create_cloud(output_header, detections)
        self.pub.publish(cloud)

    def model_states_callback(self, msg):
        self.model_states = msg

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

        total_power = max(sum(h['power'] for h in hit_subbeams), 1e-9)
        total_fused_weight = max(sum(h['fused_weight'] for h in hit_subbeams), 1e-9)

        weighted_angle = sum(h['angle'] * h['fused_weight'] for h in hit_subbeams) / total_fused_weight
        weighted_range = sum(h['range'] * h['fused_weight'] for h in hit_subbeams) / total_fused_weight

        min_angle = min(h['angle'] for h in hit_subbeams)
        max_angle = max(h['angle'] for h in hit_subbeams)
        span_deg = math.degrees(max_angle - min_angle) if hit_count > 1 else 0.0

        if hit_count == 1:
            noise_deg = float(self.get_parameter('point_target_angle_noise_deg').value)
        else:
            noise_deg = float(self.get_parameter('extended_target_angle_noise_deg').value)

        snr = self.power_to_snr(total_power)
        measured_range = self.apply_range_measurement_error(weighted_range, snr)
        measured_angle = self.apply_angle_measurement_error(weighted_angle, snr, noise_deg)

        reflectivity = float(self.get_parameter('target_material_reflectivity').value)
        base_rcs = float(self.get_parameter('target_base_rcs').value)
        radial_velocity = self.measure_radial_velocity(weighted_range, weighted_angle, snr)

        if random.random() < float(self.get_parameter('multipath_probability').value):
            measured_range += abs(random.gauss(float(self.get_parameter('multipath_range_bias_m').value), 0.6))
            snr *= 0.75

        rcs = max(0.03, reflectivity * base_rcs * hit_count)

        return {
            'source': 1.0,
            'range': measured_range,
            'azimuth': measured_angle,
            'elevation': 0.0,
            'radial_velocity': radial_velocity,
            'rcs': rcs,
            'snr': snr,
            'power': total_power,
            'z': 0.0,
            'score': snr,
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
        shape = max(0.2, float(self.get_parameter('swerling_fluctuation_shape').value))

        rcs_fluctuation = random.gammavariate(shape, 1.0 / shape)
        radar_equation = base_rcs * reflectivity * rcs_fluctuation / max(rng ** 4, 1.0)
        power = radar_equation * 1.5e8
        power *= self.beam_gain(angle)
        power *= math.exp(-float(self.get_parameter('range_attenuation_alpha').value) * rng * 0.02)
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

        sea_state = clamp(float(self.get_parameter('sea_state').value), 0.0, 1.0)
        clutter_probability = clamp(float(self.get_parameter('sea_clutter_probability_per_bin').value), 0.0, 1.0)
        if random.random() > clutter_probability * (0.35 + 0.85 * sea_state):
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

        # Sea clutter tends to be range-cell dominated rather than spatially uniform.
        rng = low + (high - low) * (random.random() ** 1.8)

        wave_height = min(
            random.weibullvariate(clutter_height_scale * (0.7 + sea_state), 1.7),
            clutter_height_max,
        )
        if wave_height < compete_min_height:
            return None

        grazing = math.atan2(float(self.get_parameter('radar_height_m').value), max(rng, 1e-3))
        grazing_gain = clamp(math.sin(grazing) * 14.0, 0.08, 1.0)
        speckle = random.gammavariate(0.9, 1.0)
        power = wave_height * (0.45 + 1.2 * sea_state) * grazing_gain * speckle
        power *= self.range_decay(rng) * self.beam_gain(bin_center)
        if near_field_radius > low and rng < near_field_radius:
            # Blend smoothly from the attenuated wake region to normal sea clutter
            # so a hard range boundary cannot create an artificial ring.
            ratio = (rng - low) / max(near_field_radius - low, 1e-3)
            attenuation = near_field_attenuation + (1.0 - near_field_attenuation) * ratio
            power *= max(0.0, min(1.0, attenuation))
        power += random.gauss(0.0, clutter_random_stddev)
        power = max(0.0, power)

        snr = self.power_to_snr(power)
        velocity = random.gauss(0.0, 0.08 + 0.28 * sea_state)
        return {
            'source': 2.0,
            'range': self.apply_range_measurement_error(rng, snr),
            'azimuth': self.apply_angle_measurement_error(bin_center, snr, 0.35),
            'elevation': math.atan2(wave_height, max(rng, 1e-3)),
            'radial_velocity': self.quantize(velocity, float(self.get_parameter('doppler_resolution_mps').value)),
            'rcs': max(0.03, wave_height * (0.4 + sea_state)),
            'snr': snr,
            'power': power,
            'z': wave_height,
            'score': snr * 0.85,
        }

    def build_side_lobe_false_alarm(self, group, min_range, max_range):
        if not group or random.random() > float(self.get_parameter('side_lobe_false_alarm_rate').value):
            return None
        rng = random.uniform(min_range, max_range)
        angle = random.choice(group)['angle'] + random.gauss(0.0, math.radians(0.9))
        power = random.expovariate(1.0 / 0.08)
        snr = self.power_to_snr(power)
        return {
            'source': 2.0,
            'range': rng,
            'azimuth': angle,
            'elevation': 0.0,
            'radial_velocity': random.gauss(0.0, 0.35),
            'rcs': 0.05,
            'snr': snr,
            'power': power,
            'z': 0.0,
            'score': snr * 0.55,
            'target_type': 0.0,
            'beam_count': 0.0,
            'angular_extent_deg': 0.0,
            'max_direction_weight': 0.0,
            'weighted_beam_score': 0.0,
        }

    def rank_candidates(self, *candidates):
        valid = [candidate for candidate in candidates if candidate is not None]
        return sorted(valid, key=lambda c: c['score'], reverse=True)

    def range_decay(self, rng):
        alpha = float(self.get_parameter('range_attenuation_alpha').value)
        return 1.0 / (1.0 + alpha * rng * rng)

    def beam_gain(self, angle_rad):
        half_power = math.radians(float(self.get_parameter('horizontal_fov_deg').value) * 0.5)
        x = abs(angle_rad) / max(half_power, 1e-6)
        side_floor = 0.04
        return clamp(math.exp(-2.8 * x * x) + side_floor, side_floor, 1.0)

    def power_to_snr(self, power):
        noise = max(1e-6, float(self.get_parameter('thermal_noise_power').value))
        return max(0.0, 10.0 * math.log10(1.0 + max(power, 0.0) / noise))

    def apply_range_measurement_error(self, rng, snr):
        std = float(self.get_parameter('range_noise_floor_m').value)
        std += float(self.get_parameter('range_noise_snr_scale_m').value) / math.sqrt(snr + 1.0)
        measured = rng + random.gauss(0.0, std)
        return max(0.0, self.quantize(measured, float(self.get_parameter('range_resolution_m').value)))

    def apply_angle_measurement_error(self, angle, snr, base_noise_deg):
        std_deg = base_noise_deg + float(self.get_parameter('angle_noise_snr_scale_deg').value) / math.sqrt(snr + 1.0)
        return angle + math.radians(random.gauss(0.0, std_deg))

    def measure_radial_velocity(self, radar_range, radar_angle, snr):
        true_velocity = self.lookup_truth_radial_velocity(radar_range, radar_angle)
        velocity_std = float(self.get_parameter('radial_velocity_noise_stddev').value)
        if true_velocity is None:
            true_velocity = 0.0
        noise = random.gauss(0.0, velocity_std + 0.25 / math.sqrt(snr + 1.0))
        return self.quantize(true_velocity + noise, float(self.get_parameter('doppler_resolution_mps').value))

    def lookup_truth_radial_velocity(self, radar_range, radar_angle):
        if not bool(self.get_parameter('use_ground_truth_doppler').value):
            return None
        if self.model_states is None:
            return None

        usv_index = self.find_model(str(self.get_parameter('usv_model_name').value))
        if usv_index < 0:
            return None
        if usv_index >= len(self.model_states.pose) or usv_index >= len(self.model_states.twist):
            return None

        usv_pose = self.model_states.pose[usv_index]
        usv_twist = self.model_states.twist[usv_index]
        usv_yaw = self.yaw_from_quaternion(usv_pose.orientation)
        cos_yaw = math.cos(usv_yaw)
        sin_yaw = math.sin(usv_yaw)
        bearing_base = self.wrap_angle(radar_angle + float(self.get_parameter('mount_yaw_rad').value))

        best_index = -1
        best_score = float('inf')
        range_gate = float(self.get_parameter('doppler_match_range_gate_m').value)
        angle_gate = math.radians(float(self.get_parameter('doppler_match_angle_gate_deg').value))

        for index, name in enumerate(self.model_states.name):
            if index >= len(self.model_states.pose) or index >= len(self.model_states.twist):
                continue
            if index == usv_index or not self.is_doppler_target(name):
                continue
            pose = self.model_states.pose[index]
            dx_world = pose.position.x - usv_pose.position.x
            dy_world = pose.position.y - usv_pose.position.y
            dx_base = cos_yaw * dx_world + sin_yaw * dy_world
            dy_base = -sin_yaw * dx_world + cos_yaw * dy_world
            model_range = math.hypot(dx_base, dy_base)
            model_bearing = math.atan2(dy_base, dx_base)
            range_error = abs(model_range - radar_range)
            angle_error = abs(self.wrap_angle(model_bearing - bearing_base))
            if range_error > range_gate or angle_error > angle_gate:
                continue
            score = (range_error / max(range_gate, 1e-3)) ** 2 + (angle_error / max(angle_gate, 1e-3)) ** 2
            if score < best_score:
                best_index = index
                best_score = score

        if best_index < 0:
            return None

        target_twist = self.model_states.twist[best_index]
        rel_vx_world = target_twist.linear.x - usv_twist.linear.x
        rel_vy_world = target_twist.linear.y - usv_twist.linear.y
        los_x_base = math.cos(bearing_base)
        los_y_base = math.sin(bearing_base)
        los_x_world = cos_yaw * los_x_base - sin_yaw * los_y_base
        los_y_world = sin_yaw * los_x_base + cos_yaw * los_y_base
        return rel_vx_world * los_x_world + rel_vy_world * los_y_world

    def is_doppler_target(self, name):
        target_names = list(self.get_parameter('doppler_target_names').value)
        if not target_names:
            return True
        return name in target_names

    def find_model(self, name):
        if self.model_states is None:
            return -1
        try:
            return self.model_states.name.index(name)
        except ValueError:
            return -1

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

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
            data.extend(struct.pack('<ffffffffffffff', *p))

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
