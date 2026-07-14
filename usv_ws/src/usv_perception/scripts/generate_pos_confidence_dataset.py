#!/usr/bin/env python3
"""Record a supervised C3 position-confidence dataset from Gazebo Classic.

The recorder deliberately separates data sources:
  * sensor topics provide the four network inputs only;
  * /model_states is used only while recording to rasterize supervision labels.

Each sample is a compressed NumPy archive compatible with c3_pos_confidence/convert.py:
  dehaze_camera [N, 3], gated_camera [N, 3], sonar [N, 3], radar [N, 4]
  target_heatmap [1, H, W], target_offset [2, H, W]

No world-frame target coordinate or Gazebo model name is saved into the sample.  This
prevents accidentally using simulator ground truth as an inference-time feature.
"""

import json
import math
import os
import random
import shutil
import struct
import time
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from c3_sonar_driver.msg import SonarDetect
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


DEFAULT_RADAR_TOPICS = [
    f'/mmwave/{sector}/{height}/detections'
    for height in ('h10m', 'h4m', 'h1p9m', 'h1p5m', 'h1m')
    for sector in ('front', 'right', 'back', 'left')
]

DEFAULT_TARGET_MODELS = [
    'moving_vessel',
]

FLOAT_FORMATS = {
    PointField.FLOAT32: 'f',
    PointField.FLOAT64: 'd',
}


def yaw_from_quaternion(quaternion) -> float:
    """Return planar yaw; labels use the USV body frame, not the world frame."""
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def field_lookup(message: PointCloud2) -> Dict[str, PointField]:
    return {field.name: field for field in message.fields}


def read_scalar(data: bytes, base: int, field: Optional[PointField], bigendian: bool) -> float:
    if field is None or field.datatype not in FLOAT_FORMATS:
        return 0.0
    prefix = '>' if bigendian else '<'
    return float(struct.unpack_from(prefix + FLOAT_FORMATS[field.datatype], data, base + field.offset)[0])


def cloud_to_array(message: PointCloud2, include_velocity: bool) -> np.ndarray:
    """Decode only features that the position-confidence network consumes."""
    fields = field_lookup(message)
    if 'x' not in fields or 'y' not in fields or message.point_step <= 0:
        return np.empty((0, 4 if include_velocity else 3), dtype=np.float32)

    z_field = fields.get('z')
    velocity_field = fields.get('radial_velocity')
    point_count = int(message.width) * int(message.height)
    columns = 4 if include_velocity else 3
    points = np.empty((point_count, columns), dtype=np.float32)
    valid_count = 0
    raw = bytes(message.data)

    for index in range(point_count):
        base = index * int(message.point_step)
        if base + int(message.point_step) > len(raw):
            break
        x = read_scalar(raw, base, fields['x'], message.is_bigendian)
        y = read_scalar(raw, base, fields['y'], message.is_bigendian)
        z = read_scalar(raw, base, z_field, message.is_bigendian)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        points[valid_count, :3] = (x, y, z)
        if include_velocity:
            velocity = read_scalar(raw, base, velocity_field, message.is_bigendian)
            points[valid_count, 3] = velocity if math.isfinite(velocity) else 0.0
        valid_count += 1
    return points[:valid_count]


class PosConfidenceDatasetRecorder(Node):
    """Collect delayed/noisy sensor inputs and independently rasterized labels."""

    def __init__(self):
        super().__init__('pos_confidence_dataset_recorder')
        self.declare_parameter('output_dir', 'data/pos_confidence_sim_2000')
        self.declare_parameter('sample_count', 2000)
        # The timer runs at 20 Hz so scene refresh and sensor settling can be
        # controlled independently. One labelled sample is saved per 0.5 s scene.
        self.declare_parameter('sample_period_s', 0.05)
        self.declare_parameter('max_frame_age_s', 0.70)
        self.declare_parameter('max_points_per_modality', 1200)
        self.declare_parameter('random_seed', 20260714)
        self.declare_parameter('max_dataset_size', 2000)
        self.declare_parameter('scene_samples', 1)
        self.declare_parameter('scene_settle_s', 0.45)

        self.declare_parameter('model_states_topic', '/model_states')
        self.declare_parameter('usv_model_name', 'wamv')
        self.declare_parameter('target_model_names', DEFAULT_TARGET_MODELS)
        self.declare_parameter('refresh_target_model_name', 'moving_vessel')
        self.declare_parameter('refresh_target_enabled', True)
        self.declare_parameter('set_entity_state_service', '/set_entity_state')
        self.declare_parameter('refresh_min_range_m', 8.0)
        self.declare_parameter('refresh_max_range_m', 100.0)
        self.declare_parameter('refresh_lateral_limit_m', 70.0)
        self.declare_parameter('refresh_target_z_m', 0.42)
        self.declare_parameter('refresh_max_speed_mps', 1.2)
        # Keep half of the scenes inside sonar range while retaining long-range
        # samples where sonar correctly contributes no target return.
        self.declare_parameter('refresh_sonar_visible_probability', 0.50)
        self.declare_parameter('refresh_sonar_visible_max_range_m', 52.0)
        self.declare_parameter('sonar_accumulation_s', 0.80)

        self.declare_parameter('dehaze_topic', '/depth_camera/dehazed_points')
        self.declare_parameter('gated_topic', '/gated_camera/pseudocolor/detection_points')
        self.declare_parameter('sonar_topic', '/sonar/detect')
        self.declare_parameter('radar_topics', DEFAULT_RADAR_TOPICS)
        self.declare_parameter('require_all_modalities_seen', True)

        # depth_image_to_pointcloud2 publishes in the ROS optical camera convention.
        # The remaining selected topics are already emitted in base_link.
        self.declare_parameter('dehaze_camera_optical_frame', True)
        self.declare_parameter('dehaze_camera_x_offset_m', 1.65)
        self.declare_parameter('dehaze_camera_y_offset_m', 0.0)
        self.declare_parameter('dehaze_camera_z_offset_m', 0.98)

        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 150.0)
        self.declare_parameter('y_min', -75.0)
        self.declare_parameter('y_max', 75.0)
        self.declare_parameter('voxel_size', 1.0)

        self.output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser().resolve()
        self.prepare_output_directory()
        self.split_dirs = {
            split: self.output_dir / split / 'samples'
            for split in ('train', 'val', 'test')
        }
        for directory in self.split_dirs.values():
            directory.mkdir(parents=True, exist_ok=True)
        self.sample_count = max(1, int(self.get_parameter('sample_count').value))
        self.max_dataset_size = max(1, int(self.get_parameter('max_dataset_size').value))
        self.max_age_s = max(0.05, float(self.get_parameter('max_frame_age_s').value))
        self.max_points = max(1, int(self.get_parameter('max_points_per_modality').value))
        self.scene_samples = max(1, int(self.get_parameter('scene_samples').value))
        self.scene_settle_s = max(0.0, float(self.get_parameter('scene_settle_s').value))
        self.sonar_accumulation_s = max(0.05, float(self.get_parameter('sonar_accumulation_s').value))
        self.rng = np.random.default_rng(int(self.get_parameter('random_seed').value))
        self.split_rng = random.Random(int(self.get_parameter('random_seed').value))

        self.x_min = float(self.get_parameter('x_min').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.y_min = float(self.get_parameter('y_min').value)
        self.y_max = float(self.get_parameter('y_max').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        if self.x_max <= self.x_min or self.y_max <= self.y_min or self.voxel_size <= 0.0:
            raise ValueError('Invalid grid bounds or voxel_size.')
        self.width = int(round((self.x_max - self.x_min) / self.voxel_size))
        self.height = int(round((self.y_max - self.y_min) / self.voxel_size))

        self.clouds: Dict[str, Tuple[np.ndarray, float]] = {}
        self.radar_clouds: Dict[str, Tuple[np.ndarray, float]] = {}
        self.sonar_frames = deque()
        self.model_states: Optional[ModelStates] = None
        self.seen_modalities = set()
        self.sample_records: List[Dict[str, object]] = []
        self.samples_by_split = {split: [] for split in self.split_dirs}
        self.next_sample_id = 0
        self.samples_recorded_this_run = 0
        self.load_existing_records()
        while len(self.sample_records) > self.max_dataset_size:
            self.evict_oldest_sample()
        self.active_scene_index = -1
        self.active_scene_split: Optional[str] = None
        self.scene_started_at: Optional[float] = None
        self.expected_target_world: Optional[Tuple[float, float]] = None
        self.finished = False

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointCloud2, str(self.get_parameter('dehaze_topic').value),
            lambda message: self.on_cloud('dehaze_camera', message, False, True), sensor_qos)
        self.create_subscription(
            PointCloud2, str(self.get_parameter('gated_topic').value),
            lambda message: self.on_cloud('gated_camera', message, False, False), sensor_qos)
        self.create_subscription(
            SonarDetect, str(self.get_parameter('sonar_topic').value), self.on_sonar, 20)
        for topic in self.get_parameter('radar_topics').value:
            self.create_subscription(
                PointCloud2, str(topic), lambda message, name=str(topic): self.on_radar(name, message), sensor_qos)
        self.create_subscription(
            ModelStates, str(self.get_parameter('model_states_topic').value), self.on_model_states, 10)
        self.set_entity_state_client = self.create_client(
            SetEntityState, str(self.get_parameter('set_entity_state_service').value))

        period = max(0.05, float(self.get_parameter('sample_period_s').value))
        self.timer = self.create_timer(period, self.record_if_ready)
        self.write_metadata(partial=True)
        self.get_logger().info(
            f'Appending {self.sample_count} samples to {self.output_dir}; current={len(self.sample_records)}/'
            f'{self.max_dataset_size}; grid={self.width}x{self.height}.')

    def prepare_output_directory(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def sequence_from_path(path: str) -> int:
        try:
            return int(Path(path).stem)
        except ValueError:
            return -1

    def choose_split(self) -> str:
        ratios = {'train': 0.70, 'val': 0.15, 'test': 0.15}
        return min(ratios, key=lambda split: len(self.samples_by_split[split]) / ratios[split])

    def load_existing_records(self):
        metadata_path = self.output_dir / 'metadata.json'
        raw_records = []
        if metadata_path.exists():
            try:
                raw_records = json.loads(metadata_path.read_text(encoding='utf-8')).get('samples', [])
            except (json.JSONDecodeError, OSError):
                self.get_logger().warn(f'Ignoring unreadable metadata: {metadata_path}')

        records = []
        for item in raw_records:
            if isinstance(item, dict) and isinstance(item.get('path'), str):
                records.append(dict(item))
            elif isinstance(item, str):
                records.append({'path': item})
        if not records:
            for split in self.split_dirs:
                for path in sorted(self.split_dirs[split].glob('*.npz')):
                    records.append({'path': path.relative_to(self.output_dir).as_posix(), 'split': split})
            # Migrate the initial recorder layout only once, preserving every file.
            for path in sorted((self.output_dir / 'samples').glob('*.npz')):
                records.append({'path': path.relative_to(self.output_dir).as_posix()})

        seen_paths = set()
        for record in sorted(records, key=lambda item: (int(item.get('sequence', -1)), self.sequence_from_path(item['path']))):
            relative = str(record['path'])
            source = self.output_dir / relative
            if relative in seen_paths or not source.exists():
                continue
            split = str(record.get('split', ''))
            if split not in self.split_dirs:
                parts = Path(relative).parts
                split = parts[0] if parts and parts[0] in self.split_dirs else self.choose_split()
                destination = self.split_dirs[split] / source.name
                if source != destination:
                    if destination.exists():
                        raise RuntimeError(f'Cannot migrate duplicate dataset sample: {destination}')
                    shutil.move(str(source), str(destination))
                    source = destination
                relative = source.relative_to(self.output_dir).as_posix()
            sequence = int(record.get('sequence', self.sequence_from_path(relative)))
            self.sample_records.append({'path': relative, 'split': split, 'sequence': sequence})
            self.samples_by_split[split].append(relative)
            self.next_sample_id = max(self.next_sample_id, sequence + 1)
            seen_paths.add(relative)

    def evict_oldest_sample(self):
        if not self.sample_records:
            return
        record = self.sample_records.pop(0)
        relative = str(record['path'])
        path = self.output_dir / relative
        if path.exists():
            path.unlink()
        split = str(record['split'])
        if split in self.samples_by_split and relative in self.samples_by_split[split]:
            self.samples_by_split[split].remove(relative)

    def on_cloud(self, modality: str, message: PointCloud2, include_velocity: bool, optical_frame: bool):
        points = cloud_to_array(message, include_velocity)
        if optical_frame and bool(self.get_parameter('dehaze_camera_optical_frame').value):
            points = self.optical_to_base(points)
        self.clouds[modality] = (points, time.monotonic())
        self.seen_modalities.add(modality)

    def on_radar(self, topic: str, message: PointCloud2):
        self.radar_clouds[topic] = (cloud_to_array(message, True), time.monotonic())
        self.seen_modalities.add('radar')

    def on_sonar(self, message: SonarDetect):
        points = cloud_to_array(message.cloud, False)
        if points.size == 0 and math.isfinite(message.position.x) and math.isfinite(message.position.y):
            points = np.array([[message.position.x, message.position.y, message.position.z]], dtype=np.float32)
        timestamp = time.monotonic()
        self.clouds['sonar'] = (points, timestamp)
        self.sonar_frames.append((points, timestamp))
        keep_after = timestamp - max(self.max_age_s, self.sonar_accumulation_s) * 2.0
        while self.sonar_frames and self.sonar_frames[0][1] < keep_after:
            self.sonar_frames.popleft()
        self.seen_modalities.add('sonar')

    def on_model_states(self, message: ModelStates):
        self.model_states = message

    def optical_to_base(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points
        result = points.copy()
        # Optical cloud: x=right, y=down, z=forward.  base_link: x=forward, y=left, z=up.
        result[:, 0] = float(self.get_parameter('dehaze_camera_x_offset_m').value) + points[:, 2]
        result[:, 1] = float(self.get_parameter('dehaze_camera_y_offset_m').value) - points[:, 0]
        result[:, 2] = float(self.get_parameter('dehaze_camera_z_offset_m').value) - points[:, 1]
        return result

    def fresh_points(self, modality: str, columns: int, now: float) -> np.ndarray:
        if modality == 'sonar':
            frames = [
                points for points, timestamp in self.sonar_frames
                if now - timestamp <= self.sonar_accumulation_s and
                (self.scene_started_at is None or timestamp >= self.scene_started_at)
            ]
            if not frames:
                return np.empty((0, columns), dtype=np.float32)
            return self.limit_points(np.concatenate(frames, axis=0), columns)
        frame = self.clouds.get(modality)
        if frame is None or now - frame[1] > self.max_age_s:
            return np.empty((0, columns), dtype=np.float32)
        if self.scene_started_at is not None and frame[1] < self.scene_started_at:
            return np.empty((0, columns), dtype=np.float32)
        return self.limit_points(frame[0], columns)

    def fresh_radar_points(self, now: float) -> np.ndarray:
        frames = [
            points for points, timestamp in self.radar_clouds.values()
            if now - timestamp <= self.max_age_s and
            (self.scene_started_at is None or timestamp >= self.scene_started_at)
        ]
        if not frames:
            return np.empty((0, 4), dtype=np.float32)
        return self.limit_points(np.concatenate(frames, axis=0), 4)

    def limit_points(self, points: np.ndarray, columns: int) -> np.ndarray:
        if points.size == 0:
            return np.empty((0, columns), dtype=np.float32)
        points = np.asarray(points, dtype=np.float32).reshape((-1, columns))
        if points.shape[0] <= self.max_points:
            return points
        indices = self.rng.choice(points.shape[0], size=self.max_points, replace=False)
        return points[indices]

    def record_if_ready(self):
        if self.finished or self.model_states is None:
            return
        required = {'dehaze_camera', 'gated_camera', 'sonar', 'radar'}
        if bool(self.get_parameter('require_all_modalities_seen').value) and not required.issubset(self.seen_modalities):
            missing = ', '.join(sorted(required - self.seen_modalities))
            self.get_logger().info(f'Waiting for first messages from: {missing}', throttle_duration_sec=5.0)
            return

        expected_scene = self.samples_recorded_this_run // self.scene_samples
        if expected_scene != self.active_scene_index:
            self.start_scene(expected_scene)
            return
        if self.scene_started_at is None or time.monotonic() - self.scene_started_at < self.scene_settle_s:
            return
        if not self.target_is_at_requested_scene():
            self.get_logger().info('Waiting for Gazebo to apply the randomized target position.', throttle_duration_sec=3.0)
            return

        labels = self.make_labels(self.model_states)
        if labels is None:
            self.get_logger().warn('Waiting for the configured USV model in /model_states.', throttle_duration_sec=5.0)
            return
        heatmap, offsets, target_count = labels
        now = time.monotonic()
        sample_index = self.samples_recorded_this_run
        split = self.active_scene_split
        if split is None:
            return
        if len(self.sample_records) >= self.max_dataset_size:
            self.evict_oldest_sample()
        sequence = self.next_sample_id
        path = self.split_dirs[split] / f'{sequence:09d}.npz'
        temporary = self.split_dirs[split] / f'{sequence:09d}.partial.npz'

        np.savez_compressed(
            temporary,
            dehaze_camera=self.fresh_points('dehaze_camera', 3, now),
            gated_camera=self.fresh_points('gated_camera', 3, now),
            sonar=self.fresh_points('sonar', 3, now),
            radar=self.fresh_radar_points(now),
            target_heatmap=heatmap,
            target_offset=offsets,
            target_count=np.asarray([target_count], dtype=np.int16),
        )
        os.replace(temporary, path)
        relative_path = path.relative_to(self.output_dir).as_posix()
        self.sample_records.append({
            'path': relative_path,
            'split': split,
            'scene_index': self.active_scene_index,
            'sequence': sequence,
        })
        self.samples_by_split[split].append(relative_path)
        self.next_sample_id += 1
        self.samples_recorded_this_run += 1
        self.write_metadata(partial=True)

        if (sample_index + 1) % 25 == 0 or sample_index + 1 == self.sample_count:
            self.get_logger().info(
                f'Collected {sample_index + 1}/{self.sample_count}; retained={len(self.sample_records)}/'
                f'{self.max_dataset_size}; labels={target_count}.')
        if self.samples_recorded_this_run >= self.sample_count:
            self.finalize()

    def start_scene(self, scene_index: int):
        if scene_index >= math.ceil(self.sample_count / self.scene_samples):
            return
        if bool(self.get_parameter('refresh_target_enabled').value):
            if not self.set_entity_state_client.service_is_ready():
                self.get_logger().info('Waiting for /set_entity_state before refreshing the target.', throttle_duration_sec=3.0)
                return
            pose = self.random_target_pose()
            if pose is None:
                self.get_logger().info('Waiting for the USV model before refreshing the target.', throttle_duration_sec=3.0)
                return
            target_x, target_y, yaw, speed = pose
            state = EntityState()
            state.name = str(self.get_parameter('refresh_target_model_name').value)
            state.reference_frame = 'world'
            state.pose.position.x = target_x
            state.pose.position.y = target_y
            state.pose.position.z = float(self.get_parameter('refresh_target_z_m').value)
            state.pose.orientation.z = math.sin(0.5 * yaw)
            state.pose.orientation.w = math.cos(0.5 * yaw)
            state.twist.linear.x = speed * math.cos(yaw)
            state.twist.linear.y = speed * math.sin(yaw)
            request = SetEntityState.Request()
            request.state = state
            self.set_entity_state_client.call_async(request)
            self.expected_target_world = (target_x, target_y)
        else:
            self.expected_target_world = None
        self.active_scene_index = scene_index
        self.active_scene_split = self.choose_split()
        self.scene_started_at = time.monotonic()
        self.get_logger().info(
            f'Started scene {scene_index + 1} for {self.active_scene_split}.')

    def random_target_pose(self) -> Optional[Tuple[float, float, float, float]]:
        if self.model_states is None:
            return None
        names = list(self.model_states.name)
        try:
            usv_index = names.index(str(self.get_parameter('usv_model_name').value))
        except ValueError:
            return None
        if usv_index >= len(self.model_states.pose):
            return None
        usv_pose = self.model_states.pose[usv_index]
        usv_yaw = yaw_from_quaternion(usv_pose.orientation)
        minimum = max(1.0, float(self.get_parameter('refresh_min_range_m').value))
        maximum = min(100.0, float(self.get_parameter('refresh_max_range_m').value))
        visible_probability = min(1.0, max(0.0, float(
            self.get_parameter('refresh_sonar_visible_probability').value)))
        visible_maximum = min(maximum, float(
            self.get_parameter('refresh_sonar_visible_max_range_m').value))
        if visible_maximum <= minimum:
            visible_probability = 0.0
        elif self.split_rng.random() < visible_probability:
            maximum = visible_maximum
        else:
            minimum = visible_maximum
        lateral_limit = min(abs(float(self.get_parameter('refresh_lateral_limit_m').value)), abs(self.y_max) - 0.5)
        if maximum <= minimum or lateral_limit <= 0.0:
            raise ValueError('Invalid randomized target range configuration.')
        for _ in range(100):
            radius = math.sqrt(self.split_rng.uniform(minimum * minimum, maximum * maximum))
            bearing = self.split_rng.uniform(-math.pi * 0.5, math.pi * 0.5)
            local_x, local_y = radius * math.cos(bearing), radius * math.sin(bearing)
            if abs(local_y) <= lateral_limit:
                target_x = usv_pose.position.x + math.cos(usv_yaw) * local_x - math.sin(usv_yaw) * local_y
                target_y = usv_pose.position.y + math.sin(usv_yaw) * local_x + math.cos(usv_yaw) * local_y
                target_yaw = self.split_rng.uniform(-math.pi, math.pi)
                speed = self.split_rng.uniform(0.0, float(self.get_parameter('refresh_max_speed_mps').value))
                return target_x, target_y, target_yaw, speed
        return None

    def target_is_at_requested_scene(self) -> bool:
        if self.expected_target_world is None:
            return True
        if self.model_states is None:
            return False
        names = list(self.model_states.name)
        try:
            target_index = names.index(str(self.get_parameter('refresh_target_model_name').value))
        except ValueError:
            return False
        if target_index >= len(self.model_states.pose):
            return False
        target_pose = self.model_states.pose[target_index]
        return math.hypot(
            target_pose.position.x - self.expected_target_world[0],
            target_pose.position.y - self.expected_target_world[1]) <= 1.0

    def make_labels(self, states: ModelStates) -> Optional[Tuple[np.ndarray, np.ndarray, int]]:
        names = list(states.name)
        try:
            usv_index = names.index(str(self.get_parameter('usv_model_name').value))
        except ValueError:
            return None
        if usv_index >= len(states.pose):
            return None

        usv_pose = states.pose[usv_index]
        yaw = yaw_from_quaternion(usv_pose.orientation)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        selected = set(self.get_parameter('target_model_names').value)
        heatmap = np.zeros((1, self.height, self.width), dtype=np.float32)
        offsets = np.zeros((2, self.height, self.width), dtype=np.float32)
        occupied_cells = {}

        for index, name in enumerate(names):
            if name not in selected or index >= len(states.pose):
                continue
            pose = states.pose[index]
            dx = pose.position.x - usv_pose.position.x
            dy = pose.position.y - usv_pose.position.y
            # Rotate world displacement into the current base_link frame.
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            gx = math.floor((local_x - self.x_min) / self.voxel_size)
            gy = math.floor((local_y - self.y_min) / self.voxel_size)
            if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
                continue

            center_x = self.x_min + (gx + 0.5) * self.voxel_size
            center_y = self.y_min + (gy + 0.5) * self.voxel_size
            distance_to_center = (local_x - center_x) ** 2 + (local_y - center_y) ** 2
            previous = occupied_cells.get((gy, gx))
            if previous is None or distance_to_center < previous[0]:
                occupied_cells[(gy, gx)] = (distance_to_center, local_x - center_x, local_y - center_y)

        for (gy, gx), (_, offset_x, offset_y) in occupied_cells.items():
            # A single positive cell keeps the offset in [-0.5, 0.5] m, matching
            # LocalizationHead's bounded offset output and offset_sigma_loss mask.
            heatmap[0, gy, gx] = 1.0
            offsets[0, gy, gx] = offset_x
            offsets[1, gy, gx] = offset_y
        return heatmap, offsets, len(occupied_cells)

    def write_metadata(self, partial: bool):
        self.write_split_manifests()
        metadata = {
            'format_version': 1,
            'complete': not partial,
            'samples_requested_this_run': self.sample_count,
            'sample_count_written': len(self.sample_records),
            'max_dataset_size': self.max_dataset_size,
            'next_sample_id': self.next_sample_id,
            'samples': self.sample_records,
            'samples_by_split': self.samples_by_split,
            'split_policy': 'Scene-level random split: 70% train, 15% val, 15% test.',
            'scene_samples': self.scene_samples,
            'scene_settle_s': self.scene_settle_s,
            'refresh_policy': {
                'target_model': str(self.get_parameter('refresh_target_model_name').value),
                'enabled': bool(self.get_parameter('refresh_target_enabled').value),
                'max_range_m': min(100.0, float(self.get_parameter('refresh_max_range_m').value)),
                'sonar_visible_probability': float(
                    self.get_parameter('refresh_sonar_visible_probability').value),
                'sonar_visible_max_range_m': float(
                    self.get_parameter('refresh_sonar_visible_max_range_m').value),
            },
            'input_contract': {
                'dehaze_camera': '[N,3] float32 base_link xyz',
                'gated_camera': '[N,3] float32 base_link xyz',
                'sonar': '[N,3] float32 base_link xyz',
                'radar': '[N,4] float32 base_link xyz + radial_velocity',
            },
            'labels': {
                'target_heatmap': '[1,H,W] float32 one positive cell per target',
                'target_offset': '[2,H,W] float32 meters relative to the positive cell center',
                'target_count': '[1] int16 number of labelled cells',
            },
            'grid': {
                'x_min': self.x_min, 'x_max': self.x_max,
                'y_min': self.y_min, 'y_max': self.y_max,
                'voxel_size': self.voxel_size, 'width': self.width, 'height': self.height,
            },
            'target_model_names': list(self.get_parameter('target_model_names').value),
            'ground_truth_policy': 'Used only at recording time to create labels; never saved as network input.',
        }
        temporary = self.output_dir / 'metadata.partial.json'
        temporary.write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
        os.replace(temporary, self.output_dir / 'metadata.json')

    def write_split_manifests(self):
        for split, samples in self.samples_by_split.items():
            content = ''.join(f'{path}\n' for path in samples)
            (self.output_dir / f'{split}.txt').write_text(content, encoding='utf-8')
            (self.output_dir / split / 'manifest.txt').write_text(content, encoding='utf-8')

    def finalize(self):
        if self.finished:
            return
        self.finished = True
        self.timer.cancel()
        self.write_metadata(partial=False)
        self.get_logger().info(f'Dataset complete: {len(self.sample_records)} samples in {self.output_dir}.')
        # Let ros2 run return after the requested sample count is reached.
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PosConfidenceDatasetRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Collection interrupted; retaining the already written samples.')
    finally:
        if not node.finished:
            node.write_metadata(partial=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
