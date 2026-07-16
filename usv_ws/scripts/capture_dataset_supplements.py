#!/usr/bin/env python3
"""Capture supplemental YOLO dataset images from the USV Gazebo scene.

The script positions one target model at a time in front of the ship and UAV
cameras, then saves four dataset styles:
  - ship gated pseudo-color range view
  - ship traditional RGB
  - UAV/plane gated pseudo-color range view
  - UAV/plane traditional RGB

Images are intentionally saved without labels. Use the manifest to filter and
label them before adding them to a training split.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Image


CLASS_TO_MODEL = {
    "moving_vessel": ["moving_vessel"],
    "fishing_boat": ["small_fishing_boat", "survey_boat"],
    "obstacle": ["floating_obstacle", "floating_container"],
    "research_platform": ["research_platform"],
    "service_boat": ["service_boat"],
    "ship_far": ["cargo_ship_far", "anchored_tanker"],
}

MODEL_Z = {
    "moving_vessel": 0.42,
    "small_fishing_boat": 0.34,
    "survey_boat": 0.36,
    "floating_obstacle": 0.24,
    "drift_debris": 0.16,
    "floating_container": 0.20,
    "research_platform": 0.45,
    "service_boat": 0.38,
    "cargo_ship_far": 0.48,
    "anchored_tanker": 0.48,
    "scout_uav": 12.0,
}

BACKGROUND_MODELS_TO_HIDE = [
    "fishnet_buoy",
    "channel_buoy_north",
    "channel_buoy_south",
    "navigation_marker_port",
    "navigation_marker_starboard",
    "net_line_a",
]
ALL_MODELS = sorted({model for models in CLASS_TO_MODEL.values() for model in models} | set(BACKGROUND_MODELS_TO_HIDE))
DATASET_DIRS = {
    "new_gated_camera": "new_gated_camera/images",
    "traditional_camera": "traditional_camera/images",
    "plane_gated": "plane_gated/images",
    "plane_traditional": "plane_traditional/images",
}


@dataclass(frozen=True)
class CaptureScene:
    class_name: str
    model_name: str
    distance: float
    lateral: float
    yaw: float
    uav_distance: float
    uav_lateral: float
    uav_altitude: float


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q = Quaternion()
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    q.w = cr * cp * cy + sr * sp * sy
    return q


def image_msg_to_array(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    if encoding in ("rgb8", "bgr8"):
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)
        array = array[:, : msg.width, :].copy()
        if encoding == "bgr8":
            array = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
        return array
    if encoding in ("rgba8", "bgra8"):
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 4, 4)
        array = array[:, : msg.width, :3].copy()
        if encoding == "bgra8":
            array = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
        return array
    if encoding in ("mono8", "8uc1"):
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        return array[:, : msg.width].copy()
    if encoding == "32fc1":
        row_values = msg.step // 4
        array = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, row_values)
        return array[:, : msg.width].copy()
    if encoding == "16uc1":
        row_values = msg.step // 2
        array = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, row_values)
        return array[:, : msg.width].astype(np.float32) / 1000.0
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def resize_512(image_rgb: np.ndarray) -> np.ndarray:
    if image_rgb.ndim == 2:
        image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_GRAY2RGB)
    return cv2.resize(image_rgb, (512, 512), interpolation=cv2.INTER_AREA)


def apply_gate(gray: np.ndarray, depth: np.ndarray, near: float, far: float) -> np.ndarray:
    center = 0.5 * (near + far)
    sigma = max((far - near) / 2.35, 0.1)
    clean_depth = np.where(np.isfinite(depth), depth, 0.0)
    mask = (clean_depth >= near) & (clean_depth <= far)
    normalized = (clean_depth - center) / sigma
    weighted = gray.astype(np.float32) * np.exp(-0.5 * normalized * normalized) * 1.8
    out = np.zeros(gray.shape, dtype=np.uint8)
    out[mask] = np.clip(weighted[mask], 0, 255).astype(np.uint8)
    return out


def range_view(raw_rgb: np.ndarray, depth: np.ndarray, gates: tuple[tuple[float, float], ...]) -> np.ndarray:
    if depth.shape[:2] != raw_rgb.shape[:2]:
        depth = cv2.resize(depth, (raw_rgb.shape[1], raw_rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
    gray = cv2.cvtColor(raw_rgb, cv2.COLOR_RGB2GRAY)
    near = apply_gate(gray, depth, *gates[0])
    mid = apply_gate(gray, depth, *gates[1])
    far = apply_gate(gray, depth, *gates[2])
    pseudo_rgb = np.dstack([near, mid, far])
    return cv2.addWeighted(pseudo_rgb, 0.85, raw_rgb, 0.15, 0.0)


class SupplementCapture(Node):
    def __init__(self, output_root: Path, seed: int):
        super().__init__("dataset_supplement_capture")
        self.output_root = output_root
        self.random = random.Random(seed)
        self.latest: dict[str, tuple[float, np.ndarray]] = {}
        self.client = self.create_client(SetEntityState, "/set_entity_state")
        self.create_subscription(Image, "/gated_camera/image_raw", lambda msg: self.on_image("ship_rgb", msg), 10)
        self.create_subscription(Image, "/depth_camera/image_raw", lambda msg: self.on_image("traditional", msg), 10)
        self.create_subscription(Image, "/depth_camera/depth/image_raw", lambda msg: self.on_image("ship_depth", msg), 10)
        self.create_subscription(Image, "/uav/gated_camera/image_raw", lambda msg: self.on_image("uav_rgb", msg), 10)
        self.create_subscription(Image, "/uav/gated_camera/depth/image_raw", lambda msg: self.on_image("uav_depth", msg), 10)

    def on_image(self, key: str, msg: Image) -> None:
        try:
            self.latest[key] = (time.monotonic(), image_msg_to_array(msg))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"{key}: {exc}")

    def wait_for_inputs(self, timeout_s: float = 20.0) -> None:
        required = {"ship_rgb", "traditional", "ship_depth", "uav_rgb", "uav_depth"}
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if required.issubset(self.latest):
                return
        missing = sorted(required - set(self.latest))
        raise TimeoutError(f"Timed out waiting for image topics: {missing}")

    def call_set_state(self, name: str, x: float, y: float, z: float, yaw: float, pitch: float = 0.0) -> None:
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = name
        request.state.reference_frame = "world"
        request.state.pose.position.x = float(x)
        request.state.pose.position.y = float(y)
        request.state.pose.position.z = float(z)
        request.state.pose.orientation = quaternion_from_euler(0.0, pitch, yaw)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or not future.result() or not future.result().success:
            status = future.result().status_message if future.done() and future.result() else "timeout"
            self.get_logger().warn(f"SetEntityState failed for {name}: {status}")

    def hide_all_targets(self) -> None:
        for index, name in enumerate(ALL_MODELS):
            self.call_set_state(name, -180.0 - index * 8.0, 120.0 + index * 6.0, MODEL_Z.get(name, 0.4), 0.0)

    def set_ship_scene(self, scene: CaptureScene) -> None:
        self.hide_all_targets()
        self.call_set_state(
            scene.model_name,
            scene.distance,
            scene.lateral,
            MODEL_Z.get(scene.model_name, 0.4),
            scene.yaw,
        )
        self.call_set_state("scout_uav", -140.0, 140.0, MODEL_Z["scout_uav"], -0.75)

    def set_uav_scene(self, scene: CaptureScene) -> None:
        self.hide_all_targets()
        target_x = 90.0 + scene.distance * 0.35
        target_y = 70.0 + scene.lateral * 0.5
        self.call_set_state(
            scene.model_name,
            target_x,
            target_y,
            MODEL_Z.get(scene.model_name, 0.4),
            scene.yaw,
        )
        uav_x = target_x - scene.uav_distance
        uav_y = target_y + scene.uav_lateral
        yaw_to_target = math.atan2(target_y - uav_y, target_x - uav_x)
        self.call_set_state("scout_uav", uav_x, uav_y, scene.uav_altitude, yaw_to_target)

    def fresh_copy(self, key: str) -> np.ndarray:
        return self.latest[key][1].copy()

    def save_outputs(
        self,
        scene: CaptureScene,
        index: int,
        manifest: csv.writer,
        outputs: dict[str, np.ndarray],
    ) -> None:
        stem = (
            f"{index:04d}_{scene.class_name}_{scene.model_name}"
            f"_d{scene.distance:.1f}_y{scene.lateral:.1f}_yaw{scene.yaw:.2f}"
        ).replace("-", "m").replace(".", "p")
        for dataset, image in outputs.items():
            rel_dir = Path(DATASET_DIRS[dataset])
            out_dir = self.output_root / rel_dir
            out_dir.mkdir(parents=True, exist_ok=True)
            path = out_dir / f"{stem}.jpg"
            cv2.imwrite(str(path), cv2.cvtColor(resize_512(image), cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), 94])
            manifest.writerow([
                path.relative_to(self.output_root).as_posix(),
                dataset,
                scene.class_name,
                scene.model_name,
                f"{scene.distance:.2f}",
                f"{scene.lateral:.2f}",
                f"{scene.yaw:.3f}",
                f"{scene.uav_altitude:.2f}",
            ])

    def save_ship_scene(self, scene: CaptureScene, index: int, manifest: csv.writer) -> None:
        ship_rgb = self.fresh_copy("ship_rgb")
        traditional = self.fresh_copy("traditional")
        ship_depth = self.fresh_copy("ship_depth")
        ship_gated = range_view(ship_rgb, ship_depth, ((2.0, 18.0), (12.0, 42.0), (32.0, 88.0)))
        self.save_outputs(
            scene,
            index,
            manifest,
            {
                "new_gated_camera": ship_gated,
                "traditional_camera": traditional,
            },
        )

    def save_uav_scene(self, scene: CaptureScene, index: int, manifest: csv.writer) -> None:
        uav_rgb = self.fresh_copy("uav_rgb")
        uav_depth = self.fresh_copy("uav_depth")
        plane_gated = range_view(uav_rgb, uav_depth, ((2.0, 18.0), (12.0, 42.0), (32.0, 88.0)))
        self.save_outputs(
            scene,
            index,
            manifest,
            {
                "plane_gated": plane_gated,
                "plane_traditional": uav_rgb,
            },
        )


def build_scenes(seed: int, counts: dict[str, int]) -> list[CaptureScene]:
    rng = random.Random(seed)
    scenes: list[CaptureScene] = []
    for class_name, count in counts.items():
        models = CLASS_TO_MODEL[class_name]
        for index in range(count):
            model = models[index % len(models)]
            if class_name == "ship_far":
                distance = rng.uniform(42.0, 82.0)
                lateral = rng.uniform(-10.0, 10.0)
                uav_distance = rng.uniform(26.0, 52.0)
                uav_altitude = rng.uniform(10.0, 22.0)
            elif class_name == "obstacle":
                distance = rng.uniform(5.5, 22.0)
                lateral = rng.uniform(-3.8, 3.8)
                uav_distance = rng.uniform(18.0, 34.0)
                uav_altitude = rng.uniform(4.0, 8.0)
            elif class_name == "research_platform":
                distance = rng.uniform(16.0, 46.0)
                lateral = rng.uniform(-7.0, 7.0)
                uav_distance = rng.uniform(18.0, 34.0)
                uav_altitude = rng.uniform(10.0, 18.0)
            else:
                distance = rng.uniform(10.0, 44.0)
                lateral = rng.uniform(-6.5, 6.5)
                uav_distance = rng.uniform(14.0, 32.0)
                uav_altitude = rng.uniform(8.0, 17.0)
            scenes.append(CaptureScene(
                class_name=class_name,
                model_name=model,
                distance=distance,
                lateral=lateral,
                yaw=rng.uniform(-0.75, 0.75),
                uav_distance=uav_distance,
                uav_lateral=rng.uniform(-2.5, 2.5),
                uav_altitude=uav_altitude,
            ))
    rng.shuffle(scenes)
    return scenes


def parse_counts(values: list[str]) -> dict[str, int]:
    defaults = {
        "ship_far": 40,
        "obstacle": 30,
        "moving_vessel": 8,
        "fishing_boat": 8,
        "research_platform": 8,
        "service_boat": 8,
    }
    for value in values:
        name, _, raw_count = value.partition("=")
        name = name.strip()
        if name not in defaults or not raw_count:
            raise ValueError(f"Bad --count value {value!r}. Use class=count, e.g. ship_far=40")
        defaults[name] = int(raw_count)
    return defaults


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    parser.add_argument("--output-root", default=f"/home/hu/yolo/supplement_captures_{stamp}")
    parser.add_argument("--seed", type=int, default=20260715)
    parser.add_argument("--settle", type=float, default=0.35, help="Seconds to wait after moving a target")
    parser.add_argument("--count", action="append", default=[], help="Override class count, e.g. ship_far=50")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_root = Path(args.output_root).expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    counts = parse_counts(args.count)
    scenes = build_scenes(args.seed, counts)

    rclpy.init()
    node = SupplementCapture(output_root, args.seed)
    try:
        if not node.client.wait_for_service(timeout_sec=20.0):
            raise TimeoutError("Service /set_entity_state is not available")
        node.wait_for_inputs()
        manifest_path = output_root / "manifest.csv"
        with manifest_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["image", "dataset", "suggested_class", "model", "distance_m", "lateral_m", "yaw_rad", "uav_altitude_m"])
            for index, scene in enumerate(scenes, 1):
                node.set_ship_scene(scene)
                deadline = time.monotonic() + args.settle
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                node.save_ship_scene(scene, index, writer)
                node.set_uav_scene(scene)
                deadline = time.monotonic() + args.settle
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                node.save_uav_scene(scene, index, writer)
                if index % 10 == 0 or index == len(scenes):
                    node.get_logger().info(f"captured {index}/{len(scenes)} scenes -> {output_root}")
        print(output_root)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
