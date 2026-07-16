#!/usr/bin/env python3
"""Capture C3 supplemental images with at most three visible targets per frame."""

from __future__ import annotations

import argparse
import csv
import math
import random
import re
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
    "floating_container": 0.20,
    "research_platform": 0.45,
    "service_boat": 0.38,
    "cargo_ship_far": 0.48,
    "anchored_tanker": 0.48,
    "scout_uav": 12.0,
}

BACKGROUND_MODELS_TO_HIDE = [
    "fishnet_buoy",
    "drift_debris",
    "channel_buoy_north",
    "channel_buoy_south",
    "navigation_marker_port",
    "navigation_marker_starboard",
    "net_line_a",
]
ALL_MODELS = sorted({model for models in CLASS_TO_MODEL.values() for model in models} | set(BACKGROUND_MODELS_TO_HIDE))

DATASET_DIRS = {
    "camera": "camera/images",
    "gated_camera": "gated_camera/images",
    "plane": "plane/images",
    "plane_gated": "plane_gated/images",
}


@dataclass(frozen=True)
class SceneObject:
    class_name: str
    model_name: str
    distance: float
    lateral: float
    yaw: float


@dataclass(frozen=True)
class CaptureScene:
    objects: list[SceneObject]
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
    weighted = gray.astype(np.float32) * np.exp(-0.5 * ((clean_depth - center) / sigma) ** 2) * 1.8
    out = np.zeros(gray.shape, dtype=np.uint8)
    out[mask] = np.clip(weighted[mask], 0, 255).astype(np.uint8)
    return out


def range_view(raw_rgb: np.ndarray, depth: np.ndarray, gates: tuple[tuple[float, float], ...]) -> np.ndarray:
    if depth.shape[:2] != raw_rgb.shape[:2]:
        depth = cv2.resize(depth, (raw_rgb.shape[1], raw_rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
    gray = cv2.cvtColor(raw_rgb, cv2.COLOR_RGB2GRAY)
    pseudo_rgb = np.dstack([
        apply_gate(gray, depth, *gates[0]),
        apply_gate(gray, depth, *gates[1]),
        apply_gate(gray, depth, *gates[2]),
    ])
    return cv2.addWeighted(pseudo_rgb, 0.85, raw_rgb, 0.15, 0.0)


def slug(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_+-]+", "_", value).strip("_")


class C3ExampleCapture(Node):
    def __init__(self, output_root: Path):
        super().__init__("c3example_capture")
        self.output_root = output_root
        self.latest: dict[str, tuple[float, np.ndarray]] = {}
        self.client = self.create_client(SetEntityState, "/set_entity_state")
        self.create_subscription(Image, "/depth_camera/image_raw", lambda msg: self.on_image("ship_rgb", msg), 10)
        self.create_subscription(Image, "/depth_camera/depth/image_raw", lambda msg: self.on_image("ship_depth", msg), 10)
        self.create_subscription(Image, "/gated_camera/image_raw", lambda msg: self.on_image("ship_gated_rgb", msg), 10)
        self.create_subscription(Image, "/uav/camera/image_raw", lambda msg: self.on_image("uav_rgb", msg), 10)
        self.create_subscription(Image, "/uav/camera/depth/image_raw", lambda msg: self.on_image("uav_depth", msg), 10)
        self.create_subscription(Image, "/uav/gated_camera/image_raw", lambda msg: self.on_image("uav_gated_rgb", msg), 10)
        self.create_subscription(Image, "/uav/gated_camera/depth/image_raw", lambda msg: self.on_image("uav_gated_depth", msg), 10)

    def on_image(self, key: str, msg: Image) -> None:
        try:
            self.latest[key] = (time.monotonic(), image_msg_to_array(msg))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"{key}: {exc}")

    def wait_for_inputs(self, timeout_s: float = 30.0) -> None:
        required = {
            "ship_rgb",
            "ship_depth",
            "ship_gated_rgb",
            "uav_rgb",
            "uav_depth",
            "uav_gated_rgb",
            "uav_gated_depth",
        }
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if required.issubset(self.latest):
                return
        raise TimeoutError(f"Timed out waiting for image topics: {sorted(required - set(self.latest))}")

    def call_set_state(self, name: str, x: float, y: float, z: float, yaw: float) -> None:
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = name
        request.state.reference_frame = "world"
        request.state.pose.position.x = float(x)
        request.state.pose.position.y = float(y)
        request.state.pose.position.z = float(z)
        request.state.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or not future.result() or not future.result().success:
            status = future.result().status_message if future.done() and future.result() else "timeout"
            self.get_logger().warn(f"SetEntityState failed for {name}: {status}")

    def hide_all_targets(self) -> None:
        for index, name in enumerate(ALL_MODELS):
            self.call_set_state(name, -220.0 - index * 8.0, 140.0 + index * 6.0, MODEL_Z.get(name, 0.4), 0.0)

    def set_ship_scene(self, scene: CaptureScene) -> None:
        self.hide_all_targets()
        for obj in scene.objects:
            self.call_set_state(obj.model_name, obj.distance, obj.lateral, MODEL_Z.get(obj.model_name, 0.4), obj.yaw)
        self.call_set_state("scout_uav", -150.0, 150.0, MODEL_Z["scout_uav"], -0.75)

    def set_uav_scene(self, scene: CaptureScene) -> None:
        self.hide_all_targets()
        primary = scene.objects[0]
        center_x = 96.0 + primary.distance * 0.20
        center_y = 72.0 + primary.lateral * 0.20
        center_index = (len(scene.objects) - 1) * 0.5
        for index, obj in enumerate(scene.objects):
            offset_x = (obj.distance - primary.distance) * 0.10
            offset_y = (obj.lateral - primary.lateral) * 0.25 + (index - center_index) * 2.4
            self.call_set_state(
                obj.model_name,
                center_x + offset_x,
                center_y + offset_y,
                MODEL_Z.get(obj.model_name, 0.4),
                obj.yaw,
            )
        uav_x = center_x - scene.uav_distance
        uav_y = center_y + scene.uav_lateral
        yaw_to_scene = math.atan2(center_y - uav_y, center_x - uav_x)
        self.call_set_state("scout_uav", uav_x, uav_y, scene.uav_altitude, yaw_to_scene)

    def settle(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_copy(self, key: str) -> np.ndarray:
        return self.latest[key][1].copy()

    def save_outputs(self, scene: CaptureScene, index: int, manifest: csv.writer, outputs: dict[str, np.ndarray]) -> None:
        classes = [obj.class_name for obj in scene.objects]
        models = [obj.model_name for obj in scene.objects]
        stem = slug(f"{index:04d}_{len(scene.objects)}obj_{'+'.join(classes)}")
        for dataset, image in outputs.items():
            out_dir = self.output_root / DATASET_DIRS[dataset]
            out_dir.mkdir(parents=True, exist_ok=True)
            path = out_dir / f"{stem}.jpg"
            cv2.imwrite(str(path), cv2.cvtColor(resize_512(image), cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), 94])
            manifest.writerow([
                path.relative_to(self.output_root).as_posix(),
                dataset,
                len(scene.objects),
                ";".join(classes),
                ";".join(models),
                ";".join(f"{obj.distance:.2f}" for obj in scene.objects),
                ";".join(f"{obj.lateral:.2f}" for obj in scene.objects),
                ";".join(f"{obj.yaw:.3f}" for obj in scene.objects),
                f"{scene.uav_altitude:.2f}",
            ])

    def save_ship_outputs(self, scene: CaptureScene, index: int, manifest: csv.writer) -> None:
        ship_rgb = self.image_copy("ship_rgb")
        ship_depth = self.image_copy("ship_depth")
        ship_gated_rgb = self.image_copy("ship_gated_rgb")
        ship_gated = range_view(ship_gated_rgb, ship_depth, ((2.0, 18.0), (12.0, 42.0), (32.0, 88.0)))
        self.save_outputs(scene, index, manifest, {"camera": ship_rgb, "gated_camera": ship_gated})

    def save_uav_outputs(self, scene: CaptureScene, index: int, manifest: csv.writer) -> None:
        uav_rgb = self.image_copy("uav_rgb")
        uav_gated_rgb = self.image_copy("uav_gated_rgb")
        uav_gated_depth = self.image_copy("uav_gated_depth")
        plane_gated = range_view(uav_gated_rgb, uav_gated_depth, ((10.0, 45.0), (45.0, 160.0), (120.0, 260.0)))
        self.save_outputs(scene, index, manifest, {"plane": uav_rgb, "plane_gated": plane_gated})


def weighted_class(rng: random.Random) -> str:
    names = ["ship_far", "obstacle", "service_boat", "fishing_boat", "moving_vessel", "research_platform"]
    weights = [48, 32, 6, 6, 4, 4]
    return rng.choices(names, weights=weights, k=1)[0]


def choose_model(rng: random.Random, class_name: str, used: set[str]) -> str | None:
    available = [model for model in CLASS_TO_MODEL[class_name] if model not in used]
    if not available:
        return None
    return rng.choice(available)


def object_for_class(rng: random.Random, class_name: str, model_name: str) -> SceneObject:
    if class_name == "ship_far":
        distance = rng.uniform(48.0, 95.0)
        lateral = rng.uniform(-11.0, 11.0)
    elif class_name == "obstacle":
        distance = rng.uniform(18.0, 58.0)
        lateral = rng.uniform(-8.0, 8.0)
    else:
        distance = rng.uniform(28.0, 72.0)
        lateral = rng.uniform(-9.5, 9.5)
    return SceneObject(class_name, model_name, distance, lateral, rng.uniform(-0.75, 0.75))


def build_scenes(seed: int, scene_count: int, max_objects: int) -> list[CaptureScene]:
    rng = random.Random(seed)
    scenes: list[CaptureScene] = []
    max_objects = max(1, min(3, max_objects))
    for _ in range(scene_count):
        object_count = min(max_objects, rng.choices([1, 2, 3], weights=[48, 36, 16], k=1)[0])
        used_models: set[str] = set()
        objects: list[SceneObject] = []
        while len(objects) < object_count:
            class_name = weighted_class(rng)
            model_name = choose_model(rng, class_name, used_models)
            if model_name is None:
                fallback = [
                    (cls, model)
                    for cls, models in CLASS_TO_MODEL.items()
                    for model in models
                    if model not in used_models
                ]
                if not fallback:
                    break
                class_name, model_name = rng.choice(fallback)
            used_models.add(model_name)
            objects.append(object_for_class(rng, class_name, model_name))
        objects.sort(key=lambda obj: 0 if obj.class_name == "ship_far" else 1 if obj.class_name == "obstacle" else 2)
        scenes.append(CaptureScene(
            objects=objects,
            uav_distance=rng.uniform(32.0, 56.0),
            uav_lateral=rng.uniform(-3.0, 3.0),
            uav_altitude=rng.uniform(12.0, 20.0),
        ))
    return scenes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", default="/home/hu/c3example")
    parser.add_argument("--scenes", type=int, default=120)
    parser.add_argument("--max-objects", type=int, default=3)
    parser.add_argument("--seed", type=int, default=20260716)
    parser.add_argument("--settle", type=float, default=0.38)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_root = Path(args.output_root).expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    scenes = build_scenes(args.seed, args.scenes, args.max_objects)

    rclpy.init()
    node = C3ExampleCapture(output_root)
    try:
        if not node.client.wait_for_service(timeout_sec=20.0):
            raise TimeoutError("Service /set_entity_state is not available")
        node.wait_for_inputs()
        manifest_path = output_root / "manifest.csv"
        with manifest_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "image",
                "dataset",
                "num_objects",
                "classes",
                "models",
                "distances_m",
                "laterals_m",
                "yaws_rad",
                "uav_altitude_m",
            ])
            for index, scene in enumerate(scenes, 1):
                node.set_ship_scene(scene)
                node.settle(args.settle)
                node.save_ship_outputs(scene, index, writer)
                node.set_uav_scene(scene)
                node.settle(args.settle)
                node.save_uav_outputs(scene, index, writer)
                if index % 10 == 0 or index == len(scenes):
                    node.get_logger().info(f"captured {index}/{len(scenes)} scenes -> {output_root}")
        print(output_root)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
