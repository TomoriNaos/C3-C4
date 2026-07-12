#!/usr/bin/env python3
"""Capture paired normal/dehazed and gated pseudo-color images from Gazebo."""

from __future__ import annotations

import csv
import math
import os
import random
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image


OUT_ROOT = Path(os.environ.get("C3EXAMPLE_DIR", str(Path.home() / "c3example"))).expanduser()

CLASS_NAMES = [
    "small_fishing_boat",
    "moving_vessel",
    "research_platform",
    "service_boat",
    "survey_boat",
    "cargo_ship_far",
    "anchored_tanker",
    "obstacle",
]

PRIMARY_MODELS = [
    ("small_fishing_boat", 0, 10.0, 0.34),
    ("moving_vessel", 1, 11.0, 0.42),
    ("research_platform", 2, 24.0, 2.0),
    ("service_boat", 3, 11.5, 0.38),
    ("survey_boat", 4, 11.0, 0.36),
    ("cargo_ship_far", 5, 26.0, 1.05),
    ("anchored_tanker", 6, 28.0, 1.05),
]

OBSTACLE_MODELS = [
    ("fishnet_buoy", 7, 10.0, 0.30),
    ("floating_obstacle", 7, 10.0, 0.24),
    ("drift_debris", 7, 9.5, 0.16),
    ("floating_container", 7, 11.5, 0.28),
    ("net_line_a", 7, 11.5, 0.10),
    ("channel_buoy_north", 7, 10.5, 0.55),
    ("channel_buoy_south", 7, 10.5, 0.55),
    ("navigation_marker_port", 7, 10.5, 0.55),
    ("navigation_marker_starboard", 7, 10.5, 0.55),
]

ALL_MODELS = PRIMARY_MODELS + OBSTACLE_MODELS
FOCUS_COMPLEX_NAMES = {"small_fishing_boat", "moving_vessel", "research_platform", "service_boat", "survey_boat"}
FOCUS_COMPLEX_MODELS = [item for item in PRIMARY_MODELS if item[0] in FOCUS_COMPLEX_NAMES]
RARE_COMPLEX_MODELS = [item for item in PRIMARY_MODELS if item[0] not in FOCUS_COMPLEX_NAMES]


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def image_to_bgr(msg: Image) -> np.ndarray | None:
    if msg.encoding in ("rgb8", "R8G8B8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if msg.encoding == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
    if msg.encoding in ("rgba8", "bgra8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
        code = cv2.COLOR_RGBA2BGR if msg.encoding == "rgba8" else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(arr, code)
    return None


def image_to_depth(msg: Image) -> np.ndarray | None:
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
    if msg.encoding in ("16UC1", "mono16"):
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) * 0.001
    return None


def dehaze_like_runtime(bgr: np.ndarray, depth: np.ndarray | None) -> np.ndarray:
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(l)
    out = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    if depth is not None and depth.shape[:2] == bgr.shape[:2]:
      valid = np.isfinite(depth) & (depth > 0.2)
      if valid.any():
          haze = np.clip(np.exp(-0.018 * depth), 0.18, 1.0).astype(np.float32)
          haze = cv2.GaussianBlur(haze, (0, 0), 2.0)
          haze3 = np.repeat(haze[:, :, None], 3, axis=2)
          f = out.astype(np.float32) / 255.0
          air = np.percentile(f.reshape(-1, 3), 99.5, axis=0)
          rec = np.clip((f - air) / np.maximum(haze3, 0.18) + air, 0.0, 1.0)
          out = np.clip(0.65 * rec * 255.0 + 0.35 * out.astype(np.float32), 0, 255).astype(np.uint8)
    return out


def gated_pseudocolor(bgr: np.ndarray, depth: np.ndarray | None) -> np.ndarray:
    if depth is None or depth.shape[:2] != bgr.shape[:2]:
        return bgr.copy()
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
    d = depth.copy()
    d[~np.isfinite(d)] = 0.0

    def gate(near: float, far: float) -> np.ndarray:
        center = 0.5 * (near + far)
        half = max(0.5 * (far - near), 1e-3)
        w = np.clip(1.0 - np.abs(d - center) / half, 0.0, 1.0)
        w[(d < near) | (d > far)] = 0.0
        return gray * w

    def boost(channel: np.ndarray) -> np.ndarray:
        nonzero = channel[channel > 1e-4]
        if nonzero.size:
            ref = float(np.percentile(nonzero, 98.0))
            if ref > 1e-4:
                channel = np.clip(channel * min(3.8, 0.78 / ref), 0.0, 1.0)
        return np.power(channel, 0.72)

    near = boost(gate(2.0, 18.0))
    mid = boost(gate(12.0, 42.0))
    far = boost(gate(32.0, 88.0))
    rgb = np.dstack([near, mid, far])
    rgb = np.clip(rgb * 255.0, 0, 255).astype(np.uint8)
    pseudo = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    return cv2.addWeighted(pseudo, 0.72, bgr, 0.28, 8.0)


class CaptureNode:
    def __init__(self) -> None:
        self.node = rclpy.create_node("capture_c3example_dataset")
        self.client = self.node.create_client(SetEntityState, "/set_entity_state")
        self.frames: dict[str, Image] = {}
        topics = [
            ("/gated_camera/image_raw", "ship_rgb"),
            ("/depth_camera/depth/image_raw", "ship_depth"),
            ("/uav/gated_camera/image_raw", "uav_rgb"),
            ("/uav/gated_camera/depth/image_raw", "uav_depth"),
        ]
        for topic, key in topics:
            self.node.create_subscription(Image, topic, lambda msg, k=key: self._on_image(k, msg), 10)

    def _on_image(self, key: str, msg: Image) -> None:
        self.frames[key] = msg

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_ready(self) -> None:
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for /set_entity_state")
        deadline = time.time() + 20.0
        while time.time() < deadline and rclpy.ok():
            self.spin_for(0.2)
            if {"ship_rgb", "ship_depth", "uav_rgb", "uav_depth"}.issubset(self.frames):
                return
        raise RuntimeError("Camera topics are not ready")

    def set_model(self, name: str, x: float, y: float, z: float, yaw: float, pitch: float = 0.0) -> None:
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.reference_frame = "world"
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation = quat_from_rpy(0.0, pitch, yaw)
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

    def hide_all(self) -> None:
        self.set_model("wamv", 0.0, 0.0, 0.32, 0.0)
        for idx, (name, _, _, z) in enumerate(ALL_MODELS):
            self.set_model(name, -600.0 - idx * 6.0, 350.0, -20.0 - z, 0.0)
        self.set_model("scout_uav", -500.0, -500.0, -30.0, 0.0)

    def save_pair(
        self,
        normal_path: Path,
        gated_path: Path,
        uav: bool = False,
        usv_x: float = 0.0,
        usv_y: float = 0.0,
        usv_yaw: float = 0.0,
    ) -> None:
        self.set_model("wamv", usv_x, usv_y, 0.32, usv_yaw)
        self.spin_for(0.35)
        rgb_key = "uav_rgb" if uav else "ship_rgb"
        depth_key = "uav_depth" if uav else "ship_depth"
        bgr = image_to_bgr(self.frames[rgb_key])
        depth = image_to_depth(self.frames[depth_key])
        if bgr is None:
            raise RuntimeError(f"Unsupported image encoding on {rgb_key}: {self.frames[rgb_key].encoding}")
        if depth is not None and depth.shape[:2] != bgr.shape[:2]:
            depth = cv2.resize(depth, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)
        normal = dehaze_like_runtime(bgr, depth)
        gated = gated_pseudocolor(normal, depth)
        normal_path.parent.mkdir(parents=True, exist_ok=True)
        gated_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(normal_path), normal)
        cv2.imwrite(str(gated_path), gated)


def write_classes() -> None:
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    (OUT_ROOT / "classes.txt").write_text("\n".join(CLASS_NAMES) + "\n", encoding="utf-8")
    (OUT_ROOT / "README.txt").write_text(
        "类别顺序:\n"
        "0 small_fishing_boat\n1 moving_vessel\n2 research_platform\n3 service_boat\n"
        "4 survey_boat\n5 cargo_ship_far\n6 anchored_tanker\n7 obstacle\n\n"
        "normal/ 为普通去雾相机图像，gated/ 为门控伪彩色图像。\n"
        "complex 中同名图片为一一对应的同一场景。\n"
        "标注时必须标出每张图里所有可见目标，否则漏标目标会被训练成背景。\n"
        "普通相机和门控相机按各自图片分别标注：普通相机可见但门控图不可辨认时，门控图不要强行标。\n",
        encoding="utf-8",
    )


def capture_single(node: CaptureNode, manifest: list[dict[str, object]]) -> None:
    for name, cls, distance, z in PRIMARY_MODELS:
        for view in range(6):
            yaw = math.radians(view * 60)
            node.hide_all()
            node.set_model(name, distance, 0.0, z, yaw)
            stem = f"{cls:02d}_{name}_yaw{view * 60:03d}.png"
            node.save_pair(OUT_ROOT / "normal" / "single" / stem, OUT_ROOT / "gated" / "single" / stem)
            manifest.append({"subset": "single", "file": stem, "model": name, "class_id": cls, "class_name": CLASS_NAMES[cls]})

    for name, cls, distance, z in OBSTACLE_MODELS:
        for view, yaw_deg in enumerate((0, 90)):
            node.hide_all()
            node.set_model(name, distance, 0.0, z, math.radians(yaw_deg))
            stem = f"{cls:02d}_{name}_yaw{yaw_deg:03d}.png"
            node.save_pair(OUT_ROOT / "normal" / "single" / stem, OUT_ROOT / "gated" / "single" / stem)
            manifest.append({"subset": "single", "file": stem, "model": name, "class_id": cls, "class_name": CLASS_NAMES[cls]})


def capture_complex(node: CaptureNode, manifest: list[dict[str, object]]) -> None:
    rng = random.Random(20260712)
    for idx in range(100):
        node.hide_all()
        primary = FOCUS_COMPLEX_MODELS[idx % len(FOCUS_COMPLEX_MODELS)]
        scene = [primary]
        if rng.random() < 0.72:
            candidates = [item for item in FOCUS_COMPLEX_MODELS if item[0] != primary[0]]
            scene.append(rng.choice(candidates))
        if rng.random() < 0.28:
            scene.append(rng.choice(OBSTACLE_MODELS))
        elif rng.random() < 0.10:
            scene.append(rng.choice(RARE_COMPLEX_MODELS))
        scene = scene[:3]
        visible = []
        base_x = 12.0 + rng.uniform(-1.2, 2.4)
        for j, (name, cls, _, z) in enumerate(scene):
            size_offset = 10.0 if cls in (2, 5) else (7.0 if cls == 6 else 0.0)
            x = base_x + size_offset + 4.8 * j + rng.uniform(-0.8, 1.1)
            y = (-1.25 + j * 1.05) + rng.uniform(-0.45, 0.45)
            if j == 1 and rng.random() < 0.65:
                y *= 0.35
            yaw = rng.uniform(-math.pi, math.pi)
            node.set_model(name, x, y, z, yaw)
            visible.append(f"{name}:{cls}")
        usv_x = rng.uniform(-0.45, 0.45)
        usv_y = rng.uniform(-0.35, 0.35)
        usv_yaw = rng.uniform(math.radians(-4.0), math.radians(4.0))
        stem = f"complex_{idx:03d}.png"
        node.save_pair(
            OUT_ROOT / "normal" / "complex" / stem,
            OUT_ROOT / "gated" / "complex" / stem,
            usv_x=usv_x,
            usv_y=usv_y,
            usv_yaw=usv_yaw,
        )
        manifest.append({"subset": "complex", "file": stem, "models": ";".join(visible)})


def capture_uav_topdown(node: CaptureNode, manifest: list[dict[str, object]]) -> None:
    for name, cls, distance, z in PRIMARY_MODELS:
        for view in range(6):
            yaw = math.radians(view * 60)
            node.hide_all()
            node.set_model(name, 20.0, 0.0, z, yaw)
            altitude = 46.0 if cls in (2, 5, 6) else 32.0
            node.set_model("scout_uav", 19.0, 0.0, altitude, math.pi, pitch=math.radians(80.0))
            stem = f"{cls:02d}_{name}_top_yaw{view * 60:03d}.png"
            node.save_pair(OUT_ROOT / "normal" / "uav_topdown" / stem, OUT_ROOT / "gated" / "uav_topdown" / stem, uav=True)
            manifest.append({"subset": "uav_topdown", "file": stem, "model": name, "class_id": cls, "class_name": CLASS_NAMES[cls]})

    for name, cls, distance, z in OBSTACLE_MODELS:
        for view, yaw_deg in enumerate((0, 90)):
            node.hide_all()
            node.set_model(name, 20.0, 0.0, z, math.radians(yaw_deg))
            node.set_model("scout_uav", 19.0, 0.0, 30.0, math.pi, pitch=math.radians(80.0))
            stem = f"{cls:02d}_{name}_top_yaw{yaw_deg:03d}.png"
            node.save_pair(OUT_ROOT / "normal" / "uav_topdown" / stem, OUT_ROOT / "gated" / "uav_topdown" / stem, uav=True)
            manifest.append({"subset": "uav_topdown", "file": stem, "model": name, "class_id": cls, "class_name": CLASS_NAMES[cls]})


def main() -> None:
    rclpy.init()
    write_classes()
    node = CaptureNode()
    node.wait_ready()
    manifest: list[dict[str, object]] = []
    capture_single(node, manifest)
    capture_complex(node, manifest)
    capture_uav_topdown(node, manifest)
    with (OUT_ROOT / "manifest.csv").open("w", newline="", encoding="utf-8") as f:
        keys = sorted({k for row in manifest for k in row.keys()})
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(manifest)
    node.hide_all()
    node.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
