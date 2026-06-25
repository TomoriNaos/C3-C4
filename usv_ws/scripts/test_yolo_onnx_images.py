#!/usr/bin/env python3
"""Run a YOLO ONNX model on a directory of images and print compact results."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="Path to ONNX model.")
    parser.add_argument("--images", required=True, help="Image file or directory.")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold.")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU threshold.")
    parser.add_argument("--limit", type=int, default=24, help="Maximum images to test.")
    parser.add_argument("--save-dir", default="", help="Optional directory for annotated images.")
    return parser.parse_args()


def model_names(session: ort.InferenceSession) -> list[str]:
    raw = session.get_modelmeta().custom_metadata_map.get("names", "")
    if not raw:
        return []
    try:
        parsed = ast.literal_eval(raw)
    except (SyntaxError, ValueError):
        return []
    if isinstance(parsed, dict):
        return [str(parsed[key]) for key in sorted(parsed)]
    if isinstance(parsed, list):
        return [str(item) for item in parsed]
    return []


def image_paths(path: Path, limit: int) -> list[Path]:
    if path.is_file():
        return [path]
    files = [p for p in sorted(path.iterdir()) if p.suffix.lower() in IMAGE_EXTENSIONS]
    return files[:limit]


def preprocess(image: np.ndarray, width: int, height: int) -> np.ndarray:
    resized = cv2.resize(image, (width, height))
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    tensor = rgb.astype(np.float32) / 255.0
    return np.transpose(tensor, (2, 0, 1))[None, ...]


def detections_from_output(
    output: np.ndarray,
    image_shape: tuple[int, int],
    input_size: tuple[int, int],
    conf: float,
) -> tuple[list[list[int]], list[float], list[int], float]:
    data = output[0] if output.ndim == 3 else output
    if data.shape[0] < data.shape[1] and data.shape[0] <= 256:
        data = data.T
    if data.shape[1] <= 4:
        return [], [], [], 0.0

    image_h, image_w = image_shape
    input_w, input_h = input_size
    x_factor = image_w / float(input_w)
    y_factor = image_h / float(input_h)

    boxes: list[list[int]] = []
    scores: list[float] = []
    classes: list[int] = []
    max_score = 0.0
    for row in data:
        class_scores = row[4:]
        class_id = int(np.argmax(class_scores))
        score = float(class_scores[class_id])
        max_score = max(max_score, score)
        if score < conf:
            continue
        cx, cy, width, height = row[:4]
        left = int(max(0, (cx - width * 0.5) * x_factor))
        top = int(max(0, (cy - height * 0.5) * y_factor))
        right = int(min(image_w, (cx + width * 0.5) * x_factor))
        bottom = int(min(image_h, (cy + height * 0.5) * y_factor))
        if right <= left or bottom <= top:
            continue
        boxes.append([left, top, right - left, bottom - top])
        scores.append(score)
        classes.append(class_id)
    return boxes, scores, classes, max_score


def main() -> None:
    args = parse_args()
    session = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    input_info = session.get_inputs()[0]
    input_name = input_info.name
    _, _, input_h, input_w = [int(v) for v in input_info.shape]
    output_name = session.get_outputs()[0].name
    names = model_names(session)

    save_dir = Path(args.save_dir).expanduser() if args.save_dir else None
    if save_dir:
        save_dir.mkdir(parents=True, exist_ok=True)

    print(f"model={args.model}")
    print(f"input={input_w}x{input_h} names={names}")
    for path in image_paths(Path(args.images).expanduser(), args.limit):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        tensor = preprocess(image, input_w, input_h)
        output = session.run([output_name], {input_name: tensor})[0]
        boxes, scores, classes, max_score = detections_from_output(
            output, image.shape[:2], (input_w, input_h), args.conf)
        keep = cv2.dnn.NMSBoxes(boxes, scores, args.conf, args.iou)
        keep_indices = np.array(keep).reshape(-1).tolist() if len(keep) else []
        labels = [names[classes[i]] if classes[i] < len(names) else str(classes[i]) for i in keep_indices]
        best = max((scores[i] for i in keep_indices), default=0.0)
        print(f"{path.name}: detections={len(keep_indices)} best={best:.3f} max_raw={max_score:.3f} labels={labels}")

        if save_dir:
            annotated = image.copy()
            for i in keep_indices:
                x, y, w, h = boxes[i]
                label = names[classes[i]] if classes[i] < len(names) else str(classes[i])
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 220, 255), 2)
                cv2.putText(
                    annotated, f"{label} {scores[i]:.2f}", (x, max(18, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2, cv2.LINE_AA)
            cv2.imwrite(str(save_dir / path.name), annotated)


if __name__ == "__main__":
    main()
