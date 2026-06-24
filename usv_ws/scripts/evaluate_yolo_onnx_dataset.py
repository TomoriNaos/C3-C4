#!/usr/bin/env python3
"""Evaluate a YOLO ONNX detector on a YOLO-format dataset and save visual checks."""

from __future__ import annotations

import argparse
import ast
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
import yaml


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


@dataclass
class Box:
    cls: int
    score: float
    xyxy: tuple[float, float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument("--data", required=True, help="YOLO data.yaml")
    parser.add_argument("--split", default="train")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.50, help="IoU threshold for TP matching.")
    parser.add_argument("--nms-iou", type=float, default=0.45)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--save-dir", default="/tmp/usv_yolo_eval")
    parser.add_argument("--save-examples", type=int, default=24)
    parser.add_argument(
        "--example-order",
        choices=("worst", "best"),
        default="worst",
        help="Save examples with the most or fewest FP/FN first.",
    )
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


def split_image_dir(data_yaml: Path, cfg: dict, split: str) -> Path:
    root = data_yaml.parent
    raw = cfg.get(split)
    candidates: list[Path] = []
    if raw:
        candidates.append((root / raw).resolve())
    if split == "val":
        candidates.extend([(root / "valid" / "images").resolve(), (root / "val" / "images").resolve()])
    candidates.append((root / split / "images").resolve())
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def load_gt(label_path: Path, width: int, height: int) -> list[Box]:
    boxes: list[Box] = []
    if not label_path.exists():
        return boxes
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cls, cx, cy, w, h = [float(v) for v in parts[:5]]
        x1 = (cx - w * 0.5) * width
        y1 = (cy - h * 0.5) * height
        x2 = (cx + w * 0.5) * width
        y2 = (cy + h * 0.5) * height
        boxes.append(Box(int(cls), 1.0, (x1, y1, x2, y2)))
    return boxes


def preprocess(image: np.ndarray, input_w: int, input_h: int) -> np.ndarray:
    resized = cv2.resize(image, (input_w, input_h))
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    tensor = rgb.astype(np.float32) / 255.0
    return np.transpose(tensor, (2, 0, 1))[None, ...]


def parse_predictions(
    output: np.ndarray,
    image_shape: tuple[int, int],
    input_size: tuple[int, int],
    model_to_dataset_class: dict[int, int],
    conf: float,
    nms_iou: float,
) -> list[Box]:
    data = output[0] if output.ndim == 3 else output
    if data.shape[0] < data.shape[1] and data.shape[0] <= 256:
        data = data.T
    if data.shape[1] <= 4:
        return []
    image_h, image_w = image_shape
    input_w, input_h = input_size
    x_factor = image_w / float(input_w)
    y_factor = image_h / float(input_h)

    boxes_xywh: list[list[int]] = []
    boxes_xyxy: list[tuple[float, float, float, float]] = []
    scores: list[float] = []
    classes: list[int] = []
    for row in data:
        class_scores = row[4:]
        model_class = int(np.argmax(class_scores))
        if model_class not in model_to_dataset_class:
            continue
        score = float(class_scores[model_class])
        if score < conf:
            continue
        cx, cy, width, height = row[:4]
        x1 = max(0.0, float((cx - width * 0.5) * x_factor))
        y1 = max(0.0, float((cy - height * 0.5) * y_factor))
        x2 = min(float(image_w), float((cx + width * 0.5) * x_factor))
        y2 = min(float(image_h), float((cy + height * 0.5) * y_factor))
        if x2 <= x1 or y2 <= y1:
            continue
        boxes_xyxy.append((x1, y1, x2, y2))
        boxes_xywh.append([int(x1), int(y1), int(x2 - x1), int(y2 - y1)])
        scores.append(score)
        classes.append(model_to_dataset_class[model_class])
    keep = cv2.dnn.NMSBoxes(boxes_xywh, scores, conf, nms_iou)
    keep_indices = np.array(keep).reshape(-1).tolist() if len(keep) else []
    return [Box(classes[i], scores[i], boxes_xyxy[i]) for i in keep_indices]


def iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    denom = area_a + area_b - inter
    return inter / denom if denom > 0.0 else 0.0


def match(preds: list[Box], gts: list[Box], iou_thr: float) -> tuple[int, int, int, set[int], set[int]]:
    matched_gt: set[int] = set()
    matched_pred: set[int] = set()
    for pred_index, pred in sorted(enumerate(preds), key=lambda item: item[1].score, reverse=True):
        best_iou = 0.0
        best_gt = -1
        for gt_index, gt in enumerate(gts):
            if gt_index in matched_gt or gt.cls != pred.cls:
                continue
            value = iou(pred.xyxy, gt.xyxy)
            if value > best_iou:
                best_iou = value
                best_gt = gt_index
        if best_gt >= 0 and best_iou >= iou_thr:
            matched_gt.add(best_gt)
            matched_pred.add(pred_index)
    tp = len(matched_gt)
    fp = len(preds) - len(matched_pred)
    fn = len(gts) - len(matched_gt)
    return tp, fp, fn, matched_pred, matched_gt


def draw_example(
    image: np.ndarray,
    preds: list[Box],
    gts: list[Box],
    matched_pred: set[int],
    matched_gt: set[int],
    names: list[str],
    out_path: Path,
) -> None:
    canvas = image.copy()
    for i, gt in enumerate(gts):
        color = (0, 210, 0) if i in matched_gt else (0, 0, 255)
        x1, y1, x2, y2 = [int(v) for v in gt.xyxy]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        cv2.putText(canvas, f"GT {names[gt.cls]}", (x1, max(16, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    for i, pred in enumerate(preds):
        color = (0, 255, 255) if i in matched_pred else (255, 0, 255)
        x1, y1, x2, y2 = [int(v) for v in pred.xyxy]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            canvas, f"P {names[pred.cls]} {pred.score:.2f}", (x1, min(canvas.shape[0] - 8, y2 + 16)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    cv2.imwrite(str(out_path), canvas)


def main() -> None:
    args = parse_args()
    data_yaml = Path(args.data).expanduser()
    cfg = yaml.safe_load(data_yaml.read_text())
    dataset_names = list(cfg["names"])
    image_dir = split_image_dir(data_yaml, cfg, args.split)
    label_dir = Path(str(image_dir).replace("/images", "/labels"))
    image_paths = [p for p in sorted(image_dir.iterdir()) if p.suffix.lower() in IMAGE_EXTENSIONS]
    if args.limit > 0:
        image_paths = image_paths[: args.limit]

    session = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    input_info = session.get_inputs()[0]
    _, _, input_h, input_w = [int(v) for v in input_info.shape]
    output_name = session.get_outputs()[0].name
    names = model_names(session)
    model_to_dataset_class = {i: dataset_names.index(name) for i, name in enumerate(names) if name in dataset_names}

    out_dir = Path(args.save_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    totals = {
        "tp": 0,
        "fp": 0,
        "fn": 0,
        "images": len(image_paths),
        "boxes": 0,
        "predictions": 0,
    }
    per_class = {i: {"tp": 0, "fp": 0, "fn": 0, "gt": 0, "pred": 0} for i in range(len(dataset_names))}
    examples: list[tuple[int, Path, np.ndarray, list[Box], list[Box], set[int], set[int]]] = []

    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        gts = load_gt(label_dir / f"{path.stem}.txt", image.shape[1], image.shape[0])
        tensor = preprocess(image, input_w, input_h)
        output = session.run([output_name], {input_info.name: tensor})[0]
        preds = parse_predictions(output, image.shape[:2], (input_w, input_h), model_to_dataset_class, args.conf, args.nms_iou)
        tp, fp, fn, matched_pred, matched_gt = match(preds, gts, args.iou)
        totals["tp"] += tp
        totals["fp"] += fp
        totals["fn"] += fn
        totals["boxes"] += len(gts)
        totals["predictions"] += len(preds)
        for i, gt in enumerate(gts):
            per_class[gt.cls]["gt"] += 1
            if i in matched_gt:
                per_class[gt.cls]["tp"] += 1
            else:
                per_class[gt.cls]["fn"] += 1
        for i, pred in enumerate(preds):
            per_class[pred.cls]["pred"] += 1
            if i not in matched_pred:
                per_class[pred.cls]["fp"] += 1
        severity = fn * 10 + fp
        examples.append((severity, path, image, preds, gts, matched_pred, matched_gt))

    precision = totals["tp"] / max(1, totals["tp"] + totals["fp"])
    recall = totals["tp"] / max(1, totals["tp"] + totals["fn"])
    summary = {
        "model": str(args.model),
        "data": str(data_yaml),
        "split": args.split,
        "dataset_names": dataset_names,
        "model_names": names,
        "model_to_dataset_class": model_to_dataset_class,
        "totals": totals,
        "precision_iou50": precision,
        "recall_iou50": recall,
        "per_class": {
            dataset_names[i]: {
                **stats,
                "precision": stats["tp"] / max(1, stats["tp"] + stats["fp"]),
                "recall": stats["tp"] / max(1, stats["tp"] + stats["fn"]),
            }
            for i, stats in per_class.items()
        },
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    reverse = args.example_order == "worst"
    for rank, item in enumerate(sorted(examples, key=lambda x: x[0], reverse=reverse)[: args.save_examples]):
        _, path, image, preds, gts, matched_pred, matched_gt = item
        draw_example(image, preds, gts, matched_pred, matched_gt, dataset_names, out_dir / f"{rank:02d}_{path.name}")


if __name__ == "__main__":
    main()
