#!/usr/bin/env python3
"""Build pseudo-color gated images from STF slices and export YOLO labels.

The STF subset stores synchronized gated0/gated1/gated2 grayscale images plus
KITTI-style labels. This tool makes a training/testing folder without touching
the ROS workspace code or the source dataset.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2


DEFAULT_CLASS_MAP = {
    "passengercar": "vehicle",
    "passengercar_is_group": "vehicle",
    "largevehicle": "vehicle",
    "vehicle": "vehicle",
    "ridablevehicle": "vehicle",
    "pedestrian": "pedestrian",
    "pedestrian_is_group": "pedestrian",
    "obstacle": "obstacle",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert a Seeing Through Fog gated subset to YOLO format.")
    parser.add_argument("--dataset", default="/home/hu/STF_Dataset", help="STF subset root directory.")
    parser.add_argument("--out", required=True, help="Output YOLO directory.")
    parser.add_argument("--labels-dir", default="gated_labels_TMPv2", help="KITTI-style label directory under dataset.")
    parser.add_argument("--split", default="train", choices=["train", "val", "test"], help="Output split name.")
    parser.add_argument("--val-every", type=int, default=0, help="If >0, every Nth sample is written to val.")
    parser.add_argument(
        "--class-mode",
        default="mapped",
        choices=["mapped", "generic"],
        help="mapped keeps vehicle/pedestrian/obstacle; generic maps every valid class to gated_object.",
    )
    parser.add_argument("--min-box-size", type=float, default=4.0, help="Drop boxes smaller than this in width or height.")
    parser.add_argument("--limit", type=int, default=0, help="Optional maximum number of images to export.")
    return parser.parse_args()


def read_gray(path: Path):
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(path)
    return image


def make_pseudo(near, mid, far):
    # OpenCV writes BGR. Put far/mid/near into B/G/R so color encodes range.
    return cv2.merge([far, mid, near])


def yolo_line(cls_id: int, box: tuple[float, float, float, float], width: int, height: int) -> str | None:
    x1, y1, x2, y2 = box
    x1 = max(0.0, min(float(width), x1))
    x2 = max(0.0, min(float(width), x2))
    y1 = max(0.0, min(float(height), y1))
    y2 = max(0.0, min(float(height), y2))
    bw = x2 - x1
    bh = y2 - y1
    if bw <= 1.0 or bh <= 1.0:
        return None
    cx = (x1 + 0.5 * bw) / width
    cy = (y1 + 0.5 * bh) / height
    return f"{cls_id} {cx:.6f} {cy:.6f} {bw / width:.6f} {bh / height:.6f}"


def parse_label_file(path: Path, class_to_id: dict[str, int], class_mode: str, min_box_size: float, width: int, height: int):
    labels = []
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) < 8:
            continue
        source_class = parts[0].lower()
        if source_class == "dontcare":
            continue
        if class_mode == "generic":
            target_class = "gated_object"
        else:
            target_class = DEFAULT_CLASS_MAP.get(source_class)
            if target_class is None:
                continue
        try:
            x1, y1, x2, y2 = map(float, parts[4:8])
        except ValueError:
            continue
        if x2 - x1 < min_box_size or y2 - y1 < min_box_size:
            continue
        line = yolo_line(class_to_id[target_class], (x1, y1, x2, y2), width, height)
        if line:
            labels.append(line)
    return labels


def write_data_yaml(out_root: Path, classes: list[str]) -> None:
    names = ", ".join(f"'{name}'" for name in classes)
    (out_root / "data.yaml").write_text(
        f"path: {out_root.resolve()}\ntrain: images/train\nval: images/val\ntest: images/test\nnames: [{names}]\n",
        encoding="utf-8",
    )


def main() -> None:
    args = parse_args()
    dataset = Path(args.dataset).expanduser()
    out_root = Path(args.out).expanduser()
    label_dir = dataset / args.labels_dir
    near_dir = dataset / "gated0_rect8"
    mid_dir = dataset / "gated1_rect8"
    far_dir = dataset / "gated2_rect8"

    classes = ["gated_object"] if args.class_mode == "generic" else ["vehicle", "pedestrian", "obstacle"]
    class_to_id = {name: idx for idx, name in enumerate(classes)}

    exported = {"train": 0, "val": 0, "test": 0}
    boxes = {"train": 0, "val": 0, "test": 0}
    missing = []

    label_files = sorted(label_dir.glob("*.txt"))
    if args.limit > 0:
        label_files = label_files[: args.limit]

    for index, label_path in enumerate(label_files):
        stem = label_path.stem
        near_path = near_dir / f"{stem}.png"
        mid_path = mid_dir / f"{stem}.png"
        far_path = far_dir / f"{stem}.png"
        if not (near_path.exists() and mid_path.exists() and far_path.exists()):
            missing.append(stem)
            continue

        split = args.split
        if args.val_every > 0 and (index + 1) % args.val_every == 0:
            split = "val"

        near = read_gray(near_path)
        mid = read_gray(mid_path)
        far = read_gray(far_path)
        pseudo = make_pseudo(near, mid, far)
        height, width = near.shape[:2]
        label_lines = parse_label_file(label_path, class_to_id, args.class_mode, args.min_box_size, width, height)
        if not label_lines:
            continue

        image_out = out_root / "images" / split / f"{stem}.jpg"
        label_out = out_root / "labels" / split / f"{stem}.txt"
        image_out.parent.mkdir(parents=True, exist_ok=True)
        label_out.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(image_out), pseudo, [int(cv2.IMWRITE_JPEG_QUALITY), 94])
        label_out.write_text("\n".join(label_lines) + "\n", encoding="utf-8")
        exported[split] += 1
        boxes[split] += len(label_lines)

    write_data_yaml(out_root, classes)
    summary = {
        "dataset": str(dataset),
        "output": str(out_root),
        "class_mode": args.class_mode,
        "classes": classes,
        "exported_images": exported,
        "exported_boxes": boxes,
        "missing_triplets": len(missing),
        "missing_examples": missing[:20],
    }
    (out_root / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
