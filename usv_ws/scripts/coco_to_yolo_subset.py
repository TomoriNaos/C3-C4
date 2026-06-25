#!/usr/bin/env python3
"""Convert a COCO detection dataset subset to YOLO labels.

The script is intentionally standalone: it does not import ROS packages and does
not touch the simulation workspace except for the output directory you choose.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
from collections import Counter, defaultdict
from pathlib import Path


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def norm_name(name: str) -> str:
    return " ".join(name.replace("_", " ").replace("-", " ").lower().split())


def parse_maps(items: list[str]) -> dict[str, str]:
    mapping: dict[str, str] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"--map must be SOURCE=TARGET, got: {item}")
        source, target = item.split("=", 1)
        source = source.strip()
        target = target.strip()
        if not source or not target:
            raise ValueError(f"--map must be SOURCE=TARGET, got: {item}")
        mapping[norm_name(source)] = target
    return mapping


def load_json_mapping(path: Path | None) -> dict[str, str]:
    if path is None:
        return {}
    with path.open("r", encoding="utf-8") as f:
        raw = json.load(f)
    if not isinstance(raw, dict):
        raise ValueError("--map-json must contain a JSON object")
    return {norm_name(str(k)): str(v) for k, v in raw.items()}


def index_images(images_root: Path) -> dict[str, Path]:
    indexed: dict[str, Path] = {}
    for path in images_root.rglob("*"):
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES:
            rel = path.relative_to(images_root).as_posix()
            indexed.setdefault(rel, path)
            indexed.setdefault(path.name, path)
    return indexed


def clip_box(
    x: float, y: float, w: float, h: float, img_w: float, img_h: float
) -> tuple[float, float, float, float] | None:
    x1 = max(0.0, x)
    y1 = max(0.0, y)
    x2 = min(img_w, x + w)
    y2 = min(img_h, y + h)
    clipped_w = x2 - x1
    clipped_h = y2 - y1
    if clipped_w <= 1.0 or clipped_h <= 1.0:
        return None
    return x1, y1, clipped_w, clipped_h


def link_or_copy(src: Path, dst: Path, symlink: bool) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() or dst.is_symlink():
        return
    if symlink:
        os.symlink(src.resolve(), dst)
    else:
        shutil.copy2(src, dst)


def write_data_yaml(path: Path, class_names: list[str]) -> None:
    names = ", ".join(f"'{name}'" for name in class_names)
    text = (
        f"path: {path.resolve()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n"
        f"names: [{names}]\n"
    )
    (path / "data.yaml").write_text(text, encoding="utf-8")


def convert(args: argparse.Namespace) -> None:
    images_root = Path(args.images).expanduser()
    annotation_path = Path(args.annotations).expanduser()
    out_root = Path(args.out).expanduser()
    split = args.split

    with annotation_path.open("r", encoding="utf-8") as f:
        coco = json.load(f)

    categories = {int(cat["id"]): str(cat["name"]) for cat in coco.get("categories", [])}
    cli_mapping = parse_maps(args.map)
    json_mapping = load_json_mapping(Path(args.map_json).expanduser() if args.map_json else None)
    category_mapping = {**json_mapping, **cli_mapping}

    if category_mapping:
        target_names = sorted(set(category_mapping.values()))
    else:
        target_names = sorted(set(categories.values()))
    class_to_id = {name: i for i, name in enumerate(target_names)}

    image_index = index_images(images_root)
    image_meta = {int(img["id"]): img for img in coco.get("images", [])}
    anns_by_image: dict[int, list[dict]] = defaultdict(list)
    skipped_categories = Counter()

    for ann in coco.get("annotations", []):
        if ann.get("iscrowd", 0):
            continue
        source_name = categories.get(int(ann["category_id"]), "")
        if category_mapping:
            target_name = category_mapping.get(norm_name(source_name))
            if target_name is None:
                skipped_categories[source_name] += 1
                continue
        else:
            target_name = source_name
        ann = dict(ann)
        ann["_target_name"] = target_name
        anns_by_image[int(ann["image_id"])].append(ann)

    output_images = out_root / "images" / split
    output_labels = out_root / "labels" / split
    output_images.mkdir(parents=True, exist_ok=True)
    output_labels.mkdir(parents=True, exist_ok=True)

    exported_images = 0
    exported_boxes = 0
    missing_images = []
    class_counts = Counter()

    for image_id, anns in sorted(anns_by_image.items()):
        img = image_meta.get(image_id)
        if img is None:
            continue
        file_name = str(img["file_name"])
        src = image_index.get(file_name) or image_index.get(Path(file_name).name)
        if src is None:
            missing_images.append(file_name)
            continue

        width = float(img["width"])
        height = float(img["height"])
        dst_image = output_images / Path(file_name).name
        label_path = output_labels / (dst_image.stem + ".txt")

        lines = []
        for ann in anns:
            clipped = clip_box(*map(float, ann["bbox"]), width, height)
            if clipped is None:
                continue
            x, y, w, h = clipped
            xc = (x + w / 2.0) / width
            yc = (y + h / 2.0) / height
            nw = w / width
            nh = h / height
            target_name = ann["_target_name"]
            class_id = class_to_id[target_name]
            lines.append(f"{class_id} {xc:.6f} {yc:.6f} {nw:.6f} {nh:.6f}")
            class_counts[target_name] += 1

        if not lines:
            continue
        label_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        link_or_copy(src, dst_image, args.symlink)
        exported_images += 1
        exported_boxes += len(lines)

    write_data_yaml(out_root, target_names)
    summary = {
        "annotation_file": str(annotation_path),
        "images_root": str(images_root),
        "output": str(out_root),
        "split": split,
        "classes": target_names,
        "exported_images": exported_images,
        "exported_boxes": exported_boxes,
        "class_counts": dict(class_counts),
        "skipped_categories": dict(skipped_categories),
        "missing_images": missing_images[:50],
        "missing_images_count": len(missing_images),
    }
    (out_root / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"Exported {exported_images} images and {exported_boxes} boxes to {out_root}")
    print(f"Classes: {', '.join(target_names)}")
    if skipped_categories:
        print("Skipped source categories:", dict(skipped_categories))
    if missing_images:
        print(f"Missing images: {len(missing_images)}; first examples: {missing_images[:5]}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert COCO detection annotations to a YOLO dataset subset.")
    parser.add_argument("--images", required=True, help="Root directory containing source images.")
    parser.add_argument("--annotations", required=True, help="COCO annotation JSON path.")
    parser.add_argument("--out", required=True, help="Output YOLO dataset directory.")
    parser.add_argument("--split", default="train", choices=["train", "val", "test"], help="Output split name.")
    parser.add_argument(
        "--map",
        action="append",
        default=[],
        help="Category mapping SOURCE=TARGET. Repeat for multiple source classes.",
    )
    parser.add_argument("--map-json", help="JSON object mapping source category names to target category names.")
    parser.add_argument("--symlink", action="store_true", help="Symlink images instead of copying them.")
    args = parser.parse_args()
    convert(args)


if __name__ == "__main__":
    main()
