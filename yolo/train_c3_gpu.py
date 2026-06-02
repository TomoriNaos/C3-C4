#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
import tempfile
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


YOLO_ENV_PYTHON = Path("/home/hu/yolo_env/bin/python")
if (
    __name__ == "__main__"
    and YOLO_ENV_PYTHON.exists()
    and Path(sys.prefix).resolve() != YOLO_ENV_PYTHON.parent.parent.resolve()
    and os.environ.get("C3_YOLO_NO_REEXEC") != "1"
):
    os.execv(str(YOLO_ENV_PYTHON), [str(YOLO_ENV_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])

from ultralytics import YOLO
import yaml


ROOT_DIR = Path("/home/hu/yolo")
DEFAULT_RUN_PREFIX = "c3_gpu_train"
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
LABEL_SUFFIXES = (".txt",)
CLASS_NAMES = ["buoy", "debris_container", "fishing_boat", "floating_obstacle", "platform", "vessel"]
LOCAL_YOLOV8N_PT_CANDIDATES = (
    ROOT_DIR / "yolov8n.pt",
    ROOT_DIR / "smboat.v4i.yolov8" / "yolov8n.pt",
)
PACKAGE_YOLOV8N_YAML = "yolov8n.yaml"


@dataclass(frozen=True)
class DatasetConfig:
    key: str
    root: Path

    @property
    def train_images(self) -> Path:
        return self.root / "train" / "images"

    @property
    def train_labels(self) -> Path:
        return self.root / "train" / "labels"


DATASETS = {
    "gated_camera": DatasetConfig("gated_camera", ROOT_DIR / "gated_camera.v2i.yolov8"),
    "vessel": DatasetConfig("vessel", ROOT_DIR / "vessel.v1i.yolov8"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GPU train script for gated_camera and vessel datasets")
    parser.add_argument("--check", action="store_true", help="Only print environment and dataset status")
    parser.add_argument(
        "--dataset",
        choices=["all", *DATASETS.keys()],
        default="all",
        help="Dataset to train. Default trains gated_camera and vessel as independent runs.",
    )
    parser.add_argument(
        "--model",
        default="auto",
        help=(
            "Model to train. Use auto for local yolov8n.pt if available, "
            "or yolov8n.yaml for offline training from scratch."
        ),
    )
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", default="0", help="Use 0 for GPU, or cpu")
    parser.add_argument("--name", default=DEFAULT_RUN_PREFIX, help="Run name prefix")
    parser.add_argument("--project", default="/home/hu/yolo/runs/detect")
    parser.add_argument("--exist-ok", action="store_true")
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument("--cache", action="store_true")
    parser.add_argument("--amp", action="store_true", help="Enable AMP. Disabled by default to avoid online AMP checks.")
    parser.add_argument("--export-onnx", action="store_true", help="Export best.pt to ONNX after training")
    return parser.parse_args()


def count_files(directory: Path, suffixes: tuple[str, ...]) -> int:
    if not directory.exists():
        return 0
    return sum(1 for path in directory.iterdir() if path.is_file() and path.suffix.lower() in suffixes)


def format_class_counts(counts: Counter[int]) -> str:
    if not counts:
        return "none"
    parts = []
    for class_id in range(len(CLASS_NAMES)):
        parts.append(f"{class_id}:{CLASS_NAMES[class_id]}={counts.get(class_id, 0)}")
    extras = sorted(class_id for class_id in counts if class_id < 0 or class_id >= len(CLASS_NAMES))
    parts.extend(f"{class_id}=INVALID:{counts[class_id]}" for class_id in extras)
    return ", ".join(parts)


def read_label_counts(labels_dir: Path) -> Counter[int]:
    counts: Counter[int] = Counter()
    if not labels_dir.exists():
        return counts

    for label_file in sorted(labels_dir.glob("*.txt")):
        for line_number, line in enumerate(label_file.read_text(encoding="utf-8").splitlines(), 1):
            stripped = line.strip()
            if not stripped:
                continue
            first_value = stripped.split(maxsplit=1)[0]
            try:
                counts[int(float(first_value))] += 1
            except ValueError as exc:
                raise RuntimeError(f"Invalid class id in {label_file}:{line_number}: {first_value!r}") from exc

    return counts


def read_label_classes(labels_dir: Path) -> set[int]:
    return set(read_label_counts(labels_dir))


def format_classes(classes: set[int]) -> str:
    return ", ".join(str(class_id) for class_id in sorted(classes)) if classes else "none"


def image_stems(images_dir: Path) -> set[str]:
    if not images_dir.exists():
        return set()
    return {path.stem for path in images_dir.iterdir() if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES}


def label_stems(labels_dir: Path) -> set[str]:
    if not labels_dir.exists():
        return set()
    return {path.stem for path in labels_dir.iterdir() if path.is_file() and path.suffix.lower() in LABEL_SUFFIXES}


def cache_paths(dataset: DatasetConfig) -> list[Path]:
    return [
        dataset.train_labels.with_suffix(".cache"),
        dataset.train_images.with_suffix(".cache"),
    ]


def read_ultralytics_cache_counts(cache_path: Path) -> tuple[int, Counter[int], int, tuple | None]:
    try:
        import numpy as np
    except Exception:
        return 0, Counter(), 0, None

    data = np.load(cache_path, allow_pickle=True).item()
    counts: Counter[int] = Counter()
    records = data.get("labels", [])
    for record in records:
        cls_values = record.get("cls", [])
        for value in np.array(cls_values).reshape(-1):
            counts[int(value)] += 1
    return len(records), counts, len(data.get("msgs", [])), data.get("results")


def print_cache_status(dataset: DatasetConfig, label_counts: Counter[int]) -> None:
    for cache_path in cache_paths(dataset):
        if not cache_path.exists():
            continue
        try:
            records, cache_counts, message_count, results = read_ultralytics_cache_counts(cache_path)
        except Exception as exc:
            print(f"  cache: {cache_path} (cannot read: {exc})")
            continue

        print(f"  cache: {cache_path}")
        print(f"    records: {records}, messages: {message_count}, results: {results}")
        print(f"    cached class counts: {format_class_counts(cache_counts)}")
        if cache_counts != label_counts:
            print("    warning: cache class counts differ from raw labels; delete cache before training.")


def remove_ultralytics_caches(dataset: DatasetConfig) -> None:
    for cache_path in cache_paths(dataset):
        if cache_path.exists():
            cache_path.unlink()
            print(f"removed stale Ultralytics cache: {cache_path}")


def selected_datasets(args: argparse.Namespace) -> list[DatasetConfig]:
    if args.dataset == "all":
        return list(DATASETS.values())
    return [DATASETS[args.dataset]]


def check_environment(datasets: list[DatasetConfig]) -> None:
    print(f"python: {sys.executable}")
    print(f"class names ({len(CLASS_NAMES)}): {CLASS_NAMES}")
    for dataset in datasets:
        labels = label_stems(dataset.train_labels)
        images = image_stems(dataset.train_images)
        label_counts = read_label_counts(dataset.train_labels)
        print(f"dataset[{dataset.key}]: {dataset.root}")
        print(f"  images: {count_files(dataset.train_images, IMAGE_SUFFIXES)}")
        print(f"  labels: {count_files(dataset.train_labels, LABEL_SUFFIXES)}")
        print(f"  label classes: {format_classes(read_label_classes(dataset.train_labels))}")
        print(f"  raw class counts: {format_class_counts(label_counts)}")
        print(f"  missing label files: {len(images - labels)}")
        print(f"  orphan label files: {len(labels - images)}")
        print_cache_status(dataset, label_counts)

    try:
        import torch

        print(f"torch: {torch.__version__}")
        print(f"cuda available: {torch.cuda.is_available()}")
        print(f"cuda version: {torch.version.cuda}")
        if torch.cuda.is_available():
            print(f"gpu 0: {torch.cuda.get_device_name(0)}")
    except Exception as exc:
        print(f"torch check failed: {exc}")


def require_dataset(dataset: DatasetConfig) -> None:
    if not dataset.train_images.exists():
        raise FileNotFoundError(f"Missing image directory: {dataset.train_images}")
    if not dataset.train_labels.exists():
        raise FileNotFoundError(f"Missing label directory: {dataset.train_labels}")
    if count_files(dataset.train_images, IMAGE_SUFFIXES) == 0:
        raise RuntimeError(f"No training images in {dataset.train_images}")
    if count_files(dataset.train_labels, LABEL_SUFFIXES) == 0:
        raise RuntimeError(f"No training labels in {dataset.train_labels}")

    images = image_stems(dataset.train_images)
    labels = label_stems(dataset.train_labels)
    missing_labels = sorted(images - labels)
    orphan_labels = sorted(labels - images)
    if missing_labels:
        raise RuntimeError(f"{dataset.key} has images without labels, first examples: {missing_labels[:5]}")
    if orphan_labels:
        raise RuntimeError(f"{dataset.key} has labels without images, first examples: {orphan_labels[:5]}")

    label_counts = read_label_counts(dataset.train_labels)
    label_classes = set(label_counts)
    invalid_classes = sorted(class_id for class_id in label_classes if class_id < 0 or class_id >= len(CLASS_NAMES))
    if invalid_classes:
        raise RuntimeError(
            f"{dataset.key} has labels outside 0-{len(CLASS_NAMES) - 1}: {invalid_classes}. "
            f"Class names are {CLASS_NAMES}."
        )

    missing_classes = sorted(set(range(len(CLASS_NAMES))) - label_classes)
    if missing_classes:
        print(f"warning: {dataset.key} labels do not contain class ids: {missing_classes}")

    print(f"{dataset.key} raw class counts: {format_class_counts(label_counts)}")


def make_runtime_data_yaml(dataset: DatasetConfig) -> Path:
    content = {
        "path": str(dataset.root),
        "train": "train/images",
        "val": "train/images",
        "test": "train/images",
        "nc": len(CLASS_NAMES),
        "names": CLASS_NAMES,
    }
    data_yaml = Path(tempfile.gettempdir()) / f"{dataset.key}_c3_gpu_data.yaml"
    data_yaml.write_text(yaml.safe_dump(content, sort_keys=False, allow_unicode=False), encoding="utf-8")
    return data_yaml


def resolve_model(model_arg: str) -> str:
    if model_arg == "auto":
        for model_path in LOCAL_YOLOV8N_PT_CANDIDATES:
            if model_path.exists():
                return str(model_path)
        return PACKAGE_YOLOV8N_YAML

    model_path = Path(model_arg).expanduser()
    if model_path.exists():
        return str(model_path)

    if model_arg.endswith(".pt"):
        raise FileNotFoundError(
            f"Model weights not found: {model_arg}\n"
            "This script is offline-safe and will not auto-download weights.\n"
            f"Use --model {PACKAGE_YOLOV8N_YAML} to train from scratch, or pass an existing .pt file."
        )

    return model_arg


def run_name_for_dataset(args: argparse.Namespace, dataset: DatasetConfig, total_datasets: int) -> str:
    if total_datasets > 1 or args.name == DEFAULT_RUN_PREFIX:
        return f"{args.name}_{dataset.key}"
    return args.name


def train_dataset(
    dataset: DatasetConfig,
    args: argparse.Namespace,
    model_path: str,
    total_datasets: int,
) -> Path:
    require_dataset(dataset)
    remove_ultralytics_caches(dataset)
    data_yaml = make_runtime_data_yaml(dataset)
    run_name = run_name_for_dataset(args, dataset, total_datasets)

    print(f"\ntraining dataset: {dataset.key}")
    print(f"runtime data yaml: {data_yaml}")
    print(f"run name: {run_name}")
    print(f"device: {args.device}")

    # Create a fresh YOLO object for each dataset so the runs are independent.
    model = YOLO(model_path)
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        device=args.device,
        project=args.project,
        name=run_name,
        exist_ok=args.exist_ok,
        patience=args.patience,
        cache=args.cache,
        amp=args.amp and args.device != "cpu",
    )

    trainer = getattr(model, "trainer", None)
    save_dir = Path(getattr(trainer, "save_dir", Path(args.project) / run_name))
    best_pt = save_dir / "weights" / "best.pt"
    print(f"best weights: {best_pt}")

    if args.export_onnx:
        if not best_pt.exists():
            raise FileNotFoundError(f"Cannot export; missing {best_pt}")
        exported = YOLO(str(best_pt)).export(format="onnx", imgsz=args.imgsz, device=args.device)
        print(f"exported onnx: {exported}")

    return best_pt


def main() -> None:
    args = parse_args()
    datasets = selected_datasets(args)
    check_environment(datasets)
    if args.check:
        return

    model_path = resolve_model(args.model)
    print(f"model: {model_path}")
    print("training datasets independently; one best.pt will be produced per run")

    best_weights: list[tuple[str, Path]] = []
    for dataset in datasets:
        best_weights.append((dataset.key, train_dataset(dataset, args, model_path, len(datasets))))

    print("\ntraining complete")
    for dataset_key, best_pt in best_weights:
        print(f"{dataset_key} best.pt: {best_pt}")


if __name__ == "__main__":
    main()
