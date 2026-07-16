from __future__ import annotations

import argparse
import os
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path


ROOT_DIR = Path("/home/hu/yolo")
YOLO_ENV_PYTHON = Path("/home/hu/yolo_env/bin/python")
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
LABEL_SUFFIXES = (".txt",)
PACKAGE_YOLOV8N_YAML = "yolov8n.yaml"
DEFAULT_CLASSES = [
    "fishing_boat",
    "moving_vessel",
    "obstacle",
    "research_platform",
    "service_boat",
    "ship_far",
]


@dataclass(frozen=True)
class DatasetSpec:
    key: str
    dataset_dir: Path
    run_name: str
    class_names: list[str]

    @property
    def source_data_yaml(self) -> Path:
        return self.dataset_dir / "data.yaml"


def reexec_if_needed(script_path: Path) -> None:
    expected_prefix = YOLO_ENV_PYTHON.parent.parent.resolve()
    if (
        YOLO_ENV_PYTHON.exists()
        and Path(sys.prefix).resolve() != expected_prefix
        and os.environ.get("YOLO_TRAIN_NO_REEXEC") != "1"
    ):
        os.execv(str(YOLO_ENV_PYTHON), [str(YOLO_ENV_PYTHON), str(script_path), *sys.argv[1:]])


def parse_args(spec: DatasetSpec) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=f"Train YOLOv8 on {spec.key}")
    parser.add_argument("--check", action="store_true", help="Only print environment and dataset status")
    parser.add_argument(
        "--model",
        default="auto",
        help=(
            "Model to train. auto uses a local yolov8n.pt if present, "
            "otherwise yolov8n.yaml for offline training from scratch."
        ),
    )
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", default="0", help="Use 0 for GPU, or cpu")
    parser.add_argument("--name", default=spec.run_name, help="Run directory name under --project")
    parser.add_argument("--project", default=str(ROOT_DIR / "runs" / "detect"))
    parser.add_argument(
        "--increment",
        action="store_true",
        help="Create an incremented run directory instead of reusing the fixed run name",
    )
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument("--cache", action="store_true")
    parser.add_argument("--amp", action="store_true", help="Enable AMP. Disabled by default to avoid online AMP checks.")
    parser.add_argument("--export-onnx", action="store_true", help="Export best.pt to ONNX after training")
    return parser.parse_args()


def count_files(directory: Path, suffixes: tuple[str, ...]) -> int:
    if not directory.exists():
        return 0
    return sum(1 for path in directory.iterdir() if path.is_file() and path.suffix.lower() in suffixes)


def read_label_classes(labels_dir: Path) -> set[int]:
    classes: set[int] = set()
    if not labels_dir.exists():
        return classes

    for label_file in sorted(labels_dir.glob("*.txt")):
        for line_number, line in enumerate(label_file.read_text(encoding="utf-8").splitlines(), 1):
            stripped = line.strip()
            if not stripped:
                continue
            first_value = stripped.split(maxsplit=1)[0]
            try:
                classes.add(int(first_value))
            except ValueError as exc:
                raise RuntimeError(f"Invalid class id in {label_file}:{line_number}: {first_value!r}") from exc

    return classes


def format_classes(classes: set[int]) -> str:
    return ", ".join(str(class_id) for class_id in sorted(classes)) if classes else "none"


def split_images_dir(spec: DatasetSpec, split: str) -> Path:
    return spec.dataset_dir / split / "images"


def split_labels_dir(spec: DatasetSpec, split: str) -> Path:
    return spec.dataset_dir / split / "labels"


def check_environment(spec: DatasetSpec) -> None:
    print(f"python: {sys.executable}")
    print(f"dataset: {spec.dataset_dir}")
    print(f"source data.yaml: {spec.source_data_yaml}")
    print(f"class names ({len(spec.class_names)}): {spec.class_names}")

    for split in ("train", "valid", "test"):
        images_dir = split_images_dir(spec, split)
        labels_dir = split_labels_dir(spec, split)
        print(f"{split} images: {count_files(images_dir, IMAGE_SUFFIXES)}")
        print(f"{split} labels: {count_files(labels_dir, LABEL_SUFFIXES)}")
        print(f"{split} label classes: {format_classes(read_label_classes(labels_dir))}")

    try:
        import torch

        print(f"torch: {torch.__version__}")
        print(f"cuda available: {torch.cuda.is_available()}")
        print(f"cuda version: {torch.version.cuda}")
        if torch.cuda.is_available():
            print(f"gpu 0: {torch.cuda.get_device_name(0)}")
    except Exception as exc:
        print(f"torch check failed: {exc}")


def require_split(spec: DatasetSpec, split: str, require_labels: bool) -> None:
    images_dir = split_images_dir(spec, split)
    labels_dir = split_labels_dir(spec, split)
    if not images_dir.exists():
        raise FileNotFoundError(f"Missing image directory: {images_dir}")
    if require_labels and not labels_dir.exists():
        raise FileNotFoundError(f"Missing label directory: {labels_dir}")
    if count_files(images_dir, IMAGE_SUFFIXES) == 0:
        raise RuntimeError(f"No images in {images_dir}")
    if require_labels and count_files(labels_dir, LABEL_SUFFIXES) == 0:
        raise RuntimeError(f"No labels in {labels_dir}")


def require_dataset(spec: DatasetSpec) -> None:
    require_split(spec, "train", require_labels=True)
    require_split(spec, "valid", require_labels=True)

    if split_images_dir(spec, "test").exists():
        require_split(spec, "test", require_labels=False)

    all_classes: set[int] = set()
    for split in ("train", "valid", "test"):
        all_classes.update(read_label_classes(split_labels_dir(spec, split)))

    invalid_classes = sorted(class_id for class_id in all_classes if class_id < 0 or class_id >= len(spec.class_names))
    if invalid_classes:
        raise RuntimeError(
            f"{spec.key} has labels outside 0-{len(spec.class_names) - 1}: {invalid_classes}. "
            f"Class names are {spec.class_names}."
        )

    missing_classes = sorted(set(range(len(spec.class_names))) - all_classes)
    if missing_classes:
        print(f"warning: {spec.key} labels do not contain class ids: {missing_classes}")


def make_runtime_data_yaml(spec: DatasetSpec) -> Path:
    import yaml

    content = {
        "path": str(spec.dataset_dir),
        "train": "train/images",
        "val": "valid/images",
        "test": "test/images",
        "nc": len(spec.class_names),
        "names": spec.class_names,
    }
    data_yaml = Path(tempfile.gettempdir()) / f"{spec.key}_runtime_data.yaml"
    data_yaml.write_text(yaml.safe_dump(content, sort_keys=False, allow_unicode=False), encoding="utf-8")
    return data_yaml


def local_model_candidates() -> list[Path]:
    explicit = [
        ROOT_DIR / "yolov8n.pt",
        ROOT_DIR / "weights" / "yolov8n.pt",
    ]
    discovered = sorted(ROOT_DIR.glob("**/yolov8n.pt"))

    candidates: list[Path] = []
    for path in [*explicit, *discovered]:
        if path not in candidates:
            candidates.append(path)
    return candidates


def resolve_model(model_arg: str) -> str:
    if model_arg == "auto":
        for model_path in local_model_candidates():
            if model_path.exists():
                return str(model_path)
        print(f"warning: no local yolov8n.pt found; using {PACKAGE_YOLOV8N_YAML} from scratch")
        return PACKAGE_YOLOV8N_YAML

    model_path = Path(model_arg).expanduser()
    if model_path.exists():
        return str(model_path)

    if model_arg.endswith(".pt"):
        raise FileNotFoundError(
            f"Model weights not found: {model_arg}\n"
            "Use --model yolov8n.yaml to train from scratch, or pass an existing .pt file."
        )

    return model_arg


def train(spec: DatasetSpec, args: argparse.Namespace) -> Path:
    from ultralytics import YOLO

    require_dataset(spec)
    data_yaml = make_runtime_data_yaml(spec)
    model_path = resolve_model(args.model)

    print(f"runtime data yaml: {data_yaml}")
    print(f"model: {model_path}")
    print(f"project: {args.project}")
    print(f"name: {args.name}")
    print(f"device: {args.device}")

    model = YOLO(model_path)
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        device=args.device,
        project=args.project,
        name=args.name,
        exist_ok=not args.increment,
        patience=args.patience,
        cache=args.cache,
        amp=args.amp and args.device != "cpu",
    )

    trainer = getattr(model, "trainer", None)
    save_dir = Path(getattr(trainer, "save_dir", Path(args.project) / args.name))
    best_pt = save_dir / "weights" / "best.pt"
    print(f"best weights: {best_pt}")

    if args.export_onnx:
        if not best_pt.exists():
            raise FileNotFoundError(f"Cannot export; missing {best_pt}")
        exported = YOLO(str(best_pt)).export(format="onnx", imgsz=args.imgsz, device=args.device)
        print(f"exported onnx: {exported}")

    return best_pt


def main_for_dataset(spec: DatasetSpec) -> None:
    args = parse_args(spec)
    check_environment(spec)
    if args.check:
        return

    best_pt = train(spec, args)
    print(f"done: {best_pt}")
