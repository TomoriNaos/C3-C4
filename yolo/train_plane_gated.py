from __future__ import annotations

from pathlib import Path

from yolo_train_common import DEFAULT_CLASSES, ROOT_DIR, DatasetSpec, main_for_dataset, reexec_if_needed


reexec_if_needed(Path(__file__).resolve())


if __name__ == "__main__":
    main_for_dataset(
        DatasetSpec(
            key="plane_gated",
            dataset_dir=ROOT_DIR / "plane_gated.v2i.yolov8",
            run_name="plane_gated",
            class_names=DEFAULT_CLASSES,
        )
    )
