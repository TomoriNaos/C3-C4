from __future__ import annotations

from pathlib import Path

from yolo_train_common import DEFAULT_CLASSES, ROOT_DIR, DatasetSpec, main_for_dataset, reexec_if_needed


reexec_if_needed(Path(__file__).resolve())


if __name__ == "__main__":
    main_for_dataset(
        DatasetSpec(
            key="new_gated_camera",
            dataset_dir=ROOT_DIR / "new_gated_camera.v2i.yolov8",
            run_name="new_gated_camera",
            class_names=DEFAULT_CLASSES,
        )
    )
