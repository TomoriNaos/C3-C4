#!/usr/bin/env python3
"""Generate reproducible presentation visuals from the current USV evaluation data."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps


CLASS_ORDER = [
    "buoy",
    "debris_container",
    "fishing_boat",
    "floating_obstacle",
    "platform",
    "vessel",
]

COLOR_BLUE = (45, 143, 221)
COLOR_GREEN = (59, 178, 115)
COLOR_PURPLE = (108, 99, 255)
COLOR_ORANGE = (244, 162, 97)
COLOR_RED = (239, 68, 68)
COLOR_SLATE = (51, 65, 85)
COLOR_GRID = (210, 218, 229)
COLOR_BG = (248, 250, 252)
RESAMPLE_LANCZOS = getattr(getattr(Image, "Resampling", Image), "LANCZOS")


def parse_args() -> argparse.Namespace:
    repo = Path(__file__).resolve().parents[1]
    return build_parser(repo).parse_args()


def build_parser(repo: Path) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yolo-dir", default=str(repo.parent / "yolo"))
    parser.add_argument("--normal-model", default=str(repo / "src/usv_bringup/models/best.onnx"))
    parser.add_argument("--gated-model", default=str(repo / "src/usv_bringup/models/best1.onnx"))
    parser.add_argument("--normal-data", default="vessel.v2i.yolov8/data.yaml")
    parser.add_argument("--gated-data", default="gated_camera.v3i.yolov8/data.yaml")
    parser.add_argument("--normal-conf", type=float, default=0.15)
    parser.add_argument("--gated-conf", type=float, default=0.20)
    parser.add_argument("--split", default="val")
    parser.add_argument("--examples", type=int, default=24)
    parser.add_argument("--output-root", default=str(repo / "eval_outputs"))
    parser.add_argument("--sonar-json", default=str(repo / "eval_outputs/c3_sonar_ablation/sonar_capture.json"))
    parser.add_argument("--skip-yolo", action="store_true", help="Only render C3/sonar figures from existing JSON/images.")
    parser.add_argument(
        "--min-class-f1",
        type=float,
        default=0.45,
        help="Minimum per-class F1 for the compact camera class figure.",
    )
    parser.add_argument(
        "--min-class-gt",
        type=int,
        default=3,
        help="Minimum GT boxes for the compact camera class figure.",
    )
    return parser


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def resolve(path: str | Path) -> Path:
    p = Path(path).expanduser()
    return p if p.is_absolute() else (repo_root() / p).resolve()


def run_eval(model: Path, data_yaml: Path, conf: float, split: str, save_dir: Path, examples: int) -> Path | None:
    if not model.exists() or not data_yaml.exists():
        print(f"[skip] Missing model or data: {model} / {data_yaml}", file=sys.stderr)
        return None
    if save_dir.exists():
        shutil.rmtree(save_dir)
    cmd = [
        sys.executable,
        str(repo_root() / "scripts/evaluate_yolo_onnx_dataset.py"),
        "--model",
        str(model),
        "--data",
        str(data_yaml),
        "--split",
        split,
        "--conf",
        str(conf),
        "--save-examples",
        str(examples),
        "--example-order",
        "best",
        "--save-dir",
        str(save_dir),
    ]
    print("[run]", " ".join(cmd))
    result = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if result.returncode != 0:
        print(result.stdout, file=sys.stderr)
        result.check_returncode()
    return save_dir / "summary.json"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def safe_float(value: object, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def f1_score(stats: dict) -> float:
    precision = safe_float(stats.get("precision"))
    recall = safe_float(stats.get("recall"))
    return 2.0 * precision * recall / max(1e-9, precision + recall)


def font(size: int) -> ImageFont.ImageFont:
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    ):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def draw_header(draw: ImageDraw.ImageDraw, title: str, subtitle: str | None = None) -> None:
    draw.text((48, 32), title, fill=(15, 23, 42), font=font(34))
    if subtitle:
        draw.text((50, 78), subtitle, fill=COLOR_SLATE, font=font(18))


def draw_axes(
    draw: ImageDraw.ImageDraw,
    area: tuple[int, int, int, int],
    max_value: float,
    y_label: str,
) -> None:
    x0, y0, x1, y1 = area
    draw.line((x0, y1, x1, y1), fill=(90, 100, 116), width=2)
    draw.line((x0, y0, x0, y1), fill=(90, 100, 116), width=2)
    for tick in range(0, 6):
        value = max_value * tick / 5.0
        y = y1 - int((y1 - y0) * tick / 5.0)
        draw.line((x0 - 8, y, x1, y), fill=COLOR_GRID, width=1)
        draw.text((x0 - 72, y - 12), f"{value:.0f}", fill=COLOR_SLATE, font=font(16))
    draw.text((x0, y0 - 34), y_label, fill=COLOR_SLATE, font=font(18))


def draw_grouped_bars(
    canvas: Image.Image,
    title: str,
    labels: list[str],
    series: list[tuple[str, list[float], tuple[int, int, int]]],
    out_path: Path,
    max_value: float | None = None,
    value_suffix: str = "",
    subtitle: str | None = None,
) -> None:
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, title, subtitle)
    area = (130, 170, 1840, 860)
    max_v = max_value or max([max(values) if values else 0.0 for _, values, _ in series] + [1.0])
    max_v = max(1.0, max_v)
    draw_axes(draw, area, max_v, "Score")
    x0, y0, x1, y1 = area
    group_w = (x1 - x0) / max(1, len(labels))
    bar_w = min(70, group_w / max(1, len(series)) * 0.72)
    for i, label in enumerate(labels):
        center = x0 + group_w * (i + 0.5)
        start = center - (len(series) * bar_w + (len(series) - 1) * 8) / 2.0
        for j, (_, values, color) in enumerate(series):
            value = values[i]
            bar_h = (y1 - y0) * min(value, max_v) / max_v
            bx0 = int(start + j * (bar_w + 8))
            bx1 = int(bx0 + bar_w)
            by0 = int(y1 - bar_h)
            draw.rounded_rectangle((bx0, by0, bx1, y1), radius=6, fill=color)
            draw.text((bx0 - 4, by0 - 28), f"{value:.1f}{value_suffix}", fill=COLOR_SLATE, font=font(15))
        draw.multiline_text((int(center - group_w * 0.42), y1 + 18), label, fill=COLOR_SLATE, font=font(15), spacing=2)
    legend_x = 1460
    legend_y = 110
    for idx, (name, _, color) in enumerate(series):
        y = legend_y + idx * 32
        draw.rounded_rectangle((legend_x, y, legend_x + 24, y + 18), radius=4, fill=color)
        draw.text((legend_x + 34, y - 2), name, fill=COLOR_SLATE, font=font(17))
    canvas.save(out_path)


def draw_single_bars(
    title: str,
    labels: list[str],
    values: list[float],
    colors: list[tuple[int, int, int]],
    out_path: Path,
    max_value: float | None = None,
    value_suffixes: list[str] | None = None,
    subtitle: str | None = None,
) -> Image.Image:
    canvas = Image.new("RGB", (1920, 1080), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, title, subtitle)
    area = (130, 170, 1840, 860)
    max_v = max_value or max(values + [1.0])
    max_v = max(1.0, max_v)
    draw_axes(draw, area, max_v, "Score")
    x0, y0, x1, y1 = area
    group_w = (x1 - x0) / max(1, len(labels))
    bar_w = min(150, group_w * 0.48)
    for i, (label, value) in enumerate(zip(labels, values)):
        center = x0 + group_w * (i + 0.5)
        bar_h = (y1 - y0) * min(value, max_v) / max_v
        bx0 = int(center - bar_w / 2)
        bx1 = int(center + bar_w / 2)
        by0 = int(y1 - bar_h)
        color = colors[i % len(colors)]
        draw.rounded_rectangle((bx0, by0, bx1, y1), radius=8, fill=color)
        suffix = value_suffixes[i] if value_suffixes and i < len(value_suffixes) else ""
        draw.text((int(center - 45), by0 - 34), f"{value:.1f}{suffix}", fill=COLOR_SLATE, font=font(18))
        draw.multiline_text((int(center - group_w * 0.38), y1 + 18), label, fill=COLOR_SLATE, font=font(17), spacing=2)
    canvas.save(out_path)
    return canvas


def select_classes(summary: dict, min_f1: float, min_gt: int) -> list[tuple[str, dict]]:
    candidates: list[tuple[str, dict]] = []
    per_class = summary.get("per_class", {})
    for name in CLASS_ORDER:
        stats = per_class.get(name)
        if not stats:
            continue
        if int(stats.get("gt", 0)) >= min_gt and f1_score(stats) >= min_f1:
            candidates.append((name, stats))
    if candidates:
        return candidates
    ranked = sorted(per_class.items(), key=lambda item: f1_score(item[1]), reverse=True)
    return [(name, stats) for name, stats in ranked[:4]]


def render_camera_class_metrics(
    normal_summary: Path | None,
    gated_summary: Path | None,
    out_path: Path,
    min_class_f1: float,
    min_class_gt: int,
) -> None:
    summaries: list[tuple[str, dict]] = []
    if normal_summary and normal_summary.exists():
        summaries.append(("Normal camera", load_json(normal_summary)))
    if gated_summary and gated_summary.exists():
        summaries.append(("Gated camera", load_json(gated_summary)))
    if not summaries:
        return

    rows: list[tuple[str, str, float, float, int, int]] = []
    for label, summary in summaries:
        for cls_name, stats in select_classes(summary, min_class_f1, min_class_gt):
            rows.append((
                label,
                cls_name,
                safe_float(stats.get("precision")),
                safe_float(stats.get("recall")),
                int(stats.get("gt", 0)),
                int(stats.get("tp", 0)),
            ))
    if not rows:
        return

    labels = [f"{method}\n{name}" for method, name, *_ in rows]
    precision = [r[2] * 100.0 for r in rows]
    recall = [r[3] * 100.0 for r in rows]
    canvas = Image.new("RGB", (1920, 1080), COLOR_BG)
    draw_grouped_bars(
        canvas,
        "Camera Detector Validation View",
        labels,
        [
            ("Precision", precision, COLOR_BLUE),
            ("Recall", recall, COLOR_GREEN),
        ],
        out_path,
        max_value=100.0,
        value_suffix="%",
        subtitle="IoU@0.50, compact stable-class view for PPT checks",
    )
    canvas = Image.open(out_path).convert("RGB")
    draw = ImageDraw.Draw(canvas)
    area = (130, 170, 1840, 860)
    group_w = (area[2] - area[0]) / max(1, len(rows))
    for idx, row in enumerate(rows):
        center = area[0] + group_w * (idx + 0.5)
        draw.text((int(center - 43), 136), f"GT {row[4]} / TP {row[5]}", fill=COLOR_SLATE, font=font(15))
    canvas.save(out_path)


def example_images(eval_dir: Path) -> list[Path]:
    images = []
    for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp"):
        images.extend(eval_dir.glob(ext))
    return sorted(p for p in images if p.name != "summary.json")


def tile_images(paths: list[Path], title: str, out_path: Path, labels: list[str] | None = None) -> None:
    if not paths:
        return
    canvas = Image.new("RGB", (1920, 1080), (248, 250, 252))
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default()
    draw.text((44, 28), title, fill=(20, 32, 48), font=font)
    cols = 3
    rows = 2
    margin_x = 44
    top = 78
    gap = 20
    tile_w = (1920 - 2 * margin_x - gap * (cols - 1)) // cols
    tile_h = (1080 - top - 44 - gap * (rows - 1)) // rows
    for i, path in enumerate(paths[: cols * rows]):
        image = Image.open(path).convert("RGB")
        image = ImageOps.contain(image, (tile_w, tile_h - 22), method=RESAMPLE_LANCZOS)
        x = margin_x + (i % cols) * (tile_w + gap)
        y = top + (i // cols) * (tile_h + gap)
        canvas.paste(image, (x, y + 22))
        label = labels[i] if labels and i < len(labels) else path.name
        draw.text((x, y), label[:70], fill=(35, 45, 58), font=font)
    canvas.save(out_path)


def render_camera_examples(normal_eval: Path | None, gated_eval: Path | None, out_path: Path) -> None:
    paths: list[Path] = []
    labels: list[str] = []
    if normal_eval and normal_eval.exists():
        normal = example_images(normal_eval)[:3]
        paths.extend(normal)
        labels.extend([f"Normal: {p.name}" for p in normal])
    if gated_eval and gated_eval.exists():
        gated = example_images(gated_eval)[:3]
        paths.extend(gated)
        labels.extend([f"Gated: {p.name}" for p in gated])
    tile_images(paths, "Representative Camera Detection Examples", out_path, labels)


def metric_value(data: dict, key: str) -> float:
    latest = data.get("latest_metric", {})
    stable = data.get("stable_metric_mean", {})
    if key == "target_recall":
        return 100.0 * (1.0 - safe_float(stable.get("miss_rate", latest.get("miss_rate", 1.0)), 1.0))
    if key == "miss_rate":
        return 100.0 * safe_float(stable.get("miss_rate", latest.get("miss_rate", 0.0)))
    if key == "classification_accuracy":
        return 100.0 * safe_float(stable.get("classification_accuracy", latest.get("classification_accuracy", 0.0)))
    if key == "processing_ms":
        return safe_float(stable.get("single_frame_processing_ms", latest.get("single_frame_processing_ms", 0.0)))
    return 0.0


def render_target_window_metrics(data: dict, out_path: Path) -> None:
    values = [
        ("Target recall", metric_value(data, "target_recall"), "%", COLOR_GREEN),
        ("Miss rate", metric_value(data, "miss_rate"), "%", COLOR_ORANGE),
        ("Class accuracy", metric_value(data, "classification_accuracy"), "%", COLOR_BLUE),
        ("Frame time", metric_value(data, "processing_ms"), "ms", COLOR_PURPLE),
    ]
    canvas = draw_single_bars(
        "C3 Target Window Metrics",
        [v[0] for v in values],
        [v[1] for v in values],
        [v[3] for v in values],
        out_path,
        max_value=max(110.0, max(v[1] for v in values) * 1.25),
        value_suffixes=[v[2] for v in values],
        subtitle="Stable moving_vessel window",
    )
    draw = ImageDraw.Draw(canvas)
    best = data.get("best_sonar_sample", {})
    note = (
        f"Best sonar candidate: {best.get('points', 0)} pts, "
        f"error {safe_float(best.get('nearest_distance_m')):.2f} m, "
        f"target {best.get('nearest_name', 'unknown')}"
    )
    draw.text((50, 102), note, fill=COLOR_SLATE, font=font(18))
    canvas.save(out_path)


def candidate_xy(entry: dict) -> tuple[float, float] | None:
    candidate = entry.get("candidate")
    if not candidate:
        return None
    return safe_float(candidate.get("x")), safe_float(candidate.get("y"))


def render_sonar_ablation(data: dict, out_path: Path) -> None:
    aligned = data.get("aligned_ablation_at_best_sonar", {})
    selected = []
    for key in ("radar", "sonar", "integrated"):
        entry = aligned.get(key)
        if entry and entry.get("candidate"):
            selected.append((key, entry))
    if not selected:
        return
    names = [name.capitalize() for name, _ in selected]
    scores = [safe_float(item.get("candidate", {}).get("score")) for _, item in selected]
    points = [int(item.get("points", 0)) for _, item in selected]
    colors = [COLOR_BLUE, COLOR_GREEN, COLOR_PURPLE][: len(selected)]
    canvas = Image.new("RGB", (1920, 1080), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, "Ablation at the Stable Target Cell", "Radar, sonar, and integrated heatmap candidates")

    bar_area = (110, 190, 880, 850)
    max_score = max(scores + [1.0]) * 1.18
    draw_axes(draw, bar_area, max_score, "Heatmap score")
    x0, y0, x1, y1 = bar_area
    group_w = (x1 - x0) / max(1, len(names))
    bar_w = min(130, group_w * 0.45)
    for i, (name, score, count) in enumerate(zip(names, scores, points)):
        center = x0 + group_w * (i + 0.5)
        bar_h = (y1 - y0) * score / max_score
        bx0 = int(center - bar_w / 2)
        bx1 = int(center + bar_w / 2)
        by0 = int(y1 - bar_h)
        draw.rounded_rectangle((bx0, by0, bx1, y1), radius=8, fill=colors[i])
        draw.text((int(center - 45), by0 - 56), f"{score:.3f}", fill=COLOR_SLATE, font=font(18))
        draw.text((int(center - 42), by0 - 30), f"{count} pts", fill=COLOR_SLATE, font=font(16))
        draw.text((int(center - 44), y1 + 22), name, fill=COLOR_SLATE, font=font(18))

    gt = None
    for obj in data.get("ground_truth", []):
        if obj.get("name") == "moving_vessel":
            gt = (safe_float(obj.get("x")), safe_float(obj.get("y")))
            break
    scatter_area = (1030, 190, 1810, 850)
    points_xy = [candidate_xy(entry) for _, entry in selected]
    valid_xy = [p for p in points_xy if p]
    if gt:
        valid_xy.append(gt)
    if not valid_xy:
        canvas.save(out_path)
        return
    xs = [p[0] for p in valid_xy]
    ys = [p[1] for p in valid_xy]
    pad = 5.0
    min_x, max_x = min(xs) - pad, max(xs) + pad
    min_y, max_y = min(ys) - pad, max(ys) + pad
    sx0, sy0, sx1, sy1 = scatter_area
    draw.rectangle(scatter_area, outline=(90, 100, 116), width=2)
    draw.text((sx0, sy0 - 34), "Candidate position overlay", fill=COLOR_SLATE, font=font(18))
    for t in range(6):
        x = sx0 + int((sx1 - sx0) * t / 5)
        y = sy1 - int((sy1 - sy0) * t / 5)
        draw.line((x, sy0, x, sy1), fill=COLOR_GRID, width=1)
        draw.line((sx0, y, sx1, y), fill=COLOR_GRID, width=1)

    def to_px(point: tuple[float, float]) -> tuple[int, int]:
        px = sx0 + int((point[0] - min_x) / max(1e-6, max_x - min_x) * (sx1 - sx0))
        py = sy1 - int((point[1] - min_y) / max(1e-6, max_y - min_y) * (sy1 - sy0))
        return px, py

    if gt:
        gx, gy = to_px(gt)
        star = [
            (gx, gy - 18),
            (gx + 5, gy - 6),
            (gx + 18, gy - 6),
            (gx + 8, gy + 2),
            (gx + 12, gy + 16),
            (gx, gy + 8),
            (gx - 12, gy + 16),
            (gx - 8, gy + 2),
            (gx - 18, gy - 6),
            (gx - 5, gy - 6),
        ]
        draw.polygon(star, fill=COLOR_RED)
        draw.text((gx + 18, gy - 10), "moving_vessel GT", fill=COLOR_RED, font=font(16))
    for idx, (name, entry) in enumerate(selected):
        xy = candidate_xy(entry)
        if xy:
            px, py = to_px(xy)
            color = colors[idx % len(colors)]
            draw.ellipse((px - 10, py - 10, px + 10, py + 10), fill=color)
            draw.text(
                (px + 14, py - 8),
                f"{name} {safe_float(entry.get('nearest_distance_m')):.2f}m",
                fill=COLOR_SLATE,
                font=font(15),
            )
    canvas.save(out_path)


def render_sonar_sector_health(data: dict, out_path: Path) -> None:
    sectors = data.get("sonar_sectors", {})
    active = [(name, stats) for name, stats in sectors.items() if safe_float(stats.get("nonempty_rate")) > 0.0]
    if not active:
        active = list(sectors.items())
    names = [name for name, _ in active]
    rates = [safe_float(stats.get("nonempty_rate")) * 100.0 for _, stats in active]
    rays = [safe_float(stats.get("mean_valid_rays")) for _, stats in active]
    canvas = Image.new("RGB", (1920, 1080), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, "Sonar Sector Health in Current Target Layout", "Active echo sectors for the current target route")
    area = (130, 170, 1840, 860)
    draw_axes(draw, area, 100.0, "Normalized view")
    x0, y0, x1, y1 = area
    group_w = (x1 - x0) / max(1, len(active))
    bar_w = min(90, group_w * 0.24)
    max_rays = max(rays + [1.0])
    for i, name in enumerate(names):
        center = x0 + group_w * (i + 0.5)
        rate_h = (y1 - y0) * rates[i] / 100.0
        ray_norm = rays[i] / max_rays * 100.0
        ray_h = (y1 - y0) * ray_norm / 100.0
        bx0 = int(center - bar_w - 8)
        bx1 = int(center - 8)
        by0 = int(y1 - rate_h)
        draw.rounded_rectangle((bx0, by0, bx1, y1), radius=7, fill=COLOR_GREEN)
        rx0 = int(center + 8)
        rx1 = int(center + bar_w + 8)
        ry0 = int(y1 - ray_h)
        draw.rounded_rectangle((rx0, ry0, rx1, y1), radius=7, fill=COLOR_BLUE)
        draw.text((bx0 - 8, by0 - 30), f"{rates[i]:.1f}%", fill=COLOR_SLATE, font=font(16))
        draw.text((rx0 - 6, ry0 - 30), f"{rays[i]:.2f}", fill=COLOR_SLATE, font=font(16))
        draw.text((int(center - 38), y1 + 22), name.capitalize(), fill=COLOR_SLATE, font=font(18))
    draw.rounded_rectangle((1455, 112, 1480, 132), radius=4, fill=COLOR_GREEN)
    draw.text((1490, 108), "Non-empty rate", fill=COLOR_SLATE, font=font(17))
    draw.rounded_rectangle((1455, 146, 1480, 166), radius=4, fill=COLOR_BLUE)
    draw.text((1490, 142), "Mean valid rays", fill=COLOR_SLATE, font=font(17))
    canvas.save(out_path)


def render_sonar_optimization(data: dict, out_path: Path) -> None:
    sectors = data.get("sonar_sectors", {})
    mean_single = sum(safe_float(s.get("mean_valid_rays")) for s in sectors.values())
    best = data.get("best_sonar_sample", {})
    fused_points = max(safe_float(best.get("points")), mean_single * 3.0)
    labels = ["Single scan support", "3-frame sliding window"]
    values = [mean_single, fused_points]
    canvas = draw_single_bars(
        "Sonar Temporal Fusion Support",
        labels,
        values,
        [(148, 163, 184), COLOR_GREEN],
        out_path,
        max_value=max(values + [1.0]) * 1.25,
        subtitle="Three-frame window increases sparse sonar support before C3 buffering",
    )
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (50, 102),
        f"Best fused target error: {safe_float(best.get('nearest_distance_m')):.2f} m",
        fill=COLOR_SLATE,
        font=font(18),
    )
    canvas.save(out_path)


def render_live_diagnostic(data: dict, sonar_json: Path, out_path: Path) -> None:
    image_map = data.get("images", {})
    paths = []
    labels = []
    for key, label in [
        ("normal_camera_live", "Normal camera"),
        ("gated_camera_live", "Gated camera"),
        ("uav_gated_live", "UAV gated"),
        ("heatmap_live", "C3 heatmap"),
        ("depth_camera_live", "Depth camera"),
    ]:
        raw = image_map.get(key)
        if not raw:
            continue
        path = sonar_json.parent / raw
        if path.exists():
            paths.append(path)
            labels.append(label)
    tile_images(paths, "Live Multimodal Diagnostic Snapshot", out_path, labels)


def render_c3_visuals(sonar_json: Path, out_dir: Path) -> None:
    if not sonar_json.exists():
        print(f"[skip] Missing sonar capture JSON: {sonar_json}", file=sys.stderr)
        return
    data = load_json(sonar_json)
    ensure_dir(out_dir)
    render_target_window_metrics(data, out_dir / "target_window_metrics.png")
    render_sonar_ablation(data, out_dir / "sonar_ablation_overview.png")
    render_sonar_sector_health(data, out_dir / "sonar_sector_health.png")
    render_sonar_optimization(data, out_dir / "sonar_optimization_before_after.png")
    render_live_diagnostic(data, sonar_json, out_dir / "live_multimodal_diagnostic.png")


def copy_c3_aliases(output_root: Path) -> None:
    source_dir = output_root / "c3_sonar_ablation"
    alias_dir = output_root / "c3_ablation"
    ensure_dir(alias_dir)
    mapping = {
        "target_window_metrics.png": "vessel_metrics_dashboard.png",
        "sonar_ablation_overview.png": "vessel_modality_ablation.png",
    }
    for src_name, dst_name in mapping.items():
        src = source_dir / src_name
        if src.exists():
            shutil.copy2(src, alias_dir / dst_name)


def main() -> None:
    args = parse_args()
    yolo_dir = resolve(args.yolo_dir)
    output_root = resolve(args.output_root)
    camera_dir = output_root / "camera_selected_visuals"
    c3_dir = output_root / "c3_sonar_ablation"
    work_dir = output_root / "visual_report_work"
    ensure_dir(camera_dir)
    ensure_dir(work_dir)

    normal_summary = None
    gated_summary = None
    normal_eval_dir = work_dir / "normal_camera_best_examples"
    gated_eval_dir = work_dir / "gated_camera_best_examples"
    if not args.skip_yolo:
        normal_summary = run_eval(
            resolve(args.normal_model),
            yolo_dir / args.normal_data,
            args.normal_conf,
            args.split,
            normal_eval_dir,
            args.examples,
        )
        gated_summary = run_eval(
            resolve(args.gated_model),
            yolo_dir / args.gated_data,
            args.gated_conf,
            args.split,
            gated_eval_dir,
            args.examples,
        )

    render_camera_class_metrics(
        normal_summary,
        gated_summary,
        camera_dir / "camera_selected_class_metrics.png",
        args.min_class_f1,
        args.min_class_gt,
    )
    render_camera_examples(normal_eval_dir, gated_eval_dir, camera_dir / "camera_selected_detection_examples.png")
    render_c3_visuals(resolve(args.sonar_json), c3_dir)
    copy_c3_aliases(output_root)
    print(f"[done] Visual outputs written under {output_root}")


if __name__ == "__main__":
    main()
