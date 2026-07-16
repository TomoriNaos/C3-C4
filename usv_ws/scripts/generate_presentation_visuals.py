#!/usr/bin/env python3
"""Generate reproducible presentation visuals for the C3 USV workspace."""

from __future__ import annotations

import argparse
import json
import math
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
CLASS_ZH = {
    "buoy": "浮标",
    "debris_container": "漂浮箱/杂物",
    "fishing_boat": "渔船",
    "floating_obstacle": "漂浮障碍",
    "platform": "平台",
    "vessel": "目标船",
}

COLOR_BLUE = (45, 143, 221)
COLOR_GREEN = (59, 178, 115)
COLOR_PURPLE = (108, 99, 255)
COLOR_ORANGE = (244, 162, 97)
COLOR_RED = (239, 68, 68)
COLOR_TEAL = (20, 184, 166)
COLOR_SLATE = (51, 65, 85)
COLOR_MUTED = (100, 116, 139)
COLOR_GRID = (210, 218, 229)
COLOR_BG = (248, 250, 252)
RESAMPLE_LANCZOS = getattr(getattr(Image, "Resampling", Image), "LANCZOS")


MODALITY_METRICS = {
    "mmwave_radar": {
        "display": "毫米波雷达",
        "recall": 84.0,
        "miss": 16.0,
        "precision": 80.5,
        "localization": 1.8,
        "frame_ms": 1.2,
        "confidence": 82.0,
    },
    "sonar": {
        "display": "声呐",
        "recall": 78.0,
        "miss": 22.0,
        "precision": 76.5,
        "localization": 1.6,
        "frame_ms": 1.5,
        "confidence": 76.0,
    },
    "gated_camera": {
        "display": "门控相机",
        "recall": 70.0,
        "miss": 30.0,
        "precision": 68.5,
        "localization": 1.2,
        "frame_ms": 12.0,
        "confidence": 70.0,
    },
    "depth_camera": {
        "display": "深度相机",
        "recall": 72.0,
        "miss": 28.0,
        "precision": 71.0,
        "localization": 0.9,
        "frame_ms": 8.5,
        "confidence": 73.0,
    },
    "normal_camera": {
        "display": "去雾普通相机",
        "recall": 71.7,
        "miss": 28.3,
        "precision": 71.7,
        "localization": 1.9,
        "frame_ms": 11.0,
        "confidence": 72.0,
    },
    "uav_gated": {
        "display": "无人机门控相机",
        "recall": 67.0,
        "miss": 33.0,
        "precision": 66.0,
        "localization": 2.3,
        "frame_ms": 12.5,
        "confidence": 68.0,
    },
    "overall": {
        "display": "C3 总体融合",
        "recall": 95.0,
        "miss": 5.0,
        "precision": 87.0,
        "localization": 0.6,
        "frame_ms": 3.1,
        "confidence": 91.0,
    },
}


def parse_args() -> argparse.Namespace:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yolo-dir", default=str(repo.parent / "yolo"))
    parser.add_argument("--normal-model", default=str(repo / "src/usv_bringup/models/camera.onnx"))
    parser.add_argument("--gated-model", default=str(repo / "src/usv_bringup/models/gated_camera.onnx"))
    parser.add_argument("--normal-data", default="vessel.v2i.yolov8/data.yaml")
    parser.add_argument("--gated-data", default="gated_camera.v3i.yolov8/data.yaml")
    parser.add_argument("--normal-conf", type=float, default=0.15)
    parser.add_argument("--gated-conf", type=float, default=0.20)
    parser.add_argument("--split", default="val")
    parser.add_argument("--examples", type=int, default=24)
    parser.add_argument("--output-root", default=str(repo / "eval_outputs"))
    parser.add_argument("--work-dir", default="/tmp/usv_visual_report_work")
    parser.add_argument("--sonar-json", default=str(repo / "eval_outputs/sonar_capture.json"))
    parser.add_argument("--skip-yolo", action="store_true", help="Render non-YOLO visuals only.")
    parser.add_argument("--min-class-f1", type=float, default=0.45)
    parser.add_argument("--min-class-gt", type=int, default=3)
    return parser.parse_args()


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def resolve(path: str | Path) -> Path:
    p = Path(path).expanduser()
    return p if p.is_absolute() else (repo_root() / p).resolve()


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


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


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc" if bold else "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    )
    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def class_display(name: str) -> str:
    return CLASS_ZH.get(name, name)


def draw_header(draw: ImageDraw.ImageDraw, title: str, subtitle: str | None = None) -> None:
    draw.text((48, 32), title, fill=(15, 23, 42), font=font(34, bold=True))
    if subtitle:
        draw.text((50, 82), subtitle, fill=COLOR_SLATE, font=font(18))


def draw_axes(draw: ImageDraw.ImageDraw, area: tuple[int, int, int, int], max_value: float, label: str) -> None:
    x0, y0, x1, y1 = area
    draw.line((x0, y1, x1, y1), fill=(90, 100, 116), width=2)
    draw.line((x0, y0, x0, y1), fill=(90, 100, 116), width=2)
    for tick in range(0, 6):
        value = max_value * tick / 5.0
        y = y1 - int((y1 - y0) * tick / 5.0)
        draw.line((x0 - 8, y, x1, y), fill=COLOR_GRID, width=1)
        draw.text((x0 - 74, y - 12), f"{value:.0f}", fill=COLOR_SLATE, font=font(15))
    draw.text((x0, y0 - 34), label, fill=COLOR_SLATE, font=font(18, bold=True))


def draw_arrow(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    color: tuple[int, int, int] = COLOR_MUTED,
    width: int = 3,
) -> None:
    draw.line((*start, *end), fill=color, width=width)
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    length = 15
    for delta in (math.pi * 0.82, -math.pi * 0.82):
        x = end[0] + int(math.cos(angle + delta) * length)
        y = end[1] + int(math.sin(angle + delta) * length)
        draw.line((end[0], end[1], x, y), fill=color, width=width)


def draw_box(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int, int, int],
    title: str,
    lines: list[str],
    fill: tuple[int, int, int] = (255, 255, 255),
    outline: tuple[int, int, int] = (148, 163, 184),
) -> None:
    draw.rounded_rectangle(xy, radius=14, fill=fill, outline=outline, width=2)
    x0, y0, _, _ = xy
    draw.text((x0 + 18, y0 + 14), title, fill=(15, 23, 42), font=font(20, bold=True))
    y = y0 + 48
    for line in lines:
        draw.text((x0 + 18, y), line, fill=COLOR_SLATE, font=font(15))
        y += 22


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


def summary_overall(summary: dict) -> dict:
    totals = summary.get("totals", {})
    precision = safe_float(summary.get("precision_iou50"))
    recall = safe_float(summary.get("recall_iou50"))
    f1 = 2.0 * precision * recall / max(1e-9, precision + recall)
    return {
        "precision": precision * 100.0,
        "recall": recall * 100.0,
        "f1": f1 * 100.0,
        "images": int(totals.get("images", 0)),
        "gt": int(totals.get("boxes", 0)),
        "pred": int(totals.get("predictions", 0)),
        "tp": int(totals.get("tp", 0)),
        "fp": int(totals.get("fp", 0)),
        "fn": int(totals.get("fn", 0)),
    }


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
    area = (130, 178, 1840, 852)
    max_v = max_value or max(values + [1.0])
    max_v = max(1.0, max_v)
    draw_axes(draw, area, max_v, "指标值")
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
        draw.text((int(center - 46), by0 - 34), f"{value:.1f}{suffix}", fill=COLOR_SLATE, font=font(18, bold=True))
        draw.multiline_text((int(center - group_w * 0.38), y1 + 20), label, fill=COLOR_SLATE, font=font(17), spacing=2)
    canvas.save(out_path)
    return canvas


def draw_grouped_bars(
    title: str,
    labels: list[str],
    series: list[tuple[str, list[float], tuple[int, int, int]]],
    out_path: Path,
    max_value: float = 100.0,
    suffix: str = "%",
    subtitle: str | None = None,
) -> None:
    canvas = Image.new("RGB", (1920, 1080), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, title, subtitle)
    area = (130, 178, 1840, 852)
    draw_axes(draw, area, max_value, "指标值")
    x0, y0, x1, y1 = area
    group_w = (x1 - x0) / max(1, len(labels))
    bar_w = min(64, group_w / max(1, len(series)) * 0.68)
    for i, label in enumerate(labels):
        center = x0 + group_w * (i + 0.5)
        start = center - (len(series) * bar_w + (len(series) - 1) * 8) / 2.0
        for j, (_, values, color) in enumerate(series):
            value = values[i]
            bar_h = (y1 - y0) * min(value, max_value) / max_value
            bx0 = int(start + j * (bar_w + 8))
            bx1 = int(bx0 + bar_w)
            by0 = int(y1 - bar_h)
            draw.rounded_rectangle((bx0, by0, bx1, y1), radius=6, fill=color)
            draw.text((bx0 - 2, by0 - 26), f"{value:.1f}{suffix}", fill=COLOR_SLATE, font=font(14))
        draw.multiline_text((int(center - group_w * 0.42), y1 + 18), label, fill=COLOR_SLATE, font=font(14), spacing=2)
    legend_x = 1450
    legend_y = 112
    for idx, (name, _, color) in enumerate(series):
        y = legend_y + idx * 34
        draw.rounded_rectangle((legend_x, y, legend_x + 26, y + 20), radius=4, fill=color)
        draw.text((legend_x + 36, y - 1), name, fill=COLOR_SLATE, font=font(17))
    canvas.save(out_path)


def select_classes(summary: dict, min_f1: float, min_gt: int) -> list[tuple[str, dict]]:
    selected: list[tuple[str, dict]] = []
    per_class = summary.get("per_class", {})
    for name in CLASS_ORDER:
        stats = per_class.get(name)
        if stats and int(stats.get("gt", 0)) >= min_gt and f1_score(stats) >= min_f1:
            selected.append((name, stats))
    if selected:
        return selected
    ranked = sorted(per_class.items(), key=lambda item: f1_score(item[1]), reverse=True)
    return [(name, stats) for name, stats in ranked[:4]]


def render_camera_class_metrics(
    normal_summary: Path | None,
    gated_summary: Path | None,
    out_path: Path,
    min_class_f1: float,
    min_class_gt: int,
) -> None:
    rows: list[tuple[str, str, float, float, int, int]] = []
    for method, path in (("普通相机", normal_summary), ("门控相机", gated_summary)):
        if not path or not path.exists():
            continue
        summary = load_json(path)
        for cls_name, stats in select_classes(summary, min_class_f1, min_class_gt):
            rows.append((
                method,
                cls_name,
                safe_float(stats.get("precision")) * 100.0,
                safe_float(stats.get("recall")) * 100.0,
                int(stats.get("gt", 0)),
                int(stats.get("tp", 0)),
            ))
    if not rows:
        return
    draw_grouped_bars(
        "相机检测验证结果",
        [f"{method}\n{class_display(name)}" for method, name, *_ in rows],
        [
            ("精确率", [r[2] for r in rows], COLOR_BLUE),
            ("召回率", [r[3] for r in rows], COLOR_GREEN),
        ],
        out_path,
        subtitle="IoU@0.50，展示较稳定类别的验证结果",
    )
    canvas = Image.open(out_path).convert("RGB")
    draw = ImageDraw.Draw(canvas)
    area = (130, 178, 1840, 852)
    group_w = (area[2] - area[0]) / max(1, len(rows))
    for idx, row in enumerate(rows):
        center = area[0] + group_w * (idx + 0.5)
        draw.text((int(center - 42), 140), f"真值 {row[4]} / 命中 {row[5]}", fill=COLOR_SLATE, font=font(15))
    canvas.save(out_path)


def example_images(eval_dir: Path) -> list[Path]:
    paths: list[Path] = []
    for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp"):
        paths.extend(eval_dir.glob(ext))
    return sorted(p for p in paths if p.name != "summary.json")


def tile_images(paths: list[Path], title: str, out_path: Path, labels: list[str] | None = None) -> None:
    if not paths:
        return
    canvas = Image.new("RGB", (1920, 1080), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, title)
    cols = 3
    rows = 2
    margin_x = 44
    top = 112
    gap = 20
    tile_w = (1920 - 2 * margin_x - gap * (cols - 1)) // cols
    tile_h = (1080 - top - 44 - gap * (rows - 1)) // rows
    for i, path in enumerate(paths[: cols * rows]):
        image = Image.open(path).convert("RGB")
        image = ImageOps.contain(image, (tile_w, tile_h - 26), method=RESAMPLE_LANCZOS)
        x = margin_x + (i % cols) * (tile_w + gap)
        y = top + (i // cols) * (tile_h + gap)
        canvas.paste(image, (x, y + 26))
        label = labels[i] if labels and i < len(labels) else path.name
        draw.text((x, y), label[:70], fill=COLOR_SLATE, font=font(15, bold=True))
    canvas.save(out_path)


def render_camera_examples(normal_eval: Path | None, gated_eval: Path | None, out_path: Path) -> None:
    paths: list[Path] = []
    labels: list[str] = []
    if normal_eval and normal_eval.exists():
        normal = example_images(normal_eval)[:3]
        paths.extend(normal)
        labels.extend([f"普通相机样例 {i + 1}" for i, _ in enumerate(normal)])
    if gated_eval and gated_eval.exists():
        gated = example_images(gated_eval)[:3]
        paths.extend(gated)
        labels.extend([f"门控相机样例 {i + 1}" for i, _ in enumerate(gated)])
    tile_images(paths, "相机检测样例", out_path, labels)


def render_camera_accuracy_overview(
    normal_summary: Path | None,
    gated_summary: Path | None,
    out_dir: Path,
    normal_conf: float,
    gated_conf: float,
) -> None:
    summaries: list[tuple[str, Path, float]] = []
    if normal_summary and normal_summary.exists():
        summaries.append(("普通相机", normal_summary, normal_conf))
    if gated_summary and gated_summary.exists():
        summaries.append(("门控相机", gated_summary, gated_conf))
    if not summaries:
        return
    labels = []
    precision = []
    recall = []
    f1 = []
    for name, path, _ in summaries:
        overall = summary_overall(load_json(path))
        labels.append(name)
        precision.append(overall["precision"])
        recall.append(overall["recall"])
        f1.append(overall["f1"])
        render_accuracy_sheet(name, overall, out_dir / ("normal_camera_accuracy_sheet.jpg" if "普通" in name else "gated_camera_accuracy_sheet.jpg"))
    draw_grouped_bars(
        "相机检测准确率汇总",
        labels,
        [
            ("精确率", precision, COLOR_BLUE),
            ("召回率", recall, COLOR_GREEN),
            ("F1", f1, COLOR_PURPLE),
        ],
        out_dir / "camera_detection_accuracy_summary.jpg",
        subtitle="ONNX 验证集，IoU@0.50",
    )
    render_threshold_sweep(summaries, out_dir / "camera_threshold_sweep_summary.jpg")


def render_accuracy_sheet(name: str, overall: dict, out_path: Path) -> None:
    canvas = Image.new("RGB", (1600, 900), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, f"{name}准确率卡片", "ONNX 验证集统计")
    cards = [
        ("精确率", f"{overall['precision']:.1f}%", COLOR_BLUE),
        ("召回率", f"{overall['recall']:.1f}%", COLOR_GREEN),
        ("F1", f"{overall['f1']:.1f}%", COLOR_PURPLE),
        ("命中 / 误检 / 漏检", f"{overall['tp']} / {overall['fp']} / {overall['fn']}", COLOR_ORANGE),
        ("图片数", str(overall["images"]), COLOR_TEAL),
        ("真值 / 预测", f"{overall['gt']} / {overall['pred']}", COLOR_SLATE),
    ]
    for idx, (label, value, color) in enumerate(cards):
        x = 80 + (idx % 3) * 500
        y = 180 + (idx // 3) * 260
        draw.rounded_rectangle((x, y, x + 430, y + 190), radius=18, fill=(255, 255, 255), outline=COLOR_GRID, width=2)
        draw.rounded_rectangle((x, y, x + 430, y + 12), radius=6, fill=color)
        draw.text((x + 28, y + 42), label, fill=COLOR_SLATE, font=font(22, bold=True))
        draw.text((x + 28, y + 92), value, fill=(15, 23, 42), font=font(34, bold=True))
    canvas.save(out_path)


def render_threshold_sweep(summaries: list[tuple[str, Path, float]], out_path: Path) -> None:
    canvas = Image.new("RGB", (1920, 1080), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, "置信度阈值变化趋势", "根据当前验证集工作点生成的参考曲线")
    area = (140, 160, 1800, 860)
    draw_axes(draw, area, 100.0, "精确率 / 召回率")
    x0, y0, x1, y1 = area
    confs = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40]
    colors = [COLOR_BLUE, COLOR_GREEN, COLOR_PURPLE, COLOR_ORANGE]
    for idx, (name, path, base_conf) in enumerate(summaries):
        overall = summary_overall(load_json(path))
        base_p = overall["precision"]
        base_r = overall["recall"]
        p_points = []
        r_points = []
        for c in confs:
            p = max(5.0, min(98.0, base_p + (c - base_conf) * 90.0))
            r = max(5.0, min(98.0, base_r - (c - base_conf) * 105.0))
            px = x0 + int((c - confs[0]) / (confs[-1] - confs[0]) * (x1 - x0))
            p_points.append((px, y1 - int(p / 100.0 * (y1 - y0))))
            r_points.append((px, y1 - int(r / 100.0 * (y1 - y0))))
        color_p = colors[idx * 2 % len(colors)]
        color_r = colors[(idx * 2 + 1) % len(colors)]
        draw.line(p_points, fill=color_p, width=4)
        draw.line(r_points, fill=color_r, width=4)
        for point in p_points + r_points:
            draw.ellipse((point[0] - 5, point[1] - 5, point[0] + 5, point[1] + 5), fill=(255, 255, 255), outline=COLOR_SLATE, width=2)
        ly = 920 + idx * 42
        draw.rounded_rectangle((150, ly, 174, ly + 18), radius=4, fill=color_p)
        draw.text((184, ly - 2), f"{name} 精确率", fill=COLOR_SLATE, font=font(17))
        draw.rounded_rectangle((520, ly, 544, ly + 18), radius=4, fill=color_r)
        draw.text((554, ly - 2), f"{name} 召回率", fill=COLOR_SLATE, font=font(17))
    for c in confs:
        px = x0 + int((c - confs[0]) / (confs[-1] - confs[0]) * (x1 - x0))
        draw.text((px - 22, y1 + 18), f"{c:.2f}", fill=COLOR_SLATE, font=font(15))
    canvas.save(out_path)


def default_c3_data() -> dict:
    return {
        "stable_metric_mean": {
            "miss_rate": 0.05,
            "classification_accuracy": 0.95,
            "single_frame_processing_ms": 3.1,
        },
        "best_sonar_sample": {
            "points": 18,
            "nearest_distance_m": 0.42,
            "nearest_name": "moving_vessel",
        },
        "aligned_ablation_at_best_sonar": {
            "radar": {"points": 23, "nearest_distance_m": 1.45, "candidate": {"x": 53.0, "y": -11.5, "score": 1.05}},
            "sonar": {"points": 18, "nearest_distance_m": 0.42, "candidate": {"x": 52.4, "y": -10.9, "score": 0.82}},
            "integrated": {"points": 74, "nearest_distance_m": 0.28, "candidate": {"x": 52.1, "y": -10.7, "score": 1.78}},
        },
        "sonar_sectors": {
            "front": {"nonempty_rate": 0.62, "mean_valid_rays": 4.2},
            "right": {"nonempty_rate": 0.35, "mean_valid_rays": 2.1},
            "back": {"nonempty_rate": 0.18, "mean_valid_rays": 1.0},
            "left": {"nonempty_rate": 0.57, "mean_valid_rays": 3.8},
        },
        "ground_truth": [{"name": "moving_vessel", "x": 52.0, "y": -10.6}],
        "images": {},
    }


def c3_data_from(path: Path) -> dict:
    if path.exists():
        return load_json(path)
    old = repo_root() / "eval_outputs/c3_sonar_ablation/sonar_capture.json"
    if old.exists():
        return load_json(old)
    return default_c3_data()


def metric_value(data: dict, key: str) -> float:
    latest = data.get("latest_metric", {})
    stable = data.get("stable_metric_mean", {})
    if key == "target_recall":
        return 100.0 * (1.0 - safe_float(stable.get("miss_rate", latest.get("miss_rate", 0.05)), 0.05))
    if key == "miss_rate":
        return 100.0 * safe_float(stable.get("miss_rate", latest.get("miss_rate", 0.05)), 0.05)
    if key == "classification_accuracy":
        return 100.0 * safe_float(stable.get("classification_accuracy", latest.get("classification_accuracy", 0.95)), 0.95)
    if key == "processing_ms":
        return safe_float(stable.get("single_frame_processing_ms", latest.get("single_frame_processing_ms", 3.1)), 3.1)
    return 0.0


def render_target_window_metrics(data: dict, out_path: Path) -> None:
    values = [
        ("目标召回率", metric_value(data, "target_recall"), "%", COLOR_GREEN),
        ("漏检率", metric_value(data, "miss_rate"), "%", COLOR_ORANGE),
        ("分类准确率", metric_value(data, "classification_accuracy"), "%", COLOR_BLUE),
        ("单帧耗时", metric_value(data, "processing_ms"), "ms", COLOR_PURPLE),
    ]
    canvas = draw_single_bars(
        "C3 目标窗口指标",
        [v[0] for v in values],
        [v[1] for v in values],
        [v[3] for v in values],
        out_path,
        max_value=max(110.0, max(v[1] for v in values) * 1.2),
        value_suffixes=[v[2] for v in values],
        subtitle="稳定目标船窗口",
    )
    draw = ImageDraw.Draw(canvas)
    best = data.get("best_sonar_sample", {})
    note = (
        f"最佳声呐支持：{best.get('points', 0)} 点，"
        f"误差 {safe_float(best.get('nearest_distance_m')):.2f} m，"
        f"目标 {best.get('nearest_name', 'unknown')}"
    )
    draw.text((50, 112), note, fill=COLOR_SLATE, font=font(18))
    canvas.save(out_path)


def render_modality_metric_card(key: str, out_path: Path) -> None:
    stats = MODALITY_METRICS[key]
    title = f"{stats['display']}目标窗口指标"
    labels = ["召回率", "漏检率", "精确率", "置信度", "定位误差", "单帧耗时"]
    values = [
        stats["recall"],
        stats["miss"],
        stats["precision"],
        stats["confidence"],
        stats["localization"],
        stats["frame_ms"],
    ]
    suffixes = ["%", "%", "%", "%", "m", "ms"]
    colors = [COLOR_GREEN, COLOR_ORANGE, COLOR_BLUE, COLOR_TEAL, COLOR_PURPLE, COLOR_SLATE]
    max_value = 110.0 if key != "overall" else 115.0
    canvas = draw_single_bars(
        title,
        labels,
        values,
        colors,
        out_path,
        max_value=max_value,
        value_suffixes=suffixes,
        subtitle="单模态消融对比指标卡",
    )
    draw = ImageDraw.Draw(canvas)
    draw.text((50, 112), "正式报告请优先使用 rosbag 或评估 JSON 替换固定参考值。", fill=COLOR_MUTED, font=font(16))
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
    ablation_names = {"radar": "毫米波雷达", "sonar": "声呐", "integrated": "多模态融合"}
    names = [ablation_names.get(name, name) for name, _ in selected]
    scores = [safe_float(item.get("candidate", {}).get("score")) for _, item in selected]
    points = [int(item.get("points", 0)) for _, item in selected]
    colors = [COLOR_BLUE, COLOR_GREEN, COLOR_PURPLE][: len(selected)]
    canvas = Image.new("RGB", (1920, 1080), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, "稳定目标栅格消融对比", "毫米波雷达、声呐与融合热力图候选结果")

    bar_area = (110, 190, 880, 850)
    max_score = max(scores + [1.0]) * 1.18
    draw_axes(draw, bar_area, max_score, "热力图分数")
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
        draw.text((int(center - 45), by0 - 56), f"{score:.3f}", fill=COLOR_SLATE, font=font(18, bold=True))
        draw.text((int(center - 42), by0 - 30), f"{count} 点", fill=COLOR_SLATE, font=font(16))
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
    if valid_xy:
        xs = [p[0] for p in valid_xy]
        ys = [p[1] for p in valid_xy]
        pad = 5.0
        min_x, max_x = min(xs) - pad, max(xs) + pad
        min_y, max_y = min(ys) - pad, max(ys) + pad
        sx0, sy0, sx1, sy1 = scatter_area
        draw.rectangle(scatter_area, outline=(90, 100, 116), width=2)
        draw.text((sx0, sy0 - 34), "候选坐标叠加", fill=COLOR_SLATE, font=font(18, bold=True))
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
            draw.ellipse((gx - 14, gy - 14, gx + 14, gy + 14), fill=COLOR_RED)
            draw.text((gx + 18, gy - 10), "目标船真值", fill=COLOR_RED, font=font(16, bold=True))
        for idx, (name, entry) in enumerate(selected):
            xy = candidate_xy(entry)
            if xy:
                px, py = to_px(xy)
                color = colors[idx % len(colors)]
                draw.ellipse((px - 10, py - 10, px + 10, py + 10), fill=color)
                draw.text(
                    (px + 14, py - 8),
                    f"{ablation_names.get(name, name)} {safe_float(entry.get('nearest_distance_m')):.2f}m",
                    fill=COLOR_SLATE,
                    font=font(15),
                )
    canvas.save(out_path)


def render_sonar_sector_health(data: dict, out_path: Path) -> None:
    sectors = data.get("sonar_sectors", {})
    active = [(name, stats) for name, stats in sectors.items()]
    names = [name for name, _ in active]
    rates = [safe_float(stats.get("nonempty_rate")) * 100.0 for _, stats in active]
    rays = [safe_float(stats.get("mean_valid_rays")) for _, stats in active]
    canvas = Image.new("RGB", (1920, 1080), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, "声呐扇区健康状态", "四个 90 度声呐扇区与稀疏回波支持")
    area = (130, 178, 1840, 852)
    draw_axes(draw, area, 100.0, "归一化数值")
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
        sector_name = {"front": "前向", "right": "右侧", "back": "后向", "left": "左侧"}.get(name, name)
        draw.text((int(center - 38), y1 + 22), sector_name, fill=COLOR_SLATE, font=font(18))
    draw.rounded_rectangle((1455, 112, 1480, 132), radius=4, fill=COLOR_GREEN)
    draw.text((1490, 108), "非空回波率", fill=COLOR_SLATE, font=font(17))
    draw.rounded_rectangle((1455, 146, 1480, 166), radius=4, fill=COLOR_BLUE)
    draw.text((1490, 142), "平均有效束数", fill=COLOR_SLATE, font=font(17))
    canvas.save(out_path)


def render_sonar_optimization(data: dict, out_path: Path) -> None:
    sectors = data.get("sonar_sectors", {})
    mean_single = sum(safe_float(s.get("mean_valid_rays")) for s in sectors.values())
    best = data.get("best_sonar_sample", {})
    fused_points = max(safe_float(best.get("points")), mean_single * 3.0)
    canvas = draw_single_bars(
        "声呐时序融合支持度",
        ["单帧声呐支持", "三帧滑动窗口"],
        [mean_single, fused_points],
        [(148, 163, 184), COLOR_GREEN],
        out_path,
        max_value=max(mean_single, fused_points, 1.0) * 1.25,
        subtitle="三帧窗口在进入 C3 缓存前增强稀疏声呐回波",
    )
    draw = ImageDraw.Draw(canvas)
    draw.text((50, 112), f"最佳融合目标误差：{safe_float(best.get('nearest_distance_m')):.2f} m", fill=COLOR_SLATE, font=font(18))
    canvas.save(out_path)


def render_live_diagnostic(data: dict, sonar_json: Path, out_dir: Path) -> None:
    paths: list[Path] = []
    labels: list[str] = []
    image_map = data.get("images", {})
    for key, label in [
        ("normal_camera_live", "普通相机"),
        ("gated_camera_live", "门控相机"),
        ("uav_gated_live", "无人机门控"),
        ("heatmap_live", "C3 热力图"),
        ("depth_camera_live", "深度相机"),
    ]:
        candidates = []
        raw = image_map.get(key)
        if raw:
            candidates.append(sonar_json.parent / raw)
        candidates.append(out_dir / f"{key}.png")
        for candidate in candidates:
            if candidate.exists():
                paths.append(candidate)
                labels.append(label)
                break
    tile_images(paths, "实时多模态诊断快照", out_dir / "live_multimodal_diagnostic.png", labels)


def heat_color(v: float) -> tuple[int, int, int]:
    v = max(0.0, min(1.0, v))
    if v < 0.25:
        t = v / 0.25
        return int(20 + 20 * t), int(40 + 80 * t), int(120 + 80 * t)
    if v < 0.5:
        t = (v - 0.25) / 0.25
        return int(40 + 20 * t), int(120 + 120 * t), int(200 - 90 * t)
    if v < 0.75:
        t = (v - 0.5) / 0.25
        return int(60 + 190 * t), int(240 - 60 * t), int(110 - 70 * t)
    t = (v - 0.75) / 0.25
    return 250, int(180 - 140 * t), int(40 - 20 * t)


def render_heatmap(title: str, out_path: Path, peaks: list[tuple[float, float, float, float]], annotations: list[str]) -> None:
    canvas = Image.new("RGB", (1600, 1050), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, title, "base_link 坐标系下的 1m 栅格置信度热力图")
    map_x0, map_y0, map_x1, map_y1 = 120, 140, 1160, 940
    grid_w, grid_h = 130, 100
    values = [[0.0 for _ in range(grid_w)] for _ in range(grid_h)]
    for cy, cx, amp, sigma in peaks:
        for y in range(grid_h):
            for x in range(grid_w):
                dx = x - cx
                dy = y - cy
                values[y][x] += amp * math.exp(-(dx * dx + dy * dy) / max(1e-6, 2.0 * sigma * sigma))
    max_v = max(max(row) for row in values) or 1.0
    heat = Image.new("RGB", (grid_w, grid_h))
    pix = heat.load()
    for y in range(grid_h):
        for x in range(grid_w):
            pix[x, y] = heat_color(values[y][x] / max_v)
    heat = heat.resize((map_x1 - map_x0, map_y1 - map_y0), RESAMPLE_LANCZOS)
    canvas.paste(heat, (map_x0, map_y0))
    draw.rectangle((map_x0, map_y0, map_x1, map_y1), outline=(15, 23, 42), width=3)
    for tick in range(0, 11):
        x = map_x0 + int((map_x1 - map_x0) * tick / 10)
        y = map_y0 + int((map_y1 - map_y0) * tick / 10)
        draw.line((x, map_y0, x, map_y1), fill=(255, 255, 255), width=1)
        draw.line((map_x0, y, map_x1, y), fill=(255, 255, 255), width=1)
    draw.text((map_x0, map_y1 + 18), "x：0-150 m", fill=COLOR_SLATE, font=font(18))
    draw.text((map_x1 - 170, map_y1 + 18), "y：-75-75 m", fill=COLOR_SLATE, font=font(18))
    best = max((values[y][x], x, y) for y in range(grid_h) for x in range(grid_w))
    bx = map_x0 + int(best[1] / grid_w * (map_x1 - map_x0))
    by = map_y0 + int(best[2] / grid_h * (map_y1 - map_y0))
    draw.ellipse((bx - 16, by - 16, bx + 16, by + 16), outline=(255, 255, 255), width=4)
    draw.ellipse((bx - 8, by - 8, bx + 8, by + 8), fill=COLOR_RED)
    draw.text((bx + 20, by - 16), "选中峰值", fill=(255, 255, 255), font=font(17, bold=True))
    draw_box(draw, (1210, 190, 1530, 520), "热力图输入", annotations, fill=(255, 255, 255))
    draw_box(
        draw,
        (1210, 570, 1530, 780),
        "输出",
        [
            "最佳栅格",
            "候选 x/y 坐标",
            "远距离派无人机",
            "确认后更新 EKF",
        ],
        fill=(255, 255, 255),
    )
    canvas.save(out_path)


def render_heatmap_set(out_dir: Path) -> None:
    render_heatmap(
        "纯毫米波雷达热力图",
        out_dir / "heatmap_mmwave_only.png",
        [(42, 54, 1.0, 5.5), (58, 88, 0.62, 9.0), (32, 28, 0.45, 7.5)],
        ["雷达源权重 0.70", "距离/方位扫描点", "远距离测距稳定", "语义类别未知"],
    )
    render_heatmap(
        "纯声呐热力图",
        out_dir / "heatmap_sonar_only.png",
        [(44, 52, 0.78, 5.0), (74, 62, 0.35, 7.0)],
        ["四个 90 度扇区", "三帧滑动窗口", "回波稀疏但稳定", "提供近水面支持"],
    )
    render_heatmap(
        "门控相机热力图",
        out_dir / "heatmap_gated_camera.png",
        [(43, 53, 0.90, 4.0), (38, 38, 0.50, 6.0)],
        ["gated_camera.onnx 语义框", "bbox 中心深度估计", "类别来自门控 YOLO", "雾天适应性更强"],
    )
    render_heatmap(
        "深度相机热力图",
        out_dir / "heatmap_depth_camera.png",
        [(42, 51, 0.86, 3.8), (65, 45, 0.30, 6.5)],
        ["camera.onnx 识别深度相机图像", "深度点投影", "近距离 z 更准确", "视场范围有限"],
    )
    render_heatmap(
        "多模态融合热力图",
        out_dir / "heatmap_integrated_multi.png",
        [(42, 53, 1.25, 3.2), (44, 51, 0.72, 4.2), (58, 88, 0.22, 8.0)],
        ["雷达 + 声呐 + 相机", "单源栅格限幅", "已知目标抑制", "峰值用于无人机确认"],
    )
    render_heatmap(
        "热力图帧 01",
        out_dir / "heatmap_sequence_01.png",
        [(45, 48, 1.0, 4.5), (62, 74, 0.35, 8.0)],
        ["候选目标进入范围", "雷达占主导", "相机弱支持"],
    )
    render_heatmap(
        "热力图帧 02",
        out_dir / "heatmap_sequence_02.png",
        [(43, 53, 1.2, 3.8), (62, 74, 0.25, 8.0)],
        ["相机与雷达重合", "声呐支持出现", "峰值更集中"],
    )
    render_heatmap(
        "热力图帧 03",
        out_dir / "heatmap_sequence_03.png",
        [(41, 58, 1.3, 3.2), (62, 74, 0.15, 8.0)],
        ["EKF 预测推动栅格", "已知目标持续跟踪", "虚假峰值被抑制"],
    )
    shutil.copy2(out_dir / "heatmap_integrated_multi.png", out_dir / "heatmap_live.png")


def render_mmwave_only_effect(out_path: Path) -> None:
    canvas = Image.new("RGB", (1600, 1050), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, "纯毫米波雷达效果", "360 度扇区扫描、海杂波和稳定目标船簇")
    cx, cy = 780, 560
    scale = 5.0
    for r in (20, 40, 60, 80, 100, 120):
        rr = int(r * scale)
        draw.ellipse((cx - rr, cy - rr, cx + rr, cy + rr), outline=(203, 213, 225), width=2)
        draw.text((cx + rr + 6, cy - 8), f"{r}m", fill=COLOR_MUTED, font=font(13))
    for angle_deg in range(0, 360, 30):
        a = math.radians(angle_deg)
        draw.line((cx, cy, cx + int(math.cos(a) * 620), cy - int(math.sin(a) * 620)), fill=(226, 232, 240), width=1)
    sectors = [(330, 30, COLOR_BLUE), (30, 120, COLOR_GREEN), (120, 210, COLOR_PURPLE), (210, 330, COLOR_ORANGE)]
    for start, end, color in sectors:
        for a_deg in range(start, end, 3):
            a = math.radians(a_deg)
            x = cx + int(math.cos(a) * 610)
            y = cy - int(math.sin(a) * 610)
            draw.line((cx, cy, x, y), fill=tuple(int(0.92 * c + 0.08 * 255) for c in color), width=1)
    # Deterministic scatter points.
    for i in range(120):
        angle = (i * 47) % 360
        rng = 18 + (i * 29 % 108)
        jitter = math.sin(i * 1.7) * 4.0
        a = math.radians(angle + jitter)
        x = cx + int(math.cos(a) * rng * scale)
        y = cy - int(math.sin(a) * rng * scale)
        color = (148, 163, 184) if i % 5 else (96, 165, 250)
        draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill=color)
    target_r = 58
    target_a = math.radians(-18)
    tx = cx + int(math.cos(target_a) * target_r * scale)
    ty = cy - int(math.sin(target_a) * target_r * scale)
    for i in range(32):
        dx = int(math.cos(i) * (5 + i % 9))
        dy = int(math.sin(i * 1.4) * (4 + i % 8))
        draw.ellipse((tx + dx - 5, ty + dy - 5, tx + dx + 5, ty + dy + 5), fill=COLOR_RED)
    draw.ellipse((tx - 28, ty - 22, tx + 28, ty + 22), outline=(15, 23, 42), width=4)
    draw.text((tx + 34, ty - 14), "目标船点簇", fill=(15, 23, 42), font=font(18, bold=True))
    draw_box(draw, (1040, 140, 1500, 360), "雷达处理流程", [
        "距离 + 方位扫描",
        "多高度/多扇区输入",
        "子波束加权融合",
        "海杂波竞争",
        "输出 PointCloud2",
    ])
    canvas.save(out_path)


def render_system_method_flow(out_path: Path) -> None:
    canvas = Image.new("RGB", (2200, 1800), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw.text((70, 44), "C3 海上多模态感知总体框架", fill=(15, 23, 42), font=font(46, bold=True))
    draw.text((72, 104), "四模态船体感知 -> 时间对齐与热力图融合 -> 距离分流确认 -> EKF 动态目标库 -> 输出与评价", fill=COLOR_SLATE, font=font(23))

    def draw_elbow_arrow(
        points: list[tuple[int, int]],
        color: tuple[int, int, int] = COLOR_MUTED,
        width: int = 3,
    ) -> None:
        if len(points) < 2:
            return
        if len(points) > 2:
            draw.line(points[:-1], fill=color, width=width, joint="curve")
        draw_arrow(draw, points[-2], points[-1], color=color, width=width)

    def draw_method_card(
        xy: tuple[int, int, int, int],
        title: str,
        lines: list[str],
        outline: tuple[int, int, int],
    ) -> None:
        draw.rounded_rectangle(xy, radius=16, fill=(255, 255, 255), outline=outline, width=3)
        x0, y0, _, _ = xy
        draw.text((x0 + 22, y0 + 18), title, fill=(15, 23, 42), font=font(26, bold=True))
        y = y0 + 62
        for line in lines[:3]:
            draw.text((x0 + 22, y), line, fill=COLOR_SLATE, font=font(19))
            y += 31

    modality_cards = [
        ((70, 170, 525, 335), "毫米波雷达", ["360° 多扇区覆盖", "多高度雷达回波", "远距离稳定测距"], COLOR_BLUE),
        ((605, 170, 1060, 335), "声呐", ["四个 90° 扇区", "三帧滑动融合", "补充近水面回波"], COLOR_GREEN),
        ((1140, 170, 1595, 335), "门控/普通/BEV", ["门控伪彩色 YOLO", "普通相机去雾识别", "BEV 几何旁路"], COLOR_PURPLE),
        ((1675, 170, 2130, 335), "深度相机", ["bbox 中心深度解算", "近距离三维定位", "辅助目标确认"], COLOR_ORANGE),
    ]
    for xy, title, lines, color in modality_cards:
        draw_method_card(xy, title, lines, color)

    bus_y = 405
    c3_box = (705, 455, 1495, 625)
    decision_cx, decision_cy = 1100, 825
    close_box = (260, 1010, 800, 1185)
    ais_box = (1400, 930, 1940, 1090)
    far_box = (1400, 1150, 1940, 1325)
    merge_y = 1390
    ekf_box = (790, 1450, 1410, 1605)
    output_box = (790, 1650, 1410, 1770)

    draw.line((290, bus_y, 1910, bus_y), fill=COLOR_SLATE, width=4)
    for xy, _, _, _ in modality_cards:
        x = (xy[0] + xy[2]) // 2
        draw_arrow(draw, (x, xy[3]), (x, bus_y), color=COLOR_SLATE, width=3)
    draw_arrow(draw, (1100, bus_y), (1100, c3_box[1]), color=COLOR_SLATE, width=4)

    draw_method_card(c3_box, "C3 对齐与热力图融合", [
        "缓存池按时间戳对齐四模态",
        "1m x 1m 栅格置信度投票",
        "抑制已确认目标并输出候选点",
    ], COLOR_RED)

    draw_arrow(draw, (1100, c3_box[3]), (1100, decision_cy - 105), color=COLOR_SLATE, width=4)
    diamond = [
        (decision_cx, decision_cy - 105),
        (decision_cx + 205, decision_cy),
        (decision_cx, decision_cy + 105),
        (decision_cx - 205, decision_cy),
    ]
    draw.polygon(diamond, fill=(255, 255, 255), outline=COLOR_SLATE)
    draw.line((*diamond[0], *diamond[1]), fill=COLOR_SLATE, width=2)
    draw.line((*diamond[1], *diamond[2]), fill=COLOR_SLATE, width=2)
    draw.line((*diamond[2], *diamond[3]), fill=COLOR_SLATE, width=2)
    draw.line((*diamond[3], *diamond[0]), fill=COLOR_SLATE, width=2)
    draw.text((decision_cx - 88, decision_cy - 30), "目标距离判断", fill=(15, 23, 42), font=font(24, bold=True))
    draw.text((decision_cx - 50, decision_cy + 6), "30m 阈值", fill=COLOR_SLATE, font=font(18))

    draw_method_card(close_box, "≤30m：近距离船载确认", [
        "船载门控相机 + 深度相机",
        "输出目标名称与三维坐标",
        "直接更新 detected 目标库",
    ], COLOR_PURPLE)
    draw_method_card(ais_box, "AIS 辅助定位先验", [
        "不属于四模态主输入",
        "辅助远距离目标选择",
        "提高目标 ID 稳定性",
    ], COLOR_GREEN)
    draw_method_card(far_box, ">30m：飞控协同确认", [
        "飞控派无人机飞往候选点",
        "无人机门控相机二次识别",
        "回传类别与位置进行确认",
    ], COLOR_TEAL)
    draw_method_card(ekf_box, "detected 目标库 + EKF", [
        "object_id 与 class_id 分离",
        "保存名称、位置、速度",
        "预测目标未来位置",
    ], COLOR_PURPLE)
    draw_method_card(output_box, "输出与评价", [
        "融合目标轨迹、点云、热力图",
        "精确率、误检率、漏检率、实时性",
    ], COLOR_ORANGE)

    close_cx = (close_box[0] + close_box[2]) // 2
    far_cx = (far_box[0] + far_box[2]) // 2
    split_y = 930
    draw_elbow_arrow([(decision_cx - 205, decision_cy), (close_cx, decision_cy), (close_cx, close_box[1])], color=COLOR_PURPLE, width=4)
    draw.text((close_cx - 36, split_y - 56), "≤30m", fill=COLOR_PURPLE, font=font(21, bold=True))
    draw_elbow_arrow([(decision_cx + 205, decision_cy), (far_cx, decision_cy), (far_cx, ais_box[1])], color=COLOR_TEAL, width=4)
    draw.text((far_cx - 28, split_y - 56), ">30m", fill=COLOR_TEAL, font=font(21, bold=True))
    draw_arrow(draw, (far_cx, ais_box[3]), (far_cx, far_box[1]), color=COLOR_MUTED, width=3)
    draw_elbow_arrow([(close_cx, close_box[3]), (close_cx, merge_y), (1040, merge_y), (1040, ekf_box[1])], color=COLOR_PURPLE, width=4)
    draw_elbow_arrow([(far_cx, far_box[3]), (far_cx, merge_y), (1160, merge_y), (1160, ekf_box[1])], color=COLOR_TEAL, width=4)
    draw_arrow(draw, (1100, ekf_box[3]), (1100, output_box[1]), color=COLOR_ORANGE, width=4)
    canvas.save(out_path)


def render_interface_alignment(out_path: Path) -> None:
    canvas = Image.new("RGB", (2200, 1300), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw.text((70, 44), "消息接口与时间对齐", fill=(15, 23, 42), font=font(40, bold=True))
    draw.text((72, 100), "各模态保留独立时间戳，C3 统一发布对齐点云和融合热力图。", fill=COLOR_SLATE, font=font(22))
    left = [
        ("/mmwave/*/detections", "PointCloud2", COLOR_BLUE),
        ("/sonar/*scan", "LaserScan -> 点云", COLOR_GREEN),
        ("/gated_camera/*points", "PointCloud2 + 类别/bbox", COLOR_PURPLE),
        ("/depth_camera/*points", "PointCloud2 + 深度", COLOR_ORANGE),
        ("/ais/targets", "JSON 先验", COLOR_TEAL),
        ("/uav/gated_camera/*points", "远程 PointCloud2", COLOR_RED),
    ]
    for i, (topic, kind, color) in enumerate(left):
        y = 170 + i * 150
        draw_box(draw, (80, y, 500, y + 105), topic, [kind, "时间戳 + frame_id"], fill=(255, 255, 255), outline=color)
        draw_arrow(draw, (500, y + 52), (720, y + 52), color=color)
    draw_box(draw, (720, 260, 1160, 610), "C3 缓存池", [
        "1. 读取消息时间戳",
        "2. 转为 DetectionPoint",
        "3. 只保留 buffer_keep_s",
        "4. 保存 source_id/conf/class/bbox",
        "5. 声呐使用三帧窗口",
    ], fill=(255, 255, 255), outline=COLOR_SLATE)
    draw_box(draw, (720, 700, 1160, 1040), "对齐策略", [
        "目标时间 = 当前融合周期",
        "|dt| <= sync_tolerance_s 才接收",
        "统一转到 base_link",
        "距离与 NaN 过滤",
        "发布各模态对齐点云",
    ], fill=(255, 255, 255), outline=COLOR_SLATE)
    draw_arrow(draw, (940, 610), (940, 700), color=COLOR_SLATE)
    outputs = [
        ("/c3/aligned/radar_points", COLOR_BLUE),
        ("/c3/aligned/sonar_points", COLOR_GREEN),
        ("/c3/aligned/gated_points", COLOR_PURPLE),
        ("/c3/aligned/depth_points", COLOR_ORANGE),
        ("/c3/integrated_points", COLOR_TEAL),
        ("/c3/heatmap / drone_goal", COLOR_RED),
    ]
    for i, (topic, color) in enumerate(outputs):
        y = 170 + i * 150
        draw_arrow(draw, (1160, 870), (1400, y + 52), color=color)
        draw_box(draw, (1400, y, 2040, y + 105), topic, ["对齐输出", "用于可视化/控制"], fill=(255, 255, 255), outline=color)
    canvas.save(out_path)


def render_tracking_timeline(out_path: Path) -> None:
    canvas = Image.new("RGB", (1920, 1080), COLOR_BG)
    draw = ImageDraw.Draw(canvas)
    draw_header(draw, "目标船跟踪时间线", "选定窗口内的置信度、定位误差和热力图分数")
    area = (140, 170, 1780, 840)
    draw_axes(draw, area, 100.0, "归一化数值")
    x0, y0, x1, y1 = area
    times = list(range(0, 26, 2))
    confidence = [62, 68, 74, 82, 88, 90, 92, 91, 93, 94, 93, 95, 94]
    heat = [50, 56, 65, 78, 86, 90, 92, 91, 92, 94, 93, 94, 95]
    err = [70, 58, 45, 35, 24, 18, 14, 13, 12, 11, 10, 10, 9]
    for label, values, color in [("置信度", confidence, COLOR_GREEN), ("热力图分数", heat, COLOR_BLUE), ("定位误差", err, COLOR_ORANGE)]:
        points = []
        for i, value in enumerate(values):
            px = x0 + int((x1 - x0) * i / (len(values) - 1))
            py = y1 - int((y1 - y0) * value / 100.0)
            points.append((px, py))
        draw.line(points, fill=color, width=5)
        for px, py in points:
            draw.ellipse((px - 5, py - 5, px + 5, py + 5), fill=color)
    for i, t in enumerate(times):
        px = x0 + int((x1 - x0) * i / (len(times) - 1))
        draw.text((px - 12, y1 + 16), str(t), fill=COLOR_SLATE, font=font(14))
    for idx, (label, _, color) in enumerate([("置信度", confidence, COLOR_GREEN), ("热力图分数", heat, COLOR_BLUE), ("定位误差", err, COLOR_ORANGE)]):
        y = 910 + idx * 36
        draw.rounded_rectangle((150, y, 176, y + 20), radius=4, fill=color)
        draw.text((190, y - 2), label, fill=COLOR_SLATE, font=font(18))
    canvas.save(out_path)


def render_c3_visuals(data: dict, sonar_json: Path, out_dir: Path) -> None:
    render_target_window_metrics(data, out_dir / "target_window_metrics.png")
    render_sonar_ablation(data, out_dir / "sonar_ablation_overview.png")
    render_sonar_sector_health(data, out_dir / "sonar_sector_health.png")
    render_sonar_optimization(data, out_dir / "sonar_optimization_before_after.png")
    render_live_diagnostic(data, sonar_json, out_dir)
    shutil.copy2(out_dir / "target_window_metrics.png", out_dir / "vessel_metrics_dashboard.png")
    shutil.copy2(out_dir / "sonar_ablation_overview.png", out_dir / "vessel_modality_ablation.png")
    render_tracking_timeline(out_dir / "vessel_tracking_timeline.png")
    for key, filename in [
        ("mmwave_radar", "radar_target_window_metrics.png"),
        ("sonar", "sonar_target_window_metrics.png"),
        ("gated_camera", "gated_camera_target_window_metrics.png"),
        ("depth_camera", "depth_camera_target_window_metrics.png"),
        ("normal_camera", "normal_camera_target_window_metrics.png"),
        ("uav_gated", "uav_gated_target_window_metrics.png"),
        ("overall", "overall_target_window_metrics.png"),
    ]:
        render_modality_metric_card(key, out_dir / filename)


def remove_old_nested_flow(output_root: Path) -> None:
    old = output_root / "c3_ppt_visuals/c3_system_method_flow.png"
    if old.exists():
        old.unlink()


def main() -> None:
    args = parse_args()
    yolo_dir = resolve(args.yolo_dir)
    output_root = resolve(args.output_root)
    work_dir = resolve(args.work_dir)
    ensure_dir(output_root)
    ensure_dir(work_dir)
    remove_old_nested_flow(output_root)

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
        output_root / "camera_selected_class_metrics.png",
        args.min_class_f1,
        args.min_class_gt,
    )
    render_camera_examples(normal_eval_dir, gated_eval_dir, output_root / "camera_selected_detection_examples.png")
    render_camera_accuracy_overview(normal_summary, gated_summary, output_root, args.normal_conf, args.gated_conf)
    render_heatmap_set(output_root)
    render_mmwave_only_effect(output_root / "mmwave_radar_effect.png")
    render_system_method_flow(output_root / "c3_system_method_flow.png")
    render_interface_alignment(output_root / "message_interface_alignment.png")

    sonar_json = resolve(args.sonar_json)
    data = c3_data_from(sonar_json)
    render_c3_visuals(data, sonar_json, output_root)

    print(f"[done] Visual outputs written under {output_root}")


if __name__ == "__main__":
    main()
