#!/usr/bin/env python3
"""Visualize fixed-course curvature radius, speed, and attack speed caps."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import generate_course_speed_caps as course_caps  # noqa: E402


def finite_float(value: Any, default: float = math.nan) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def load_raw_track_points(path: Path) -> list[course_caps.TrackPoint]:
    rows: list[tuple[float, float, float]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append((
                float(row["x"]),
                float(row["y"]),
                float(row.get("width", 3.0)),
            ))
    cumulative = [0.0]
    for i in range(1, len(rows)):
        dx = rows[i][0] - rows[i - 1][0]
        dy = rows[i][1] - rows[i - 1][1]
        cumulative.append(cumulative[-1] + math.hypot(dx, dy))
    return [
        course_caps.TrackPoint(x=x, y=y, width=w, s=s)
        for (x, y, w), s in zip(rows, cumulative)
    ]


def load_laptime_log(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "time": finite_float(row.get("time")),
                "x": finite_float(row.get("x")),
                "y": finite_float(row.get("y")),
                "s": finite_float(row.get("s")),
                "speed": finite_float(row.get("speed")),
                "target_speed": finite_float(row.get("target_speed")),
                "target_speed_final": finite_float(row.get("target_speed_final")),
                "kappa": finite_float(row.get("kappa")),
                "lateral_error": finite_float(row.get("lateral_error")),
                "attack_speed_scale": finite_float(row.get("attack_selected_speed_scale")),
            })
    return [row for row in rows if math.isfinite(row["s"]) and math.isfinite(row["speed"])]


def radius_from_kappa(kappa: float) -> float:
    return 1.0 / abs(kappa) if math.isfinite(kappa) and abs(kappa) > 1e-6 else math.inf


def trajectory_radius_samples(
    log_rows: list[dict[str, float]],
    *,
    min_step_m: float = 0.75,
    smooth_half_window: int = 3,
) -> list[dict[str, float]]:
    sampled_points: list[course_caps.TrackPoint] = []
    sampled_course_s: list[float] = []
    sampled_speed: list[float] = []
    last_xy: tuple[float, float] | None = None
    path_s = 0.0
    for row in log_rows:
        x = row["x"]
        y = row["y"]
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        if last_xy is None:
            sampled_points.append(course_caps.TrackPoint(x=x, y=y, width=0.0, s=path_s))
            sampled_course_s.append(row["s"])
            sampled_speed.append(row["speed"])
            last_xy = (x, y)
            continue
        ds = math.hypot(x - last_xy[0], y - last_xy[1])
        if ds < min_step_m:
            continue
        path_s += ds
        sampled_points.append(course_caps.TrackPoint(x=x, y=y, width=0.0, s=path_s))
        sampled_course_s.append(row["s"])
        sampled_speed.append(row["speed"])
        last_xy = (x, y)
    if len(sampled_points) < 4:
        return []
    curved = course_caps.with_curvature(sampled_points, smooth_half_window=smooth_half_window)
    return [
        {
            "s": course_s,
            "speed": speed,
            "radius": radius_from_kappa(point.kappa),
            "kappa": point.kappa,
        }
        for point, course_s, speed in zip(curved, sampled_course_s, sampled_speed)
    ]


def quantiles(values: list[float], qs: list[float]) -> dict[str, float]:
    finite = sorted(v for v in values if math.isfinite(v))
    if not finite:
        return {str(q): math.nan for q in qs}
    out: dict[str, float] = {}
    for q in qs:
        idx = min(len(finite) - 1, max(0, int(q * (len(finite) - 1))))
        out[str(q)] = finite[idx]
    return out


def load_cap_payload(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    return json.loads(path.read_text())


def cap_points(payload: dict[str, Any]) -> list[dict[str, float]]:
    out: list[dict[str, float]] = []
    for item in payload.get("speed_caps", []):
        if not isinstance(item, dict):
            continue
        out.append({
            "s": finite_float(item.get("s", item.get("s_m"))),
            "speed": finite_float(item.get("speed", item.get("speed_mps"))),
            "radius": finite_float(item.get("radius", item.get("radius_m"))),
            "source_radius_m": finite_float(item.get("source_radius_m")),
        })
    return [item for item in out if math.isfinite(item["s"]) and math.isfinite(item["speed"])]


def cap_windows(payload: dict[str, Any]) -> list[dict[str, float]]:
    out: list[dict[str, float]] = []
    for item in payload.get("window_caps", []):
        if not isinstance(item, dict):
            continue
        out.append({
            "s_start": finite_float(item.get("s_start")),
            "s_end": finite_float(item.get("s_end")),
            "speed": finite_float(item.get("speed", item.get("speed_mps"))),
            "source_radius_m": finite_float(item.get("source_radius_m")),
        })
    return [
        item for item in out
        if math.isfinite(item["s_start"]) and math.isfinite(item["s_end"])
        and math.isfinite(item["speed"])
    ]


def plot_track_speed(ax, track_points, baseline, comparison, caps):
    ax.plot([p.x for p in track_points], [p.y for p in track_points], color="0.75", lw=1.0)
    if baseline:
        stride = max(1, len(baseline) // 1200)
        sc = ax.scatter(
            [r["x"] for r in baseline[::stride]],
            [r["y"] for r in baseline[::stride]],
            c=[r["speed"] for r in baseline[::stride]],
            s=7,
            cmap="viridis",
            label="baseline speed",
        )
        plt.colorbar(sc, ax=ax, label="m/s", fraction=0.046, pad=0.02)
    if comparison:
        ax.plot([r["x"] for r in comparison], [r["y"] for r in comparison],
                color="tab:red", lw=0.8, alpha=0.6, label="comparison trajectory")
    if caps:
        by_s = [(p.s, p.x, p.y) for p in track_points]
        for cap in caps:
            nearest = min(by_s, key=lambda p: abs(p[0] - cap["s"]))
            ax.scatter([nearest[1]], [nearest[2]], marker="x", color="black", s=24)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Track And Baseline Speed")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="best")


def plot_speed_profile(ax, baseline, comparison, caps, windows):
    if baseline:
        ax.plot([r["s"] for r in baseline], [r["speed"] for r in baseline],
                label="baseline speed", lw=1.0)
        if math.isfinite(baseline[0].get("target_speed_final", math.nan)):
            ax.plot([r["s"] for r in baseline], [r["target_speed_final"] for r in baseline],
                    label="baseline target final", lw=0.8, alpha=0.6)
    if comparison:
        ax.plot([r["s"] for r in comparison], [r["speed"] for r in comparison],
                label="comparison speed", lw=1.0)
    for window in windows:
        ax.axvspan(window["s_start"], window["s_end"], color="tab:red", alpha=0.08)
        ax.hlines(window["speed"], window["s_start"], window["s_end"],
                  color="tab:red", lw=1.0, alpha=0.6)
    if caps:
        ax.scatter([c["s"] for c in caps], [c["speed"] for c in caps],
                   color="black", s=14, label="cap points")
    ax.set_title("Speed Versus Course Distance")
    ax.set_xlabel("s [m]")
    ax.set_ylabel("speed [m/s]")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")


def plot_radius_profile(ax, raw_track_points, baseline, smoothing_windows, caps):
    for smooth in smoothing_windows:
        pts = course_caps.with_curvature(raw_track_points, smooth_half_window=smooth)
        s = [p.s for p in pts]
        radius = [min(radius_from_kappa(p.kappa), 100.0) for p in pts]
        ax.plot(s, radius, lw=0.9, label=f"CSV radius smooth={smooth}")
    if baseline:
        sample_stride = max(1, len(baseline) // 1800)
        sim_s = [r["s"] for r in baseline[::sample_stride]]
        sim_radius = [min(radius_from_kappa(r["kappa"]), 100.0) for r in baseline[::sample_stride]]
        ax.scatter(sim_s, sim_radius, s=4, alpha=0.35, label="laptime kappa radius")
        trajectory = trajectory_radius_samples(baseline)
        if trajectory:
            ax.scatter(
                [r["s"] for r in trajectory],
                [min(r["radius"], 100.0) for r in trajectory],
                s=6,
                color="tab:red",
                alpha=0.35,
                label="trajectory-fit radius",
            )
    if caps:
        cap_radius = [
            min(c.get("source_radius_m", math.nan), 100.0)
            for c in caps
        ]
        ax.scatter([c["s"] for c in caps], cap_radius, color="black", s=14,
                   label="cap source radius")
    ax.set_ylim(0, 80)
    ax.set_title("Curvature Radius Along Course")
    ax.set_xlabel("s [m]")
    ax.set_ylabel("radius [m] (clipped at 100)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")


def write_summary(path: Path, raw_track_points, baseline, caps, windows) -> dict[str, Any]:
    radii_from_sim = [radius_from_kappa(r["kappa"]) for r in baseline if abs(r["kappa"]) > 1e-4]
    radii_from_trajectory = [
        r["radius"] for r in trajectory_radius_samples(baseline)
        if math.isfinite(r["radius"])
    ]
    cap_source_radii = [c["source_radius_m"] for c in caps if math.isfinite(c["source_radius_m"])]
    summary = {
        "track_length_m": raw_track_points[-1].s if raw_track_points else 0.0,
        "baseline_speed": quantiles([r["speed"] for r in baseline], [0.0, 0.05, 0.25, 0.5, 0.75, 0.95, 1.0]),
        "sim_kappa_radius_m": quantiles(radii_from_sim, [0.0, 0.01, 0.05, 0.1, 0.25, 0.5]),
        "trajectory_fit_radius_m": quantiles(radii_from_trajectory, [0.0, 0.01, 0.05, 0.1, 0.25, 0.5]),
        "cap_count": len(caps),
        "window_count": len(windows),
        "cap_source_radius_m": quantiles(cap_source_radii, [0.0, 0.05, 0.25, 0.5, 0.75, 1.0]),
        "tightest_sim_radius_samples": sorted(
            [
                {"s": r["s"], "radius_m": radius_from_kappa(r["kappa"]), "speed_mps": r["speed"]}
                for r in baseline if abs(r["kappa"]) > 1e-4
            ],
            key=lambda x: x["radius_m"],
        )[:12],
    }
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track-csv", type=Path, default=REPO_ROOT / "data/ASE_autocross_sim.csv")
    parser.add_argument("--baseline-csv", type=Path, required=True)
    parser.add_argument("--comparison-csv", type=Path)
    parser.add_argument("--cap-json", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--smoothing-windows", default="2,5,10")
    args = parser.parse_args(argv)

    raw_track_points = load_raw_track_points(args.track_csv)
    baseline = load_laptime_log(args.baseline_csv)
    comparison = load_laptime_log(args.comparison_csv) if args.comparison_csv else []
    payload = load_cap_payload(args.cap_json)
    caps = cap_points(payload)
    windows = cap_windows(payload)
    smoothing_windows = [int(x) for x in args.smoothing_windows.split(",") if x.strip()]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(14, 15), constrained_layout=True)
    plot_track_speed(axes[0], raw_track_points, baseline, comparison, caps)
    plot_speed_profile(axes[1], baseline, comparison, caps, windows)
    plot_radius_profile(axes[2], raw_track_points, baseline, smoothing_windows, caps)
    fig.savefig(args.output, dpi=180)
    plt.close(fig)

    summary_path = args.summary or args.output.with_suffix(".summary.json")
    summary = write_summary(summary_path, raw_track_points, baseline, caps, windows)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
