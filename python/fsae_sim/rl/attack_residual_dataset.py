"""Dataset builder for attack-driver residual policies."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

FEATURE_NAMES = [
    "station_fraction",
    "speed",
    "course_v_ref",
    "course_target_gap",
    "dynamic_cap_gap",
    "course_lateral_error",
    "d_heading",
    "course_kappa_ref",
    "attack_dynamic_speed_cap",
    "attack_heading_speed_cap",
    "attack_margin_speed_cap",
    "mpc_predicted_min_margin",
    "attack_selected_speed_scale",
    "throttle",
    "brake",
    "healthy_underspeed",
]

ACTION_NAMES = [
    "delta_v_ref_mps",
    "delta_steer_ff_rad",
]


@dataclass(frozen=True)
class AttackResidualDataset:
    features: np.ndarray
    actions: np.ndarray
    stations: np.ndarray
    feature_names: tuple[str, ...] = tuple(FEATURE_NAMES)
    action_names: tuple[str, ...] = tuple(ACTION_NAMES)
    metadata: dict[str, object] | None = None


def _finite_float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        value = float(row.get(key, default))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _truthy(row: dict[str, str], key: str) -> float:
    return 1.0 if str(row.get(key, "")).strip().lower() in {"1", "true", "yes"} else 0.0


def _mean(values: Sequence[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return 0.0
    return sum(finite) / len(finite)


def _rows_from_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def _load_teacher_samples(path: Path | None) -> list[dict[str, float]]:
    if path is None:
        return []
    root = json.loads(path.read_text(encoding="utf-8"))
    samples = []
    for item in root.get("samples", []):
        if isinstance(item, dict):
            samples.append(
                {
                    "s": float(item.get("s", item.get("s_m", 0.0))),
                    "delta_v_ref_mps": float(
                        item.get("delta_v_ref_mps", item.get("delta_v_mps", 0.0))
                    ),
                    "delta_steer_ff_rad": float(
                        item.get("delta_steer_ff_rad", item.get("delta_steer_rad", 0.0))
                    ),
                }
            )
        elif isinstance(item, list) and len(item) >= 2:
            samples.append(
                {
                    "s": float(item[0]),
                    "delta_v_ref_mps": float(item[1]),
                    "delta_steer_ff_rad": float(item[2]) if len(item) >= 3 else 0.0,
                }
            )
    return sorted(samples, key=lambda sample: sample["s"])


def _interpolate_teacher(
    samples: Sequence[dict[str, float]],
    station: float,
    key: str,
) -> float | None:
    if not samples:
        return None
    if station <= samples[0]["s"]:
        return samples[0][key]
    if station >= samples[-1]["s"]:
        return samples[-1][key]
    hi = 1
    while hi < len(samples) and samples[hi]["s"] < station:
        hi += 1
    lo = hi - 1
    lo_s = samples[lo]["s"]
    hi_s = samples[hi]["s"]
    alpha = (station - lo_s) / max(hi_s - lo_s, 1e-9)
    return samples[lo][key] + alpha * (samples[hi][key] - samples[lo][key])


def _heuristic_action(
    rows: Sequence[dict[str, str]],
    *,
    station: float,
    teacher_samples: Sequence[dict[str, float]],
    max_delta_v_ref_mps: float,
    max_delta_steer_ff_rad: float,
) -> tuple[float, float]:
    teacher_v = _interpolate_teacher(teacher_samples, station, "delta_v_ref_mps")
    teacher_steer = _interpolate_teacher(teacher_samples, station, "delta_steer_ff_rad")
    if teacher_v is not None and teacher_steer is not None:
        return (
            float(np.clip(teacher_v, -max_delta_v_ref_mps, max_delta_v_ref_mps)),
            float(np.clip(teacher_steer, -max_delta_steer_ff_rad, max_delta_steer_ff_rad)),
        )

    target_gap = _mean([_finite_float(row, "course_target_gap") for row in rows])
    dynamic_cap_gap = _mean([_finite_float(row, "dynamic_cap_gap") for row in rows])
    throttle = _mean([_finite_float(row, "throttle") for row in rows])
    kappa = _mean([_finite_float(row, "course_kappa_ref") for row in rows])
    lateral_error = _mean([_finite_float(row, "course_lateral_error") for row in rows])
    heading = _mean([_finite_float(row, "d_heading") for row in rows])

    delta_v = 0.0
    if target_gap > 1.0 and dynamic_cap_gap > -0.25 and throttle > 0.15:
        delta_v = min(max_delta_v_ref_mps, 0.20 * target_gap)
    elif dynamic_cap_gap < -0.75 or abs(heading) > 0.25:
        delta_v = max(-max_delta_v_ref_mps, 0.25 * dynamic_cap_gap)

    steer_sign = -1.0 if kappa > 0.0 else 1.0
    steer_need = 0.0
    if abs(kappa) > 0.02:
        steer_need += 0.35 * abs(lateral_error)
        steer_need += 0.04 * abs(heading)
    delta_steer = steer_sign * min(max_delta_steer_ff_rad, steer_need)
    return float(delta_v), float(delta_steer)


def _feature_vector(
    rows: Sequence[dict[str, str]],
    station: float,
    track_length_m: float,
) -> list[float]:
    row_means = {
        "speed": _mean([_finite_float(row, "speed") for row in rows]),
        "course_v_ref": _mean([_finite_float(row, "course_v_ref") for row in rows]),
        "course_target_gap": _mean([_finite_float(row, "course_target_gap") for row in rows]),
        "dynamic_cap_gap": _mean([_finite_float(row, "dynamic_cap_gap") for row in rows]),
        "course_lateral_error": _mean([_finite_float(row, "course_lateral_error") for row in rows]),
        "d_heading": _mean([_finite_float(row, "d_heading") for row in rows]),
        "course_kappa_ref": _mean([_finite_float(row, "course_kappa_ref") for row in rows]),
        "attack_dynamic_speed_cap": _mean(
            [_finite_float(row, "attack_dynamic_speed_cap") for row in rows]
        ),
        "attack_heading_speed_cap": _mean(
            [_finite_float(row, "attack_heading_speed_cap") for row in rows]
        ),
        "attack_margin_speed_cap": _mean(
            [_finite_float(row, "attack_margin_speed_cap") for row in rows]
        ),
        "mpc_predicted_min_margin": _mean(
            [_finite_float(row, "mpc_predicted_min_margin") for row in rows]
        ),
        "attack_selected_speed_scale": _mean(
            [_finite_float(row, "attack_selected_speed_scale") for row in rows]
        ),
        "throttle": _mean([_finite_float(row, "throttle") for row in rows]),
        "brake": _mean([_finite_float(row, "brake") for row in rows]),
        "healthy_underspeed": _mean([_truthy(row, "healthy_underspeed") for row in rows]),
    }
    return [
        station / max(track_length_m, 1e-9),
        *[row_means[name] for name in FEATURE_NAMES[1:]],
    ]


def build_attack_residual_dataset(
    csv_paths: Iterable[Path],
    *,
    bin_width_m: float = 10.0,
    track_length_m: float | None = None,
    teacher_profile: Path | None = None,
    max_delta_v_ref_mps: float = 2.0,
    max_delta_steer_ff_rad: float = 0.035,
) -> AttackResidualDataset:
    """Aggregate attack logs into station-binned residual learning samples."""

    all_rows: list[dict[str, str]] = []
    source_paths = [Path(path) for path in csv_paths]
    for path in source_paths:
        all_rows.extend(_rows_from_csv(path))
    if not all_rows:
        raise ValueError("no CSV rows were provided")
    if bin_width_m <= 0.0:
        raise ValueError("bin_width_m must be positive")

    s_values = [_finite_float(row, "s") for row in all_rows]
    inferred_length = max(s_values) if s_values else 0.0
    effective_length = float(track_length_m or inferred_length)
    if effective_length <= 0.0:
        raise ValueError("track length could not be inferred")

    grouped: dict[int, list[dict[str, str]]] = {}
    for row in all_rows:
        station = _finite_float(row, "s")
        index = int(math.floor(station / bin_width_m))
        grouped.setdefault(index, []).append(row)

    teacher_samples = _load_teacher_samples(teacher_profile)
    features: list[list[float]] = []
    actions: list[tuple[float, float]] = []
    stations: list[float] = []
    for index in sorted(grouped):
        rows = grouped[index]
        station = _mean([_finite_float(row, "s") for row in rows])
        stations.append(station)
        features.append(_feature_vector(rows, station, effective_length))
        actions.append(
            _heuristic_action(
                rows,
                station=station,
                teacher_samples=teacher_samples,
                max_delta_v_ref_mps=max_delta_v_ref_mps,
                max_delta_steer_ff_rad=max_delta_steer_ff_rad,
            )
        )

    metadata = {
        "schema": "attack_residual_dataset_v1",
        "source_csv": [str(path) for path in source_paths],
        "teacher_profile": str(teacher_profile) if teacher_profile else None,
        "bin_width_m": bin_width_m,
        "track_length_m": effective_length,
        "max_delta_v_ref_mps": max_delta_v_ref_mps,
        "max_delta_steer_ff_rad": max_delta_steer_ff_rad,
    }
    return AttackResidualDataset(
        features=np.asarray(features, dtype=np.float32),
        actions=np.asarray(actions, dtype=np.float32),
        stations=np.asarray(stations, dtype=np.float32),
        metadata=metadata,
    )


def save_attack_residual_dataset(dataset: AttackResidualDataset, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        features=dataset.features,
        actions=dataset.actions,
        stations=dataset.stations,
        feature_names=np.asarray(dataset.feature_names),
        action_names=np.asarray(dataset.action_names),
        metadata=json.dumps(dataset.metadata or {}, sort_keys=True),
    )
