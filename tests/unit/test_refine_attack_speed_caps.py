"""Tests for attack speed-cap refinement."""

from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path
import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[2]
    / "python"
    / "scripts"
    / "refine_attack_speed_caps.py"
)
SPEC = importlib.util.spec_from_file_location("refine_attack_speed_caps", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
refine = importlib.util.module_from_spec(SPEC)
sys.modules["refine_attack_speed_caps"] = refine
SPEC.loader.exec_module(refine)


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def test_generate_caps_from_failed_summary_row(tmp_path: Path) -> None:
    laptime_csv = tmp_path / "failed_laptime.csv"
    _write_csv(
        laptime_csv,
        [
            {
                "time": 1.0,
                "s": 80.0,
                "speed": 9.0,
                "d_heading": 0.10,
                "mpc_predicted_min_margin": 1.2,
            },
            {
                "time": 5.0,
                "s": 112.0,
                "speed": 12.0,
                "d_heading": 0.50,
                "mpc_predicted_min_margin": 0.35,
            },
        ],
    )
    summary_csv = tmp_path / "summary.csv"
    _write_csv(
        summary_csv,
        [
            {
                "case_id": "fast_exit",
                "lap_completed": "False",
                "max_track_limit_excess": 0.4,
                "mpc_fail_count": 3,
                "csv_path": str(laptime_csv),
            }
        ],
    )

    caps = refine.generate_caps_from_summary(
        summary_csv,
        speed_factor=0.8,
        radius=12.0,
        min_speed=3.0,
    )

    assert len(caps) == 1
    assert caps[0].s == 112.0
    assert caps[0].speed == pytest.approx(9.6)
    assert caps[0].radius == 12.0
    assert caps[0].reason == "heading_margin_conflict"

    windows = refine.generate_windows_from_summary(
        summary_csv,
        speed_factor=0.8,
        min_speed=3.0,
        prefix_distance=20.0,
        post_distance=6.0,
    )

    assert len(windows) == 1
    assert windows[0].s_start == pytest.approx(92.0)
    assert windows[0].s_end == pytest.approx(118.0)
    assert windows[0].speed == pytest.approx(9.6)
    assert windows[0].source_s == pytest.approx(112.0)


def test_successful_summary_rows_do_not_generate_caps(tmp_path: Path) -> None:
    laptime_csv = tmp_path / "success_laptime.csv"
    _write_csv(
        laptime_csv,
        [{"time": 1.0, "s": 10.0, "speed": 8.0, "d_heading": 0.0}],
    )
    summary_csv = tmp_path / "summary.csv"
    _write_csv(
        summary_csv,
        [
            {
                "case_id": "baseline",
                "lap_completed": "True",
                "max_track_limit_excess": 0.0,
                "mpc_fail_count": 0,
                "csv_path": str(laptime_csv),
            }
        ],
    )

    assert refine.generate_caps_from_summary(summary_csv) == []
    assert refine.generate_windows_from_summary(summary_csv) == []
