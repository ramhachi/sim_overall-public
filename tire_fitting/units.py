"""Unit contracts and conversions for TTC tire fitting data."""

from __future__ import annotations

from typing import Tuple

import numpy as np
import pandas as pd


TTC_INPUT_UNITS = {
    "SR": "ratio",
    "SA": "deg",
    "FZ": "N",
    "IA": "deg",
    "TSTC": "degC",
}

MODEL_INPUT_UNITS = {
    "slip_ratio": "ratio",
    "slip_angle": "rad",
    "vertical_load": "N",
    "camber_angle": "rad",
}


def degrees_to_radians(values: np.ndarray | pd.Series, column_name: str) -> np.ndarray:
    """Convert a TTC angle column from degrees to radians for analytic tire models."""
    if column_name not in {"SA", "IA"}:
        raise ValueError(f"Unsupported TTC angle column: {column_name}")
    return np.deg2rad(np.asarray(values, dtype=float))


def ttc_lateral_angles_to_radians(
    frame: pd.DataFrame,
    sa_column: str = "SA",
    ia_column: str = "IA",
) -> Tuple[np.ndarray, np.ndarray]:
    """Return lateral slip and camber angles in radians from TTC degree columns."""
    missing = [column for column in (sa_column, ia_column) if column not in frame.columns]
    if missing:
        raise ValueError(f"Missing TTC angle column(s): {missing}")
    return (
        degrees_to_radians(frame[sa_column], "SA"),
        degrees_to_radians(frame[ia_column], "IA"),
    )
