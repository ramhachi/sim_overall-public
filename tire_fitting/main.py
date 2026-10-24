#!/usr/bin/env python3
"""Compatibility entrypoint for the hybrid tire fitting pipeline.

`main.py` used to contain a TMeasy-only implementation while exposing hybrid
CLI flags. Keep this public entrypoint, but delegate all fitting/export behavior
to `main_hybrid.py`, which actually fits and selects TMeasy/Magic Formula models.
"""

import sys

import numpy as np

from tire_fitting.main_hybrid import (
    load_config_file,
    main,
    merge_config_with_args,
    parse_arguments,
)


def filter_drivebrake_pure_longitudinal(drivebrake_df, sa_threshold_deg: float = 5.0):
    """Filter DriveBrake data for pure longitudinal fitting.

    TTC DriveBrake `SA` is loaded in degrees, so the threshold is also specified
    and applied in degrees.
    """
    required_cols_drivebrake = ["SR", "FX", "FZ", "SA"]
    filtered = drivebrake_df.dropna(subset=required_cols_drivebrake).copy()
    filtered = filtered[filtered["FZ"] > 0].copy()
    return filtered[np.abs(filtered["SA"]) < sa_threshold_deg].copy()


if __name__ == "__main__":
    sys.exit(main())
