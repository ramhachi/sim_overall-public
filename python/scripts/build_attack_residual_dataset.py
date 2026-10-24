#!/usr/bin/env python3
"""Build an NPZ dataset for attack residual-policy pretraining."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from fsae_sim.rl import (  # noqa: E402
    build_attack_residual_dataset,
    save_attack_residual_dataset,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", action="append", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--teacher-profile", type=Path)
    parser.add_argument("--bin-width-m", type=float, default=10.0)
    parser.add_argument("--track-length-m", type=float)
    parser.add_argument("--max-delta-v-ref-mps", type=float, default=2.0)
    parser.add_argument("--max-delta-steer-ff-rad", type=float, default=0.035)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    dataset = build_attack_residual_dataset(
        args.csv,
        bin_width_m=args.bin_width_m,
        track_length_m=args.track_length_m,
        teacher_profile=args.teacher_profile,
        max_delta_v_ref_mps=args.max_delta_v_ref_mps,
        max_delta_steer_ff_rad=args.max_delta_steer_ff_rad,
    )
    save_attack_residual_dataset(dataset, args.output)
    print(
        json.dumps(
            {
                "output": str(args.output),
                "sample_count": int(dataset.features.shape[0]),
                "feature_dim": int(dataset.features.shape[1]),
                "action_dim": int(dataset.actions.shape[1]),
                "metadata": dataset.metadata,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
