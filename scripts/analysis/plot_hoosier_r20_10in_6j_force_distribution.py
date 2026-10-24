#!/usr/bin/env python3
"""Plot vertical load vs resultant tire force for Hoosier 18.0x6.0-10 R20, 6J."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT / "data" / "tiredata"
OUTPUT_DIR = ROOT / "output" / "tire_plots"

CORNERING_RUNS = [
    DATA_DIR / "RunData_Cornering_ASCII_SI_Round9" / "B2356run27.dat",
    DATA_DIR / "RunData_Cornering_ASCII_SI_Round9" / "B2356run28.dat",
    DATA_DIR / "RunData_Cornering_ASCII_SI_Round9" / "B2356run29.dat",
]

DRIVE_BRAKE_RUNS = [
    DATA_DIR / "RunData_DriveBrake_ASCII_SI_Round9" / "B2356run68.dat",
    DATA_DIR / "RunData_DriveBrake_ASCII_SI_Round9" / "B2356run69.dat",
    DATA_DIR / "RunData_DriveBrake_ASCII_SI_Round9" / "B2356run70.dat",
]


def load_tirf_dat(path: Path, source: str) -> pd.DataFrame:
    """Read a TTC/TIRF ASCII SI .dat file with metadata and units header rows."""
    df = pd.read_csv(path, sep=r"\s+", skiprows=[0, 2], engine="python")
    df["source"] = source
    df["run"] = path.stem
    return df


def load_dataset() -> pd.DataFrame:
    frames = [
        *(load_tirf_dat(path, "Cornering") for path in CORNERING_RUNS),
        *(load_tirf_dat(path, "Drive/Brake") for path in DRIVE_BRAKE_RUNS),
    ]
    data = pd.concat(frames, ignore_index=True)
    data["vertical_load_n"] = data["FZ"].abs()
    data["resultant_force_n"] = np.hypot(data["FX"], data["FY"])
    return data.replace([np.inf, -np.inf], np.nan).dropna(
        subset=["vertical_load_n", "resultant_force_n", "FX", "FY"]
    )


def save_summary(data: pd.DataFrame, path: Path) -> None:
    summary = (
        data.groupby("source")
        .agg(
            rows=("resultant_force_n", "size"),
            vertical_load_min_n=("vertical_load_n", "min"),
            vertical_load_max_n=("vertical_load_n", "max"),
            vertical_load_mean_n=("vertical_load_n", "mean"),
            resultant_force_min_n=("resultant_force_n", "min"),
            resultant_force_max_n=("resultant_force_n", "max"),
            resultant_force_mean_n=("resultant_force_n", "mean"),
        )
        .round(2)
    )
    summary.loc["All"] = {
        "rows": len(data),
        "vertical_load_min_n": round(data["vertical_load_n"].min(), 2),
        "vertical_load_max_n": round(data["vertical_load_n"].max(), 2),
        "vertical_load_mean_n": round(data["vertical_load_n"].mean(), 2),
        "resultant_force_min_n": round(data["resultant_force_n"].min(), 2),
        "resultant_force_max_n": round(data["resultant_force_n"].max(), 2),
        "resultant_force_mean_n": round(data["resultant_force_n"].mean(), 2),
    }
    summary.to_csv(path)


def plot_distribution(data: pd.DataFrame, path: Path) -> None:
    plt.style.use("seaborn-v0_8-whitegrid")
    fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)

    hb = ax.hexbin(
        data["vertical_load_n"],
        data["resultant_force_n"],
        gridsize=75,
        mincnt=1,
        bins="log",
        cmap="viridis",
    )
    cbar = fig.colorbar(hb, ax=ax)
    cbar.set_label("Sample count (log scale)")

    for source, color in [("Cornering", "#e15759"), ("Drive/Brake", "#4e79a7")]:
        subset = data[data["source"] == source]
        ax.scatter(
            subset["vertical_load_n"],
            subset["resultant_force_n"],
            s=1.5,
            alpha=0.08,
            linewidths=0,
            color=color,
            label=source,
        )

    ax.set_title("Hoosier 18.0x6.0-10 R20 on 6J: vertical load vs resultant tire force")
    ax.set_xlabel("Vertical load |FZ| [N]")
    ax.set_ylabel("Resultant tire force sqrt(FX^2 + FY^2) [N]")
    ax.legend(markerscale=8, frameon=True)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)

    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    data = load_dataset()
    plot_distribution(data, OUTPUT_DIR / "hoosier_18x6_10_r20_6j_force_distribution.png")
    save_summary(data, OUTPUT_DIR / "hoosier_18x6_10_r20_6j_force_distribution_summary.csv")

    print(f"Loaded {len(data):,} samples")
    print(f"Output: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
