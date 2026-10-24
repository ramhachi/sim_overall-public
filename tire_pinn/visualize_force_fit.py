#!/usr/bin/env python3
"""Create quick-look force fit plots for trained PINN tire models."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
import sys
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import torch

sys.path.append(str(Path(__file__).parent.parent))

from tire_pinn.data.ttc_loader import TTCDataLoader
from tire_pinn.main_pinn_train import (
    apply_tire_profile_overrides,
    build_pinn_core_from_config,
    build_runtime_model,
    load_checkpoint_into_model,
    load_config,
    resolve_device,
)
from tire_pinn.models.pinn_model import NormalizedTirePINN


FORCE_COMPONENTS = {
    "Fx": {"index": 0, "unit": "N", "x": "SR", "source": "Db", "sweep": "sr"},
    "Fy": {"index": 1, "unit": "N", "x": "SA", "source": "Cr", "sweep": "sa"},
    "Mz": {"index": 3, "unit": "Nm", "x": "SA", "source": "Cr", "sweep": "sa"},
}


def r2_rmse(actual: np.ndarray, pred: np.ndarray) -> dict[str, float]:
    residual = pred - actual
    ss_res = float(np.sum(residual**2))
    ss_tot = float(np.sum((actual - np.mean(actual)) ** 2))
    r2 = 1.0 - ss_res / max(ss_tot, 1.0e-12)
    rmse = float(np.sqrt(np.mean(residual**2)))
    mae = float(np.mean(np.abs(residual)))
    return {"r2": r2, "rmse": rmse, "mae": mae, "n": int(actual.size)}


def load_selected_checkpoint(model_dir: Path) -> Path:
    summary_path = model_dir / "training_artifacts" / "validation_summary.json"
    if summary_path.exists():
        with summary_path.open("r", encoding="utf-8") as handle:
            summary = json.load(handle)
        checkpoint = summary.get("checkpoint")
        if checkpoint:
            return model_dir / "training_artifacts" / checkpoint
    return model_dir / "training_artifacts" / "best_r2_model.pt"


def build_model(config: dict[str, Any], checkpoint_path: Path, loader: TTCDataLoader, device: str):
    # Evaluation-only path: the checkpoint carries the training normalization,
    # so these statistics are a fallback for constructing the model.
    loader.fit_normalization(np.arange(len(loader.df)))
    norm_params = loader.get_normalization_params()
    pinn_core = build_pinn_core_from_config(config)
    base_model = NormalizedTirePINN(pinn_core)
    base_model.set_normalization_params(**norm_params)
    load_checkpoint_into_model(base_model, checkpoint_path, device)
    model = build_runtime_model(base_model, config)
    model.to(device).eval()
    return model


def predict_batches(model, x: np.ndarray, device: str, batch_size: int = 8192) -> np.ndarray:
    outputs = []
    with torch.no_grad():
        for start in range(0, len(x), batch_size):
            batch = torch.tensor(x[start : start + batch_size], dtype=torch.float32, device=device)
            outputs.append(model(batch).detach().cpu().numpy())
    return np.concatenate(outputs, axis=0)


def synchronize_if_needed(device: str) -> None:
    if device == "cuda":
        torch.cuda.synchronize()
    elif device == "mps" and hasattr(torch, "mps") and hasattr(torch.mps, "synchronize"):
        torch.mps.synchronize()


def measure_inference_time(model, x: np.ndarray, device: str) -> dict[str, float]:
    sample = x[: min(len(x), 60000)]
    _ = predict_batches(model, sample[: min(len(sample), 8192)], device)
    synchronize_if_needed(device)
    timings = []
    for _ in range(5):
        start = time.perf_counter()
        _ = predict_batches(model, sample, device)
        synchronize_if_needed(device)
        timings.append(time.perf_counter() - start)
    median_s = float(np.median(timings))
    return {
        "samples": int(len(sample)),
        "median_ms_total": median_s * 1000.0,
        "median_us_per_sample": median_s / max(len(sample), 1) * 1.0e6,
        "samples_per_second": len(sample) / median_s if median_s > 0 else math.inf,
    }


def linear_slope(df, x_col: str, y_col: str) -> float | None:
    if len(df) < 20:
        return None
    x = df[x_col].to_numpy(dtype=np.float64)
    y = df[y_col].to_numpy(dtype=np.float64)
    if np.ptp(x) < 1.0e-9:
        return None
    return float(np.polyfit(x, y, 1)[0])


def model_slope(model, component: str, fz: float, ia: float, temp: float, device: str) -> float:
    if component == "Fx":
        h = 0.005
        x_plus = np.array([[h, 0.0, fz, ia, temp]], dtype=np.float32)
        x_minus = np.array([[-h, 0.0, fz, ia, temp]], dtype=np.float32)
        denom = 2.0 * h
    elif component == "Fy":
        h = 0.1
        x_plus = np.array([[0.0, h, fz, ia, temp]], dtype=np.float32)
        x_minus = np.array([[0.0, -h, fz, ia, temp]], dtype=np.float32)
        denom = 2.0 * h
    else:
        raise ValueError(component)
    pred = predict_batches(model, np.vstack([x_plus, x_minus]), device, batch_size=2)
    index = FORCE_COMPONENTS[component]["index"]
    return float((pred[0, index] - pred[1, index]) / denom)


def compute_slope_diagnostics(df, model, device: str) -> dict[str, Any]:
    ia = float(np.median(df["IA"]))
    temp = float(np.median(df["TSTC"]))
    diagnostics: dict[str, Any] = {"Fx": [], "Fy": []}
    fz_levels = [200.0, 650.0, 950.0, 1150.0]

    for fz in fz_levels:
        fz_mask = (df["Fz"] - fz).abs() <= 90.0
        fx_df = df[
            fz_mask
            & (df["source_test_type"] == "Db")
            & (df["SA"].abs() <= 0.75)
            & (df["SR"].abs() <= 0.03)
        ]
        fy_df = df[
            fz_mask
            & (df["source_test_type"] == "Cr")
            & (df["SR"].abs() <= 0.03)
            & (df["SA"].abs() <= 1.0)
        ]
        diagnostics["Fx"].append(
            {
                "fz": fz,
                "measured_slope_n_per_sr": linear_slope(fx_df, "SR", "Fx"),
                "model_slope_n_per_sr": model_slope(model, "Fx", fz, ia, temp, device),
                "n": int(len(fx_df)),
            }
        )
        diagnostics["Fy"].append(
            {
                "fz": fz,
                "measured_slope_n_per_deg": linear_slope(fy_df, "SA", "Fy"),
                "model_slope_n_per_deg": model_slope(model, "Fy", fz, ia, temp, device),
                "n": int(len(fy_df)),
            }
        )
    return diagnostics


def section_inputs(sweep: str, values: np.ndarray, fz: float, ia: float, temp: float) -> np.ndarray:
    if sweep == "sr":
        return np.column_stack(
            [values, np.zeros_like(values), np.full_like(values, fz), np.full_like(values, ia), np.full_like(values, temp)]
        ).astype(np.float32)
    return np.column_stack(
        [np.zeros_like(values), values, np.full_like(values, fz), np.full_like(values, ia), np.full_like(values, temp)]
    ).astype(np.float32)


def make_force_sections_plot(
    tire_name: str,
    df,
    model,
    device: str,
    metrics: dict[str, Any],
    output_dir: Path,
) -> Path:
    ia = float(np.median(df["IA"]))
    temp = float(np.median(df["TSTC"]))
    fz_levels = [200.0, 650.0, 950.0, 1150.0]
    fig, axes = plt.subplots(3, 3, figsize=(18, 14))

    for row, (component, spec) in enumerate(FORCE_COMPONENTS.items()):
        index = spec["index"]
        source = spec["source"]
        x_col = spec["x"]
        sweep = spec["sweep"]
        source_df = df[df["source_test_type"] == source]
        plot_df = source_df.sample(n=min(len(source_df), 12000), random_state=7)

        ax = axes[row, 0]
        sc = ax.scatter(
            plot_df[x_col],
            plot_df[component],
            c=plot_df["Fz"],
            cmap="viridis",
            s=4,
            alpha=0.30,
            linewidths=0,
        )
        ax.set_title(f"Measured {component} data ({source})")
        ax.set_xlabel("Slip ratio SR [-]" if x_col == "SR" else "Slip angle SA [deg]")
        ax.set_ylabel(f"{component} [{spec['unit']}]")
        ax.grid(True, alpha=0.22)
        fig.colorbar(sc, ax=ax, label="Vertical load Fz [N]")

        ax = axes[row, 1]
        values = np.linspace(-0.30, 0.12, 240) if sweep == "sr" else np.linspace(-12.0, 12.0, 240)
        for fz in fz_levels:
            x_in = section_inputs(sweep, values, fz, ia, temp)
            pred = predict_batches(model, x_in, device, batch_size=4096)
            ax.plot(values, pred[:, index], label=f"Fz={fz:.0f} N", lw=2)
        near_mask = source_df["SA"].abs() <= 1.0 if sweep == "sr" else source_df["SR"].abs() <= 0.03
        near_df = source_df[near_mask].sample(n=min(int(near_mask.sum()), 6000), random_state=11)
        ax.scatter(near_df[x_col], near_df[component], s=3, alpha=0.12, c="0.25", linewidths=0)
        ax.axhline(0, color="0.3", lw=0.8)
        ax.axvline(0, color="0.3", lw=0.8)
        ax.set_title(f"PINN {component} section, IA={ia:.1f} deg, T={temp:.0f} C")
        ax.set_xlabel("Slip ratio SR [-]" if sweep == "sr" else "Slip angle SA [deg]")
        ax.set_ylabel(f"Predicted {component} [{spec['unit']}]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.22)

        ax = axes[row, 2]
        x_grid = np.linspace(-0.30, 0.12, 120) if sweep == "sr" else np.linspace(-12.0, 12.0, 120)
        fz_grid = np.linspace(180.0, 1250.0, 100)
        xx, ff = np.meshgrid(x_grid, fz_grid)
        if sweep == "sr":
            x_in = np.column_stack(
                [xx.ravel(), np.zeros(xx.size), ff.ravel(), np.full(xx.size, ia), np.full(xx.size, temp)]
            ).astype(np.float32)
        else:
            x_in = np.column_stack(
                [np.zeros(xx.size), xx.ravel(), ff.ravel(), np.full(xx.size, ia), np.full(xx.size, temp)]
            ).astype(np.float32)
        zz = predict_batches(model, x_in, device, batch_size=8192)[:, index].reshape(xx.shape)
        vmax = np.nanpercentile(np.abs(zz), 98)
        im = ax.pcolormesh(xx, ff, zz, cmap="coolwarm", shading="auto", vmin=-vmax, vmax=vmax)
        ax.set_title(f"PINN {component} prediction surface")
        ax.set_xlabel("Slip ratio SR [-]" if sweep == "sr" else "Slip angle SA [deg]")
        ax.set_ylabel("Vertical load Fz [N]")
        fig.colorbar(im, ax=ax, label=f"Predicted {component} [{spec['unit']}]")

        metric = metrics[component]
        axes[row, 1].text(
            0.02,
            0.03,
            f"R²={metric['r2']:.4f}, RMSE={metric['rmse']:.1f} {spec['unit']}",
            transform=axes[row, 1].transAxes,
            fontsize=9,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.75),
        )

    fig.suptitle(f"{tire_name}: TTC scatter and PINN prediction quicklook", fontsize=16)
    fig.tight_layout()
    path = output_dir / f"{tire_name}_forces_quicklook.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def make_predicted_vs_measured_plot(
    tire_name: str, df, pred: np.ndarray, metrics: dict[str, Any], output_dir: Path
) -> Path:
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    for ax, (component, spec) in zip(axes, FORCE_COMPONENTS.items()):
        source_df = df[df["source_test_type"] == spec["source"]]
        idx = source_df.index.to_numpy()
        if len(idx) > 18000:
            rng = np.random.default_rng(17)
            idx = rng.choice(idx, size=18000, replace=False)
        actual = df.loc[idx, component].to_numpy()
        predicted = pred[idx, spec["index"]]
        ax.scatter(actual, predicted, s=3, alpha=0.22, linewidths=0)
        lo = float(min(actual.min(), predicted.min()))
        hi = float(max(actual.max(), predicted.max()))
        ax.plot([lo, hi], [lo, hi], "r--", lw=1)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel(f"Measured {component} [{spec['unit']}]")
        ax.set_ylabel(f"Predicted {component} [{spec['unit']}]")
        ax.set_title(
            f"{component}: R²={metrics[component]['r2']:.4f}, RMSE={metrics[component]['rmse']:.1f}"
        )
        ax.grid(True, alpha=0.22)
    fig.suptitle(f"{tire_name}: source-aware predicted vs measured")
    fig.tight_layout()
    path = output_dir / f"{tire_name}_predicted_vs_measured.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def process_tire(
    tire_name: str,
    base_config: dict[str, Any],
    data_dir: Path,
    output_root: Path,
    device: str,
) -> dict[str, Any]:
    config, _ = apply_tire_profile_overrides(base_config, tire_name)
    data_config = config.get("data", {})
    output_model_root = Path(config["output"].get("runtime_root", "output/tire_models"))
    model_dir = output_model_root / tire_name
    checkpoint_path = load_selected_checkpoint(model_dir)
    if not checkpoint_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    loader = TTCDataLoader(
        normalize=True,
        tire_name=tire_name,
        data_dir=data_dir,
        max_samples_total=data_config.get("max_samples_total"),
        sampling_seed=data_config.get("sampling_seed", data_config.get("random_seed", 42)),
    )
    model = build_model(config, checkpoint_path, loader, device)

    x = loader.df[["SR", "SA", "Fz", "IA", "TSTC"]].to_numpy(np.float32)
    pred = predict_batches(model, x, device)
    metrics = {}
    for component, spec in FORCE_COMPONENTS.items():
        mask = loader.df["source_test_type"].to_numpy() == spec["source"]
        metrics[component] = r2_rmse(loader.df.loc[mask, component].to_numpy(), pred[mask, spec["index"]])

    output_dir = output_root / tire_name
    output_dir.mkdir(parents=True, exist_ok=True)
    sections_path = make_force_sections_plot(tire_name, loader.df, model, device, metrics, output_dir)
    pred_path = make_predicted_vs_measured_plot(tire_name, loader.df, pred, metrics, output_dir)
    diagnostics = compute_slope_diagnostics(loader.df, model, device)
    inference = measure_inference_time(model, x, device)

    summary = {
        "tire_name": tire_name,
        "checkpoint": str(checkpoint_path),
        "device": device,
        "metrics": metrics,
        "slope_diagnostics": diagnostics,
        "inference": inference,
        "plots": {
            "force_sections": str(sections_path),
            "predicted_vs_measured": str(pred_path),
        },
    }
    with (output_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="tire_pinn/config/pinn_config.yaml")
    parser.add_argument("--data-dir", default="data/tiredata")
    parser.add_argument("--output-dir", default="output/tire_models/visualizations/quicklook_local_stiffness_mz_20260517")
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--tire-model",
        action="append",
        dest="tire_models",
        default=[],
        help="Tire dataset identifier; repeat for multiple models.",
    )
    args = parser.parse_args()

    tire_models = args.tire_models or [
        "Hoosier_18p0X6p0-10_LCO_C2000_Rim6",
        "Hoosier_18p0X6p0-10_LCO_C2000_Rim7",
    ]
    config = load_config(args.config)
    device = resolve_device(args.device)
    output_root = Path(args.output_dir)
    output_root.mkdir(parents=True, exist_ok=True)

    summaries = []
    for tire_name in tire_models:
        print(f"Visualizing {tire_name}...")
        summaries.append(
            process_tire(
                tire_name=tire_name,
                base_config=config,
                data_dir=Path(args.data_dir),
                output_root=output_root,
                device=device,
            )
        )

    with (output_root / "summary_all.json").open("w", encoding="utf-8") as handle:
        json.dump(summaries, handle, indent=2)
    print(f"Saved summary: {output_root / 'summary_all.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
