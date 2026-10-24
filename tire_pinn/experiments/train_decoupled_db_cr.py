#!/usr/bin/env python3
"""Train an experimental decoupled tire model: Db->Fx, Cr->Fy/Mz.

This script intentionally lives under experiments/ because it is an ablation,
not a deployable replacement for the production PINN export path.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
from torch.utils.data import DataLoader, TensorDataset

sys.path.append(str(Path(__file__).resolve().parents[2]))

from tire_pinn.data.ttc_loader import INPUT_COLUMNS, OUTPUT_COLUMNS, TTCDataLoader
from tire_pinn.losses.data_loss import DataLoss, R2Score
from tire_pinn.main_pinn_train import (
    apply_tire_profile_overrides,
    build_pinn_core_from_config,
    load_config,
    resolve_device,
)
from tire_pinn.models.pinn_model import NormalizedTirePINN


COMPONENT_NAMES = ["Fx", "Fy", "Mx", "Mz"]
FORCE_COMPONENTS = {
    "Fx": {"index": 0, "source": "Db", "x": "SR", "sweep": "sr", "unit": "N"},
    "Fy": {"index": 1, "source": "Cr", "x": "SA", "sweep": "sa", "unit": "N"},
    "Mz": {"index": 3, "source": "Cr", "x": "SA", "sweep": "sa", "unit": "Nm"},
}


class DecoupledTireModel(torch.nn.Module):
    """Compose separately trained component models into one prediction tensor."""

    def __init__(self, db_fx_model: NormalizedTirePINN, cr_fymz_model: NormalizedTirePINN):
        super().__init__()
        self.db_fx_model = db_fx_model
        self.cr_fymz_model = cr_fymz_model

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        db_pred = self.db_fx_model(x)
        cr_pred = self.cr_fymz_model(x)
        pred = torch.zeros_like(db_pred)
        pred[:, 0] = db_pred[:, 0]
        pred[:, 1] = cr_pred[:, 1]
        pred[:, 3] = cr_pred[:, 3]
        return pred


def parse_hidden_dims(value: str | None, fallback: list[int]) -> list[int]:
    if value is None or not value.strip():
        return fallback
    return [int(part.strip()) for part in value.split(",") if part.strip()]


def build_model(config: dict[str, Any], norm_params: dict[str, torch.Tensor]) -> NormalizedTirePINN:
    pinn = build_pinn_core_from_config(config)
    model = NormalizedTirePINN(pinn)
    model.set_normalization_params(**norm_params)
    return model


def split_source_dataframe(
    df: pd.DataFrame,
    source: str,
    component_mask: list[float],
    random_seed: int,
    train_ratio: float = 0.8,
    val_ratio: float = 0.1,
) -> tuple[TensorDataset, TensorDataset, TensorDataset]:
    source_df = df[df["source_test_type"] == source].reset_index(drop=True)
    if source_df.empty:
        raise ValueError(f"No samples found for source_test_type={source!r}")

    x = source_df[INPUT_COLUMNS].to_numpy(dtype=np.float32)
    y = source_df[OUTPUT_COLUMNS].to_numpy(dtype=np.float32)
    mask = np.tile(np.array(component_mask, dtype=np.float32), (len(source_df), 1))

    rng = np.random.default_rng(random_seed)
    indices = rng.permutation(len(source_df))
    n_train = int(len(indices) * train_ratio)
    n_val = int(len(indices) * val_ratio)
    split_indices = {
        "train": indices[:n_train],
        "val": indices[n_train : n_train + n_val],
        "test": indices[n_train + n_val :],
    }

    datasets = []
    for split_name in ["train", "val", "test"]:
        idx = split_indices[split_name]
        datasets.append(
            TensorDataset(
                torch.tensor(x[idx]),
                torch.tensor(y[idx]),
                torch.tensor(mask[idx]),
            )
        )
    return datasets[0], datasets[1], datasets[2]


def make_loader(dataset: TensorDataset, batch_size: int, shuffle: bool) -> DataLoader:
    return DataLoader(dataset, batch_size=batch_size, shuffle=shuffle, num_workers=0)


def clone_state_dict(model: torch.nn.Module) -> dict[str, torch.Tensor]:
    return {key: value.detach().cpu().clone() for key, value in model.state_dict().items()}


def mean_active_r2(r2: torch.Tensor, active_indices: list[int]) -> float:
    return float(torch.mean(r2[active_indices]).detach().cpu().item())


def evaluate_loader(
    model: NormalizedTirePINN,
    loader: DataLoader,
    device: str,
    active_indices: list[int],
) -> dict[str, Any]:
    model.eval()
    preds = []
    targets = []
    masks = []
    with torch.no_grad():
        for x_batch, y_batch, mask_batch in loader:
            x_batch = x_batch.to(device)
            preds.append(model(x_batch).detach().cpu())
            targets.append(y_batch)
            masks.append(mask_batch)

    pred = torch.cat(preds, dim=0)
    target = torch.cat(targets, dim=0)
    mask = torch.cat(masks, dim=0)
    r2_tensor = R2Score.compute(pred, target, component_mask=mask)
    rmse = torch.sqrt(torch.mean(((pred - target) ** 2) * mask, dim=0)).numpy()
    mae = torch.mean(torch.abs(pred - target) * mask, dim=0).numpy()
    return {
        "r2": {f"r2_{name.lower()}": float(r2_tensor[i].item()) for i, name in enumerate(COMPONENT_NAMES)},
        "r2_mean_active": mean_active_r2(r2_tensor, active_indices),
        "rmse": {f"rmse_{name.lower()}": float(rmse[i]) for i, name in enumerate(COMPONENT_NAMES)},
        "mae": {f"mae_{name.lower()}": float(mae[i]) for i, name in enumerate(COMPONENT_NAMES)},
        "n": int(len(loader.dataset)),
    }


def train_component_model(
    label: str,
    model: NormalizedTirePINN,
    train_dataset: TensorDataset,
    val_dataset: TensorDataset,
    test_dataset: TensorDataset,
    active_indices: list[int],
    args: argparse.Namespace,
    output_dir: Path,
    device: str,
) -> dict[str, Any]:
    train_loader = make_loader(train_dataset, args.batch_size, shuffle=True)
    val_loader = make_loader(val_dataset, args.batch_size, shuffle=False)
    test_loader = make_loader(test_dataset, args.batch_size, shuffle=False)

    model.to(device)
    loss_fn = DataLoss(component_weights=[1.0, 1.0, 0.0, 1.0]).to(device)
    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer,
        T_max=max(args.epochs, 1),
        eta_min=args.learning_rate * 0.05,
    )

    best_score = -math.inf
    best_epoch = -1
    best_state = clone_state_dict(model)
    best_val_metrics: dict[str, Any] = {}
    patience_counter = 0
    history = []

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss_sum = 0.0
        train_batches = 0
        for x_batch, y_batch, mask_batch in train_loader:
            x_batch = x_batch.to(device)
            y_batch = y_batch.to(device)
            mask_batch = mask_batch.to(device)

            x_norm = model.normalize_input(x_batch)
            y_pred_norm = model.forward_normalized(x_norm)
            y_target_norm = (y_batch - model.output_mean) / model.output_std
            loss, _ = loss_fn(y_pred_norm, y_target_norm, component_mask=mask_batch)

            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), args.gradient_clip_norm)
            optimizer.step()
            train_loss_sum += float(loss.detach().cpu().item())
            train_batches += 1

        scheduler.step()
        val_metrics = evaluate_loader(model, val_loader, device, active_indices)
        score = float(val_metrics["r2_mean_active"])
        history.append(
            {
                "epoch": epoch,
                "train_loss": train_loss_sum / max(train_batches, 1),
                "val_r2_mean_active": score,
                "lr": float(scheduler.get_last_lr()[0]),
            }
        )

        if score > best_score:
            best_score = score
            best_epoch = epoch
            best_state = clone_state_dict(model)
            best_val_metrics = val_metrics
            patience_counter = 0
        else:
            patience_counter += 1

        if epoch == 1 or epoch % args.log_interval == 0:
            print(
                f"[{label}] epoch {epoch:03d}/{args.epochs} "
                f"train_loss={history[-1]['train_loss']:.5f} "
                f"val_r2={score:.4f} best={best_score:.4f}@{best_epoch}"
            )

        if patience_counter >= args.patience:
            print(f"[{label}] early stopping at epoch {epoch} (best epoch {best_epoch})")
            break

    model.load_state_dict(best_state)
    model.to(device)
    test_metrics = evaluate_loader(model, test_loader, device, active_indices)

    checkpoint_path = output_dir / f"{label}_model.pt"
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "label": label,
            "active_indices": active_indices,
            "best_epoch": best_epoch,
            "val_metrics": best_val_metrics,
            "test_metrics": test_metrics,
            "history": history,
        },
        checkpoint_path,
    )

    return {
        "label": label,
        "checkpoint": str(checkpoint_path),
        "best_epoch": best_epoch,
        "best_val_metrics": best_val_metrics,
        "test_metrics": test_metrics,
        "history_tail": history[-5:],
    }


def clean_db_training_dataframe(
    df: pd.DataFrame,
    enabled: bool,
    sr_deadband: float,
    fx_deadband: float,
    sa_abs: float,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    db_mask = df["source_test_type"] == "Db"
    bad_mask = (
        db_mask
        & (df["SR"] < -abs(sr_deadband))
        & (df["Fx"] > fx_deadband)
        & (df["SA"].abs() <= sa_abs)
    )
    summary = {
        "enabled": bool(enabled),
        "sr_deadband": float(sr_deadband),
        "fx_deadband": float(fx_deadband),
        "sa_abs": float(sa_abs),
        "db_samples_before": int(db_mask.sum()),
        "flagged_samples": int(bad_mask.sum()),
    }
    if not enabled:
        summary["db_samples_after"] = int(db_mask.sum())
        return df.copy(), summary

    cleaned = df.loc[~bad_mask].reset_index(drop=True)
    summary["db_samples_after"] = int((cleaned["source_test_type"] == "Db").sum())
    summary["removed_fraction_of_db"] = (
        summary["flagged_samples"] / max(summary["db_samples_before"], 1)
    )
    return cleaned, summary


def predict_batches(
    model: torch.nn.Module,
    x: np.ndarray,
    device: str,
    batch_size: int = 8192,
) -> np.ndarray:
    outputs = []
    model.eval()
    with torch.no_grad():
        for start in range(0, len(x), batch_size):
            batch = torch.tensor(x[start : start + batch_size], dtype=torch.float32, device=device)
            outputs.append(model(batch).detach().cpu().numpy())
    return np.concatenate(outputs, axis=0)


def component_metrics(actual: np.ndarray, pred: np.ndarray) -> dict[str, float]:
    residual = pred - actual
    ss_res = float(np.sum(residual**2))
    ss_tot = float(np.sum((actual - np.mean(actual)) ** 2))
    return {
        "r2": float(1.0 - ss_res / max(ss_tot, 1.0e-12)),
        "rmse": float(np.sqrt(np.mean(residual**2))),
        "mae": float(np.mean(np.abs(residual))),
        "n": int(actual.size),
    }


def evaluate_integrated_model(
    model: torch.nn.Module,
    df: pd.DataFrame,
    device: str,
) -> dict[str, Any]:
    x = df[INPUT_COLUMNS].to_numpy(dtype=np.float32)
    y = df[OUTPUT_COLUMNS].to_numpy(dtype=np.float32)
    pred = predict_batches(model, x, device)

    metrics: dict[str, Any] = {}
    for component, spec in FORCE_COMPONENTS.items():
        source_mask = (df["source_test_type"] == spec["source"]).to_numpy()
        index = spec["index"]
        metrics[component] = component_metrics(y[source_mask, index], pred[source_mask, index])

    db_mask = (df["source_test_type"] == "Db").to_numpy()
    pure_negative_sr = db_mask & (x[:, 0] < -0.02) & (np.abs(x[:, 1]) <= 1.0)
    if np.any(pure_negative_sr):
        metrics["Fx_negative_sr_sign"] = {
            "n": int(np.sum(pure_negative_sr)),
            "measured_positive_rate": float(np.mean(y[pure_negative_sr, 0] > 0.0)),
            "predicted_positive_rate": float(np.mean(pred[pure_negative_sr, 0] > 0.0)),
            "predicted_positive_over_50n_rate": float(np.mean(pred[pure_negative_sr, 0] > 50.0)),
        }
    return metrics


def synchronize_if_needed(device: str) -> None:
    if device == "cuda":
        torch.cuda.synchronize()
    elif device == "mps" and hasattr(torch, "mps") and hasattr(torch.mps, "synchronize"):
        torch.mps.synchronize()


def measure_inference_time(
    model: torch.nn.Module,
    df: pd.DataFrame,
    device: str,
) -> dict[str, float]:
    x = df[INPUT_COLUMNS].to_numpy(dtype=np.float32)
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


def section_inputs(sweep: str, values: np.ndarray, fz: float, ia: float, temp: float) -> np.ndarray:
    if sweep == "sr":
        return np.column_stack(
            [
                values,
                np.zeros_like(values),
                np.full_like(values, fz),
                np.full_like(values, ia),
                np.full_like(values, temp),
            ]
        ).astype(np.float32)
    return np.column_stack(
        [
            np.zeros_like(values),
            values,
            np.full_like(values, fz),
            np.full_like(values, ia),
            np.full_like(values, temp),
        ]
    ).astype(np.float32)


def make_quicklook_plot(
    tire_name: str,
    model: torch.nn.Module,
    df: pd.DataFrame,
    metrics: dict[str, Any],
    output_dir: Path,
    device: str,
) -> Path:
    ia = float(np.median(df["IA"]))
    temp = float(np.median(df["TSTC"]))
    fz_levels = [200.0, 650.0, 950.0, 1150.0]
    fig, axes = plt.subplots(3, 2, figsize=(13, 13))

    for row, (component, spec) in enumerate(FORCE_COMPONENTS.items()):
        index = spec["index"]
        source = spec["source"]
        x_col = spec["x"]
        source_df = df[df["source_test_type"] == source]
        plot_df = source_df.sample(n=min(len(source_df), 12000), random_state=7)

        ax = axes[row, 0]
        scatter = ax.scatter(
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
        ax.grid(True, alpha=0.25)
        cbar = fig.colorbar(scatter, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label("Vertical load Fz [N]")

        ax = axes[row, 1]
        if spec["sweep"] == "sr":
            values = np.linspace(-0.30, 0.12, 220, dtype=np.float32)
            near_df = source_df[(source_df["SA"].abs() <= 1.0)]
        else:
            values = np.linspace(-12.5, 12.5, 220, dtype=np.float32)
            near_df = source_df[(source_df["SR"].abs() <= 0.03)]

        ax.scatter(
            near_df[x_col],
            near_df[component],
            color="0.5",
            s=4,
            alpha=0.12,
            linewidths=0,
        )
        for fz in fz_levels:
            x_section = section_inputs(spec["sweep"], values, fz, ia, temp)
            pred = predict_batches(model, x_section, device, batch_size=8192)
            ax.plot(values, pred[:, index], linewidth=2.0, label=f"Fz={fz:.0f} N")

        component_metrics_text = metrics.get(component, {})
        if component_metrics_text:
            ax.text(
                0.02,
                0.03,
                f"R2={component_metrics_text['r2']:.4f}, "
                f"RMSE={component_metrics_text['rmse']:.1f}",
                transform=ax.transAxes,
                bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "alpha": 0.75},
            )
        ax.axhline(0.0, color="0.35", linewidth=0.8)
        ax.axvline(0.0, color="0.35", linewidth=0.8)
        ax.set_title(f"Decoupled {component} section, IA={ia:.1f} deg, T={temp:.0f} C")
        ax.set_xlabel("Slip ratio SR [-]" if spec["sweep"] == "sr" else "Slip angle SA [deg]")
        ax.set_ylabel(f"Predicted {component} [{spec['unit']}]")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)

    fig.suptitle(f"{tire_name}: decoupled Db/Cr experiment quicklook", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    output_path = output_dir / "decoupled_quicklook.png"
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    return output_path


def resolve_output_dir(args: argparse.Namespace) -> Path:
    tag = args.tag
    if tag is None:
        tag = "clean_sign" if args.clean_db_sign else "raw"
        tag += f"_e{args.epochs}"
    output_dir = Path(args.output_root) / args.tire_model / tag
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Experimental decoupled PINN training for Db Fx and Cr Fy/Mz."
    )
    parser.add_argument(
        "--tire-model",
        default="Hoosier_18p0X6p0-10_LCO_C2000_Rim7",
        help="Tire catalog dataset/profile identifier.",
    )
    parser.add_argument(
        "--config",
        default="tire_pinn/config/pinn_config.yaml",
        help="PINN YAML config path.",
    )
    parser.add_argument(
        "--data-dir",
        default="data/tiredata",
        help="Tire data directory.",
    )
    parser.add_argument(
        "--output-root",
        default="output/tire_models/experiments/decoupled_db_cr",
        help="Directory for experiment artifacts.",
    )
    parser.add_argument("--tag", default=None, help="Experiment output tag.")
    parser.add_argument("--device", default="auto", help="auto, cpu, cuda, or mps.")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=2048)
    parser.add_argument("--learning-rate", type=float, default=1.5e-3)
    parser.add_argument("--weight-decay", type=float, default=1.0e-5)
    parser.add_argument("--gradient-clip-norm", type=float, default=1.0)
    parser.add_argument("--patience", type=int, default=12)
    parser.add_argument("--log-interval", type=int, default=5)
    parser.add_argument("--random-seed", type=int, default=42)
    parser.add_argument("--max-samples-total", type=int, default=None)
    parser.add_argument(
        "--hidden-dims",
        default=None,
        help="Comma-separated hidden dimensions. Used only with --architecture sequential.",
    )
    parser.add_argument(
        "--architecture",
        choices=["from-config", "sequential"],
        default="from-config",
        help="Use config architecture by default; pass sequential for legacy ablations.",
    )
    parser.add_argument(
        "--clean-db-sign",
        action="store_true",
        help="Remove pure-ish negative-SR samples with positive Fx from Db training.",
    )
    parser.add_argument("--clean-sr-deadband", type=float, default=0.02)
    parser.add_argument("--clean-fx-deadband", type=float, default=50.0)
    parser.add_argument("--clean-sa-abs", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    torch.manual_seed(args.random_seed)
    np.random.seed(args.random_seed)

    config = load_config(args.config)
    config, _ = apply_tire_profile_overrides(config, args.tire_model)
    if args.architecture == "sequential":
        fallback_hidden_dims = config["model"]["hidden_dims"] or [128, 128, 64, 64, 32]
        config["model"]["hidden_dims"] = parse_hidden_dims(args.hidden_dims, fallback_hidden_dims)
        config["model"]["architecture"] = "sequential"
        config["model"].pop("head_hidden_dims", None)
        config["model"].pop("head_outputs", None)

    data_cfg = config.get("data", {})
    max_samples_total = (
        args.max_samples_total
        if args.max_samples_total is not None
        else int(data_cfg.get("max_samples_total", 60000))
    )
    device = resolve_device(args.device)
    output_dir = resolve_output_dir(args)

    print(f"Experiment output: {output_dir}")
    print(f"Device: {device}")
    print(f"Experimental architecture: {config['model'].get('architecture', 'sequential')}")
    print(f"Experimental hidden dims: {config['model']['hidden_dims']}")

    loader = TTCDataLoader(
        tire_name=args.tire_model,
        data_dir=Path(args.data_dir),
        max_samples_total=max_samples_total,
        sampling_seed=int(data_cfg.get("sampling_seed", args.random_seed)),
    )
    # Experimental ablation path: this script uses its own source split below,
    # so fit normalization explicitly on all loaded rows and record that choice.
    loader.fit_normalization(np.arange(len(loader.df)))
    norm_params = loader.get_normalization_params()
    raw_df = loader.df.reset_index(drop=True)
    training_df, clean_summary = clean_db_training_dataframe(
        raw_df,
        enabled=args.clean_db_sign,
        sr_deadband=args.clean_sr_deadband,
        fx_deadband=args.clean_fx_deadband,
        sa_abs=args.clean_sa_abs,
    )
    print("Db cleaning summary:")
    print(json.dumps(clean_summary, indent=2))

    db_datasets = split_source_dataframe(
        training_df,
        source="Db",
        component_mask=[1.0, 0.0, 0.0, 0.0],
        random_seed=args.random_seed,
    )
    cr_datasets = split_source_dataframe(
        training_df,
        source="Cr",
        component_mask=[0.0, 1.0, 0.0, 1.0],
        random_seed=args.random_seed + 1,
    )

    db_model = build_model(config, norm_params)
    cr_model = build_model(config, norm_params)

    db_summary = train_component_model(
        "db_fx",
        db_model,
        *db_datasets,
        active_indices=[0],
        args=args,
        output_dir=output_dir,
        device=device,
    )
    cr_summary = train_component_model(
        "cr_fymz",
        cr_model,
        *cr_datasets,
        active_indices=[1, 3],
        args=args,
        output_dir=output_dir,
        device=device,
    )

    integrated_model = DecoupledTireModel(db_model, cr_model).to(device)
    integrated_metrics = evaluate_integrated_model(integrated_model, raw_df, device)
    inference = measure_inference_time(integrated_model, raw_df, device)
    quicklook_path = make_quicklook_plot(
        args.tire_model,
        integrated_model,
        raw_df,
        integrated_metrics,
        output_dir,
        device,
    )

    summary = {
        "tire_model": args.tire_model,
        "device": device,
        "config": {
            "epochs_requested": args.epochs,
            "batch_size": args.batch_size,
            "learning_rate": args.learning_rate,
            "architecture": config["model"].get("architecture", "sequential"),
            "hidden_dims": config["model"]["hidden_dims"],
            "head_hidden_dims": config["model"].get("head_hidden_dims", {}),
            "max_samples_total": max_samples_total,
        },
        "cleaning": clean_summary,
        "db_fx": db_summary,
        "cr_fymz": cr_summary,
        "integrated_metrics_on_original_data": integrated_metrics,
        "inference": inference,
        "quicklook": str(quicklook_path),
    }
    summary_path = output_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    print("\nIntegrated metrics on original data:")
    print(json.dumps(integrated_metrics, indent=2))
    print("\nInference:")
    print(json.dumps(inference, indent=2))
    print(f"\nSaved summary: {summary_path}")
    print(f"Saved quicklook: {quicklook_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
