#!/usr/bin/env python3
"""
PINN tire model validation: predicted vs actual comparison plots.

Usage:
    python3 tire_pinn/validate_pinn.py
    python3 tire_pinn/validate_pinn.py --checkpoint output/pinn/best_model.pt
"""

import argparse
import yaml
import torch
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).parent.parent))

from tire_pinn.data.ttc_loader import TTCDataLoader
from tire_pinn.main_pinn_train import (
    apply_tire_profile_overrides,
    build_pinn_core_from_config,
    build_runtime_model,
)
from tire_pinn.models.pinn_model import NormalizedTirePINN


def load_model(config, checkpoint_path, device):
    """Load trained PINN model from checkpoint."""
    script_dir = Path(__file__).parent
    data_dir = script_dir.parent / "data" / "tiredata"
    tire_name = config.get("tire_model", {}).get("name", "Hoosier_18p0X6p0-10_R20_Rim6")

    config, _ = apply_tire_profile_overrides(config, tire_name)
    loader = TTCDataLoader(
        normalize=True,
        tire_name=tire_name,
        data_dir=data_dir,
        source_component_masks=config.get("data", {}).get("source_component_masks"),
    )
    # Evaluation-only path: the checkpoint carries the trained normalization buffers.
    loader.fit_normalization(np.arange(len(loader.df)))
    norm_params = loader.get_normalization_params()

    pinn_core = build_pinn_core_from_config(config)
    model = NormalizedTirePINN(pinn_core)
    model.set_normalization_params(**norm_params)

    ckpt = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model.load_state_dict(ckpt["model_state_dict"])

    model = build_runtime_model(model, config)

    model.to(device).eval()

    return model, loader


def get_test_data(loader, config):
    """Get test split raw data."""
    train_ds, val_ds, test_ds = loader.split_data(
        train_ratio=config["data"]["train_ratio"],
        val_ratio=config["data"]["val_ratio"],
        test_ratio=config["data"]["test_ratio"],
        random_seed=config["data"]["random_seed"],
        split_strategy=config["data"].get("split_strategy", "group"),
    )
    X_test = test_ds.tensors[0]
    y_test = test_ds.tensors[1]
    return X_test, y_test


def plot_pred_vs_actual(y_pred, y_test, output_dir):
    """1:1 scatter plot for all 4 outputs."""
    labels = ["Fx [N]", "Fy [N]", "Mx [Nm]", "Mz [Nm]"]
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    for i, (ax, label) in enumerate(zip(axes.flat, labels)):
        actual = y_test[:, i].numpy()
        pred = y_pred[:, i].numpy()

        ax.scatter(actual, pred, s=1, alpha=0.3, c="steelblue")
        lims = [min(actual.min(), pred.min()), max(actual.max(), pred.max())]
        ax.plot(lims, lims, "r--", lw=1, label="y=x")
        ax.set_xlabel(f"Actual {label}")
        ax.set_ylabel(f"Predicted {label}")
        ax.set_title(label)
        ax.set_aspect("equal", adjustable="box")
        ax.legend()

        # R²
        ss_res = np.sum((actual - pred) ** 2)
        ss_tot = np.sum((actual - actual.mean()) ** 2)
        r2 = 1 - ss_res / (ss_tot + 1e-8)
        rmse = np.sqrt(np.mean((actual - pred) ** 2))
        ax.text(
            0.05, 0.92, f"R²={r2:.4f}\nRMSE={rmse:.1f}",
            transform=ax.transAxes, fontsize=10,
            verticalalignment="top", bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        )

    fig.suptitle("PINN Predictions vs Actual (Test Set)", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_dir / "pred_vs_actual.png", dpi=150)
    print(f"  Saved: {output_dir / 'pred_vs_actual.png'}")
    return fig


def plot_fy_vs_sa(model, loader, device, output_dir):
    """Fy vs SA at multiple Fz levels — key physics check."""
    df = loader.df

    # Get typical IA and TSTC values (TTC data: SA/IA in degrees, TSTC in °C)
    ia_mean = df["IA"].mean()
    tstc_mean = df["TSTC"].mean()

    fz_levels = [500, 800, 1000, 1500, 1900]
    sa_sweep = np.linspace(-12.0, 12.0, 200)  # degrees (TTC raw unit)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # --- Left: PINN prediction sweep ---
    ax = axes[0]
    for fz in fz_levels:
        x_input = np.column_stack([
            np.zeros_like(sa_sweep),  # SR=0
            sa_sweep,
            np.full_like(sa_sweep, fz),
            np.full_like(sa_sweep, ia_mean),
            np.full_like(sa_sweep, tstc_mean),
        ]).astype(np.float32)

        with torch.no_grad():
            y_pred = model(torch.tensor(x_input, device=device)).cpu().numpy()

        fy_pred = y_pred[:, 1]
        ax.plot(sa_sweep, fy_pred, label=f"Fz={fz}N")

    ax.axhline(0, color="gray", lw=0.5)
    ax.axvline(0, color="gray", lw=0.5)
    ax.set_xlabel("Slip Angle [deg]")
    ax.set_ylabel("Fy [N]")
    ax.set_title("PINN: Fy vs SA (SR=0)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- Right: Actual data scatter + PINN overlay ---
    ax = axes[1]
    # Filter actual data: |SR| < 0.02 (near-zero longitudinal slip)
    mask = df["SR"].abs() < 0.02
    df_filt = df[mask]

    # SA is already in degrees in TTC data
    sc = ax.scatter(
        df_filt["SA"].values,
        df_filt["Fy"].values,
        c=df_filt["Fz"].values,
        s=2, alpha=0.3, cmap="viridis",
    )
    plt.colorbar(sc, ax=ax, label="Fz [N]")

    # Overlay PINN predictions
    for fz in [800, 1500]:
        x_input = np.column_stack([
            np.zeros_like(sa_sweep),
            sa_sweep,
            np.full_like(sa_sweep, fz),
            np.full_like(sa_sweep, ia_mean),
            np.full_like(sa_sweep, tstc_mean),
        ]).astype(np.float32)

        with torch.no_grad():
            y_pred = model(torch.tensor(x_input, device=device)).cpu().numpy()
        ax.plot(sa_sweep, y_pred[:, 1], "r-", lw=2, label=f"PINN Fz={fz}N")

    ax.axhline(0, color="gray", lw=0.5)
    ax.axvline(0, color="gray", lw=0.5)
    ax.set_xlabel("Slip Angle [deg]")
    ax.set_ylabel("Fy [N]")
    ax.set_title("Actual Data (|SR|<0.02) + PINN Overlay")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle("Lateral Force Characteristic", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_dir / "fy_vs_sa.png", dpi=150)
    print(f"  Saved: {output_dir / 'fy_vs_sa.png'}")
    return fig


def plot_fx_vs_sr(model, loader, device, output_dir):
    """Fx vs SR at multiple Fz levels."""
    df = loader.df
    ia_mean = df["IA"].mean()
    tstc_mean = df["TSTC"].mean()

    fz_levels = [500, 800, 1000, 1500, 1900]
    sr_sweep = np.linspace(-0.25, 0.25, 200)  # SR is dimensionless ratio

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # --- Left: PINN prediction sweep ---
    ax = axes[0]
    for fz in fz_levels:
        x_input = np.column_stack([
            sr_sweep,
            np.zeros_like(sr_sweep),  # SA=0 deg
            np.full_like(sr_sweep, fz),
            np.full_like(sr_sweep, ia_mean),
            np.full_like(sr_sweep, tstc_mean),
        ]).astype(np.float32)

        with torch.no_grad():
            y_pred = model(torch.tensor(x_input, device=device)).cpu().numpy()

        ax.plot(sr_sweep * 100, y_pred[:, 0], label=f"Fz={fz}N")

    ax.axhline(0, color="gray", lw=0.5)
    ax.axvline(0, color="gray", lw=0.5)
    ax.set_xlabel("Slip Ratio [%]")
    ax.set_ylabel("Fx [N]")
    ax.set_title("PINN: Fx vs SR (SA=0)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- Right: Actual data + PINN overlay ---
    ax = axes[1]
    # SA is in degrees in TTC data
    mask = df["SA"].abs() < 1.0  # less than 1 degree
    df_filt = df[mask]

    sc = ax.scatter(
        df_filt["SR"].values * 100,
        df_filt["Fx"].values,
        c=df_filt["Fz"].values,
        s=2, alpha=0.3, cmap="viridis",
    )
    plt.colorbar(sc, ax=ax, label="Fz [N]")

    for fz in [800, 1500]:
        x_input = np.column_stack([
            sr_sweep,
            np.zeros_like(sr_sweep),
            np.full_like(sr_sweep, fz),
            np.full_like(sr_sweep, ia_mean),
            np.full_like(sr_sweep, tstc_mean),
        ]).astype(np.float32)

        with torch.no_grad():
            y_pred = model(torch.tensor(x_input, device=device)).cpu().numpy()
        ax.plot(sr_sweep * 100, y_pred[:, 0], "r-", lw=2, label=f"PINN Fz={fz}N")

    ax.axhline(0, color="gray", lw=0.5)
    ax.axvline(0, color="gray", lw=0.5)
    ax.set_xlabel("Slip Ratio [%]")
    ax.set_ylabel("Fx [N]")
    ax.set_title("Actual Data (|SA|<1°) + PINN Overlay")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle("Longitudinal Force Characteristic", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_dir / "fx_vs_sr.png", dpi=150)
    print(f"  Saved: {output_dir / 'fx_vs_sr.png'}")
    return fig


def plot_zero_slip_check(model, loader, device, output_dir):
    """Check Fx, Fy at SR=0, SA=0 across Fz range — the key physics sanity check."""
    tstc_mean = loader.df["TSTC"].mean()
    fz_sweep = np.linspace(100, 1900, 100)

    x_input = np.column_stack([
        np.zeros_like(fz_sweep),   # SR=0
        np.zeros_like(fz_sweep),   # SA=0 deg
        fz_sweep,
        np.zeros_like(fz_sweep),   # IA=0 deg
        np.full_like(fz_sweep, tstc_mean),  # TSTC in °C
    ]).astype(np.float32)

    with torch.no_grad():
        y_pred = model(torch.tensor(x_input, device=device)).cpu().numpy()

    fig, axes = plt.subplots(1, 4, figsize=(16, 4))
    labels = ["Fx", "Fy", "Mx", "Mz"]
    units = ["N", "N", "Nm", "Nm"]

    for i, (ax, lbl, unit) in enumerate(zip(axes, labels, units)):
        ax.plot(fz_sweep, y_pred[:, i], "b-", lw=2)
        ax.axhline(0, color="red", ls="--", lw=1)
        ax.set_xlabel("Fz [N]")
        ax.set_ylabel(f"{lbl} [{unit}]")
        ax.set_title(f"{lbl} at SR=SA=IA=0")
        ax.grid(True, alpha=0.3)

        max_abs = np.max(np.abs(y_pred[:, i]))
        ax.text(
            0.05, 0.92, f"|max|={max_abs:.1f}{unit}",
            transform=ax.transAxes, fontsize=10,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="lightyellow" if max_abs < 5 else "lightsalmon", alpha=0.7),
        )

    fig.suptitle("Zero Slip Check: SR=SA=IA=0 (should be ~0 for Fx, Fy)", fontsize=13)
    fig.tight_layout()
    fig.savefig(output_dir / "zero_slip_check.png", dpi=150)
    print(f"  Saved: {output_dir / 'zero_slip_check.png'}")
    return fig


def plot_friction_circle(model, loader, device, output_dir):
    """Friction circle (Fx vs Fy) at fixed Fz."""
    df = loader.df
    ia_mean = df["IA"].mean()
    tstc_mean = df["TSTC"].mean()

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fz_levels = [500, 1000, 1500]

    for ax, fz in zip(axes, fz_levels):
        # Dense grid of SR × SA (SA in degrees)
        sr = np.linspace(-0.25, 0.25, 50)
        sa = np.linspace(-12.0, 12.0, 50)
        SR, SA = np.meshgrid(sr, sa)
        sr_flat = SR.ravel()
        sa_flat = SA.ravel()

        x_input = np.column_stack([
            sr_flat, sa_flat,
            np.full_like(sr_flat, fz),
            np.full_like(sr_flat, ia_mean),
            np.full_like(sr_flat, tstc_mean),
        ]).astype(np.float32)

        with torch.no_grad():
            y_pred = model(torch.tensor(x_input, device=device)).cpu().numpy()

        fx = y_pred[:, 0]
        fy = y_pred[:, 1]

        ax.scatter(fy, fx, s=1, alpha=0.5, c="steelblue")

        # Friction circle
        mu = 2.4
        theta = np.linspace(0, 2 * np.pi, 100)
        ax.plot(mu * fz * np.cos(theta), mu * fz * np.sin(theta), "r--", lw=1, label=f"mu={mu}")

        ax.set_xlabel("Fy [N]")
        ax.set_ylabel("Fx [N]")
        ax.set_title(f"Fz = {fz} N")
        ax.set_aspect("equal")
        ax.legend()
        ax.grid(True, alpha=0.3)

    fig.suptitle("Friction Circle (PINN)", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_dir / "friction_circle.png", dpi=150)
    print(f"  Saved: {output_dir / 'friction_circle.png'}")
    return fig


def main():
    parser = argparse.ArgumentParser(description="Validate PINN tire model")
    parser.add_argument("--checkpoint", type=str, default="output/pinn/best_model.pt")
    parser.add_argument("--config", type=str, default="tire_pinn/config/pinn_config.yaml")
    parser.add_argument("--device", type=str, default="cpu")
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)
    tire_name = config.get("tire_model", {}).get("name", "Hoosier_18p0X6p0-10_R20_Rim6")
    config, _ = apply_tire_profile_overrides(config, tire_name)

    output_dir = Path(config["output"]["dir"]) / "validation"
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("PINN Tire Model Validation")
    print("=" * 70)

    # Load model
    print("\nLoading model...")
    model, loader = load_model(config, args.checkpoint, args.device)

    # Get test data
    print("Preparing test data...")
    X_test, y_test = get_test_data(loader, config)

    with torch.no_grad():
        y_pred = model(X_test.to(args.device)).cpu()

    # --- Plots ---
    print("\nGenerating plots...")

    plot_pred_vs_actual(y_pred, y_test, output_dir)
    plot_fy_vs_sa(model, loader, args.device, output_dir)
    plot_fx_vs_sr(model, loader, args.device, output_dir)
    plot_zero_slip_check(model, loader, args.device, output_dir)
    plot_friction_circle(model, loader, args.device, output_dir)

    # --- Numerical summary ---
    print("\n" + "=" * 70)
    print("Zero Slip Summary (SR=SA=IA=0)")
    print("=" * 70)
    tstc_mean = loader.df["TSTC"].mean()
    fz_test = [300, 500, 800, 1000, 1500, 1900]
    for fz in fz_test:
        x_zs = torch.tensor([[0.0, 0.0, fz, 0.0, tstc_mean]], dtype=torch.float32)
        with torch.no_grad():
            y_zs = model(x_zs.to(args.device)).cpu().numpy()[0]
        print(f"  Fz={fz:5d}N: Fx={y_zs[0]:+7.1f}N  Fy={y_zs[1]:+7.1f}N  "
              f"Mx={y_zs[2]:+7.2f}Nm  Mz={y_zs[3]:+7.2f}Nm")

    print(f"\nAll plots saved to: {output_dir}/")


if __name__ == "__main__":
    main()
