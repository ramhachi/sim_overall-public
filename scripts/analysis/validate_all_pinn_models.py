#!/usr/bin/env python3
"""
Multi-model PINN tire validation: real data vs predicted + Fy leakage analysis.

This script validates all deployable PINN tire models and specifically checks
whether lateral force (Fy) appears in conditions where it should be zero:
  - DriveBrake conditions (SA ~ 0)
  - SA=0 sweep at various SR values

Usage:
    python3 validate_all_pinn_models.py [--device cpu|mps|cuda]
    python3 validate_all_pinn_models.py --tire Hoosier_18p0X6p0-10_R20
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt
import torch
import yaml

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))

from tire_pinn.models.pinn_model import TirePINN, NormalizedTirePINN, PhysicsEnforcedTireModel
from tire_pinn.data.ttc_loader import TTCDataLoader
from tire_pinn.data.tire_catalog import TireCatalog

OUTPUT_BASE = ROOT / "output" / "validation"
MODEL_BASE = ROOT / "output" / "tire_models"
DATA_DIR = ROOT / "data" / "tiredata"
CONFIG_PATH = ROOT / "tire_pinn" / "config" / "pinn_config.yaml"


# ---------------------------------------------------------------------------
# Model loading
# ---------------------------------------------------------------------------

def load_model_from_manifest(tire_name: str, device: str):
    """Load trained PINN model from training artifact checkpoint using manifest info."""
    manifest_path = MODEL_BASE / tire_name / "manifest.json"
    with open(manifest_path) as f:
        manifest = json.load(f)

    artifact_dir = ROOT / manifest["training"]["artifact_dir"]
    selected_ckpt = manifest["training"]["selected_checkpoint"]
    ckpt_path = artifact_dir / selected_ckpt
    if not ckpt_path.exists():
        ckpt_path = artifact_dir / "best_r2_model.pt"

    with open(CONFIG_PATH) as f:
        config = yaml.safe_load(f)

    loader = TTCDataLoader(normalize=True, tire_name=tire_name, data_dir=DATA_DIR)
    norm_params = loader.get_normalization_params()

    pinn_core = TirePINN(
        hidden_dims=config["model"]["hidden_dims"],
        input_dim=config["model"]["input_dim"],
        output_dim=config["model"]["output_dim"],
        use_layer_norm=config["model"]["use_layer_norm"],
    )
    model = NormalizedTirePINN(pinn_core)
    model.set_normalization_params(**norm_params)

    ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
    model.load_state_dict(ckpt["model_state_dict"])

    # Apply runtime post-processing (zero-slip gate, friction cap, symmetry)
    pp = manifest.get("validation", {}).get("physics", {})
    runtime_cfg = config.get("runtime_postprocess", {})
    if runtime_cfg.get("enabled", True):
        model = PhysicsEnforcedTireModel(
            model,
            friction_cap_mu=float(runtime_cfg.get("friction_cap_mu", 2.6)),
            zero_slip_sr_scale=float(runtime_cfg.get("zero_slip_sr_scale", 0.03)),
            zero_slip_sa_scale_deg=float(runtime_cfg.get("zero_slip_sa_scale_deg", 0.5)),
            epsilon=float(runtime_cfg.get("epsilon", 1.0e-9)),
        )

    model.to(device).eval()
    return model, loader, manifest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _r2(actual: np.ndarray, pred: np.ndarray) -> float:
    ss_res = np.sum((actual - pred) ** 2)
    ss_tot = np.sum((actual - actual.mean()) ** 2) + 1e-8
    return float(1.0 - ss_res / ss_tot)


def _rmse(actual: np.ndarray, pred: np.ndarray) -> float:
    return float(np.sqrt(np.mean((actual - pred) ** 2)))


@torch.no_grad()
def _predict(model, x_np: np.ndarray, device: str) -> np.ndarray:
    x = torch.tensor(x_np.astype(np.float32), device=device)
    return model(x).cpu().numpy()


# ---------------------------------------------------------------------------
# Plot: predicted vs actual with per-source breakdown
# ---------------------------------------------------------------------------

def plot_pred_vs_actual(model, loader, device, out_dir: Path, tire_name: str):
    df = loader.df
    ia_mean = float(df["IA"].mean())
    tstc_mean = float(df["TSTC"].mean())

    feature_cols = ["SR", "SA", "Fz", "IA", "TSTC"]
    target_cols  = ["Fx", "Fy", "Mx", "Mz"]

    x_all = df[feature_cols].values.astype(np.float32)
    y_all = df[target_cols].values.astype(np.float32)
    y_pred = _predict(model, x_all, device)

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    labels = ["Fx [N]", "Fy [N]", "Mx [Nm]", "Mz [Nm]"]

    for i, (ax, label) in enumerate(zip(axes.flat, labels)):
        actual = y_all[:, i]
        pred = y_pred[:, i]
        r2 = _r2(actual, pred)
        rmse = _rmse(actual, pred)

        # Color by source type
        if "source_test_type" in df.columns:
            for src, color in [("Db", "steelblue"), ("Cr", "darkorange"), ("Cmb", "green")]:
                mask = df["source_test_type"] == src
                if mask.sum() > 0:
                    ax.scatter(actual[mask], pred[mask], s=1, alpha=0.3,
                               c=color, label=src)
            ax.legend(markerscale=5, fontsize=8)
        else:
            ax.scatter(actual, pred, s=1, alpha=0.3, c="steelblue")

        lims = [min(actual.min(), pred.min()), max(actual.max(), pred.max())]
        ax.plot(lims, lims, "r--", lw=1)
        ax.set_xlabel(f"Actual {label}")
        ax.set_ylabel(f"Predicted {label}")
        ax.set_title(label)
        ax.set_aspect("equal", adjustable="box")
        ax.text(0.05, 0.93, f"R²={r2:.4f}\nRMSE={rmse:.1f}",
                transform=ax.transAxes, fontsize=9, verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))
        ax.grid(True, alpha=0.3)

    fig.suptitle(f"Predicted vs Actual — {tire_name}", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_dir / "pred_vs_actual.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: pred_vs_actual.png")

    # Summary per source
    print(f"\n  {'Source':<8}  {'N':>7}  {'R²_Fx':>7}  {'R²_Fy':>7}  {'RMSE_Fx':>9}  {'RMSE_Fy':>9}")
    if "source_test_type" in df.columns:
        for src in ("Db", "Cr", "Cmb"):
            mask = (df["source_test_type"] == src).values
            if mask.sum() == 0:
                continue
            r2_fx = _r2(y_all[mask, 0], y_pred[mask, 0])
            r2_fy = _r2(y_all[mask, 1], y_pred[mask, 1])
            rmse_fx = _rmse(y_all[mask, 0], y_pred[mask, 0])
            rmse_fy = _rmse(y_all[mask, 1], y_pred[mask, 1])
            print(f"  {src:<8}  {mask.sum():>7d}  {r2_fx:>7.4f}  {r2_fy:>7.4f}  {rmse_fx:>9.1f}  {rmse_fy:>9.1f}")
    all_r2_fx = _r2(y_all[:, 0], y_pred[:, 0])
    all_r2_fy = _r2(y_all[:, 1], y_pred[:, 1])
    print(f"  {'ALL':<8}  {len(df):>7d}  {all_r2_fx:>7.4f}  {all_r2_fy:>7.4f}  "
          f"  {_rmse(y_all[:,0], y_pred[:,0]):>9.1f}  {_rmse(y_all[:,1], y_pred[:,1]):>9.1f}")


# ---------------------------------------------------------------------------
# Plot: Fy leakage on DriveBrake data (MAIN CONCERN)
# ---------------------------------------------------------------------------

def plot_fy_leakage_drivebrake(model, loader, device, out_dir: Path, tire_name: str):
    """
    Check Fy on DriveBrake data.
    In pure DriveBrake conditions, SA is roughly 0 and Fy should be ~0.
    If the model predicts large |Fy| on these samples, that is 'Fy leakage'.
    """
    df = loader.df
    feature_cols = ["SR", "SA", "Fz", "IA", "TSTC"]

    # DriveBrake subset: either flagged by source or low |SA|
    if "source_test_type" in df.columns:
        db_mask = df["source_test_type"] == "Db"
    else:
        db_mask = df["SA"].abs() < 1.5  # fallback: SA within ±1.5°

    df_db = df[db_mask].copy()
    if len(df_db) == 0:
        print("  [skip] No DriveBrake data found for Fy leakage check")
        return

    x_db = df_db[feature_cols].values.astype(np.float32)
    y_db_pred = _predict(model, x_db, device)
    fy_pred = y_db_pred[:, 1]
    fy_actual = df_db["Fy"].values if "Fy" in df_db.columns else np.zeros(len(df_db))

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    # 1) Fy_pred vs SA on DB samples
    ax = axes[0]
    sc = ax.scatter(df_db["SA"].values, fy_pred, c=df_db["SR"].values,
                    s=4, alpha=0.5, cmap="RdBu_r", vmin=-0.3, vmax=0.3)
    plt.colorbar(sc, ax=ax, label="SR")
    ax.axhline(0, color="red", lw=1.5, ls="--", label="Fy=0")
    ax.axvline(0, color="gray", lw=0.8)
    ax.set_xlabel("Slip Angle SA [deg]")
    ax.set_ylabel("Predicted Fy [N]")
    ax.set_title("PINN Fy (DriveBrake data)\n→ should be near 0 when SA≈0")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Text summary
    sa_zero_mask = df_db["SA"].abs() < 0.5
    if sa_zero_mask.sum() > 0:
        fy_at_sa0 = fy_pred[sa_zero_mask.values]
        ax.text(0.05, 0.92, f"|SA|<0.5°: mean|Fy|={np.mean(np.abs(fy_at_sa0)):.1f}N\n"
                f"max|Fy|={np.max(np.abs(fy_at_sa0)):.1f}N",
                transform=ax.transAxes, fontsize=9, verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8))

    # 2) Histogram of |Fy_pred| for |SA| < 1 deg on DB data
    ax = axes[1]
    sa_near0 = df_db["SA"].abs() < 1.0
    fy_near0 = np.abs(fy_pred[sa_near0.values])
    ax.hist(fy_near0, bins=50, color="steelblue", edgecolor="white", alpha=0.8)
    ax.axvline(20.0, color="red", lw=2, ls="--", label="20 N threshold")
    ax.axvline(50.0, color="orange", lw=1.5, ls="--", label="50 N")
    ax.set_xlabel("|Predicted Fy| [N]")
    ax.set_ylabel("Count")
    ax.set_title(f"|Fy| distribution (DriveBrake, |SA|<1°)\nN={sa_near0.sum()}")
    ax.legend()
    ax.grid(True, alpha=0.3)
    pct_over20 = 100 * (fy_near0 > 20).mean()
    pct_over50 = 100 * (fy_near0 > 50).mean()
    ax.text(0.95, 0.92, f">20N: {pct_over20:.1f}%\n>50N: {pct_over50:.1f}%",
            transform=ax.transAxes, fontsize=9, ha="right", verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8))

    # 3) Model sweep: Fy vs SR at SA=0 (should stay ~0 throughout)
    ax = axes[2]
    tstc_mean = float(df_db["TSTC"].mean())
    ia_mean = float(df_db["IA"].mean())
    sr_sweep = np.linspace(-0.30, 0.30, 300)
    for fz in [500, 1000, 1500]:
        x_sweep = np.column_stack([
            sr_sweep,
            np.zeros_like(sr_sweep),       # SA=0
            np.full_like(sr_sweep, fz),
            np.full_like(sr_sweep, ia_mean),
            np.full_like(sr_sweep, tstc_mean),
        ]).astype(np.float32)
        y_sw = _predict(model, x_sweep, device)
        ax.plot(sr_sweep * 100, y_sw[:, 1], label=f"Fz={fz}N")

    ax.axhline(0, color="red", lw=1.5, ls="--")
    ax.axhline(20, color="orange", lw=1, ls=":", alpha=0.7, label="±20N")
    ax.axhline(-20, color="orange", lw=1, ls=":", alpha=0.7)
    ax.set_xlabel("Slip Ratio SR [%]")
    ax.set_ylabel("Fy [N]")
    ax.set_title("Fy vs SR (SA=IA=0)\n→ should be ~0 for all SR")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle(f"Fy Leakage Analysis — {tire_name}", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_dir / "fy_leakage_drivebrake.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: fy_leakage_drivebrake.png")

    # Numerical report
    print(f"\n  === Fy Leakage on DriveBrake data ({len(df_db)} samples) ===")
    if sa_near0.sum() > 0:
        print(f"  |SA|<1°  (N={sa_near0.sum()}):  "
              f"mean|Fy|={np.mean(fy_near0):.1f}N, "
              f"p95|Fy|={np.percentile(fy_near0, 95):.1f}N, "
              f"max|Fy|={np.max(fy_near0):.1f}N, "
              f">20N: {pct_over20:.1f}%")
    if sa_zero_mask.sum() > 0:
        fy_at_sa0 = fy_pred[sa_zero_mask.values]
        print(f"  |SA|<0.5° (N={sa_zero_mask.sum()}):  "
              f"mean|Fy|={np.mean(np.abs(fy_at_sa0)):.1f}N, "
              f"max|Fy|={np.max(np.abs(fy_at_sa0)):.1f}N")


# ---------------------------------------------------------------------------
# Plot: Fy vs SA (cornering curve with zero-crossing check)
# ---------------------------------------------------------------------------

def plot_fy_vs_sa(model, loader, device, out_dir: Path, tire_name: str):
    df = loader.df
    ia_mean = float(df["IA"].mean())
    tstc_mean = float(df["TSTC"].mean())

    fz_levels = [500, 800, 1000, 1500, 1900]
    sa_sweep = np.linspace(-14.0, 14.0, 300)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    ax = axes[0]
    for fz in fz_levels:
        x_inp = np.column_stack([
            np.zeros_like(sa_sweep), sa_sweep,
            np.full_like(sa_sweep, fz),
            np.full_like(sa_sweep, ia_mean),
            np.full_like(sa_sweep, tstc_mean),
        ]).astype(np.float32)
        y_pred = _predict(model, x_inp, device)
        ax.plot(sa_sweep, y_pred[:, 1], label=f"Fz={fz}N")

    ax.axhline(0, color="gray", lw=0.8)
    ax.axvline(0, color="red", lw=1.5, ls="--", label="SA=0")
    # Annotate Fy at SA=0
    tstc_m = tstc_mean
    for fz in [800, 1500]:
        x0 = np.array([[0.0, 0.0, fz, ia_mean, tstc_m]], dtype=np.float32)
        y0 = _predict(model, x0, device)[0, 1]
        ax.annotate(f"Fy(0)={y0:.1f}N", xy=(0, y0), fontsize=8,
                    xytext=(1, y0 + 30), arrowprops=dict(arrowstyle="->", lw=0.8))
    ax.set_xlabel("SA [deg]")
    ax.set_ylabel("Fy [N]")
    ax.set_title("PINN: Fy vs SA (SR=0)\n→ Fy(SA=0) should be ~0")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    if "source_test_type" in df.columns:
        cr_mask = df["source_test_type"] == "Cr"
        df_cr = df[cr_mask & (df["SR"].abs() < 0.02)]
    else:
        df_cr = df[df["SR"].abs() < 0.02]
    sc = ax.scatter(df_cr["SA"].values, df_cr["Fy"].values,
                    c=df_cr["Fz"].values, s=2, alpha=0.3, cmap="viridis")
    plt.colorbar(sc, ax=ax, label="Fz [N]")
    for fz in [800, 1500]:
        x_inp = np.column_stack([
            np.zeros_like(sa_sweep), sa_sweep,
            np.full_like(sa_sweep, fz),
            np.full_like(sa_sweep, ia_mean),
            np.full_like(sa_sweep, tstc_mean),
        ]).astype(np.float32)
        y_pred = _predict(model, x_inp, device)
        ax.plot(sa_sweep, y_pred[:, 1], lw=2, label=f"PINN Fz={fz}N")
    ax.axhline(0, color="gray", lw=0.8)
    ax.axvline(0, color="red", lw=1, ls="--")
    ax.set_xlabel("SA [deg]")
    ax.set_ylabel("Fy [N]")
    ax.set_title("Actual Cornering Data + PINN Overlay (|SR|<0.02)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle(f"Lateral Force Characteristic — {tire_name}", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_dir / "fy_vs_sa.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: fy_vs_sa.png")


# ---------------------------------------------------------------------------
# Plot: Fx vs SR
# ---------------------------------------------------------------------------

def plot_fx_vs_sr(model, loader, device, out_dir: Path, tire_name: str):
    df = loader.df
    ia_mean = float(df["IA"].mean())
    tstc_mean = float(df["TSTC"].mean())

    fz_levels = [500, 800, 1000, 1500, 1900]
    sr_sweep = np.linspace(-0.30, 0.30, 300)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    ax = axes[0]
    for fz in fz_levels:
        x_inp = np.column_stack([
            sr_sweep, np.zeros_like(sr_sweep),
            np.full_like(sr_sweep, fz),
            np.full_like(sr_sweep, ia_mean),
            np.full_like(sr_sweep, tstc_mean),
        ]).astype(np.float32)
        y_pred = _predict(model, x_inp, device)
        ax.plot(sr_sweep * 100, y_pred[:, 0], label=f"Fz={fz}N")

    ax.axhline(0, color="gray", lw=0.8)
    ax.axvline(0, color="red", lw=1.5, ls="--")
    ax.set_xlabel("SR [%]")
    ax.set_ylabel("Fx [N]")
    ax.set_title("PINN: Fx vs SR (SA=0)\n→ Fx(SR=0) should be ~0")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    if "source_test_type" in df.columns:
        db_mask = df["source_test_type"] == "Db"
        df_db = df[db_mask & (df["SA"].abs() < 1.0)]
    else:
        df_db = df[df["SA"].abs() < 1.0]
    sc = ax.scatter(df_db["SR"].values * 100, df_db["Fx"].values,
                    c=df_db["Fz"].values, s=2, alpha=0.3, cmap="viridis")
    plt.colorbar(sc, ax=ax, label="Fz [N]")
    for fz in [800, 1500]:
        x_inp = np.column_stack([
            sr_sweep, np.zeros_like(sr_sweep),
            np.full_like(sr_sweep, fz),
            np.full_like(sr_sweep, ia_mean),
            np.full_like(sr_sweep, tstc_mean),
        ]).astype(np.float32)
        y_pred = _predict(model, x_inp, device)
        ax.plot(sr_sweep * 100, y_pred[:, 0], lw=2, label=f"PINN Fz={fz}N")
    ax.axhline(0, color="gray", lw=0.8)
    ax.axvline(0, color="red", lw=1, ls="--")
    ax.set_xlabel("SR [%]")
    ax.set_ylabel("Fx [N]")
    ax.set_title("Actual DriveBrake Data + PINN Overlay (|SA|<1°)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle(f"Longitudinal Force Characteristic — {tire_name}", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_dir / "fx_vs_sr.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: fx_vs_sr.png")


# ---------------------------------------------------------------------------
# Plot: Zero-slip sweep (SA=0 across all SR, SR=0 across all SA)
# ---------------------------------------------------------------------------

def plot_zero_slip_check(model, loader, device, out_dir: Path, tire_name: str):
    """
    Most important Fy-leakage check.
    Row 1: Fx, Fy, Mz at SR=SA=IA=0 vs Fz   (should be 0 for all)
    Row 2: Fy vs SR at SA=0 for multiple Fz   (should stay ~0 across SR)
    Row 3: Fx vs SA at SR=0 for multiple Fz   (should stay ~0 across SA)
    """
    df = loader.df
    ia_mean = float(df["IA"].mean())
    tstc_mean = float(df["TSTC"].mean())
    fz_sweep = np.linspace(100, 2000, 200)
    sr_sweep = np.linspace(-0.30, 0.30, 300)
    sa_sweep = np.linspace(-14.0, 14.0, 300)
    fz_levels = [500, 1000, 1500]

    fig, axes = plt.subplots(3, 3, figsize=(16, 12))

    # ---- Row 0: Fx, Fy, Mz at SR=SA=IA=0 vs Fz ----
    x0 = np.column_stack([
        np.zeros_like(fz_sweep), np.zeros_like(fz_sweep),
        fz_sweep, np.zeros_like(fz_sweep),
        np.full_like(fz_sweep, tstc_mean),
    ]).astype(np.float32)
    y0 = _predict(model, x0, device)

    for col, (idx, label, unit) in enumerate([(0, "Fx", "N"), (1, "Fy", "N"), (3, "Mz", "Nm")]):
        ax = axes[0, col]
        ax.plot(fz_sweep, y0[:, idx], "b-", lw=2)
        ax.axhline(0, color="red", ls="--", lw=1.5)
        ax.fill_between(fz_sweep, -20, 20, alpha=0.1, color="green", label="±20N tolerance")
        ax.set_xlabel("Fz [N]")
        ax.set_ylabel(f"{label} [{unit}]")
        ax.set_title(f"{label} at SR=SA=IA=0\n(should be 0)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        max_abs = np.max(np.abs(y0[:, idx]))
        color = "lightgreen" if max_abs < 5 else ("lightyellow" if max_abs < 20 else "lightsalmon")
        ax.text(0.04, 0.95, f"max|{label}|={max_abs:.2f}{unit}",
                transform=ax.transAxes, fontsize=9, va="top",
                bbox=dict(boxstyle="round", facecolor=color, alpha=0.8))

    # ---- Row 1: Fy vs SR at SA=0 (Fy leakage into DriveBrake axis) ----
    for col, fz in enumerate(fz_levels):
        ax = axes[1, col]
        x_sr = np.column_stack([
            sr_sweep, np.zeros_like(sr_sweep),
            np.full_like(sr_sweep, fz),
            np.full_like(sr_sweep, ia_mean),
            np.full_like(sr_sweep, tstc_mean),
        ]).astype(np.float32)
        y_sr = _predict(model, x_sr, device)
        fy_sr = y_sr[:, 1]

        ax.plot(sr_sweep * 100, fy_sr, "b-", lw=2)
        ax.fill_between(sr_sweep * 100, -20, 20, alpha=0.15, color="green")
        ax.axhline(0, color="red", ls="--", lw=1.5)
        ax.set_xlabel("SR [%]")
        ax.set_ylabel("Fy [N]")
        ax.set_title(f"Fy vs SR (SA=0, Fz={fz}N)\n→ should be ~0")
        ax.grid(True, alpha=0.3)
        max_fy = np.max(np.abs(fy_sr))
        pct_ok = 100 * (np.abs(fy_sr) < 20).mean()
        color = "lightgreen" if max_fy < 20 else ("lightyellow" if max_fy < 50 else "lightsalmon")
        ax.text(0.04, 0.95, f"max|Fy|={max_fy:.1f}N\n<20N: {pct_ok:.0f}%",
                transform=ax.transAxes, fontsize=9, va="top",
                bbox=dict(boxstyle="round", facecolor=color, alpha=0.8))

    # ---- Row 2: Fx vs SA at SR=0 (Fx leakage into Cornering axis) ----
    for col, fz in enumerate(fz_levels):
        ax = axes[2, col]
        x_sa = np.column_stack([
            np.zeros_like(sa_sweep), sa_sweep,
            np.full_like(sa_sweep, fz),
            np.full_like(sa_sweep, ia_mean),
            np.full_like(sa_sweep, tstc_mean),
        ]).astype(np.float32)
        y_sa = _predict(model, x_sa, device)
        fx_sa = y_sa[:, 0]

        ax.plot(sa_sweep, fx_sa, "darkorange", lw=2)
        ax.fill_between(sa_sweep, -50, 50, alpha=0.1, color="green")
        ax.axhline(0, color="red", ls="--", lw=1.5)
        ax.set_xlabel("SA [deg]")
        ax.set_ylabel("Fx [N]")
        ax.set_title(f"Fx vs SA (SR=0, Fz={fz}N)\n→ rolling resistance only")
        ax.grid(True, alpha=0.3)
        max_fx = np.max(np.abs(fx_sa))
        ax.text(0.04, 0.95, f"max|Fx|={max_fx:.1f}N",
                transform=ax.transAxes, fontsize=9, va="top",
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))

    fig.suptitle(f"Zero-Slip Physics Check — {tire_name}", fontsize=14)
    fig.tight_layout()
    fig.savefig(out_dir / "zero_slip_check.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: zero_slip_check.png")

    # Print numerical table
    print(f"\n  === Zero-slip point check (SR=SA=IA=0) ===")
    print(f"  {'Fz[N]':>8}  {'Fx[N]':>9}  {'Fy[N]':>9}  {'Mx[Nm]':>9}  {'Mz[Nm]':>9}")
    for fz in [200, 400, 600, 800, 1000, 1200, 1500, 1800]:
        x_pt = np.array([[0.0, 0.0, float(fz), 0.0, tstc_mean]], dtype=np.float32)
        y_pt = _predict(model, x_pt, device)[0]
        flag = " ★ LEAKAGE" if abs(y_pt[1]) > 20 else ""
        print(f"  {fz:>8}  {y_pt[0]:>+9.2f}  {y_pt[1]:>+9.2f}  {y_pt[2]:>+9.2f}  {y_pt[3]:>+9.2f}{flag}")

    print(f"\n  === Fy(SR) at SA=0 max values ===")
    for fz in fz_levels:
        x_sr = np.column_stack([
            sr_sweep, np.zeros_like(sr_sweep),
            np.full_like(sr_sweep, fz),
            np.full_like(sr_sweep, ia_mean),
            np.full_like(sr_sweep, tstc_mean),
        ]).astype(np.float32)
        fy_sr = _predict(model, x_sr, device)[:, 1]
        print(f"  Fz={fz}N: max|Fy|={np.max(np.abs(fy_sr)):.2f}N  "
              f"at SR={sr_sweep[np.argmax(np.abs(fy_sr))]*100:.1f}%  "
              f"p95|Fy|={np.percentile(np.abs(fy_sr), 95):.2f}N")


# ---------------------------------------------------------------------------
# Plot: Residuals — signed error vs input variables
# ---------------------------------------------------------------------------

def plot_residuals(model, loader, device, out_dir: Path, tire_name: str):
    df = loader.df
    feature_cols = ["SR", "SA", "Fz", "IA", "TSTC"]
    target_cols  = ["Fx", "Fy", "Mx", "Mz"]

    x_all = df[feature_cols].values.astype(np.float32)
    y_all = df[target_cols].values.astype(np.float32)
    y_pred = _predict(model, x_all, device)

    res_fx = y_pred[:, 0] - y_all[:, 0]
    res_fy = y_pred[:, 1] - y_all[:, 1]

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))

    # Fy residual vs SA
    ax = axes[0, 0]
    sc = ax.scatter(df["SA"].values, res_fy, c=df["Fz"].values,
                    s=2, alpha=0.2, cmap="viridis")
    plt.colorbar(sc, ax=ax, label="Fz [N]")
    ax.axhline(0, color="red", lw=1)
    ax.set_xlabel("SA [deg]")
    ax.set_ylabel("Fy residual [N]  (pred - actual)")
    ax.set_title("Fy residual vs SA")
    ax.grid(True, alpha=0.3)

    # Fy residual vs SR
    ax = axes[0, 1]
    sc = ax.scatter(df["SR"].values * 100, res_fy, c=df["Fz"].values,
                    s=2, alpha=0.2, cmap="viridis")
    plt.colorbar(sc, ax=ax, label="Fz [N]")
    ax.axhline(0, color="red", lw=1)
    ax.set_xlabel("SR [%]")
    ax.set_ylabel("Fy residual [N]")
    ax.set_title("Fy residual vs SR")
    ax.grid(True, alpha=0.3)

    # Fy residual vs Fz
    ax = axes[0, 2]
    ax.scatter(df["Fz"].values, res_fy, s=2, alpha=0.2, c="darkorange")
    ax.axhline(0, color="red", lw=1)
    ax.set_xlabel("Fz [N]")
    ax.set_ylabel("Fy residual [N]")
    ax.set_title("Fy residual vs Fz")
    ax.grid(True, alpha=0.3)

    # Fx residual vs SR
    ax = axes[1, 0]
    sc = ax.scatter(df["SR"].values * 100, res_fx, c=df["Fz"].values,
                    s=2, alpha=0.2, cmap="viridis")
    plt.colorbar(sc, ax=ax, label="Fz [N]")
    ax.axhline(0, color="red", lw=1)
    ax.set_xlabel("SR [%]")
    ax.set_ylabel("Fx residual [N]")
    ax.set_title("Fx residual vs SR")
    ax.grid(True, alpha=0.3)

    # Fx residual vs SA
    ax = axes[1, 1]
    sc = ax.scatter(df["SA"].values, res_fx, c=df["Fz"].values,
                    s=2, alpha=0.2, cmap="viridis")
    plt.colorbar(sc, ax=ax, label="Fz [N]")
    ax.axhline(0, color="red", lw=1)
    ax.set_xlabel("SA [deg]")
    ax.set_ylabel("Fx residual [N]")
    ax.set_title("Fx residual vs SA")
    ax.grid(True, alpha=0.3)

    # Per-source residual box plot
    ax = axes[1, 2]
    if "source_test_type" in df.columns:
        sources = ["Db", "Cr", "Cmb"]
        fy_by_src = [res_fy[df["source_test_type"].values == s] for s in sources]
        labels = [f"{s}\n(N={len(v)})" for s, v in zip(sources, fy_by_src) if len(v) > 0]
        data = [v for v in fy_by_src if len(v) > 0]
        bp = ax.boxplot(data, labels=labels, notch=False, sym="", patch_artist=True,
                        medianprops=dict(color="red", lw=2))
        for patch, color in zip(bp["boxes"], ["steelblue", "darkorange", "green"]):
            patch.set_facecolor(color)
            patch.set_alpha(0.6)
        ax.axhline(0, color="gray", lw=0.8)
        ax.set_ylabel("Fy residual [N]")
        ax.set_title("Fy residual by source type")
        ax.grid(True, alpha=0.3, axis="y")
    else:
        ax.text(0.5, 0.5, "No source info", transform=ax.transAxes, ha="center")

    fig.suptitle(f"Prediction Residuals — {tire_name}", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_dir / "residuals.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: residuals.png")


# ---------------------------------------------------------------------------
# Single model validation
# ---------------------------------------------------------------------------

def validate_one(tire_name: str, device: str) -> dict:
    print(f"\n{'='*70}")
    print(f"  Validating: {tire_name}")
    print(f"{'='*70}")

    out_dir = OUTPUT_BASE / tire_name
    out_dir.mkdir(parents=True, exist_ok=True)

    model, loader, manifest = load_model_from_manifest(tire_name, device)

    print(f"  Dataset: {len(loader.df):,} samples")
    print(f"  R²_Fx={manifest['validation']['r2']['r2_fx']:.4f}  "
          f"R²_Fy={manifest['validation']['r2']['r2_fy']:.4f}")

    plot_pred_vs_actual(model, loader, device, out_dir, tire_name)
    plot_fy_leakage_drivebrake(model, loader, device, out_dir, tire_name)
    plot_fy_vs_sa(model, loader, device, out_dir, tire_name)
    plot_fx_vs_sr(model, loader, device, out_dir, tire_name)
    plot_zero_slip_check(model, loader, device, out_dir, tire_name)
    plot_residuals(model, loader, device, out_dir, tire_name)

    print(f"\n  All plots → {out_dir}/")
    return {"tire_name": tire_name, "out_dir": str(out_dir)}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=str, default="cpu",
                        help="Inference device: cpu | mps | cuda")
    parser.add_argument("--tire", type=str, default=None,
                        help="Validate one specific tire (default: all deployable)")
    args = parser.parse_args()

    device = args.device
    if device == "mps" and not torch.backends.mps.is_available():
        print("MPS not available, falling back to cpu")
        device = "cpu"
    if device == "cuda" and not torch.cuda.is_available():
        print("CUDA not available, falling back to cpu")
        device = "cpu"

    # Determine which tires to validate
    with open(MODEL_BASE / "registry.json") as f:
        registry = json.load(f)

    if args.tire:
        tire_names = [args.tire]
    else:
        tire_names = [t for t, info in registry["models"].items() if info.get("deployable")]

    print(f"Validating {len(tire_names)} model(s) on device={device}")
    OUTPUT_BASE.mkdir(parents=True, exist_ok=True)

    results = []
    for name in tire_names:
        try:
            results.append(validate_one(name, device))
        except Exception as e:
            print(f"  [ERROR] {name}: {e}")
            import traceback
            traceback.print_exc()

    print(f"\n{'='*70}")
    print(f"Done. Plots saved to: {OUTPUT_BASE}/")
    for r in results:
        print(f"  {r['tire_name']} → {r['out_dir']}")


if __name__ == "__main__":
    main()
