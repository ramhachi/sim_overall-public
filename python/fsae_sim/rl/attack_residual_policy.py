"""Residual-policy parameterizations for fixed-course attack training."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class ResidualBasisConfig:
    track_length_m: float = 1000.0
    basis_count: int = 24
    output_samples: int = 128
    basis_width_m: float | None = None
    max_delta_v_ref_mps: float = 2.0
    max_delta_steer_ff_rad: float = 0.035
    max_delta_ax_bias_mps2: float = 1.0
    max_delta_heading_relax_rad: float = 0.08
    basis_delta_v_ref_mps: float = 0.75
    basis_delta_steer_ff_rad: float = 0.010
    basis_delta_ax_bias_mps2: float = 0.25
    basis_delta_heading_relax_rad: float = 0.015
    closed_loop: bool = True


def _clip_action(action: Sequence[float]) -> np.ndarray:
    values = np.asarray(action, dtype=np.float64).reshape(-1)
    if not np.all(np.isfinite(values)):
        raise ValueError("residual basis action contains non-finite values")
    return np.clip(values, -1.0, 1.0)


def residual_basis_dim(config: ResidualBasisConfig) -> int:
    return 4 * max(1, int(config.basis_count))


def residual_basis_centers(config: ResidualBasisConfig) -> np.ndarray:
    if config.track_length_m <= 0.0:
        raise ValueError("track_length_m must be positive")
    count = max(1, int(config.basis_count))
    return np.linspace(0.0, config.track_length_m, count, endpoint=False)


def residual_basis_stations(config: ResidualBasisConfig) -> np.ndarray:
    if config.track_length_m <= 0.0:
        raise ValueError("track_length_m must be positive")
    count = max(2, int(config.output_samples))
    return np.linspace(0.0, config.track_length_m, count, endpoint=False)


def _wrapped_distance(stations: np.ndarray, centers: np.ndarray, length: float) -> np.ndarray:
    raw = np.abs(stations[:, None] - centers[None, :])
    if length <= 0.0:
        return raw
    return np.minimum(raw, length - raw)


def residual_basis_matrix(
    stations: np.ndarray,
    config: ResidualBasisConfig,
) -> np.ndarray:
    centers = residual_basis_centers(config)
    if config.basis_width_m is None:
        width = config.track_length_m / max(float(config.basis_count), 1.0)
    else:
        width = max(float(config.basis_width_m), 1e-6)
    if config.closed_loop:
        distance = _wrapped_distance(stations, centers, config.track_length_m)
    else:
        distance = np.abs(stations[:, None] - centers[None, :])
    weights = np.exp(-0.5 * np.square(distance / width))
    row_sums = np.sum(weights, axis=1, keepdims=True)
    return weights / np.maximum(row_sums, 1e-12)


def basis_action_to_profile(
    action: Sequence[float],
    config: ResidualBasisConfig,
    *,
    description: str = "Residual profile generated from a basis-policy action.",
) -> dict[str, Any]:
    dim = residual_basis_dim(config)
    values = _clip_action(action)
    if values.size != dim:
        raise ValueError(f"expected basis action dimension {dim}, got {values.size}")

    coefficients = values.reshape(config.basis_count, 4)
    stations = residual_basis_stations(config)
    weights = residual_basis_matrix(stations, config)
    delta_v_coeff = np.ascontiguousarray(coefficients[:, 0])
    delta_steer_coeff = np.ascontiguousarray(coefficients[:, 1])
    delta_ax_coeff = np.ascontiguousarray(coefficients[:, 2])
    delta_heading_coeff = np.ascontiguousarray(coefficients[:, 3])
    delta_v = np.sum(weights * delta_v_coeff[None, :], axis=1)
    delta_steer = np.sum(weights * delta_steer_coeff[None, :], axis=1)
    delta_ax = np.sum(weights * delta_ax_coeff[None, :], axis=1)
    delta_heading = np.sum(weights * delta_heading_coeff[None, :], axis=1)
    samples = []
    for station, dv_coeff, steer_coeff, ax_coeff, heading_coeff in zip(
        stations,
        delta_v,
        delta_steer,
        delta_ax,
        delta_heading,
    ):
        samples.append(
            {
                "s": float(station),
                "delta_v_ref_mps": float(
                    np.clip(dv_coeff, -1.0, 1.0) * config.max_delta_v_ref_mps
                ),
                "delta_steer_ff_rad": float(
                    np.clip(steer_coeff, -1.0, 1.0) * config.max_delta_steer_ff_rad
                ),
                "delta_ax_bias_mps2": float(
                    np.clip(ax_coeff, -1.0, 1.0) * config.max_delta_ax_bias_mps2
                ),
                "delta_heading_relax_rad": float(
                    np.clip(heading_coeff, 0.0, 1.0) * config.max_delta_heading_relax_rad
                ),
            }
        )
    return {
        "schema": "attack_residual_profile_v1",
        "description": description,
        "limits": {
            "max_delta_v_ref_mps": float(abs(config.max_delta_v_ref_mps)),
            "max_delta_steer_ff_rad": float(abs(config.max_delta_steer_ff_rad)),
            "max_delta_ax_bias_mps2": float(abs(config.max_delta_ax_bias_mps2)),
            "max_delta_heading_relax_rad": float(abs(config.max_delta_heading_relax_rad)),
        },
        "samples": samples,
        "metadata": {
            "generator": "python/fsae_sim/rl/attack_residual_policy.py",
            "policy_mode": "basis",
            "basis_count": int(config.basis_count),
            "basis_width_m": (
                float(config.basis_width_m)
                if config.basis_width_m is not None
                else None
            ),
            "output_samples": int(config.output_samples),
            "track_length_m": float(config.track_length_m),
        },
    }


def delta_basis_action_to_profile(
    action: Sequence[float],
    base_profile: Mapping[str, Any],
    config: ResidualBasisConfig,
    *,
    description: str = "Residual profile generated from a delta-basis policy action.",
) -> dict[str, Any]:
    """Apply low-dimensional basis corrections on top of an existing profile."""

    dim = residual_basis_dim(config)
    values = _clip_action(action)
    if values.size != dim:
        raise ValueError(f"expected basis action dimension {dim}, got {values.size}")

    coefficients = values.reshape(config.basis_count, 4)
    stations = residual_basis_stations(config)
    weights = residual_basis_matrix(stations, config)
    delta_v_coeff = np.ascontiguousarray(coefficients[:, 0])
    delta_steer_coeff = np.ascontiguousarray(coefficients[:, 1])
    delta_ax_coeff = np.ascontiguousarray(coefficients[:, 2])
    delta_heading_coeff = np.ascontiguousarray(coefficients[:, 3])
    residual_v = np.sum(weights * delta_v_coeff[None, :], axis=1)
    residual_steer = np.sum(weights * delta_steer_coeff[None, :], axis=1)
    residual_ax = np.sum(weights * delta_ax_coeff[None, :], axis=1)
    residual_heading = np.sum(weights * delta_heading_coeff[None, :], axis=1)
    base_v, base_steer, base_ax, base_heading = _sample_profile_on_stations(
        base_profile,
        stations,
    )

    samples = []
    for (
        station,
        base_dv,
        base_dsteer,
        base_dax,
        base_dheading,
        dv_coeff,
        steer_coeff,
        ax_coeff,
        heading_coeff,
    ) in zip(
        stations,
        base_v,
        base_steer,
        base_ax,
        base_heading,
        residual_v,
        residual_steer,
        residual_ax,
        residual_heading,
    ):
        samples.append(
            {
                "s": float(station),
                "delta_v_ref_mps": float(
                    np.clip(
                        base_dv + dv_coeff * config.basis_delta_v_ref_mps,
                        -config.max_delta_v_ref_mps,
                        config.max_delta_v_ref_mps,
                    )
                ),
                "delta_steer_ff_rad": float(
                    np.clip(
                        base_dsteer + steer_coeff * config.basis_delta_steer_ff_rad,
                        -config.max_delta_steer_ff_rad,
                        config.max_delta_steer_ff_rad,
                    )
                ),
                "delta_ax_bias_mps2": float(
                    np.clip(
                        base_dax + ax_coeff * config.basis_delta_ax_bias_mps2,
                        -config.max_delta_ax_bias_mps2,
                        config.max_delta_ax_bias_mps2,
                    )
                ),
                "delta_heading_relax_rad": float(
                    np.clip(
                        base_dheading + heading_coeff * config.basis_delta_heading_relax_rad,
                        0.0,
                        config.max_delta_heading_relax_rad,
                    )
                ),
            }
        )

    return {
        "schema": "attack_residual_profile_v1",
        "description": description,
        "limits": {
            "max_delta_v_ref_mps": float(abs(config.max_delta_v_ref_mps)),
            "max_delta_steer_ff_rad": float(abs(config.max_delta_steer_ff_rad)),
            "max_delta_ax_bias_mps2": float(abs(config.max_delta_ax_bias_mps2)),
            "max_delta_heading_relax_rad": float(abs(config.max_delta_heading_relax_rad)),
        },
        "samples": samples,
        "metadata": {
            "generator": "python/fsae_sim/rl/attack_residual_policy.py",
            "policy_mode": "delta-basis",
            "basis_count": int(config.basis_count),
            "basis_width_m": (
                float(config.basis_width_m)
                if config.basis_width_m is not None
                else None
            ),
            "basis_delta_v_ref_mps": float(config.basis_delta_v_ref_mps),
            "basis_delta_steer_ff_rad": float(config.basis_delta_steer_ff_rad),
            "basis_delta_ax_bias_mps2": float(config.basis_delta_ax_bias_mps2),
            "basis_delta_heading_relax_rad": float(config.basis_delta_heading_relax_rad),
            "output_samples": int(config.output_samples),
            "track_length_m": float(config.track_length_m),
        },
    }


def _sample_profile_on_stations(
    profile: Mapping[str, Any],
    stations: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    samples = sorted(profile.get("samples", []), key=lambda item: float(item.get("s", 0.0)))
    if not samples:
        zeros = np.zeros_like(stations)
        return zeros, zeros, zeros, zeros
    sample_s = np.asarray([float(item.get("s", item.get("s_m", 0.0))) for item in samples])
    sample_v = np.asarray(
        [
            float(item.get("delta_v_ref_mps", item.get("delta_v_mps", 0.0)))
            for item in samples
        ]
    )
    sample_steer = np.asarray(
        [
            float(item.get("delta_steer_ff_rad", item.get("delta_steer_rad", 0.0)))
            for item in samples
        ]
    )
    sample_ax = np.asarray(
        [
            float(item.get("delta_ax_bias_mps2", item.get("delta_ax_mps2", 0.0)))
            for item in samples
        ]
    )
    sample_heading = np.asarray(
        [
            float(item.get("delta_heading_relax_rad", item.get("heading_relax_rad", 0.0)))
            for item in samples
        ]
    )
    delta_v = np.interp(stations, sample_s, sample_v, left=sample_v[0], right=sample_v[-1])
    delta_steer = np.interp(
        stations,
        sample_s,
        sample_steer,
        left=sample_steer[0],
        right=sample_steer[-1],
    )
    delta_ax = np.interp(stations, sample_s, sample_ax, left=sample_ax[0], right=sample_ax[-1])
    delta_heading = np.interp(
        stations,
        sample_s,
        sample_heading,
        left=sample_heading[0],
        right=sample_heading[-1],
    )
    return delta_v, delta_steer, delta_ax, delta_heading


def profile_to_basis_action(
    profile: Mapping[str, Any],
    config: ResidualBasisConfig,
) -> np.ndarray:
    stations = residual_basis_stations(config)
    weights = residual_basis_matrix(stations, config)
    delta_v, delta_steer, delta_ax, delta_heading = _sample_profile_on_stations(
        profile,
        stations,
    )
    v_target = delta_v / max(abs(config.max_delta_v_ref_mps), 1e-12)
    steer_target = delta_steer / max(abs(config.max_delta_steer_ff_rad), 1e-12)
    ax_target = delta_ax / max(abs(config.max_delta_ax_bias_mps2), 1e-12)
    heading_target = delta_heading / max(abs(config.max_delta_heading_relax_rad), 1e-12)
    v_coeff, *_ = np.linalg.lstsq(weights, v_target, rcond=None)
    steer_coeff, *_ = np.linalg.lstsq(weights, steer_target, rcond=None)
    ax_coeff, *_ = np.linalg.lstsq(weights, ax_target, rcond=None)
    heading_coeff, *_ = np.linalg.lstsq(weights, heading_target, rcond=None)
    action = np.empty(residual_basis_dim(config), dtype=np.float64)
    action[0::4] = v_coeff
    action[1::4] = steer_coeff
    action[2::4] = ax_coeff
    action[3::4] = heading_coeff
    return np.clip(action, -1.0, 1.0)


def load_profile_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def basis_reconstruction_rmse(
    profile: Mapping[str, Any],
    config: ResidualBasisConfig,
) -> dict[str, float]:
    action = profile_to_basis_action(profile, config)
    reconstructed = basis_action_to_profile(action, config)
    stations = residual_basis_stations(config)
    source_v, source_steer, source_ax, source_heading = _sample_profile_on_stations(
        profile,
        stations,
    )
    recon_v, recon_steer, recon_ax, recon_heading = _sample_profile_on_stations(
        reconstructed,
        stations,
    )
    return {
        "delta_v_ref_rmse_mps": float(math.sqrt(float(np.mean(np.square(source_v - recon_v))))),
        "delta_steer_ff_rmse_rad": float(
            math.sqrt(float(np.mean(np.square(source_steer - recon_steer))))
        ),
        "delta_ax_bias_rmse_mps2": float(
            math.sqrt(float(np.mean(np.square(source_ax - recon_ax))))
        ),
        "delta_heading_relax_rmse_rad": float(
            math.sqrt(float(np.mean(np.square(source_heading - recon_heading))))
        ),
    }
