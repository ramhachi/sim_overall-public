#!/usr/bin/env python3
"""
Main entry point for PINN tire model training and deployment gating.
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys
from typing import Any, Dict

import yaml

sys.path.append(str(Path(__file__).parent.parent))

torch = None
TireCatalog = None
TTCDataLoader = None
CPPExporter = None
R2Score = None
NormalizedTirePINN = None
PhysicsEnforcedTireModel = None
TirePINN = None
CollocationSampler = None
PINNTrainer = None
PhysicsValidator = None


def ensure_training_dependencies() -> None:
    global torch
    global TireCatalog, TTCDataLoader, CPPExporter, R2Score
    global NormalizedTirePINN, PhysicsEnforcedTireModel, TirePINN
    global CollocationSampler, PINNTrainer, PhysicsValidator

    if torch is not None:
        return

    import torch as torch_module

    from tire_pinn.data.tire_catalog import TireCatalog as TireCatalogClass
    from tire_pinn.data.ttc_loader import TTCDataLoader as TTCDataLoaderClass
    from tire_pinn.export.cpp_exporter import CPPExporter as CPPExporterClass
    from tire_pinn.losses.data_loss import R2Score as R2ScoreClass
    from tire_pinn.models.pinn_model import (
        NormalizedTirePINN as NormalizedTirePINNClass,
        PhysicsEnforcedTireModel as PhysicsEnforcedTireModelClass,
        TirePINN as TirePINNClass,
    )
    from tire_pinn.training.collocation_sampler import CollocationSampler as CollocationSamplerClass
    from tire_pinn.training.trainer import PINNTrainer as PINNTrainerClass
    from tire_pinn.validation.physics_tests import PhysicsValidator as PhysicsValidatorClass

    torch = torch_module
    TireCatalog = TireCatalogClass
    TTCDataLoader = TTCDataLoaderClass
    CPPExporter = CPPExporterClass
    R2Score = R2ScoreClass
    NormalizedTirePINN = NormalizedTirePINNClass
    PhysicsEnforcedTireModel = PhysicsEnforcedTireModelClass
    TirePINN = TirePINNClass
    CollocationSampler = CollocationSamplerClass
    PINNTrainer = PINNTrainerClass
    PhysicsValidator = PhysicsValidatorClass


def load_config(config_path: str) -> dict:
    with open(config_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


_PHYSICS_SECTION_KEYS: Dict[str, set[str]] = {
    "zero_slip": {
        "force_threshold", "moment_threshold",
        "force_threshold_n", "moment_threshold_nm", "n_colloc_points",
    },
    "symmetry": {"n_colloc_points"},
    "friction_circle": {"mu_cap", "n_colloc_points"},
    "monotonicity": {"enable", "n_colloc_points", "epsilon"},
    "combined_slip": {"mu_cap", "margin", "n_colloc_points"},
    "stiffness_load": {
        "sr_delta", "sa_delta_deg", "load_step_ratio", "min_load_step",
        "normalized_margin", "n_colloc_points",
    },
    "mz_trail": {
        "small_sa_deg", "large_sa_deg", "min_abs_sa_deg", "max_trail_m",
        "min_trail_m", "trail_margin_m", "min_lateral_force_n",
        "sign_loss_weight", "odd_loss_weight", "n_colloc_points",
    },
    "camber_thrust": {
        "camber_mu_cap", "n_colloc_points", "ia_deg", "fx_weight", "fx_threshold_n",
    },
    "section_smoothness": {
        "sr_delta", "sa_delta_deg", "mu_cap", "mz_scale_m", "force_deadband",
        "mz_deadband", "mz_weight", "slope_sign_weight", "n_colloc_points",
    },
    "fy_sa_zero": {"force_threshold", "n_colloc_points"},
    "high_sa_lateral_load": {
        "min_abs_sa_deg", "drop_margin", "mu_cap", "n_colloc_points",
    },
}


def validate_physics_config(config: Dict[str, Any]) -> None:
    """Reject physics keys that the trainer does not actually consume."""
    physics = config.get("physics", {})
    if not isinstance(physics, dict):
        raise ValueError("config.physics must be a mapping")
    for section, values in physics.items():
        if section not in _PHYSICS_SECTION_KEYS:
            raise ValueError(f"Unsupported physics section: {section}")
        if not isinstance(values, dict):
            raise ValueError(f"config.physics.{section} must be a mapping")
        unknown = set(values) - _PHYSICS_SECTION_KEYS[section]
        if unknown:
            raise ValueError(
                f"Unsupported keys in config.physics.{section}: {sorted(unknown)}"
            )


def deep_merge_dict(base: Dict[str, Any], overrides: Dict[str, Any]) -> Dict[str, Any]:
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge_dict(base[key], value)
        else:
            base[key] = value
    return base


def apply_tire_profile_overrides(
    config: Dict[str, Any], tire_name: str
) -> tuple[Dict[str, Any], Dict[str, Any] | None]:
    profiles = config.get("tire_profiles", {})
    overrides = profiles.get(tire_name)
    if not overrides:
        return config, None

    merged = copy.deepcopy(config)
    deep_merge_dict(merged, overrides)
    return merged, overrides


def resolve_device(requested: str | None) -> str:
    ensure_training_dependencies()
    mps_available = hasattr(torch.backends, "mps") and torch.backends.mps.is_available()

    if requested and requested != "auto":
        if requested == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but not available.")
        if requested == "mps" and not mps_available:
            raise RuntimeError("MPS requested but not available.")
        return requested

    if torch.cuda.is_available():
        return "cuda"
    if mps_available:
        return "mps"
    return "cpu"


def load_checkpoint_into_model(
    model: NormalizedTirePINN, checkpoint_path: Path, device: str
) -> None:
    ensure_training_dependencies()
    checkpoint = torch.load(checkpoint_path, map_location=device)
    state_dict = checkpoint.get("model_state_dict", checkpoint)
    model.load_state_dict(state_dict)


def summarize_validation_results(
    tire_name: str,
    dataset_info: Dict[str, Any],
    r2_dict: Dict[str, float],
    physics_results: Dict[str, dict],
    r2_target: float,
    mz_r2_target: float = 0.0,
) -> Dict[str, Any]:
    r2_mean = (r2_dict["r2_fx"] + r2_dict["r2_fy"]) / 2.0
    r2_min = min(r2_dict["r2_fx"], r2_dict["r2_fy"])
    mz_r2 = r2_dict["r2_mz"]
    dataset_complete = bool(dataset_info["has_db"] and dataset_info["has_cr"])
    # Skipped checks are "not evaluated", not failed; the measured gates below
    # still block deployment when they were skipped.
    physics_passed = all(
        result["pass"] for result in physics_results.values() if not result.get("skipped", False)
    )
    measured_gate_names = ("measured_friction_circle", "measured_combined_slip")
    missing_measured_gates = [
        name
        for name in measured_gate_names
        if name not in physics_results or physics_results[name].get("skipped", False)
    ]
    measured_gates_present = not missing_measured_gates

    deployment_reasons = []
    if not dataset_complete:
        deployment_reasons.append(
            "Dataset is incomplete (requires both DriveBrake and Cornering data)."
        )
    if r2_min < r2_target:
        deployment_reasons.append(f"Fx/Fy R² minimum {r2_min:.4f} is below target {r2_target:.4f}.")
    if mz_r2_target > 0.0 and mz_r2 < mz_r2_target:
        deployment_reasons.append(
            f"Mz R² {mz_r2:.4f} is below target {mz_r2_target:.4f} (vehicle SAT uses Mz)."
        )
    if not physics_passed:
        deployment_reasons.append("One or more physics validation checks failed.")
    if not measured_gates_present:
        deployment_reasons.append(
            f"Measured holdout gates were not evaluated: {', '.join(missing_measured_gates)}."
        )

    deployable = (
        dataset_complete
        and physics_passed
        and measured_gates_present
        and r2_min >= r2_target
        and (mz_r2_target <= 0.0 or mz_r2 >= mz_r2_target)
    )

    return {
        "tire_name": tire_name,
        "dataset_complete": dataset_complete,
        "r2": r2_dict,
        "r2_mean": r2_mean,
        "r2_min": r2_min,
        "r2_target": r2_target,
        "mz_r2": mz_r2,
        "mz_r2_target": mz_r2_target,
        "physics": {
            name: {
                "pass": result["pass"],
                "skipped": result.get("skipped", False),
                "metrics": result["metrics"],
                "message": result["message"],
            }
            for name, result in physics_results.items()
        },
        "deployable": deployable,
        "deployment_reasons": deployment_reasons,
    }


def resolve_training_exit_code(
    validation_summary: Dict[str, Any], allow_nondeployable: bool = False, **legacy_kwargs: Any
) -> int:
    """Return the CLI exit code for a completed training/evaluation run."""
    if "require_deployable" in legacy_kwargs:
        allow_nondeployable = not bool(legacy_kwargs["require_deployable"])
    if validation_summary.get("deployable", False):
        return 0
    return 0 if allow_nondeployable else 1


def build_runtime_model(
    model: NormalizedTirePINN,
    config: Dict[str, Any],
) -> PhysicsEnforcedTireModel | NormalizedTirePINN:
    runtime_cfg = config.get("runtime_postprocess", {})
    if not runtime_cfg.get("enabled", False):
        return model

    return PhysicsEnforcedTireModel(
        base_model=model,
        friction_cap_mu=float(runtime_cfg.get("friction_cap_mu", 2.6)),
        zero_slip_sr_scale=float(runtime_cfg.get("zero_slip_sr_scale", 0.03)),
        zero_slip_sa_scale_deg=float(runtime_cfg.get("zero_slip_sa_scale_deg", 0.5)),
        symmetry_sr_scale=float(runtime_cfg.get("symmetry_sr_scale", 0.05)),
        symmetry_ia_scale_deg=float(runtime_cfg.get("symmetry_ia_scale_deg", 0.75)),
        sa_zero_fy_scale_deg=float(runtime_cfg.get("sa_zero_fy_scale_deg", 0.3)),
        epsilon=float(runtime_cfg.get("epsilon", 1.0e-9)),
    )


def build_pinn_core_from_config(config: Dict[str, Any]) -> TirePINN:
    """Build a TirePINN while preserving sequential defaults for older configs."""
    ensure_training_dependencies()
    model_cfg = config["model"]
    return TirePINN(
        hidden_dims=model_cfg["hidden_dims"],
        input_dim=model_cfg["input_dim"],
        output_dim=model_cfg["output_dim"],
        use_layer_norm=model_cfg["use_layer_norm"],
        architecture=model_cfg.get("architecture", "sequential"),
        head_hidden_dims=model_cfg.get("head_hidden_dims"),
        head_outputs=model_cfg.get("head_outputs"),
    )


def evaluate_checkpoint(
    checkpoint_name: str,
    checkpoint_path: Path,
    model: NormalizedTirePINN,
    device: str,
    eval_loader: Any,
    eval_data_frame: Any,
    dataset_info: Dict[str, Any],
    tire_model_name: str,
    config: Dict[str, Any],
    coverage_data_frame: Any = None,
) -> Dict[str, Any]:
    load_checkpoint_into_model(model, checkpoint_path, device)
    runtime_model = build_runtime_model(model, config)
    runtime_model.to(device).eval()

    test_preds = []
    test_targets = []
    test_component_masks = []
    with torch.no_grad():
        for batch in eval_loader:
            if len(batch) == 3:
                x_batch, y_batch, component_mask = batch
            else:
                x_batch, y_batch = batch
                component_mask = None
            x_batch = x_batch.to(device)
            y_batch = y_batch.to(device)
            test_preds.append(runtime_model(x_batch))
            test_targets.append(y_batch)
            if component_mask is not None:
                test_component_masks.append(component_mask.to(device))

    test_preds = torch.cat(test_preds)
    test_targets = torch.cat(test_targets)
    test_component_mask = torch.cat(test_component_masks) if test_component_masks else None
    r2_dict = R2Score.compute_dict(
        test_preds,
        test_targets,
        component_mask=test_component_mask,
    )

    validation_targets = config["validation_targets"]
    symmetry_mean_threshold = float(
        validation_targets.get(
            "symmetry_mean_max_error",
            validation_targets.get("symmetry_max_error", 0.25),
        )
    )
    symmetry_p95_threshold = float(
        validation_targets.get(
            "symmetry_p95_max_error",
            validation_targets.get("symmetry_max_error", 0.45),
        )
    )

    validator = PhysicsValidator(
        runtime_model,
        data_frame=eval_data_frame,
        device=device,
        coverage_data_frame=coverage_data_frame,
    )
    physics_results = validator.run_all_tests(
        zero_threshold=float(
            validation_targets.get(
                "zero_slip_max_force_n",
                validation_targets.get("zero_slip_max_error", 20.0),
            )
        ),
        zero_moment_threshold=float(validation_targets.get("zero_slip_max_moment_nm", 50.0)),
        symmetry_mean_threshold=symmetry_mean_threshold,
        symmetry_p95_threshold=symmetry_p95_threshold,
        max_friction_violation_rate=float(validation_targets.get("friction_violation_rate", 0.05)),
        measured_friction_violation_rate=float(
            validation_targets.get("measured_friction_violation_rate", 0.15)
        ),
        measured_friction_mu_cap=float(
            validation_targets.get("measured_friction_mu_cap", 2.6)
        ),
        measured_combined_slip_violation_rate=float(
            validation_targets.get("measured_combined_slip_violation_rate", 0.35)
        ),
        measured_combined_slip_coverage_tolerance=float(
            validation_targets.get("measured_combined_slip_coverage_tolerance", 1.0)
        ),
        measured_combined_slip_min_coverage=int(
            validation_targets.get("measured_combined_slip_min_coverage", 50)
        ),
        max_monotonicity_violation_rate=float(
            validation_targets.get("monotonicity_violation_rate", 0.10)
        ),
        monotonicity_negative_gradient_tolerance=float(
            validation_targets.get("monotonicity_negative_gradient_tolerance", 0.10)
        ),
        monotonicity_empirical_margin=float(
            validation_targets.get("monotonicity_empirical_margin", 0.0)
        ),
        max_combined_slip_violation_rate=float(
            validation_targets.get("combined_slip_violation_rate", 0.35)
        ),
        combined_slip_margin=float(validation_targets.get("combined_slip_margin", 0.05)),
        max_stiffness_load_violation_rate=float(
            validation_targets.get("stiffness_load_violation_rate", 0.50)
        ),
        stiffness_load_drop_margin=float(
            validation_targets.get("stiffness_load_drop_margin", 0.03)
        ),
        max_high_sa_lateral_load_violation_rate=float(
            validation_targets.get("high_sa_lateral_load_violation_rate", 0.05)
        ),
        high_sa_lateral_load_drop_margin=float(
            validation_targets.get("high_sa_lateral_load_drop_margin", 0.01)
        ),
        high_sa_min_abs_sa_deg=float(validation_targets.get("high_sa_min_abs_sa_deg", 6.0)),
        max_mz_negative_trail_rate=float(validation_targets.get("mz_negative_trail_rate", 0.20)),
        max_mz_trail_cap_rate=float(validation_targets.get("mz_trail_cap_rate", 0.25)),
        max_mz_trail_growth_rate=float(validation_targets.get("mz_trail_growth_rate", 0.65)),
        max_pneumatic_trail_m=float(validation_targets.get("mz_max_trail_m", 0.18)),
        max_camber_asymmetry=float(validation_targets.get("camber_mean_asymmetry", 0.50)),
        max_camber_cap_violation_rate=float(
            validation_targets.get("camber_cap_violation_rate", 0.15)
        ),
        camber_mu_cap=float(validation_targets.get("camber_mu_cap", 0.25)),
        min_camber_coverage_per_side=int(
            validation_targets.get("camber_min_coverage_per_side", 50)
        ),
    )

    validation_summary = summarize_validation_results(
        tire_name=tire_model_name,
        dataset_info=dataset_info,
        r2_dict=r2_dict,
        physics_results=physics_results,
        r2_target=float(validation_targets["r2_min"]),
        mz_r2_target=float(validation_targets.get("mz_r2_min", 0.0)),
    )
    validation_summary["checkpoint"] = checkpoint_name

    return {
        "checkpoint": checkpoint_name,
        "checkpoint_path": str(checkpoint_path),
        "runtime_model": runtime_model,
        "validation_summary": validation_summary,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Train PINN tire model")
    parser.add_argument("--config", type=str, default="tire_pinn/config/pinn_config.yaml")
    parser.add_argument("--device", type=str, default=None, help="auto/cuda/mps/cpu")
    parser.add_argument("--batch-size", type=int, default=None)
    parser.add_argument("--epochs", type=int, default=None)
    parser.add_argument(
        "--resume", type=str, default=None, help="Resume trainer state from checkpoint"
    )
    parser.add_argument(
        "--init-checkpoint",
        type=str,
        default=None,
        help="Initialize model weights from checkpoint without restoring optimizer state",
    )
    parser.add_argument(
        "--tire-model",
        type=str,
        default=None,
        help='Dataset name; use "list" to print the catalog',
    )
    parser.add_argument("--list-tires", action="store_true")
    parser.add_argument("--data-dir", type=str, default=None)
    parser.add_argument("--manifest", type=str, default=None)
    parser.add_argument(
        "--max-samples",
        type=int,
        default=None,
        help="Optional cap on total training samples using stratified subsampling.",
    )
    parser.add_argument(
        "--eval-only",
        action="store_true",
        help="Skip training; evaluate and export existing checkpoints in the artifacts directory",
    )
    parser.add_argument(
        "--sampling-seed",
        type=int,
        default=None,
        help="Random seed used for subsampling when --max-samples is set.",
    )
    parser.add_argument(
        "--require-deployable",
        action="store_true",
        help="Deprecated compatibility flag; deployable validation is required by default.",
    )
    parser.add_argument(
        "--allow-nondeployable",
        action="store_true",
        help="Return exit code 0 for non-deployable models. Use only for exploratory training.",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    ensure_training_dependencies()

    script_dir = Path(__file__).parent
    data_dir = Path(args.data_dir) if args.data_dir else script_dir.parent / "data" / "tiredata"
    catalog = TireCatalog(data_dir, manifest_path=args.manifest)

    if args.list_tires or (args.tire_model and args.tire_model.lower() == "list"):
        catalog.print_catalog()
        return 0

    tire_model_name = args.tire_model or config.get("tire_model", {}).get("name", "auto")
    if tire_model_name.lower() == "auto":
        deployable = catalog.get_deployable_datasets()
        complete = catalog.get_complete_datasets()
        candidates = deployable or complete or catalog.list_tire_models()
        if not candidates:
            raise ValueError("No TTC datasets found.")
        tire_model_name = candidates[0]

    dataset_info = catalog.get_tire_info(tire_model_name)
    if dataset_info is None:
        raise ValueError(f"Unknown tire dataset: {tire_model_name}")

    config, applied_profile = apply_tire_profile_overrides(config, tire_model_name)
    validate_physics_config(config)
    data_config = config.get("data", {})
    device = resolve_device(args.device or config["hardware"].get("device", "auto"))
    batch_size = args.batch_size or config["training"]["batch_size"]
    max_epochs = args.epochs or config["training"]["max_epochs"]
    max_samples = (
        args.max_samples if args.max_samples is not None else data_config.get("max_samples_total")
    )
    sampling_seed = (
        args.sampling_seed
        if args.sampling_seed is not None
        else data_config.get("sampling_seed", data_config.get("random_seed", 42))
    )

    print("=" * 80)
    print("PINN Tire Model Training")
    print("=" * 80)
    print(f"Dataset:    {tire_model_name}")
    print(f"Device:     {device}")
    print(f"Batch size: {batch_size}")
    print(f"Epochs:     {max_epochs}")
    print(f"Max samples:{max_samples if max_samples else 'all'}")
    print(f"Deployable dataset: {dataset_info['deployable']}")
    if applied_profile:
        print("Profile:    tire-specific overrides applied")
    print("=" * 80)

    output_root = Path(config["output"].get("runtime_root", "output/tire_models"))
    model_dir = output_root / tire_model_name
    artifacts_dir = model_dir / "training_artifacts"
    artifacts_dir.mkdir(parents=True, exist_ok=True)

    print("\n[1/6] Loading TTC data...")
    data_loader = TTCDataLoader(
        normalize=True,
        tire_name=tire_model_name,
        data_dir=data_dir,
        manifest_path=args.manifest,
        max_samples_total=max_samples,
        sampling_seed=sampling_seed,
        source_component_masks=data_config.get("source_component_masks"),
        pressure_filter_kpa=data_config.get("pressure_filter_kpa"),
        pressure_tolerance_kpa=data_config.get("pressure_tolerance_kpa", 3.5),
        mirror_negative_ia=bool(data_config.get("mirror_negative_ia", False)),
    )
    train_loader, val_loader, test_loader = data_loader.create_dataloaders(
        batch_size=batch_size,
        train_ratio=data_config["train_ratio"],
        val_ratio=data_config["val_ratio"],
        test_ratio=data_config["test_ratio"],
        random_seed=data_config["random_seed"],
        num_workers=config["hardware"]["num_workers"],
        split_strategy=data_config.get("split_strategy", "group"),
    )
    train_frame = data_loader.get_split_frame("train")
    val_frame = data_loader.get_split_frame("val")
    test_frame = data_loader.get_split_frame("test")
    split_metadata = data_loader.get_split_metadata()
    print(f"Split interpretation: {split_metadata['interpretation']}")
    print(f"Generalization scope: {split_metadata['generalization_scope']}")
    norm_params = data_loader.get_normalization_params()

    print("\n[2/6] Creating PINN model...")
    pinn_core = build_pinn_core_from_config(config)
    model = NormalizedTirePINN(pinn_core)
    model.set_normalization_params(
        input_mean=norm_params["input_mean"],
        input_std=norm_params["input_std"],
        output_mean=norm_params["output_mean"],
        output_std=norm_params["output_std"],
    )

    if args.init_checkpoint:
        print(f"  Initializing from checkpoint: {args.init_checkpoint}")
        load_checkpoint_into_model(model, Path(args.init_checkpoint), device)

    print(f"  Parameters: {pinn_core.count_parameters():,}")

    print("\n[3/6] Setting up collocation sampler...")
    colloc_sampler = CollocationSampler(
        input_mean=norm_params["input_mean"],
        input_std=norm_params["input_std"],
        sr_range=tuple(config["input_ranges"]["sr"]),
        sa_range=tuple(config["input_ranges"]["sa"]),
        fz_range=tuple(config["input_ranges"]["fz"]),
        ia_range=tuple(config["input_ranges"]["ia"]),
        tstc_range=tuple(config["input_ranges"]["tstc"]),
        device=device,
    )

    print("\n[4/6] Initializing trainer...")
    trainer = PINNTrainer(
        model=model,
        train_loader=train_loader,
        val_loader=val_loader,
        collocation_sampler=colloc_sampler,
        device=device,
        learning_rate=config["training"]["learning_rate"],
        weight_decay=config["training"]["weight_decay"],
        output_dir=artifacts_dir,
        curriculum_total_epochs=max_epochs,
        curriculum_final_weights=config.get("curriculum", {}).get("final_weights"),
        data_loss_config=config.get("data_loss", {}),
        physics_config=config.get("physics", {}),
        gradient_clip_norm=float(config["training"].get("gradient_clip_norm", 1.0)),
    )
    trainer.set_normalization_params_for_physics(data_loader)

    if args.resume:
        trainer.load_checkpoint(args.resume)

    print("\n[5/6] Starting training...")
    if args.eval_only:
        print("  --eval-only: skipping training, evaluating existing checkpoints.")
    else:
        trainer.train(
            max_epochs=max_epochs,
            patience=config["training"]["patience"],
            val_interval=config["training"]["val_interval"],
        )

    print("\n[6/6] Evaluating and validating...")
    candidate_names = ["best_r2_model.pt", "best_model.pt", "final_model.pt"]
    evaluated_candidates = []
    seen_paths = set()
    for checkpoint_name in candidate_names:
        checkpoint_path = artifacts_dir / checkpoint_name
        if not checkpoint_path.exists():
            continue
        resolved = str(checkpoint_path.resolve())
        if resolved in seen_paths:
            continue
        seen_paths.add(resolved)
        print(f"\nEvaluating checkpoint: {checkpoint_name}")
        evaluated_candidates.append(
            evaluate_checkpoint(
                checkpoint_name=checkpoint_name,
                checkpoint_path=checkpoint_path,
                model=model,
                device=device,
                eval_loader=val_loader,
                eval_data_frame=val_frame,
                dataset_info=dataset_info,
                tire_model_name=tire_model_name,
                config=config,
                coverage_data_frame=train_frame,
            )
        )

    if not evaluated_candidates:
        raise RuntimeError("No training checkpoints were produced.")

    def candidate_sort_key(candidate: Dict[str, Any]) -> tuple[bool, float, float]:
        summary = candidate["validation_summary"]
        physics_passed = all(result["pass"] is not False for result in summary["physics"].values())
        return (
            physics_passed,
            summary["r2_min"],
            summary["r2_mean"],
        )

    deployable_candidates = [
        candidate
        for candidate in evaluated_candidates
        if candidate["validation_summary"]["deployable"]
    ]
    selected = (
        max(
            deployable_candidates,
            key=candidate_sort_key,
        )
        if deployable_candidates
        else max(
            evaluated_candidates,
            key=candidate_sort_key,
        )
    )

    load_checkpoint_into_model(model, Path(selected["checkpoint_path"]), device)
    runtime_model = build_runtime_model(model, config)
    runtime_model.to(device).eval()

    validation_summary = selected["validation_summary"]
    val_r2 = validation_summary["r2"]

    # Selection is frozen at this point. Evaluate the chosen checkpoint on the
    # held-out test split exactly once for the final report.
    test_candidate = evaluate_checkpoint(
        checkpoint_name=selected["checkpoint"],
        checkpoint_path=Path(selected["checkpoint_path"]),
        model=model,
        device=device,
        eval_loader=test_loader,
        eval_data_frame=test_frame,
        dataset_info=dataset_info,
        tire_model_name=tire_model_name,
        config=config,
        coverage_data_frame=train_frame,
    )
    test_summary = test_candidate["validation_summary"]
    test_r2 = test_summary["r2"]

    print("\nSelected Checkpoint:")
    print(f"  {selected['checkpoint']}")
    print("\nValidation Set Performance (selection):")
    print(f"  R² (Fx): {val_r2['r2_fx']:.4f}")
    print(f"  R² (Fy): {val_r2['r2_fy']:.4f}")
    print(f"  R² (Mx): {val_r2['r2_mx']:.4f}")
    print(f"  R² (Mz): {val_r2['r2_mz']:.4f}")
    print("\nHeld-out Test Set Performance (reported once):")
    print(f"  R² (Fx): {test_r2['r2_fx']:.4f}")
    print(f"  R² (Fy): {test_r2['r2_fy']:.4f}")
    print(f"  R² (Mx): {test_r2['r2_mx']:.4f}")
    print(f"  R² (Mz): {test_r2['r2_mz']:.4f}")

    training_summary = {
        "device": device,
        "batch_size": batch_size,
        "epochs": max_epochs,
        "artifact_dir": str(artifacts_dir),
        "best_val_r2_mean": float(trainer.best_r2_mean)
        if trainer.best_r2_mean != float("-inf")
        else None,
        "model_parameters": pinn_core.count_parameters(),
        "n_samples": len(data_loader.df),
        "max_samples_requested": max_samples,
        "sampling_seed": sampling_seed,
        "split": split_metadata,
        "normalization": {
            "fit_scope": "train",
            "input_mean": norm_params["input_mean"].tolist(),
            "input_std": norm_params["input_std"].tolist(),
            "output_mean": norm_params["output_mean"].tolist(),
            "output_std": norm_params["output_std"].tolist(),
        },
        "validation_summary": validation_summary,
        "test_summary": test_summary,
        "selected_checkpoint": selected["checkpoint"],
        "evaluated_candidates": [
            {
                "checkpoint": candidate["checkpoint"],
                "checkpoint_path": candidate["checkpoint_path"],
                "deployable": candidate["validation_summary"]["deployable"],
                "r2_min": candidate["validation_summary"]["r2_min"],
                "r2_mean": candidate["validation_summary"]["r2_mean"],
            }
            for candidate in evaluated_candidates
        ],
    }
    input_bounds = data_loader.get_input_bounds()

    validation_summary_path = artifacts_dir / "validation_summary.json"
    with validation_summary_path.open("w", encoding="utf-8") as handle:
        json.dump(validation_summary, handle, indent=2)
    test_summary_path = artifacts_dir / "test_summary.json"
    with test_summary_path.open("w", encoding="utf-8") as handle:
        json.dump(test_summary, handle, indent=2)
    candidate_results_path = artifacts_dir / "candidate_results.json"
    with candidate_results_path.open("w", encoding="utf-8") as handle:
        json.dump(
            [candidate["validation_summary"] for candidate in evaluated_candidates],
            handle,
            indent=2,
        )

    print("\nDeployment Gate:")
    if validation_summary["deployable"]:
        print("  ✓ PASS: model is deployable")
        exporter = CPPExporter(runtime_model, model_dir)
        exporter.export(
            tire_name=tire_model_name,
            dataset_info=dataset_info,
            validation_summary=validation_summary,
            training_summary=training_summary,
            input_bounds=input_bounds,
        )
        print(f"\nDeployable runtime assets written to: {model_dir}")
        return 0

    print("  ✗ FAIL: model is not deployable")
    for reason in validation_summary["deployment_reasons"]:
        print(f"    - {reason}")
    print(f"\nTraining artifacts kept at: {artifacts_dir}")
    print("Runtime export skipped because validation gate did not pass.")
    return resolve_training_exit_code(
        validation_summary, allow_nondeployable=args.allow_nondeployable
    )


if __name__ == "__main__":
    raise SystemExit(main())
