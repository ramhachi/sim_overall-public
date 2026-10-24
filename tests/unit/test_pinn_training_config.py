from pathlib import Path

from tire_pinn.main_pinn_train import (
    apply_tire_profile_overrides,
    load_config,
    resolve_training_exit_code,
)


def test_apply_tire_profile_overrides_merges_mrf_recipe():
    config = load_config(str(Path("tire_pinn/config/pinn_config.yaml")))

    merged, overrides = apply_tire_profile_overrides(config, "MRF_18p0X6p0-10")

    assert overrides is not None
    assert merged["training"]["batch_size"] == 2048
    assert merged["training"]["max_epochs"] == 100
    assert merged["training"]["patience"] == 15
    assert merged["data"]["max_samples_total"] == 120000
    assert merged["curriculum"]["final_weights"]["mono"] == 0.30
    assert merged["curriculum"]["final_weights"]["combined"] == 0.02
    assert merged["curriculum"]["final_weights"]["stiffness"] == 0.005
    assert merged["curriculum"]["final_weights"]["high_sa_lateral"] == 0.05
    assert merged["curriculum"]["final_weights"]["mz_trail"] == 0.01
    assert merged["curriculum"]["final_weights"]["camber"] == 0.01
    assert merged["hardware"]["num_workers"] == 0


def test_apply_tire_profile_overrides_merges_hoosier_recipe():
    config = load_config(str(Path("tire_pinn/config/pinn_config.yaml")))

    merged, overrides = apply_tire_profile_overrides(config, "Hoosier_18p0X6p0-10_R20_Rim6")

    assert overrides is not None
    assert merged["training"]["batch_size"] == 2048
    assert merged["training"]["max_epochs"] == 80
    assert merged["data"]["max_samples_total"] == 40000
    assert merged["hardware"]["num_workers"] == 0
    assert merged["validation_targets"]["r2_min"] == 0.90
    assert merged["validation_targets"]["friction_violation_rate"] == 0.12
    assert merged["validation_targets"]["monotonicity_empirical_margin"] == 0.08
    assert merged["validation_targets"]["high_sa_lateral_load_violation_rate"] == 0.05
    assert merged["validation_targets"]["high_sa_lateral_load_drop_margin"] == 0.01
    assert merged["validation_targets"]["mz_negative_trail_rate"] == 0.25
    assert merged["validation_targets"]["camber_mu_cap"] == 0.50
    assert merged["validation_targets"]["camber_min_coverage_per_side"] == 50


def test_default_pinn_architecture_is_deployable_multi_head():
    config = load_config(str(Path("tire_pinn/config/pinn_config.yaml")))

    model = config["model"]
    assert model["architecture"] == "multi_head"
    assert model["hidden_dims"] == []
    assert model["head_hidden_dims"]["fx"] == [128, 128, 64, 64, 32]
    assert model["head_hidden_dims"]["fy_mz"] == [128, 128, 64, 64, 32]
    assert model["head_hidden_dims"]["mx"] == [64, 64, 32]


def test_lco_profiles_use_random_interpolation_split():
    config = load_config(str(Path("tire_pinn/config/pinn_config.yaml")))

    for tire_name in (
        "Hoosier_18p0X6p0-10_LCO_C2000_Rim6",
        "Hoosier_18p0X6p0-10_LCO_C2000_Rim7",
    ):
        merged, overrides = apply_tire_profile_overrides(config, tire_name)

        assert overrides is not None
        assert merged["data"]["split_strategy"] == "random"
        assert merged["validation_targets"]["friction_violation_rate"] == 0.12
        assert merged["validation_targets"]["monotonicity_empirical_margin"] == 0.08
        assert merged["validation_targets"]["high_sa_lateral_load_violation_rate"] == 0.05
        assert merged["validation_targets"]["high_sa_lateral_load_drop_margin"] == 0.01
        assert merged["validation_targets"]["mz_negative_trail_rate"] == 0.20
        assert merged["validation_targets"]["camber_mu_cap"] == 0.50
        assert merged["validation_targets"]["camber_min_coverage_per_side"] == 50


def test_default_validation_target_r2_min_is_relaxed_to_point_nine():
    config = load_config(str(Path("tire_pinn/config/pinn_config.yaml")))

    assert config["validation_targets"]["r2_min"] == 0.90


def test_default_curriculum_includes_extended_physics_constraints():
    config = load_config(str(Path("tire_pinn/config/pinn_config.yaml")))

    final_weights = config["curriculum"]["final_weights"]
    assert final_weights["combined"] == 0.50
    assert final_weights["stiffness"] == 0.05
    assert final_weights["high_sa_lateral"] == 0.10
    assert final_weights["mz_trail"] == 0.10
    assert final_weights["camber"] == 0.10


def test_default_validation_targets_include_extended_physics_checks():
    config = load_config(str(Path("tire_pinn/config/pinn_config.yaml")))

    targets = config["validation_targets"]
    assert targets["combined_slip_violation_rate"] == 0.35
    assert targets["stiffness_load_violation_rate"] == 0.50
    assert targets["high_sa_lateral_load_violation_rate"] == 0.05
    assert targets["high_sa_lateral_load_drop_margin"] == 0.01
    assert targets["mz_negative_trail_rate"] == 0.20
    assert targets["camber_mean_asymmetry"] == 0.50
    assert targets["camber_mu_cap"] == 0.25


def test_apply_tire_profile_overrides_returns_original_for_unknown_tire():
    config = load_config(str(Path("tire_pinn/config/pinn_config.yaml")))

    merged, overrides = apply_tire_profile_overrides(config, "Unknown_Profile")

    assert overrides is None
    assert merged["training"]["batch_size"] == config["training"]["batch_size"]
    assert merged["data"]["max_samples_total"] == config["data"]["max_samples_total"]


def test_resolve_training_exit_code_rejects_nondeployable_completion_by_default():
    validation_summary = {"deployable": False}

    assert resolve_training_exit_code(validation_summary) == 1


def test_resolve_training_exit_code_can_allow_exploratory_nondeployable_output():
    validation_summary = {"deployable": False}

    assert resolve_training_exit_code(validation_summary, allow_nondeployable=True) == 0
