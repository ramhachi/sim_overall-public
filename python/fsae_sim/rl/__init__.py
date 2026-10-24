"""Residual-learning utilities for the attack driver.

The C++ attack driver owns the real-time control loop.  This package keeps the
learning side portable by optimizing the small residual profile consumed by
``--attack-residual-profile``.
"""

from .attack_residual_dataset import (
    AttackResidualDataset,
    build_attack_residual_dataset,
    save_attack_residual_dataset,
)
from .attack_residual_env import (
    AttackResidualEnvConfig,
    AttackResidualProfileEnv,
    AttackRunResult,
    action_to_profile,
    profile_to_action,
)
from .attack_residual_policy import (
    ResidualBasisConfig,
    basis_action_to_profile,
    basis_reconstruction_rmse,
    delta_basis_action_to_profile,
    profile_to_basis_action,
    residual_basis_dim,
)

__all__ = [
    "AttackResidualDataset",
    "AttackResidualEnvConfig",
    "AttackResidualProfileEnv",
    "AttackRunResult",
    "ResidualBasisConfig",
    "action_to_profile",
    "basis_action_to_profile",
    "basis_reconstruction_rmse",
    "delta_basis_action_to_profile",
    "profile_to_action",
    "profile_to_basis_action",
    "residual_basis_dim",
    "build_attack_residual_dataset",
    "save_attack_residual_dataset",
]
