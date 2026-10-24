"""Loss functions for PINN training."""

from .data_loss import DataLoss, R2Score
from .physics_losses import (
    ZeroSlipLoss,
    SymmetryLoss,
    FrictionCircleLoss,
    CombinedSlipLoss,
    StiffnessLoadSensitivityLoss,
    HighSaLateralLoadSensitivityLoss,
    MzPneumaticTrailLoss,
    CamberThrustLoss,
    SectionSmoothnessLoss,
    MonotonicityLoss,
    FySaZeroLoss,
    PhysicsLossManager,
)

__all__ = [
    "DataLoss",
    "R2Score",
    "ZeroSlipLoss",
    "SymmetryLoss",
    "FrictionCircleLoss",
    "CombinedSlipLoss",
    "StiffnessLoadSensitivityLoss",
    "HighSaLateralLoadSensitivityLoss",
    "MzPneumaticTrailLoss",
    "CamberThrustLoss",
    "SectionSmoothnessLoss",
    "MonotonicityLoss",
    "FySaZeroLoss",
    "PhysicsLossManager",
]
