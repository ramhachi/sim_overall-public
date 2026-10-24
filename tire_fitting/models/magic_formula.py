"""
Simplified Pacejka Magic Formula tire model.

This implementation is intentionally lightweight: it supports the subset of
parameters used by the repository's fitting and export pipeline while keeping
the same TireModel interface as TMeasy and Hybrid models.
"""

from __future__ import annotations

from typing import Dict, Union

import numpy as np

from tire_fitting.models.base import TireModel


class MagicFormulaModel(TireModel):
    """Magic Formula model with longitudinal, lateral, and optional temperature terms."""

    def __init__(self, params: Dict[str, float] | None = None):
        self.params = params.copy() if params is not None else {}

    def calculate_fx(self, slip_ratio: float, fz: float, **kwargs) -> float:
        self.validate_inputs(fz)

        temperature = kwargs.get("temperature", self.params.get("T_ref", 60.0))
        fz0 = self.params.get("fz0", 1200.0)
        dfz = (fz - fz0) / max(fz0, 1e-6)

        pDx1 = self.params.get("pDx1", 1.5)
        pCx1 = self.params.get("pCx1", 1.7)
        pEx1 = self.params.get("pEx1", -0.2)
        pKx1 = self.params.get("pKx1", 18.0)
        pKx2 = self.params.get("pKx2", 0.0)
        pKx3 = self.params.get("pKx3", 0.0)
        pDx2_temp = self.params.get("pDx2_temp", 0.0)
        t_ref = self.params.get("T_ref", 60.0)

        mu_x = max(0.1, pDx1 + pDx2_temp * (temperature - t_ref))
        D = mu_x * fz
        C = pCx1
        stiffness = max(1.0, (pKx1 + pKx2 * dfz) * np.exp(pKx3 * dfz) * fz)
        B = stiffness / max(C * D, 1e-6)
        E = pEx1

        return float(self._magic_formula(slip_ratio, B, C, D, E))

    def calculate_fy(self, slip_angle: float, fz: float, **kwargs) -> float:
        self.validate_inputs(fz)

        camber_angle = kwargs.get("camber_angle", 0.0)
        fz0 = self.params.get("fz0", 1200.0)
        dfz = (fz - fz0) / max(fz0, 1e-6)

        pDy1 = self.params.get("pDy1", 1.8)
        pCy1 = self.params.get("pCy1", 1.4)
        pEy1 = self.params.get("pEy1", -0.2)
        pKy1 = self.params.get("pKy1", 16.0)
        pKy2 = self.params.get("pKy2", 0.0)
        pKy3 = self.params.get("pKy3", 0.0)

        D = pDy1 * fz
        C = pCy1
        stiffness = max(1.0, (pKy1 + pKy2 * dfz) * np.exp(pKy3 * dfz) * fz)
        B = stiffness / max(C * D, 1e-6)
        E = pEy1

        fy = self._magic_formula(slip_angle, B, C, D, E)
        camber_gain = self.params.get("pKy_cam", 0.0)
        return float(fy + camber_gain * camber_angle * fz)

    def get_model_type(self) -> str:
        return "magic_formula"

    def get_parameters(self) -> Dict[str, float]:
        return self.params.copy()

    def set_parameters(self, params: Dict[str, float]) -> None:
        self.params.update(params)

    @staticmethod
    def _magic_formula(
        slip: Union[float, np.ndarray],
        B: float,
        C: float,
        D: float,
        E: float,
    ) -> Union[float, np.ndarray]:
        x = np.atleast_1d(slip)
        inner = B * x
        y = D * np.sin(C * np.arctan(inner - E * (inner - np.arctan(inner))))
        return y.item() if np.isscalar(slip) else y
