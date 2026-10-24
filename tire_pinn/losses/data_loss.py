"""
Data-driven loss functions for PINN tire model.
"""

from __future__ import annotations

import torch
import torch.nn as nn


class DataLoss(nn.Module):
    """
    MSE loss on TTC tire test data.
    Component-wise weighting to balance Fx, Fy, Mx, Mz.
    """

    def __init__(
        self,
        component_weights: list[float] = [1.0, 1.0, 0.5, 0.5],
        normalize: bool = True,
    ):
        """
        Args:
            component_weights: [w_fx, w_fy, w_mx, w_mz]
            normalize: Whether to normalize by number of samples
        """
        super().__init__()
        self.register_buffer('weights', torch.tensor(component_weights))
        self.normalize = normalize

    def forward(
        self,
        pred: torch.Tensor,
        target: torch.Tensor,
        component_mask: torch.Tensor | None = None,
        sample_component_weight: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, dict]:
        """
        Args:
            pred: (batch_size, 4) - [Fx, Fy, Mx, Mz]
            target: (batch_size, 4) - ground truth

        Returns:
            loss: scalar weighted MSE
            metrics: dict with per-component losses
        """
        # Component-wise MSE
        mse = (pred - target) ** 2  # (batch, 4)

        if component_mask is not None:
            component_mask = component_mask.to(device=mse.device, dtype=mse.dtype)
            mse = mse * component_mask

        if sample_component_weight is not None:
            sample_component_weight = sample_component_weight.to(device=mse.device, dtype=mse.dtype)
            mse = mse * sample_component_weight

        if self.normalize:
            if component_mask is not None:
                denom_mask = component_mask
                if sample_component_weight is not None:
                    denom_mask = denom_mask * sample_component_weight
                denom = denom_mask.sum(dim=0).clamp_min(1.0)
                mse = mse.sum(dim=0) / denom
            else:
                if sample_component_weight is not None:
                    denom = sample_component_weight.sum(dim=0).clamp_min(1.0)
                    mse = mse.sum(dim=0) / denom
                else:
                    mse = mse.mean(dim=0)  # (4,)
        else:
            mse = mse.sum(dim=0)

        # Weighted sum
        weighted_mse = mse * self.weights
        total_loss = weighted_mse.sum()

        # Metrics
        metrics = {
            'loss_fx': mse[0].item(),
            'loss_fy': mse[1].item(),
            'loss_mx': mse[2].item(),
            'loss_mz': mse[3].item(),
            'loss_data_total': total_loss.item(),
        }

        return total_loss, metrics


class R2Score:
    """
    R² coefficient of determination for validation.
    """

    @staticmethod
    def compute(
        pred: torch.Tensor,
        target: torch.Tensor,
        component_mask: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """
        Args:
            pred: (N, d) predictions
            target: (N, d) ground truth

        Returns:
            (d,) R² score per output dimension
        """
        if component_mask is None:
            ss_res = ((target - pred) ** 2).sum(dim=0)
            ss_tot = ((target - target.mean(dim=0)) ** 2).sum(dim=0)
            return 1 - ss_res / (ss_tot + 1e-8)

        component_mask = component_mask.to(device=pred.device)
        r2_values = []
        for index in range(pred.shape[1]):
            active = component_mask[:, index] > 0
            if not torch.any(active):
                r2_values.append(torch.tensor(0.0, device=pred.device, dtype=pred.dtype))
                continue
            pred_i = pred[active, index]
            target_i = target[active, index]
            ss_res = ((target_i - pred_i) ** 2).sum()
            ss_tot = ((target_i - target_i.mean()) ** 2).sum()
            r2_values.append(1 - ss_res / (ss_tot + 1e-8))
        return torch.stack(r2_values)

    @staticmethod
    def compute_dict(
        pred: torch.Tensor,
        target: torch.Tensor,
        component_mask: torch.Tensor | None = None,
    ) -> dict:
        """Return R² as dict with component names."""
        r2 = R2Score.compute(pred, target, component_mask=component_mask)
        return {
            'r2_fx': r2[0].item(),
            'r2_fy': r2[1].item(),
            'r2_mx': r2[2].item(),
            'r2_mz': r2[3].item(),
        }
