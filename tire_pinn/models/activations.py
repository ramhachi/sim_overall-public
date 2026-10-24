"""
Custom activation functions for PINN tire model.
"""

import torch
import torch.nn as nn


class SwishActivation(nn.Module):
    """
    Swish (SiLU) activation function: f(x) = x * sigmoid(x)

    Properties:
    - Smooth and infinitely differentiable (C∞)
    - Non-monotonic with self-gating behavior
    - Better gradient flow than ReLU for autograd-based physics losses
    """

    def __init__(self):
        super().__init__()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x * torch.sigmoid(x)


class SwishLUT(nn.Module):
    """
    Lookup table approximation of Swish for faster C++ inference.
    Used only for validation/export - training uses exact Swish.
    """

    def __init__(self, table_size: int = 2048, x_min: float = -8.0, x_max: float = 8.0):
        super().__init__()
        self.x_min = x_min
        self.x_max = x_max
        self.table_size = table_size

        # Pre-compute lookup table
        x = torch.linspace(x_min, x_max, table_size)
        y = x * torch.sigmoid(x)
        self.register_buffer('lut', y)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # Clamp input to table range
        x_clamped = torch.clamp(x, self.x_min, self.x_max)

        # Map to table index
        idx = ((x_clamped - self.x_min) / (self.x_max - self.x_min) * (self.table_size - 1))
        idx_floor = idx.long()
        idx_ceil = torch.clamp(idx_floor + 1, max=self.table_size - 1)

        # Linear interpolation
        alpha = idx - idx_floor.float()
        return self.lut[idx_floor] * (1 - alpha) + self.lut[idx_ceil] * alpha
