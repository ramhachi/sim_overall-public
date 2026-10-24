"""
PINN (Physics-Informed Neural Network) tire model.

Key differences from standard DNN:
- LayerNorm instead of BatchNorm (collocation point compatibility)
- Swish instead of ReLU (smooth gradients for autograd)
- No dropout (physics constraints act as regularization)
"""

from __future__ import annotations

import torch
import torch.nn as nn
from collections import OrderedDict
from typing import Dict, List, Union
from .activations import SwishActivation


class TirePINN(nn.Module):
    """
    Physics-Informed Neural Network for tire force prediction.

    Architecture:
    - Input: (SR, SA, Fz, IA, TSTC) - 5D
    - Production: low-depth multi-head, no trunk, component heads
      fx/fy_mz=[128, 128, 64, 64, 32], mx=[64, 64, 32]
    - Legacy/experimental: sequential MLP via architecture="sequential"
    - Output: (Fx, Fy, Mx, Mz) - 4D
    - Activation: Swish (C∞ smooth)
    - Normalization: LayerNorm (per-sample, batch-independent)
    """

    def __init__(
        self,
        hidden_dims: List[int] | None = None,
        input_dim: int = 5,
        output_dim: int = 4,
        use_layer_norm: bool = True,
        architecture: str = "multi_head",
        head_hidden_dims: Dict[str, List[int]] | None = None,
        head_outputs: Dict[str, List[int]] | None = None,
    ):
        super().__init__()

        if hidden_dims is None:
            hidden_dims = []
        if head_hidden_dims is None and architecture == "multi_head":
            head_hidden_dims = {
                "fx": [128, 128, 64, 64, 32],
                "fy_mz": [128, 128, 64, 64, 32],
                "mx": [64, 64, 32],
            }

        self.input_dim = input_dim
        self.output_dim = output_dim
        self.hidden_dims = hidden_dims
        self.use_layer_norm = use_layer_norm
        self.architecture = architecture
        self.head_hidden_dims = head_hidden_dims or {}
        self.head_outputs = self._normalize_head_outputs(head_outputs)

        if self.architecture == "sequential":
            self.net = self._build_mlp(input_dim, hidden_dims, output_dim, use_layer_norm)
        elif self.architecture == "multi_head":
            self.trunk = self._build_feature_stack(input_dim, hidden_dims, use_layer_norm)
            self.trunk_output_dim = hidden_dims[-1] if hidden_dims else input_dim
            self.heads = nn.ModuleDict()
            for head_name, output_indices in self.head_outputs.items():
                dims = self.head_hidden_dims.get(head_name)
                if dims is None:
                    dims = self.head_hidden_dims.get("default", [128, 128, 64, 64, 32])
                self.heads[head_name] = self._build_mlp(
                    self.trunk_output_dim,
                    dims,
                    len(output_indices),
                    use_layer_norm,
                )
            self.net = nn.Sequential()
        else:
            raise ValueError(
                f"Unknown TirePINN architecture '{architecture}'. "
                "Use 'sequential' or 'multi_head'."
            )

        self._initialize_weights()

    @staticmethod
    def _build_feature_stack(
        input_dim: int,
        hidden_dims: List[int],
        use_layer_norm: bool,
    ) -> nn.Sequential:
        layers = []
        in_dim = input_dim

        for h_dim in hidden_dims:
            # Fully connected layer
            layers.append(nn.Linear(in_dim, h_dim))

            # Layer normalization (per-sample, not batch-dependent)
            if use_layer_norm:
                layers.append(nn.LayerNorm(h_dim))

            # Swish activation (smooth for autograd)
            layers.append(SwishActivation())

            in_dim = h_dim

        return nn.Sequential(*layers)

    @classmethod
    def _build_mlp(
        cls,
        input_dim: int,
        hidden_dims: List[int],
        output_dim: int,
        use_layer_norm: bool,
    ) -> nn.Sequential:
        layers = list(cls._build_feature_stack(input_dim, hidden_dims, use_layer_norm))
        in_dim = hidden_dims[-1] if hidden_dims else input_dim
        layers.append(nn.Linear(in_dim, output_dim))
        return nn.Sequential(*layers)

    @staticmethod
    def _normalize_head_outputs(
        head_outputs: Dict[str, List[int]] | None,
    ) -> "OrderedDict[str, List[int]]":
        if head_outputs is None:
            head_outputs = {
                "fx": [0],
                "fy_mz": [1, 3],
                "mx": [2],
            }
        ordered = OrderedDict()
        for name, indices in head_outputs.items():
            ordered[name] = [int(index) for index in indices]
        return ordered

    def _initialize_weights(self):
        """
        Xavier uniform initialization for stable training.
        LayerNorm parameters initialized to identity transform.
        """
        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.xavier_uniform_(module.weight)
                if module.bias is not None:
                    nn.init.zeros_(module.bias)
            elif isinstance(module, nn.LayerNorm):
                nn.init.ones_(module.weight)  # gamma = 1
                nn.init.zeros_(module.bias)  # beta = 0

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass.

        Args:
            x: (batch_size, 5) - [SR, SA, Fz, IA, TSTC]

        Returns:
            (batch_size, 4) - [Fx, Fy, Mx, Mz]
        """
        if self.architecture == "sequential":
            return self.net(x)

        features = self.trunk(x)
        output = x.new_zeros((x.shape[0], self.output_dim))
        for head_name, output_indices in self.head_outputs.items():
            head_output = self.heads[head_name](features)
            output[:, output_indices] = head_output
        return output

    def predict_with_grad(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Predict forces and compute gradients w.r.t. inputs.
        Used for monotonicity loss (∂Fy/∂Fz ≥ 0).

        Args:
            x: (batch_size, 5) with requires_grad=True

        Returns:
            pred: (batch_size, 4) - force predictions
            grads: (batch_size, 4, 5) - ∂F/∂x
        """
        assert x.requires_grad, "Input must have requires_grad=True"

        pred = self.forward(x)

        # Compute gradients for each output w.r.t. all inputs
        grads = []
        for i in range(self.output_dim):
            grad_i = torch.autograd.grad(
                outputs=pred[:, i].sum(),
                inputs=x,
                create_graph=True,
                retain_graph=True,
            )[0]
            grads.append(grad_i)

        grads = torch.stack(grads, dim=1)  # (batch, 4, 5)
        return pred, grads

    def count_parameters(self) -> int:
        """Count trainable parameters."""
        return sum(p.numel() for p in self.parameters() if p.requires_grad)

    def get_architecture_info(self) -> dict:
        """Export architecture information for C++ implementation."""
        info = {
            "architecture": self.architecture,
            "input_dim": self.input_dim,
            "output_dim": self.output_dim,
            "hidden_dims": self.hidden_dims,
            "use_layer_norm": self.use_layer_norm,
            "activation": "swish",
            "total_parameters": self.count_parameters(),
        }
        if self.architecture == "multi_head":
            info["head_hidden_dims"] = {
                name: self.head_hidden_dims.get(name, self.head_hidden_dims.get("default", []))
                for name in self.head_outputs
            }
            info["head_outputs"] = dict(self.head_outputs)
        return info

    def get_runtime_structure(self) -> dict:
        """Return module sequences in the order expected by the runtime exporter."""
        if self.architecture == "sequential":
            return {"type": "sequential", "layers": list(self.net)}
        return {
            "type": "multi_head",
            "trunk_layers": list(self.trunk),
            "heads": [
                {
                    "name": name,
                    "output_indices": indices,
                    "layers": list(self.heads[name]),
                }
                for name, indices in self.head_outputs.items()
            ],
        }


class NormalizedTirePINN(nn.Module):
    """
    Wrapper around TirePINN with input/output normalization.
    Matches MATLAB DNN pipeline (Z-score normalization).
    """

    def __init__(self, pinn_model: TirePINN):
        super().__init__()
        self.pinn = pinn_model

        # Normalization parameters (registered as buffers for state_dict)
        # Will be loaded from TTC data statistics
        self.register_buffer("input_mean", torch.zeros(5))
        self.register_buffer("input_std", torch.ones(5))
        self.register_buffer("output_mean", torch.zeros(4))
        self.register_buffer("output_std", torch.ones(4))

    def set_normalization_params(
        self,
        input_mean: torch.Tensor,
        input_std: torch.Tensor,
        output_mean: torch.Tensor,
        output_std: torch.Tensor,
    ):
        """Set normalization parameters from data statistics."""
        self.input_mean.copy_(input_mean)
        self.input_std.copy_(input_std)
        self.output_mean.copy_(output_mean)
        self.output_std.copy_(output_std)

    def normalize_input(self, x: torch.Tensor) -> torch.Tensor:
        """Z-score normalization: (x - mean) / std"""
        return (x - self.input_mean) / self.input_std

    def denormalize_output(self, y_norm: torch.Tensor) -> torch.Tensor:
        """Inverse Z-score: y = y_norm * std + mean"""
        return y_norm * self.output_std + self.output_mean

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass with normalization.

        Args:
            x: (batch_size, 5) - raw inputs [SR, SA, Fz, IA, TSTC]

        Returns:
            (batch_size, 4) - raw outputs [Fx, Fy, Mx, Mz]
        """
        x_norm = self.normalize_input(x)
        y_norm = self.pinn(x_norm)
        y = self.denormalize_output(y_norm)
        return y

    def forward_normalized(self, x_norm: torch.Tensor) -> torch.Tensor:
        """
        Forward pass in normalized space (for physics losses).

        Args:
            x_norm: (batch_size, 5) - normalized inputs

        Returns:
            (batch_size, 4) - normalized outputs
        """
        return self.pinn(x_norm)


class PhysicsEnforcedTireModel(nn.Module):
    """
    Runtime wrapper that applies lightweight physical safeguards to a trained PINN.

    The wrapped model preserves the learned force surface while:
    - canceling residual Fx/Fy/Mz offsets near SR=SA=IA=0
    - capping the combined-slip force magnitude to μ * Fz
    """

    def __init__(
        self,
        base_model: NormalizedTirePINN,
        friction_cap_mu: float = 2.6,
        zero_slip_sr_scale: float = 0.03,
        zero_slip_sa_scale_deg: float = 0.5,
        symmetry_sr_scale: float = 0.05,
        symmetry_ia_scale_deg: float = 0.75,
        sa_zero_fy_scale_deg: float = 0.3,
        epsilon: float = 1.0e-9,
    ):
        super().__init__()
        self.base_model = base_model
        self.friction_cap_mu = float(friction_cap_mu)
        self.zero_slip_sr_scale = float(zero_slip_sr_scale)
        self.zero_slip_sa_scale_deg = float(zero_slip_sa_scale_deg)
        self.symmetry_sr_scale = float(symmetry_sr_scale)
        self.symmetry_ia_scale_deg = float(symmetry_ia_scale_deg)
        self.sa_zero_fy_scale_deg = float(sa_zero_fy_scale_deg)
        self.epsilon = float(epsilon)

    def get_runtime_postprocess_config(self) -> Dict[str, Union[float, bool]]:
        """Return the post-processing settings that must be mirrored in C++ runtime."""
        return {
            "enabled": True,
            "friction_cap_mu": self.friction_cap_mu,
            "zero_slip_sr_scale": self.zero_slip_sr_scale,
            "zero_slip_sa_scale_deg": self.zero_slip_sa_scale_deg,
            "symmetry_sr_scale": self.symmetry_sr_scale,
            "symmetry_ia_scale_deg": self.symmetry_ia_scale_deg,
            "sa_zero_fy_scale_deg": self.sa_zero_fy_scale_deg,
            "epsilon": self.epsilon,
        }

    def _apply_sa_zero_fy_correction(self, x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        """
        Enforce Fy -> 0 as SA and IA approach zero for any SR value.

        Camber thrust can legitimately generate Fy at SA=0 when IA != 0, so
        this safety-net is gated by both SA and IA. The correction removes only
        the learned zero-angle residual; it intentionally does not multiply Fy by
        a gate because that would force the cornering stiffness dFy/dSA to zero.
        """
        if self.sa_zero_fy_scale_deg <= 0.0:
            return y

        zero_input = x.clone()
        zero_input[:, 1] = 0.0
        zero_input[:, 3] = 0.0
        zero_output = self._apply_zero_slip_correction(
            zero_input,
            self.base_model(zero_input),
        )
        zero_output = self._apply_symmetry_projection(zero_input, zero_output)

        sa_normalized = x[:, 1] / self.sa_zero_fy_scale_deg
        metric = sa_normalized**2
        if self.symmetry_ia_scale_deg > 0.0:
            ia_normalized = x[:, 3] / self.symmetry_ia_scale_deg
            metric = metric + ia_normalized**2
        residual_gate = torch.exp(-metric)
        y_out = y.clone()
        y_out[:, 1] = y[:, 1] - residual_gate * zero_output[:, 1]
        return y_out

    def _apply_zero_slip_correction(self, x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        if self.zero_slip_sr_scale <= 0.0 or self.zero_slip_sa_scale_deg <= 0.0:
            return y

        zero_input = x.clone()
        zero_input[:, 0] = 0.0
        zero_input[:, 1] = 0.0
        zero_input[:, 3] = 0.0
        zero_output = self.base_model(zero_input)
        slip_metric = (x[:, 0] / self.zero_slip_sr_scale) ** 2 + (
            x[:, 1] / self.zero_slip_sa_scale_deg
        ) ** 2
        gate = torch.exp(-slip_metric).unsqueeze(1)
        correction = torch.stack(
            [
                zero_output[:, 0],
                zero_output[:, 1],
                torch.zeros_like(zero_output[:, 2]),
                zero_output[:, 3],
            ],
            dim=1,
        )
        return y - gate * correction

    def _apply_symmetry_projection(self, x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        if self.symmetry_sr_scale <= 0.0 or self.symmetry_ia_scale_deg <= 0.0:
            return y

        mirror_input = x.clone()
        mirror_input[:, 1] = -mirror_input[:, 1]
        mirror_output = self._apply_zero_slip_correction(
            mirror_input,
            self.base_model(mirror_input),
        )

        symmetry_metric = (x[:, 0] / self.symmetry_sr_scale) ** 2 + (
            x[:, 3] / self.symmetry_ia_scale_deg
        ) ** 2
        gate = torch.exp(-symmetry_metric).unsqueeze(1)
        projected = torch.stack(
            [
                0.5 * (y[:, 0] + mirror_output[:, 0]),
                0.5 * (y[:, 1] - mirror_output[:, 1]),
                0.5 * (y[:, 2] + mirror_output[:, 2]),
                0.5 * (y[:, 3] - mirror_output[:, 3]),
            ],
            dim=1,
        )
        return y + gate * (projected - y)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        y = self.base_model(x)
        y = self._apply_zero_slip_correction(x, y)
        y = self._apply_symmetry_projection(x, y)
        y = self._apply_sa_zero_fy_correction(x, y)  # Must come after symmetry

        if self.friction_cap_mu > 0.0:
            force_mag = torch.sqrt(y[:, 0] ** 2 + y[:, 1] ** 2 + self.epsilon)
            limit = self.friction_cap_mu * x[:, 2].abs()
            scale = torch.minimum(
                torch.ones_like(force_mag),
                limit / (force_mag + self.epsilon),
            ).unsqueeze(1)
            y = torch.cat([y[:, 0:2] * scale, y[:, 2:]], dim=1)

        return y
