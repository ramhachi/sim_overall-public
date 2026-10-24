#!/usr/bin/env python3
"""Train a small behavior-cloned RL actor export from PID teacher CSV.

The exported files match cpp/include/fsae/driver/rl_driver.hpp:
  - rl_actor_weights.txt
  - rl_normalization_params.txt
  - rl_actor_test_data.csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset


SERVER_OBS_BOUNDS = np.array(
    [
        [0.0, 40.0],
        [-10.0, 10.0],
        [-3.0, 3.0],
        [-15.0, 15.0],
        [-15.0, 15.0],
        [-0.15, 0.15],
        [-25.0, 25.0],
        [0.0, 1.0],
        [0.0, 1.0],
        [-0.3, 0.3],
        [-0.3, 0.3],
        [-0.3, 0.3],
        [-0.3, 0.3],
        [-0.5, 0.5],
        [-0.5, 0.5],
        [-5.0, 5.0],
        [-1.0, 1.0],
        [-0.2, 0.2],
        [-0.2, 0.2],
        [-0.2, 0.2],
        [-0.2, 0.2],
        [-0.2, 0.2],
        [-0.2, 0.2],
        [0.0, 5.0],
        [0.0, 5.0],
        [0.0, 1.0],
        [1.0, 10.0],
        [1.0, 10.0],
        [1.0, 10.0],
        [1.0, 10.0],
        [-0.10, 0.10],
        [-3.0, 3.0],
        [-3.0, 3.0],
        [-20.0, 20.0],
    ],
    dtype=np.float64,
)


class ActorNet(nn.Module):
    def __init__(self, obs_dim: int = 45, act_dim: int = 2, hidden: tuple[int, ...] = (256, 256, 128)):
        super().__init__()
        layers: list[nn.Module] = []
        prev = obs_dim
        for width in hidden:
            layers.append(nn.Linear(prev, width))
            layers.append(nn.ReLU())
            prev = width
        layers.append(nn.Linear(prev, act_dim))
        layers.append(nn.Tanh())
        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def load_teacher_csv(paths: list[Path]) -> tuple[np.ndarray, np.ndarray]:
    xs: list[np.ndarray] = []
    ys: list[np.ndarray] = []
    for path in paths:
        data = np.genfromtxt(path, delimiter=",", names=True, dtype=np.float64)
        obs = np.column_stack([data[f"obs_{i}"] for i in range(1, 46)])
        act = np.column_stack([data["act_steer"], data["act_accel"]])
        mask = np.isfinite(obs).all(axis=1) & np.isfinite(act).all(axis=1)
        xs.append(obs[mask])
        ys.append(np.clip(act[mask], -1.0, 1.0))
    return np.vstack(xs), np.vstack(ys)


def make_bounds(raw_obs: np.ndarray) -> np.ndarray:
    bounds = np.zeros((45, 2), dtype=np.float64)
    bounds[:34] = SERVER_OBS_BOUNDS

    # Vehicle parameters are constant for a deployment vehicle. Center their
    # normalization on the teacher vehicle so the C++ runtime sees zeros for
    # the same parameter set, while preserving non-zero ranges.
    vehicle_values = np.median(raw_obs[:, 34:45], axis=0)
    margins = np.maximum(np.abs(vehicle_values) * 0.05, 1.0)
    bounds[34:, 0] = vehicle_values - margins
    bounds[34:, 1] = vehicle_values + margins
    return bounds


def normalize_obs(raw_obs: np.ndarray, bounds: np.ndarray) -> np.ndarray:
    ranges = np.maximum(bounds[:, 1] - bounds[:, 0], 1e-9)
    x = 2.0 * (raw_obs - bounds[:, 0]) / ranges - 1.0
    return np.clip(x, -1.0, 1.0)


def train_model(x: np.ndarray, y: np.ndarray, epochs: int, batch_size: int, lr: float, seed: int) -> ActorNet:
    torch.manual_seed(seed)
    np.random.seed(seed)
    device = torch.device("cpu")
    model = ActorNet().to(device)
    dataset = TensorDataset(
        torch.tensor(x, dtype=torch.float32),
        torch.tensor(y, dtype=torch.float32),
    )
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=True, drop_last=False)
    opt = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-5)
    loss_fn = nn.MSELoss()

    for epoch in range(1, epochs + 1):
        total = 0.0
        count = 0
        model.train()
        for xb, yb in loader:
            pred = model(xb)
            loss = loss_fn(pred, yb)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()
            total += float(loss.item()) * xb.shape[0]
            count += xb.shape[0]
        if epoch == 1 or epoch % 25 == 0 or epoch == epochs:
            print(f"epoch={epoch:04d} mse={total / max(count, 1):.6f}")
    return model


def linear_layers(model: ActorNet) -> list[nn.Linear]:
    return [m for m in model.net if isinstance(m, nn.Linear)]


def export_weights(model: ActorNet, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    names = ["fc1", "fc2", "fc3", "fcMean"]
    with (output_dir / "rl_actor_weights.txt").open("w", encoding="utf-8") as f:
        layers = linear_layers(model)
        if len(layers) != len(names):
            raise RuntimeError(f"expected {len(names)} linear layers, got {len(layers)}")
        for name, layer in zip(names, layers):
            weight = layer.weight.detach().cpu().numpy()
            bias = layer.bias.detach().cpu().numpy()
            f.write(f"LAYER {name}\n")
            f.write(f"WEIGHTS {weight.shape[0]} {weight.shape[1]}\n")
            for value in weight.reshape(-1):
                f.write(f"{value:.15e}\n")
            f.write(f"BIAS {bias.shape[0]}\n")
            for value in bias:
                f.write(f"{value:.15e}\n")


def export_normalization(bounds: np.ndarray, output_dir: Path, steer_scale: float) -> None:
    with (output_dir / "rl_normalization_params.txt").open("w", encoding="utf-8") as f:
        f.write(f"OBS_BOUNDS {bounds.shape[0]}\n")
        for lo, hi in bounds:
            f.write(f"{lo:.15e} {hi:.15e}\n")
        f.write("ACTION_SCALE 2\n")
        f.write(f"{steer_scale:.15e}\n")
        f.write("1.000000000000000e+00\n")


def export_test_data(model: ActorNet, output_dir: Path, obs_dim: int = 45, act_dim: int = 2) -> None:
    rng = np.random.default_rng(42)
    x = rng.uniform(-1.0, 1.0, size=(100, obs_dim)).astype(np.float32)
    model.eval()
    with torch.no_grad():
        y = model(torch.tensor(x)).cpu().numpy()
    with (output_dir / "rl_actor_test_data.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([*(f"obs_{i}" for i in range(obs_dim)), "act_steer", "act_accel"])
        writer.writerows(np.hstack([x, y]))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--teacher-csv", nargs="+", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--epochs", type=int, default=250)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--lr", type=float, default=2e-3)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--steer-scale", type=float, default=25.0)
    args = parser.parse_args()

    raw_obs, actions = load_teacher_csv(args.teacher_csv)
    bounds = make_bounds(raw_obs)
    obs = normalize_obs(raw_obs, bounds)
    print(f"loaded samples={obs.shape[0]} obs_dim={obs.shape[1]}")
    model = train_model(obs, actions, args.epochs, args.batch_size, args.lr, args.seed)

    model.eval()
    with torch.no_grad():
        pred = model(torch.tensor(obs, dtype=torch.float32)).numpy()
    mae = np.mean(np.abs(pred - actions), axis=0)
    rmse = np.sqrt(np.mean((pred - actions) ** 2, axis=0))
    print(f"train_mae steer={mae[0]:.5f} accel={mae[1]:.5f}")
    print(f"train_rmse steer={rmse[0]:.5f} accel={rmse[1]:.5f}")

    export_weights(model, args.output_dir)
    export_normalization(bounds, args.output_dir, args.steer_scale)
    export_test_data(model, args.output_dir)
    print(f"exported {args.output_dir}")


if __name__ == "__main__":
    main()
