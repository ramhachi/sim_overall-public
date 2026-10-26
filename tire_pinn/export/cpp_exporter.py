"""
Export trained PINN models to C++-compatible runtime assets.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import torch

from ..data.ttc_loader import INPUT_UNITS, OUTPUT_UNITS
from ..models.pinn_model import NormalizedTirePINN


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


class CPPExporter:
    """
    Export a PINN model and its metadata to a C++ runtime directory.

    Files:
      - pinn_architecture.json
      - pinn_weights.json
      - pinn_normalization.json
      - pinn_runtime.json        (single-file runtime representation for C++)
      - manifest.json            (deployment metadata)
      - test_vectors.json
    """
    DEFAULT_TEST_VECTOR_COUNT = 512
    TEST_VECTOR_SEED = 42

    def __init__(self, model: torch.nn.Module, output_dir: Path):
        self.model = model
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def _base_model(self) -> NormalizedTirePINN:
        if isinstance(self.model, NormalizedTirePINN):
            return self.model
        if hasattr(self.model, "base_model") and isinstance(
            self.model.base_model, NormalizedTirePINN
        ):
            return self.model.base_model
        raise TypeError("CPPExporter requires NormalizedTirePINN or a compatible wrapped model")

    def _postprocess_config(self) -> Optional[Dict[str, Any]]:
        if hasattr(self.model, "get_runtime_postprocess_config"):
            return self.model.get_runtime_postprocess_config()
        return None

    def export(
        self,
        tire_name: str,
        dataset_info: Dict[str, Any],
        validation_summary: Dict[str, Any],
        training_summary: Dict[str, Any],
        input_bounds: Optional[Dict[str, Dict[str, float]]] = None,
    ) -> None:
        print("Exporting PINN model to C++ runtime assets...")

        arch_path = self.output_dir / "pinn_architecture.json"
        weights_path = self.output_dir / "pinn_weights.json"
        norm_path = self.output_dir / "pinn_normalization.json"
        runtime_path = self.output_dir / "pinn_runtime.json"
        manifest_path = self.output_dir / "manifest.json"

        self._export_architecture(arch_path)
        self._export_weights(weights_path)
        self._export_normalization(norm_path)
        self._export_runtime_model(runtime_path, input_bounds=input_bounds)
        # NaN/inf metrics (e.g. a gate with zero covered samples) are not valid
        # JSON; nlohmann::json in the C++ runtime refuses to load them.
        manifest = _json_safe(
            self._build_manifest(
                tire_name=tire_name,
                dataset_info=dataset_info,
                validation_summary=validation_summary,
                training_summary=training_summary,
            )
        )
        version_manifest_rel = manifest["runtime_files"]["version_manifest"]
        version_manifest_path = self.output_dir / version_manifest_rel
        version_manifest_path.parent.mkdir(parents=True, exist_ok=True)
        with manifest_path.open("w", encoding="utf-8") as handle:
            json.dump(manifest, handle, indent=2)
        with version_manifest_path.open("w", encoding="utf-8") as handle:
            json.dump(manifest, handle, indent=2)
        self._update_registry(manifest)
        self.export_test_vectors(
            output_path=self.output_dir / "test_vectors.json",
            input_bounds=input_bounds,
        )

        print(f"  ✓ Architecture: {arch_path}")
        print(f"  ✓ Weights: {weights_path}")
        print(f"  ✓ Normalization: {norm_path}")
        print(f"  ✓ Runtime model: {runtime_path}")
        print(f"  ✓ Manifest: {manifest_path}")

    def _export_architecture(self, path: Path) -> None:
        base_model = self._base_model()
        arch_info = base_model.pinn.get_architecture_info()
        layers = []
        for name, module in base_model.pinn.named_modules():
            if isinstance(module, torch.nn.Linear):
                layers.append(
                    {
                        "name": name,
                        "type": "Linear",
                        "in_features": module.in_features,
                        "out_features": module.out_features,
                        "has_bias": module.bias is not None,
                    }
                )
            elif isinstance(module, torch.nn.LayerNorm):
                layers.append(
                    {
                        "name": name,
                        "type": "LayerNorm",
                        "normalized_shape": list(module.normalized_shape),
                        "eps": module.eps,
                    }
                )

        with path.open("w", encoding="utf-8") as handle:
            json.dump({**arch_info, "layers": layers}, handle, indent=2)

    def _export_weights(self, path: Path) -> None:
        base_model = self._base_model()
        weights_data: Dict[str, Dict[str, Any]] = {}
        for name, param in base_model.pinn.named_parameters():
            values = param.detach().cpu().numpy()
            weights_data[name] = {
                "shape": list(values.shape),
                "size": int(values.size),
                "values": [f"{float(v):.15e}" for v in values.flatten()],
            }

        with path.open("w", encoding="utf-8") as handle:
            json.dump(weights_data, handle, indent=2)

    def _export_normalization(self, path: Path) -> None:
        base_model = self._base_model()
        norm_data = {
            "input": {
                "mean": base_model.input_mean.cpu().numpy().tolist(),
                "std": base_model.input_std.cpu().numpy().tolist(),
                "names": ["SR", "SA", "Fz", "IA", "TSTC"],
                "units": [INPUT_UNITS[name] for name in ["SR", "SA", "Fz", "IA", "TSTC"]],
            },
            "output": {
                "mean": base_model.output_mean.cpu().numpy().tolist(),
                "std": base_model.output_std.cpu().numpy().tolist(),
                "names": ["Fx", "Fy", "Mx", "Mz"],
                "units": [OUTPUT_UNITS[name] for name in ["Fx", "Fy", "Mx", "Mz"]],
            },
        }
        with path.open("w", encoding="utf-8") as handle:
            json.dump(norm_data, handle, indent=2)

    @staticmethod
    def _runtime_layers_from_modules(modules: list[torch.nn.Module]) -> list[dict[str, Any]]:
        layers = []
        for module in modules:
            if isinstance(module, torch.nn.Linear):
                layers.append(
                    {
                        "type": "linear",
                        "weight": module.weight.detach().cpu().numpy().tolist(),
                        "bias": module.bias.detach().cpu().numpy().tolist()
                        if module.bias is not None
                        else None,
                    }
                )
            elif isinstance(module, torch.nn.LayerNorm):
                layers.append(
                    {
                        "type": "layer_norm",
                        "weight": module.weight.detach().cpu().numpy().tolist(),
                        "bias": module.bias.detach().cpu().numpy().tolist(),
                        "eps": module.eps,
                    }
                )
            else:
                class_name = module.__class__.__name__.lower()
                if "swish" in class_name or "silu" in class_name:
                    layers.append({"type": "swish"})
                else:
                    raise ValueError(f"Unsupported runtime layer type: {module.__class__.__name__}")
        return layers

    def _export_runtime_model(
        self,
        path: Path,
        input_bounds: Optional[Dict[str, Dict[str, float]]] = None,
    ) -> None:
        base_model = self._base_model()
        runtime_structure = base_model.pinn.get_runtime_structure()

        runtime_model = {
            "format": "pinn_runtime_v2"
            if runtime_structure["type"] == "multi_head"
            else "pinn_runtime_v1",
            "input": {
                "names": ["SR", "SA", "Fz", "IA", "TSTC"],
                "units": [INPUT_UNITS[name] for name in ["SR", "SA", "Fz", "IA", "TSTC"]],
                "mean": base_model.input_mean.cpu().numpy().tolist(),
                "std": base_model.input_std.cpu().numpy().tolist(),
            },
            "output": {
                "names": ["Fx", "Fy", "Mx", "Mz"],
                "units": [OUTPUT_UNITS[name] for name in ["Fx", "Fy", "Mx", "Mz"]],
                "mean": base_model.output_mean.cpu().numpy().tolist(),
                "std": base_model.output_std.cpu().numpy().tolist(),
            },
            "input_scope": self._input_scope(),
            "architecture": {
                "type": runtime_structure["type"],
            },
        }
        if runtime_structure["type"] == "multi_head":
            runtime_model["trunk_layers"] = self._runtime_layers_from_modules(
                runtime_structure["trunk_layers"]
            )
            runtime_model["heads"] = [
                {
                    "name": head["name"],
                    "output_indices": head["output_indices"],
                    "layers": self._runtime_layers_from_modules(head["layers"]),
                }
                for head in runtime_structure["heads"]
            ]
        else:
            runtime_model["layers"] = self._runtime_layers_from_modules(
                runtime_structure["layers"]
            )
        postprocess = self._postprocess_config()
        if postprocess:
            runtime_model["postprocess"] = postprocess
        if input_bounds:
            names = ["SR", "SA", "Fz", "IA", "TSTC"]
            runtime_model["input_bounds"] = {
                "names": names,
                "min": [input_bounds[name]["min"] for name in names],
                "max": [input_bounds[name]["max"] for name in names],
            }

        with path.open("w", encoding="utf-8") as handle:
            json.dump(runtime_model, handle, indent=2)

    def _build_manifest(
        self,
        tire_name: str,
        dataset_info: Dict[str, Any],
        validation_summary: Dict[str, Any],
        training_summary: Dict[str, Any],
    ) -> Dict[str, Any]:
        exported_at = datetime.now(timezone.utc)
        export_id = exported_at.strftime("%Y%m%dT%H%M%S%fZ")
        return {
            "format_version": 1,
            "model_family": "pinn",
            "export_id": export_id,
            "tire_name": tire_name,
            "exported_at": exported_at.isoformat(),
            "deployable": bool(validation_summary.get("deployable", False)),
            "dataset": dataset_info,
            "validation": validation_summary,
            "training": training_summary,
            "input_units": dict(INPUT_UNITS),
            "output_units": dict(OUTPUT_UNITS),
            "input_scope": self._input_scope(),
            "runtime_files": {
                "runtime_model": "pinn_runtime.json",
                "architecture": "pinn_architecture.json",
                "weights": "pinn_weights.json",
                "normalization": "pinn_normalization.json",
                "version_manifest": f"manifests/manifest_{export_id}.json",
                "test_vectors": "test_vectors.json",
            },
        }

    def _update_registry(self, manifest: Dict[str, Any]) -> None:
        registry_path = self.output_dir.parent / "registry.json"
        if registry_path.exists():
            with registry_path.open("r", encoding="utf-8") as handle:
                registry = json.load(handle)
        else:
            registry = {"format_version": 2, "models": {}, "history": {}}

        dataset_id = manifest["tire_name"]
        registry["format_version"] = max(int(registry.get("format_version", 1)), 2)
        registry.setdefault("models", {})
        registry.setdefault("history", {})

        entry = {
            "tire_name": dataset_id,
            "export_id": manifest["export_id"],
            "deployable": manifest["deployable"],
            "manifest": f"{dataset_id}/manifest.json",
            "version_manifest": f"{dataset_id}/{manifest['runtime_files']['version_manifest']}",
            "runtime_model": f"{dataset_id}/pinn_runtime.json",
            "validation": {
                "r2_fx": manifest["validation"]["r2"]["r2_fx"],
                "r2_fy": manifest["validation"]["r2"]["r2_fy"],
                "r2_mean": manifest["validation"]["r2_mean"],
            },
            "updated_at": manifest["exported_at"],
        }
        registry["models"][dataset_id] = entry
        registry["history"].setdefault(dataset_id, []).append(entry)

        with registry_path.open("w", encoding="utf-8") as handle:
            json.dump(registry, handle, indent=2)

    def export_test_vectors(
        self,
        n_samples: int = DEFAULT_TEST_VECTOR_COUNT,
        output_path: Optional[Path] = None,
        input_bounds: Optional[Dict[str, Dict[str, float]]] = None,
    ) -> None:
        if output_path is None:
            output_path = self.output_dir / "test_vectors.json"

        model_device = next(self._base_model().parameters()).device
        test_inputs = self._generate_test_inputs(n_samples, input_bounds=input_bounds).to(
            model_device
        )
        self.model.eval()
        with torch.no_grad():
            test_outputs = self.model(test_inputs).cpu().numpy()
        is_multi_head = (
            getattr(self._base_model().pinn, "architecture", "sequential") == "multi_head"
        )

        test_data = {
            "n_samples": int(len(test_inputs)),
            "inputs": test_inputs.cpu().numpy().tolist(),
            "outputs": test_outputs.tolist(),
            "tolerance": {
                "absolute": 0.25 if is_multi_head else 2e-5,
                "relative": 5e-3 if is_multi_head else 1e-3,
            },
            "input_units": dict(INPUT_UNITS),
            "output_units": dict(OUTPUT_UNITS),
            "generation": {
                "seed": self.TEST_VECTOR_SEED,
                "strategy": "fixed_zero_slip_sign_symmetry_bounds_plus_random",
                "requested_samples": int(n_samples),
            },
        }
        with output_path.open("w", encoding="utf-8") as handle:
            json.dump(test_data, handle, indent=2)

    def _generate_test_inputs(
        self,
        n_samples: int,
        input_bounds: Optional[Dict[str, Dict[str, float]]] = None,
    ) -> torch.Tensor:
        cases = [
            [0.0, 0.0, 1000.0, 0.0, 60.0],
            [0.10, 0.0, 1200.0, 0.0, 55.0],
            [-0.10, 0.0, 1200.0, 0.0, 55.0],
            [0.0, 3.0, 1200.0, 0.0, 60.0],
            [0.0, -3.0, 1200.0, 0.0, 60.0],
            [0.10, 4.0, 1500.0, 1.0, 65.0],
            [-0.08, -4.0, 1800.0, -1.5, 70.0],
            [0.05, 1.5, 800.0, 0.5, 45.0],
        ]

        names = ["SR", "SA", "Fz", "IA", "TSTC"]
        ranges = {
            "SR": (-0.2, 0.2),
            "SA": (-10.0, 10.0),
            "Fz": (400.0, 2000.0),
            "IA": (-4.0, 4.0),
            "TSTC": (25.0, 85.0),
        }
        if input_bounds:
            ranges = {
                name: (float(input_bounds[name]["min"]), float(input_bounds[name]["max"]))
                for name in names
            }

        midpoint = {name: 0.5 * (bounds[0] + bounds[1]) for name, bounds in ranges.items()}
        midpoint["SR"] = 0.0 if ranges["SR"][0] <= 0.0 <= ranges["SR"][1] else midpoint["SR"]
        midpoint["SA"] = 0.0 if ranges["SA"][0] <= 0.0 <= ranges["SA"][1] else midpoint["SA"]
        midpoint["IA"] = 0.0 if ranges["IA"][0] <= 0.0 <= ranges["IA"][1] else midpoint["IA"]

        boundary_cases = [
            [0.0, 0.0, ranges["Fz"][0], midpoint["IA"], midpoint["TSTC"]],
            [0.0, 0.0, ranges["Fz"][1], midpoint["IA"], midpoint["TSTC"]],
            [ranges["SR"][0], midpoint["SA"], midpoint["Fz"], midpoint["IA"], midpoint["TSTC"]],
            [ranges["SR"][1], midpoint["SA"], midpoint["Fz"], midpoint["IA"], midpoint["TSTC"]],
            [midpoint["SR"], ranges["SA"][0], midpoint["Fz"], midpoint["IA"], midpoint["TSTC"]],
            [midpoint["SR"], ranges["SA"][1], midpoint["Fz"], midpoint["IA"], midpoint["TSTC"]],
            [midpoint["SR"], midpoint["SA"], midpoint["Fz"], ranges["IA"][0], midpoint["TSTC"]],
            [midpoint["SR"], midpoint["SA"], midpoint["Fz"], ranges["IA"][1], midpoint["TSTC"]],
            [ranges["SR"][0], ranges["SA"][0], ranges["Fz"][1], ranges["IA"][0], ranges["TSTC"][0]],
            [ranges["SR"][1], ranges["SA"][1], ranges["Fz"][1], ranges["IA"][1], ranges["TSTC"][1]],
        ]
        cases.extend(boundary_cases)

        rng = np.random.default_rng(self.TEST_VECTOR_SEED)
        while len(cases) < n_samples:
            cases.append(
                [
                    float(rng.uniform(*ranges["SR"])),
                    float(rng.uniform(*ranges["SA"])),
                    float(rng.uniform(*ranges["Fz"])),
                    float(rng.uniform(*ranges["IA"])),
                    float(rng.uniform(*ranges["TSTC"])),
                ]
            )

        if input_bounds:
            bounded_cases = []
            for case in cases[:n_samples]:
                bounded_case = []
                for value, name in zip(case, names):
                    lower = input_bounds[name]["min"]
                    upper = input_bounds[name]["max"]
                    bounded_case.append(float(np.clip(value, lower, upper)))
                bounded_cases.append(bounded_case)
            cases = bounded_cases

        return torch.tensor(cases[:n_samples], dtype=torch.float32)

    def _input_scope(self) -> Dict[str, Any]:
        return {
            "included_inputs": ["SR", "SA", "Fz", "IA", "TSTC"],
            "included_units": dict(INPUT_UNITS),
            "not_modeled_inputs": ["inflation_pressure", "road_surface", "wear_state"],
            "note": (
                "PINN runtime is valid within the exported TTC input bounds and does not "
                "condition on pressure, road surface, or wear state."
            ),
        }
