"""
PINN (Physics-Informed Neural Network) Tire Model.

A tire force prediction model that combines:
- Data-driven learning from TTC tire test data
- Physics-based constraints (zero slip, symmetry, friction circle, monotonicity,
  combined slip, stiffness load trends, Mz trail, camber thrust, smooth sections)
- Curriculum learning for stable training

Key features:
- Swish activation for smooth gradients
- LayerNorm for batch-independent normalization
- Collocation point sampling for physics constraints
- C++ export for high-performance simulation
"""

__version__ = "0.1.0"

__all__ = [
    "TirePINN",
    "NormalizedTirePINN",
    "TTCDataLoader",
    "TireCatalog",
    "PINNTrainer",
    "CollocationSampler",
    "CurriculumScheduler",
    "DataLoss",
    "PhysicsLossManager",
    "PhysicsValidator",
    "CPPExporter",
]

_LAZY_EXPORTS = {
    "TirePINN": ("tire_pinn.models", "TirePINN"),
    "NormalizedTirePINN": ("tire_pinn.models", "NormalizedTirePINN"),
    "TTCDataLoader": ("tire_pinn.data", "TTCDataLoader"),
    "TireCatalog": ("tire_pinn.data", "TireCatalog"),
    "PINNTrainer": ("tire_pinn.training", "PINNTrainer"),
    "CollocationSampler": ("tire_pinn.training", "CollocationSampler"),
    "CurriculumScheduler": ("tire_pinn.training", "CurriculumScheduler"),
    "DataLoss": ("tire_pinn.losses", "DataLoss"),
    "PhysicsLossManager": ("tire_pinn.losses", "PhysicsLossManager"),
    "PhysicsValidator": ("tire_pinn.validation", "PhysicsValidator"),
    "CPPExporter": ("tire_pinn.export", "CPPExporter"),
}


def __getattr__(name):
    if name not in _LAZY_EXPORTS:
        raise AttributeError(f"module 'tire_pinn' has no attribute {name!r}")

    module_name, attr_name = _LAZY_EXPORTS[name]
    from importlib import import_module

    value = getattr(import_module(module_name), attr_name)
    globals()[name] = value
    return value
