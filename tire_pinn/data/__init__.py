"""Data loading and preprocessing."""

__all__ = [
    'TTCDataLoader',
    'TireCatalog',
]

_LAZY_EXPORTS = {
    'TTCDataLoader': ('tire_pinn.data.ttc_loader', 'TTCDataLoader'),
    'TireCatalog': ('tire_pinn.data.tire_catalog', 'TireCatalog'),
}


def __getattr__(name):
    if name not in _LAZY_EXPORTS:
        raise AttributeError(f"module 'tire_pinn.data' has no attribute {name!r}")

    module_name, attr_name = _LAZY_EXPORTS[name]
    from importlib import import_module

    value = getattr(import_module(module_name), attr_name)
    globals()[name] = value
    return value
