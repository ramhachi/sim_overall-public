#!/usr/bin/env python3
"""
Manual verification of TMeasy model force calculations with known parameters.
This script performs hand-calculated verification of the model implementation.
"""

import numpy as np
import sys
sys.path.insert(0, '.')

from tire_fitting.tmeasy_model import TMeasyModel


def verify_longitudinal_force():
    """Verify longitudinal force calculation with known parameters."""
    print("=" * 70)
    print("LONGITUDINAL FORCE VERIFICATION")
    print("=" * 70)
    
    model = TMeasyModel(fz_nominal=1000.0)
    
    # Test Case 1: Simple case with known values
    print("\nTest Case 1: Basic longitudinal force")
    print("-" * 70)
    sr = 0.1  # 10% slip ratio
    fz = 1000.0  # Nominal load
    mu_x_peak = 1.5
    k_mu_x = 0.2
    Kx = 20000.0
    
    # Manual calculation
    mu_x = mu_x_peak * (1 - k_mu_x * (fz - 1000.0) / 1000.0)
    print(f"  mu_x = {mu_x_peak} * (1 - {k_mu_x} * ({fz} - 1000) / 1000) = {mu_x:.6f}")
    
    xi_bar = Kx * sr / (mu_x * fz)
    print(f"  xi_bar = {Kx} * {sr} / ({mu_x:.6f} * {fz}) = {xi_bar:.6f}")
    
    s = abs(xi_bar)
    if s < 1:
        mu_eff = s * (2 - s)
        print(f"  s = {s:.6f} < 1, so mu_eff = {s:.6f} * (2 - {s:.6f}) = {mu_eff:.6f}")
    else:
        mu_eff = 1.0
        print(f"  s = {s:.6f} >= 1, so mu_eff = 1.0")
    
    fx_expected = -mu_eff * mu_x * fz * np.sign(xi_bar)
    print(f"  Fx = -{mu_eff:.6f} * {mu_x:.6f} * {fz} * sign({xi_bar:.6f})")
    print(f"  Fx_expected = {fx_expected:.2f} N")
    
    # Model calculation
    fx_model = model.compute_longitudinal_force(
        np.array([sr]), np.array([fz]), mu_x_peak, k_mu_x, Kx
    )[0]
    print(f"  Fx_model    = {fx_model:.2f} N")
    
    error = abs(fx_expected - fx_model)
    print(f"  Error       = {error:.6f} N")
    
    if error < 1e-6:
        print("  ✓ PASS: Model matches hand calculation")
    else:
        print("  ✗ FAIL: Model does not match hand calculation")
        return False
    
    # Test Case 2: Saturated condition (s >= 1)
    print("\nTest Case 2: Saturated longitudinal force (s >= 1)")
    print("-" * 70)
    sr = 0.3  # 30% slip ratio - should saturate
    fz = 800.0
    
    mu_x = mu_x_peak * (1 - k_mu_x * (fz - 1000.0) / 1000.0)
    print(f"  mu_x = {mu_x_peak} * (1 - {k_mu_x} * ({fz} - 1000) / 1000) = {mu_x:.6f}")
    
    xi_bar = Kx * sr / (mu_x * fz)
    print(f"  xi_bar = {Kx} * {sr} / ({mu_x:.6f} * {fz}) = {xi_bar:.6f}")
    
    s = abs(xi_bar)
    if s < 1:
        mu_eff = s * (2 - s)
        print(f"  s = {s:.6f} < 1, so mu_eff = {s:.6f} * (2 - {s:.6f}) = {mu_eff:.6f}")
    else:
        mu_eff = 1.0
        print(f"  s = {s:.6f} >= 1, so mu_eff = 1.0")
    
    fx_expected = -mu_eff * mu_x * fz * np.sign(xi_bar)
    print(f"  Fx_expected = {fx_expected:.2f} N")
    
    fx_model = model.compute_longitudinal_force(
        np.array([sr]), np.array([fz]), mu_x_peak, k_mu_x, Kx
    )[0]
    print(f"  Fx_model    = {fx_model:.2f} N")
    
    error = abs(fx_expected - fx_model)
    print(f"  Error       = {error:.6f} N")
    
    if error < 1e-6:
        print("  ✓ PASS: Model matches hand calculation")
    else:
        print("  ✗ FAIL: Model does not match hand calculation")
        return False
    
    return True


def verify_lateral_force():
    """Verify lateral force calculation with known parameters."""
    print("\n" + "=" * 70)
    print("LATERAL FORCE VERIFICATION")
    print("=" * 70)
    
    model = TMeasyModel(fz_nominal=1000.0)
    
    # Test Case 1: Basic lateral force without camber
    print("\nTest Case 1: Basic lateral force (no camber)")
    print("-" * 70)
    sa = np.deg2rad(5.0)  # 5 degrees slip angle
    fz = 1000.0
    camber = 0.0
    mu_y_peak = 1.4
    k_mu_y = 0.15
    Ky = 18000.0
    C_gamma = 0.05
    
    # Manual calculation
    mu_y = mu_y_peak * (1 - k_mu_y * (fz - 1000.0) / 1000.0)
    print(f"  mu_y = {mu_y_peak} * (1 - {k_mu_y} * ({fz} - 1000) / 1000) = {mu_y:.6f}")
    
    eta_bar = Ky * np.tan(sa) / (mu_y * fz)
    print(f"  eta_bar = {Ky} * tan({np.rad2deg(sa):.2f}°) / ({mu_y:.6f} * {fz})")
    print(f"  eta_bar = {Ky} * {np.tan(sa):.6f} / {mu_y * fz:.2f} = {eta_bar:.6f}")
    
    s = abs(eta_bar)
    if s < 1:
        mu_eff = s * (2 - s)
        print(f"  s = {s:.6f} < 1, so mu_eff = {s:.6f} * (2 - {s:.6f}) = {mu_eff:.6f}")
    else:
        mu_eff = 1.0
        print(f"  s = {s:.6f} >= 1, so mu_eff = 1.0")
    
    fy_base = -mu_eff * mu_y * fz * np.sign(eta_bar)
    fy_camber = C_gamma * camber * fz
    fy_expected = fy_base + fy_camber
    print(f"  Fy_base   = -{mu_eff:.6f} * {mu_y:.6f} * {fz} * sign({eta_bar:.6f}) = {fy_base:.2f} N")
    print(f"  Fy_camber = {C_gamma} * {camber} * {fz} = {fy_camber:.2f} N")
    print(f"  Fy_expected = {fy_expected:.2f} N")
    
    # Model calculation
    fy_model = model.compute_lateral_force(
        np.array([sa]), np.array([fz]), np.array([camber]),
        mu_y_peak, k_mu_y, Ky, C_gamma
    )[0]
    print(f"  Fy_model    = {fy_model:.2f} N")
    
    error = abs(fy_expected - fy_model)
    print(f"  Error       = {error:.6f} N")
    
    if error < 1e-6:
        print("  ✓ PASS: Model matches hand calculation")
    else:
        print("  ✗ FAIL: Model does not match hand calculation")
        return False
    
    # Test Case 2: Lateral force with camber
    print("\nTest Case 2: Lateral force with camber thrust")
    print("-" * 70)
    sa = np.deg2rad(3.0)
    fz = 1200.0
    camber = np.deg2rad(-2.0)  # -2 degrees camber
    
    mu_y = mu_y_peak * (1 - k_mu_y * (fz - 1000.0) / 1000.0)
    print(f"  mu_y = {mu_y_peak} * (1 - {k_mu_y} * ({fz} - 1000) / 1000) = {mu_y:.6f}")
    
    eta_bar = Ky * np.tan(sa) / (mu_y * fz)
    print(f"  eta_bar = {Ky} * tan({np.rad2deg(sa):.2f}°) / ({mu_y:.6f} * {fz}) = {eta_bar:.6f}")
    
    s = abs(eta_bar)
    if s < 1:
        mu_eff = s * (2 - s)
        print(f"  s = {s:.6f} < 1, so mu_eff = {mu_eff:.6f}")
    else:
        mu_eff = 1.0
        print(f"  s = {s:.6f} >= 1, so mu_eff = 1.0")
    
    fy_base = -mu_eff * mu_y * fz * np.sign(eta_bar)
    fy_camber = C_gamma * camber * fz
    fy_expected = fy_base + fy_camber
    print(f"  Fy_base   = {fy_base:.2f} N")
    print(f"  Fy_camber = {C_gamma} * {np.rad2deg(camber):.2f}° * {fz} = {fy_camber:.2f} N")
    print(f"  Fy_expected = {fy_expected:.2f} N")
    
    fy_model = model.compute_lateral_force(
        np.array([sa]), np.array([fz]), np.array([camber]),
        mu_y_peak, k_mu_y, Ky, C_gamma
    )[0]
    print(f"  Fy_model    = {fy_model:.2f} N")
    
    error = abs(fy_expected - fy_model)
    print(f"  Error       = {error:.6f} N")
    
    if error < 1e-6:
        print("  ✓ PASS: Model matches hand calculation")
    else:
        print("  ✗ FAIL: Model does not match hand calculation")
        return False
    
    return True


def verify_load_dependency():
    """Verify load-dependent friction coefficient."""
    print("\n" + "=" * 70)
    print("LOAD-DEPENDENT FRICTION VERIFICATION")
    print("=" * 70)
    
    model = TMeasyModel(fz_nominal=1000.0)
    
    print("\nTest: Friction coefficient at different loads")
    print("-" * 70)
    
    mu_peak = 1.5
    k_mu = 0.2
    fz_nom = 1000.0
    
    test_loads = [800.0, 1000.0, 1200.0]
    
    all_pass = True
    for fz in test_loads:
        mu_expected = mu_peak * (1 - k_mu * (fz - fz_nom) / fz_nom)
        mu_model = model._compute_load_dependent_friction(fz, mu_peak, k_mu)
        
        print(f"  Fz = {fz:.0f} N:")
        print(f"    mu_expected = {mu_peak} * (1 - {k_mu} * ({fz} - {fz_nom}) / {fz_nom})")
        print(f"    mu_expected = {mu_expected:.6f}")
        print(f"    mu_model    = {mu_model:.6f}")
        
        error = abs(mu_expected - mu_model)
        print(f"    Error       = {error:.9f}")
        
        if error < 1e-9:
            print(f"    ✓ PASS")
        else:
            print(f"    ✗ FAIL")
            all_pass = False
    
    return all_pass


def main():
    """Run all manual verifications."""
    print("\n")
    print("╔" + "=" * 68 + "╗")
    print("║" + " " * 15 + "TMeasy Model Manual Verification" + " " * 21 + "║")
    print("╚" + "=" * 68 + "╝")
    
    results = []
    
    # Run verifications
    results.append(("Longitudinal Force", verify_longitudinal_force()))
    results.append(("Lateral Force", verify_lateral_force()))
    results.append(("Load Dependency", verify_load_dependency()))
    
    # Summary
    print("\n" + "=" * 70)
    print("VERIFICATION SUMMARY")
    print("=" * 70)
    
    all_pass = True
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {name:.<50} {status}")
        if not passed:
            all_pass = False
    
    print("=" * 70)
    
    if all_pass:
        print("\n✓ All manual verifications PASSED!")
        print("  The TMeasy model implementation is correct.\n")
        return 0
    else:
        print("\n✗ Some verifications FAILED!")
        print("  Please review the implementation.\n")
        return 1


if __name__ == "__main__":
    sys.exit(main())
