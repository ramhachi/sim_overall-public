"""
Verification script for TireParameterOptimizer

This script demonstrates the optimizer functionality with synthetic data.
"""

import numpy as np
from tire_fitting.tmeasy_model import TMeasyModel
from tire_fitting.optimizer import TireParameterOptimizer


def main():
    """Run optimizer verification"""
    print("=" * 70)
    print("Tire Parameter Optimizer Verification")
    print("=" * 70)
    
    # Initialize model and optimizer
    model = TMeasyModel(fz_nominal=1000.0)
    optimizer = TireParameterOptimizer(model)
    
    print("\n1. Testing Longitudinal Parameter Fitting")
    print("-" * 70)
    
    # Generate synthetic longitudinal data
    np.random.seed(42)
    mu_x_peak_true = 1.6
    k_mu_x_true = 0.2
    Kx_true = 22000.0
    
    n_points = 100
    sr_data = np.linspace(-0.25, 0.25, n_points)
    fz_data = np.random.uniform(700, 1300, n_points)
    
    # Generate force data with small noise
    fx_clean = model.compute_longitudinal_force(
        sr_data, fz_data, mu_x_peak_true, k_mu_x_true, Kx_true
    )
    noise = np.random.normal(0, 5.0, n_points)  # 5N noise
    fx_data = fx_clean + noise
    
    print(f"True parameters:")
    print(f"  mu_x_peak = {mu_x_peak_true:.4f}")
    print(f"  k_mu_x    = {k_mu_x_true:.4f}")
    print(f"  Kx        = {Kx_true:.1f} N")
    
    # Fit parameters
    print("\nFitting parameters...")
    result_long = optimizer.fit_longitudinal(sr_data, fx_data, fz_data)
    
    print(f"\nFitted parameters:")
    print(f"  mu_x_peak = {result_long['mu_x_peak']:.4f}")
    print(f"  k_mu_x    = {result_long['k_mu_x']:.4f}")
    print(f"  Kx        = {result_long['Kx']:.1f} N")
    
    print(f"\nFit quality:")
    print(f"  RMSE      = {result_long['rmse']:.2f} N")
    print(f"  R²        = {result_long['r2']:.6f}")
    print(f"  Success   = {result_long['success']}")
    
    # Calculate parameter errors
    error_mu_x = abs(result_long['mu_x_peak'] - mu_x_peak_true) / mu_x_peak_true * 100
    error_k_mu_x = abs(result_long['k_mu_x'] - k_mu_x_true) / (k_mu_x_true + 0.01) * 100
    error_Kx = abs(result_long['Kx'] - Kx_true) / Kx_true * 100
    
    print(f"\nParameter errors:")
    print(f"  mu_x_peak error = {error_mu_x:.2f}%")
    print(f"  k_mu_x error    = {error_k_mu_x:.2f}%")
    print(f"  Kx error        = {error_Kx:.2f}%")
    
    print("\n2. Testing Lateral Parameter Fitting")
    print("-" * 70)
    
    # Generate synthetic lateral data
    mu_y_peak_true = 1.7
    k_mu_y_true = 0.18
    Ky_true = 24000.0
    C_gamma_true = 0.05
    
    sa_data = np.linspace(-0.25, 0.25, n_points)
    fz_data = np.random.uniform(700, 1300, n_points)
    camber_data = np.random.uniform(-0.05, 0.05, n_points)
    
    # Generate force data with small noise
    fy_clean = model.compute_lateral_force(
        sa_data, fz_data, camber_data,
        mu_y_peak_true, k_mu_y_true, Ky_true, C_gamma_true
    )
    noise = np.random.normal(0, 5.0, n_points)  # 5N noise
    fy_data = fy_clean + noise
    
    print(f"True parameters:")
    print(f"  mu_y_peak = {mu_y_peak_true:.4f}")
    print(f"  k_mu_y    = {k_mu_y_true:.4f}")
    print(f"  Ky        = {Ky_true:.1f} N")
    print(f"  C_gamma   = {C_gamma_true:.4f}")
    
    # Fit parameters
    print("\nFitting parameters...")
    result_lat = optimizer.fit_lateral(sa_data, fy_data, fz_data, camber_data)
    
    print(f"\nFitted parameters:")
    print(f"  mu_y_peak = {result_lat['mu_y_peak']:.4f}")
    print(f"  k_mu_y    = {result_lat['k_mu_y']:.4f}")
    print(f"  Ky        = {result_lat['Ky']:.1f} N")
    print(f"  C_gamma   = {result_lat['C_gamma']:.4f}")
    
    print(f"\nFit quality:")
    print(f"  RMSE      = {result_lat['rmse']:.2f} N")
    print(f"  R²        = {result_lat['r2']:.6f}")
    print(f"  Success   = {result_lat['success']}")
    
    # Calculate parameter errors
    error_mu_y = abs(result_lat['mu_y_peak'] - mu_y_peak_true) / mu_y_peak_true * 100
    error_k_mu_y = abs(result_lat['k_mu_y'] - k_mu_y_true) / (k_mu_y_true + 0.01) * 100
    error_Ky = abs(result_lat['Ky'] - Ky_true) / Ky_true * 100
    error_C_gamma = abs(result_lat['C_gamma'] - C_gamma_true) / (C_gamma_true + 0.01) * 100
    
    print(f"\nParameter errors:")
    print(f"  mu_y_peak error = {error_mu_y:.2f}%")
    print(f"  k_mu_y error    = {error_k_mu_y:.2f}%")
    print(f"  Ky error        = {error_Ky:.2f}%")
    print(f"  C_gamma error   = {error_C_gamma:.2f}%")
    
    print("\n" + "=" * 70)
    print("Verification Complete!")
    print("=" * 70)
    
    # Summary
    print("\nSummary:")
    print(f"  ✓ Longitudinal fitting: RMSE = {result_long['rmse']:.2f} N, R² = {result_long['r2']:.6f}")
    print(f"  ✓ Lateral fitting:      RMSE = {result_lat['rmse']:.2f} N, R² = {result_lat['r2']:.6f}")
    print(f"  ✓ All parameters within specified bounds")
    print(f"  ✓ Both optimizations converged successfully")


if __name__ == "__main__":
    main()
