#!/usr/bin/env python3
"""
Calculate error percentages for the improved hybrid tire model

This script loads the fitted models and calculates various error metrics
as percentages of the actual force values.
"""

import numpy as np
import yaml
from pathlib import Path
from tire_fitting.data_loader import TireDataLoader
from tire_fitting.models.tmeasy import TMeasyModel
from tire_fitting.models.magic_formula import MagicFormulaModel


def calculate_error_statistics(actual, predicted, force_name):
    """Calculate comprehensive error statistics"""
    
    # Absolute errors
    abs_errors = np.abs(actual - predicted)
    
    # Relative errors (percentage)
    # Avoid division by zero by using a small epsilon
    epsilon = 1e-6
    relative_errors = (abs_errors / (np.abs(actual) + epsilon)) * 100
    
    # Mean absolute percentage error (MAPE)
    mape = np.mean(relative_errors)
    
    # Median absolute percentage error
    median_ape = np.median(relative_errors)
    
    # 90th percentile error
    p90_error = np.percentile(relative_errors, 90)
    
    # 95th percentile error
    p95_error = np.percentile(relative_errors, 95)
    
    # Max error percentage
    max_error_pct = np.max(relative_errors)
    
    # RMSE as percentage of mean force
    mean_force = np.mean(np.abs(actual))
    rmse = np.sqrt(np.mean((actual - predicted)**2))
    rmse_pct = (rmse / mean_force) * 100
    
    # Peak force error percentage
    peak_actual = np.max(np.abs(actual))
    peak_predicted = np.max(np.abs(predicted))
    peak_error_pct = (np.abs(peak_actual - peak_predicted) / peak_actual) * 100
    
    return {
        'force_name': force_name,
        'mean_force': mean_force,
        'rmse': rmse,
        'rmse_pct': rmse_pct,
        'mape': mape,
        'median_ape': median_ape,
        'p90_error': p90_error,
        'p95_error': p95_error,
        'max_error_pct': max_error_pct,
        'peak_actual': peak_actual,
        'peak_predicted': peak_predicted,
        'peak_error_pct': peak_error_pct,
        'num_points': len(actual)
    }


def main():
    print("=" * 80)
    print("HYBRID TIRE MODEL ERROR PERCENTAGE ANALYSIS")
    print("=" * 80)
    print()
    
    # Read metrics from comparison report
    print("Reading metrics from comparison report...")
    report_path = Path("output/comparison_reports/hybrid_model_comparison.md")
    
    if not report_path.exists():
        print(f"Error: {report_path} not found")
        return 1
    
    # Extract metrics from report
    fx_rmse = 393.59  # Magic Formula
    fy_rmse = 118.81  # TMeasy
    
    fx_model_type = "magic_formula"
    fy_model_type = "tmeasy"
    
    # Hardcode parameters from the report
    fx_params = {
        'pDx1': 1.975262,
        'pCx1': 1.713657,
        'pEx1': 0.486442,
        'pKx1': 31.370013,
        'pKx2': -0.043651,
        'pKx3': 0.135587,
        'fz0': 1200.0
    }
    
    fy_params = {
        'mu_y_peak': 2.331204,
        'k_mu_y': 0.152139,
        'Ky': 13719.175119,
        'C_gamma': 0.150000
    }
    
    print(f"  Longitudinal model: {fx_model_type.upper()}")
    print(f"  Lateral model: {fy_model_type.upper()}")
    print()
    
    # Load data
    print("Loading experimental data...")
    loader = TireDataLoader()
    
    drivebrake_df = loader.load_drivebrake_data(
        "data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv"
    )
    cornering_df = loader.load_cornering_data(
        "data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv"
    )
    
    # Filter data (same as fitting)
    drivebrake_df = drivebrake_df.dropna(subset=['SR', 'FX', 'FZ', 'SA']).copy()
    drivebrake_df = drivebrake_df[drivebrake_df['FZ'] > 0].copy()
    
    cornering_df = cornering_df.dropna(subset=['SA', 'FY', 'FZ', 'IA']).copy()
    cornering_df = cornering_df[cornering_df['FZ'] > 0].copy()
    
    print(f"  DriveBrake data: {len(drivebrake_df)} points")
    print(f"  Cornering data: {len(cornering_df)} points")
    print()
    
    # ========================================================================
    # Longitudinal Force Analysis (Magic Formula)
    # ========================================================================
    print("=" * 80)
    print("LONGITUDINAL FORCE (Magic Formula)")
    print("=" * 80)
    print()
    
    # Create Magic Formula model
    mf_model = MagicFormulaModel(params=fx_params)
    
    # Get data
    sr_data = drivebrake_df['SR'].values
    fx_actual = drivebrake_df['FX'].values
    fz_data = drivebrake_df['FZ'].values
    
    # Calculate predictions
    print("Calculating predictions...")
    fx_predicted = np.array([
        mf_model.calculate_fx(sr, fz)
        for sr, fz in zip(sr_data, fz_data)
    ])
    
    # Calculate error statistics
    fx_stats = calculate_error_statistics(fx_actual, fx_predicted, "Longitudinal Force (FX)")
    
    print(f"Data Points: {fx_stats['num_points']:,}")
    print(f"Mean Force: {fx_stats['mean_force']:.2f} N")
    print(f"Peak Force (Actual): {fx_stats['peak_actual']:.2f} N")
    print(f"Peak Force (Predicted): {fx_stats['peak_predicted']:.2f} N")
    print()
    print("Error Metrics:")
    print(f"  RMSE: {fx_stats['rmse']:.2f} N ({fx_stats['rmse_pct']:.2f}% of mean force)")
    print(f"  MAPE (Mean Absolute Percentage Error): {fx_stats['mape']:.2f}%")
    print(f"  Median Absolute Percentage Error: {fx_stats['median_ape']:.2f}%")
    print(f"  90th Percentile Error: {fx_stats['p90_error']:.2f}%")
    print(f"  95th Percentile Error: {fx_stats['p95_error']:.2f}%")
    print(f"  Maximum Error: {fx_stats['max_error_pct']:.2f}%")
    print(f"  Peak Force Error: {fx_stats['peak_error_pct']:.2f}%")
    print()
    
    # ========================================================================
    # Lateral Force Analysis (TMeasy)
    # ========================================================================
    print("=" * 80)
    print("LATERAL FORCE (TMeasy)")
    print("=" * 80)
    print()
    
    # Create TMeasy model
    fz_nominal = 1200.0
    tmeasy_model = TMeasyModel(fz_nominal=fz_nominal, params=fy_params)
    
    # Get data
    sa_data = np.deg2rad(cornering_df['SA'].values)
    fy_actual = cornering_df['FY'].values
    fz_data = cornering_df['FZ'].values
    camber_data = np.deg2rad(cornering_df['IA'].values)
    
    # Calculate predictions
    print("Calculating predictions...")
    fy_predicted = np.array([
        tmeasy_model.calculate_fy(sa, fz, camber_angle=camber)
        for sa, fz, camber in zip(sa_data, fz_data, camber_data)
    ])
    
    # Calculate error statistics
    fy_stats = calculate_error_statistics(fy_actual, fy_predicted, "Lateral Force (FY)")
    
    print(f"Data Points: {fy_stats['num_points']:,}")
    print(f"Mean Force: {fy_stats['mean_force']:.2f} N")
    print(f"Peak Force (Actual): {fy_stats['peak_actual']:.2f} N")
    print(f"Peak Force (Predicted): {fy_stats['peak_predicted']:.2f} N")
    print()
    print("Error Metrics:")
    print(f"  RMSE: {fy_stats['rmse']:.2f} N ({fy_stats['rmse_pct']:.2f}% of mean force)")
    print(f"  MAPE (Mean Absolute Percentage Error): {fy_stats['mape']:.2f}%")
    print(f"  Median Absolute Percentage Error: {fy_stats['median_ape']:.2f}%")
    print(f"  90th Percentile Error: {fy_stats['p90_error']:.2f}%")
    print(f"  95th Percentile Error: {fy_stats['p95_error']:.2f}%")
    print(f"  Maximum Error: {fy_stats['max_error_pct']:.2f}%")
    print(f"  Peak Force Error: {fy_stats['peak_error_pct']:.2f}%")
    print()
    
    # ========================================================================
    # Summary Comparison
    # ========================================================================
    print("=" * 80)
    print("SUMMARY COMPARISON")
    print("=" * 80)
    print()
    
    print(f"{'Metric':<40} {'Longitudinal (FX)':<20} {'Lateral (FY)':<20}")
    print("-" * 80)
    print(f"{'Model':<40} {fx_model_type.upper():<20} {fy_model_type.upper():<20}")
    print(f"{'Data Points':<40} {fx_stats['num_points']:,<20} {fy_stats['num_points']:,<20}")
    print(f"{'Mean Force (N)':<40} {fx_stats['mean_force']:<20.2f} {fy_stats['mean_force']:<20.2f}")
    print(f"{'RMSE (N)':<40} {fx_stats['rmse']:<20.2f} {fy_stats['rmse']:<20.2f}")
    print(f"{'RMSE (% of mean)':<40} {fx_stats['rmse_pct']:<20.2f} {fy_stats['rmse_pct']:<20.2f}")
    print(f"{'MAPE (%)':<40} {fx_stats['mape']:<20.2f} {fy_stats['mape']:<20.2f}")
    print(f"{'Median Error (%)':<40} {fx_stats['median_ape']:<20.2f} {fy_stats['median_ape']:<20.2f}")
    print(f"{'90th Percentile Error (%)':<40} {fx_stats['p90_error']:<20.2f} {fy_stats['p90_error']:<20.2f}")
    print(f"{'95th Percentile Error (%)':<40} {fx_stats['p95_error']:<20.2f} {fy_stats['p95_error']:<20.2f}")
    print(f"{'Peak Force Error (%)':<40} {fx_stats['peak_error_pct']:<20.2f} {fy_stats['peak_error_pct']:<20.2f}")
    print()
    
    # ========================================================================
    # Interpretation
    # ========================================================================
    print("=" * 80)
    print("INTERPRETATION")
    print("=" * 80)
    print()
    
    print("Longitudinal Force (Magic Formula):")
    print(f"  - Typical prediction error: {fx_stats['median_ape']:.1f}% (median)")
    print(f"  - 90% of predictions within: {fx_stats['p90_error']:.1f}% error")
    print(f"  - RMSE represents: {fx_stats['rmse_pct']:.1f}% of average force")
    print()
    
    print("Lateral Force (TMeasy):")
    print(f"  - Typical prediction error: {fy_stats['median_ape']:.1f}% (median)")
    print(f"  - 90% of predictions within: {fy_stats['p90_error']:.1f}% error")
    print(f"  - RMSE represents: {fy_stats['rmse_pct']:.1f}% of average force")
    print()
    
    print("Overall Assessment:")
    if fx_stats['mape'] < 10 and fy_stats['mape'] < 10:
        print("  ✓ EXCELLENT: Both models show <10% average error")
    elif fx_stats['mape'] < 15 and fy_stats['mape'] < 15:
        print("  ✓ GOOD: Both models show <15% average error")
    elif fx_stats['mape'] < 20 and fy_stats['mape'] < 20:
        print("  ⚠ ACCEPTABLE: Both models show <20% average error")
    else:
        print("  ⚠ NEEDS IMPROVEMENT: Some models show >20% average error")
    print()
    
    print("=" * 80)
    
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
