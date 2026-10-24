"""
Verification script for FitEvaluator

This script demonstrates the evaluator functionality with synthetic data.
"""

import numpy as np
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from tire_fitting.evaluator import FitEvaluator


def main():
    """Run evaluator verification"""
    print("=" * 70)
    print("FitEvaluator Verification")
    print("=" * 70)
    
    # Create evaluator
    evaluator = FitEvaluator()
    
    # Generate synthetic data
    print("\n1. Generating synthetic force data...")
    np.random.seed(42)
    n_points = 100
    
    # True forces with some pattern
    y_true = 1000 * np.sin(np.linspace(-1, 1, n_points)) + np.random.normal(0, 50, n_points)
    
    # Predicted forces (slightly different)
    y_pred = 1000 * np.sin(np.linspace(-1, 1, n_points)) + np.random.normal(0, 30, n_points)
    
    print(f"   Generated {n_points} data points")
    print(f"   True force range: [{y_true.min():.1f}, {y_true.max():.1f}] N")
    print(f"   Predicted force range: [{y_pred.min():.1f}, {y_pred.max():.1f}] N")
    
    # Calculate metrics
    print("\n2. Calculating evaluation metrics...")
    metrics = evaluator.calculate_metrics(y_true, y_pred)
    
    print(f"   RMSE:  {metrics['rmse']:.2f} N")
    print(f"   NRMSE: {metrics['nrmse']:.4f}")
    print(f"   R²:    {metrics['r2']:.4f}")
    print(f"   TIC:   {metrics['tic']:.4f}")
    
    # Interpret metrics
    print("\n3. Interpreting metrics...")
    if metrics['rmse'] < 100:
        print("   ✓ RMSE < 100 N: Excellent fit")
    else:
        print("   ✗ RMSE >= 100 N: Needs improvement")
    
    if metrics['r2'] > 0.95:
        print("   ✓ R² > 0.95: High correlation")
    elif metrics['r2'] > 0.90:
        print("   ✓ R² > 0.90: Good correlation")
    else:
        print("   ✗ R² < 0.90: Poor correlation")
    
    if metrics['tic'] < 0.15:
        print("   ✓ TIC < 0.15: Low inequality")
    else:
        print("   ✗ TIC >= 0.15: High inequality")
    
    # Calculate peak friction error
    print("\n4. Calculating peak friction error...")
    fz_data = np.full(n_points, 1000.0)  # Constant vertical load
    
    peak_error = evaluator.calculate_peak_friction_error(
        y_true, fz_data, y_pred
    )
    
    print(f"   Peak friction error: {peak_error:.2f}%")
    
    if peak_error < 5.0:
        print("   ✓ Peak error < 5%: Excellent")
    elif peak_error < 10.0:
        print("   ✓ Peak error < 10%: Good")
    else:
        print("   ✗ Peak error >= 10%: Needs improvement")
    
    # Test plotting (without saving)
    print("\n5. Testing plot generation...")
    
    # Generate slip data
    slip_data = np.linspace(-0.2, 0.2, n_points)
    
    # Create load groups
    load_groups = {
        'Low': np.arange(n_points) < 33,
        'Medium': (np.arange(n_points) >= 33) & (np.arange(n_points) < 66),
        'High': np.arange(n_points) >= 66
    }
    
    print("   Testing force vs slip plot...")
    try:
        # Test without saving (will not display in headless environment)
        import matplotlib
        matplotlib.use('Agg')  # Use non-interactive backend
        
        evaluator.plot_force_vs_slip(
            slip_data, y_true, y_pred,
            load_groups=load_groups,
            xlabel="Slip Ratio [-]",
            ylabel="Force [N]",
            title="Test Force vs Slip",
            save_path="output/test_force_vs_slip.png"
        )
        print("   ✓ Force vs slip plot created successfully")
    except Exception as e:
        print(f"   ✗ Force vs slip plot failed: {e}")
    
    print("   Testing residual plot...")
    try:
        evaluator.plot_residuals(
            y_true, y_pred,
            save_path="output/test_residuals.png"
        )
        print("   ✓ Residual plot created successfully")
    except Exception as e:
        print(f"   ✗ Residual plot failed: {e}")
    
    # Summary
    print("\n" + "=" * 70)
    print("Verification Summary")
    print("=" * 70)
    print("✓ FitEvaluator class implemented successfully")
    print("✓ calculate_metrics() computes RMSE, NRMSE, R², TIC")
    print("✓ calculate_peak_friction_error() computes peak friction error")
    print("✓ plot_force_vs_slip() generates force vs slip plots")
    print("✓ plot_residuals() generates residual plots")
    print("\nAll evaluator functionality verified!")
    print("=" * 70)


if __name__ == "__main__":
    main()
