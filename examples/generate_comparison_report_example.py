"""
Example: Generate Comparison Report

This example demonstrates how to use the comparison report generator
to create a comprehensive comparison between TMeasy and Magic Formula models.

Usage:
    python examples/generate_comparison_report_example.py
"""

import numpy as np
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from tire_fitting.reporting.comparison_report import (
    generate_comparison_report,
    ComparisonReportData,
    ModelResult
)


def create_example_report():
    """Create an example comparison report with realistic data"""
    
    print("Creating example comparison report...")
    
    # Example 1: Magic Formula better for Fx, TMeasy better for Fy
    # This represents the typical scenario discovered in the hybrid tire model research
    
    model_results = {
        'tmeasy': {
            'fx': ModelResult(
                model_type='tmeasy',
                direction='fx',
                parameters={
                    'mu_x_peak': 1.85,
                    'k_mu_x': 0.18,
                    'Kx': 28500.0
                },
                metrics={
                    'rmse': 2547.3,  # Poor performance for longitudinal
                    'tic': 0.45,
                    'r2': 0.75,
                    'peak_error': 5200.8
                }
            ),
            'fy': ModelResult(
                model_type='tmeasy',
                direction='fy',
                parameters={
                    'mu_y_peak': 1.92,
                    'k_mu_y': 0.14,
                    'Ky': 22000.0,
                    'C_gamma': 0.068
                },
                metrics={
                    'rmse': 119.2,  # Excellent performance for lateral
                    'tic': 0.040,
                    'r2': 0.9937,
                    'peak_error': 312.5
                }
            )
        },
        'magic_formula': {
            'fx': ModelResult(
                model_type='magic_formula',
                direction='fx',
                parameters={
                    'pDx1': 2.15,
                    'pCx1': 1.68,
                    'pEx1': -0.08,
                    'pKx1': 23.5,
                    'pKx2': -1.35,
                    'pKx3': 0.18,
                    'fz0': 1200.0
                },
                metrics={
                    'rmse': 85.3,  # Excellent performance for longitudinal
                    'tic': 0.12,
                    'r2': 0.9845,
                    'peak_error': 245.7
                }
            ),
            'fy': ModelResult(
                model_type='magic_formula',
                direction='fy',
                parameters={
                    'pDy1': 2.08,
                    'pCy1': 1.52,
                    'pEy1': -0.18,
                    'pKy1': 21.2,
                    'pKy2': -1.15,
                    'pKy3': 0.12,
                    'fz0': 1200.0
                },
                metrics={
                    'rmse': 125.8,  # Slightly worse than TMeasy for lateral
                    'tic': 0.055,
                    'r2': 0.9912,
                    'peak_error': 298.3
                }
            )
        }
    }
    
    # Generate realistic experimental data
    # Longitudinal force data (DriveBrake)
    n_fx = 500
    sr_data = np.concatenate([
        np.linspace(-0.25, -0.01, n_fx // 2),  # Braking
        np.linspace(0.01, 0.25, n_fx // 2)     # Acceleration
    ])
    fz_fx = np.random.uniform(800, 1600, n_fx)
    
    # Generate forces with some realistic characteristics
    fx_data = np.zeros(n_fx)
    for i, (sr, fz) in enumerate(zip(sr_data, fz_fx)):
        # Simplified force model for realistic data
        mu_peak = 1.8
        fx_data[i] = mu_peak * fz * np.tanh(sr / 0.05) + np.random.randn() * 50
    
    # Lateral force data (Cornering)
    n_fy = 400
    sa_data = np.linspace(-0.15, 0.15, n_fy)
    fz_fy = np.random.uniform(800, 1600, n_fy)
    
    # Generate forces with realistic characteristics
    fy_data = np.zeros(n_fy)
    for i, (sa, fz) in enumerate(zip(sa_data, fz_fy)):
        # Simplified force model for realistic data
        mu_peak = 1.9
        fy_data[i] = mu_peak * fz * np.tanh(sa / 0.05) + np.random.randn() * 40
    
    experimental_data = {
        'fx': {
            'slip_ratio': sr_data,
            'force': fx_data,
            'fz': fz_fx
        },
        'fy': {
            'slip_angle': sa_data,
            'force': fy_data,
            'fz': fz_fy
        }
    }
    
    # Create report data with hybrid selection
    report_data = ComparisonReportData(
        model_results=model_results,
        experimental_data=experimental_data,
        selected_models={
            'fx': 'magic_formula',  # Better for longitudinal
            'fy': 'tmeasy'          # Better for lateral
        },
        selection_rationale=(
            "Magic Formula selected for longitudinal force due to significantly lower RMSE "
            "(85.3N vs 2547.3N, 96.7% improvement). TMeasy selected for lateral force due to "
            "slightly better RMSE (119.2N vs 125.8N, 5.2% improvement) and superior TIC (0.040 vs 0.055)."
        ),
        tire_name="Hoosier 18.0×6.0-10 R20"
    )
    
    # Generate report
    output_dir = "output/comparison_reports"
    report_file = generate_comparison_report(
        report_data,
        output_dir,
        "hybrid_model_comparison"
    )
    
    print(f"\n✓ Comparison report generated successfully!")
    print(f"  Report: {report_file}")
    print(f"  Plots: {output_dir}/plots/")
    print(f"\nKey findings:")
    print(f"  - Magic Formula: 96.7% better RMSE for longitudinal force")
    print(f"  - TMeasy: 5.2% better RMSE for lateral force")
    print(f"  - Hybrid configuration recommended for optimal performance")
    
    return report_file


if __name__ == "__main__":
    try:
        report_file = create_example_report()
        print(f"\n✓ Example completed successfully!")
        print(f"\nTo view the report, open: {report_file}")
    except Exception as e:
        print(f"\n✗ Error generating report: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
