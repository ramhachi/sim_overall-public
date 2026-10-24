"""
Test Comparison Report Generator

This test verifies that the comparison report generator can create
reports with metrics tables and visualizations.
"""

import pytest
import numpy as np
from pathlib import Path
import tempfile
import shutil

from tire_fitting.reporting.comparison_report import (
    generate_comparison_report,
    ComparisonReportData,
    ModelResult
)


def test_comparison_report_generation():
    """Test that comparison report can be generated with valid data"""
    
    # Create mock model results
    model_results = {
        'tmeasy': {
            'fx': ModelResult(
                model_type='tmeasy',
                direction='fx',
                parameters={
                    'mu_x_peak': 1.8,
                    'k_mu_x': 0.15,
                    'Kx': 25000.0
                },
                metrics={
                    'rmse': 150.5,
                    'tic': 0.08,
                    'r2': 0.95,
                    'peak_error': 350.2
                }
            ),
            'fy': ModelResult(
                model_type='tmeasy',
                direction='fy',
                parameters={
                    'mu_y_peak': 1.9,
                    'k_mu_y': 0.12,
                    'Ky': 20000.0,
                    'C_gamma': 0.05
                },
                metrics={
                    'rmse': 120.3,
                    'tic': 0.05,
                    'r2': 0.98,
                    'peak_error': 280.1
                }
            )
        },
        'magic_formula': {
            'fx': ModelResult(
                model_type='magic_formula',
                direction='fx',
                parameters={
                    'pDx1': 2.1,
                    'pCx1': 1.6,
                    'pEx1': -0.1,
                    'pKx1': 22.0,
                    'pKx2': -1.5,
                    'pKx3': 0.2,
                    'fz0': 1200.0
                },
                metrics={
                    'rmse': 85.2,
                    'tic': 0.04,
                    'r2': 0.99,
                    'peak_error': 200.5
                }
            ),
            'fy': ModelResult(
                model_type='magic_formula',
                direction='fy',
                parameters={
                    'pDy1': 2.0,
                    'pCy1': 1.5,
                    'pEy1': -0.15,
                    'pKy1': 20.0,
                    'pKy2': -1.2,
                    'pKy3': 0.15,
                    'fz0': 1200.0
                },
                metrics={
                    'rmse': 125.8,
                    'tic': 0.06,
                    'r2': 0.97,
                    'peak_error': 290.3
                }
            )
        }
    }
    
    # Create mock experimental data
    n_points = 100
    experimental_data = {
        'fx': {
            'slip_ratio': np.linspace(-0.2, 0.2, n_points),
            'force': np.random.randn(n_points) * 100 + 1000,
            'fz': np.ones(n_points) * 1200.0
        },
        'fy': {
            'slip_angle': np.linspace(-0.15, 0.15, n_points),
            'force': np.random.randn(n_points) * 100 + 1500,
            'fz': np.ones(n_points) * 1200.0
        }
    }
    
    # Create report data
    report_data = ComparisonReportData(
        model_results=model_results,
        experimental_data=experimental_data,
        selected_models={
            'fx': 'magic_formula',
            'fy': 'tmeasy'
        },
        selection_rationale=(
            "Magic Formula selected for Fx due to 43% lower RMSE (85.2N vs 150.5N). "
            "TMeasy selected for Fy due to slightly better RMSE (120.3N vs 125.8N)."
        ),
        tire_name="Hoosier 18.0x6.0-10 R20"
    )
    
    # Create temporary output directory
    with tempfile.TemporaryDirectory() as tmpdir:
        # Generate report
        report_file = generate_comparison_report(
            report_data,
            tmpdir,
            "test_comparison"
        )
        
        # Verify report file exists
        assert Path(report_file).exists(), "Report file should be created"
        
        # Verify report content
        with open(report_file, 'r') as f:
            content = f.read()
        
        # Check for key sections
        assert "# Tire Model Comparison Report" in content
        assert "Executive Summary" in content
        assert "Fitting Metrics Comparison" in content
        assert "Force-Slip Curve Visualizations" in content
        assert "Fitted Model Parameters" in content
        
        # Check for metrics
        assert "RMSE" in content
        assert "TIC" in content
        assert "R²" in content
        assert "Peak Error" in content
        
        # Check for model names
        assert "TMEASY" in content
        assert "MAGIC_FORMULA" in content
        
        # Check for tire name
        assert "Hoosier 18.0x6.0-10 R20" in content
        
        # Verify plots directory exists
        plots_dir = Path(tmpdir) / "plots"
        assert plots_dir.exists(), "Plots directory should be created"
        
        # Verify plot files exist
        fx_plot = plots_dir / "test_comparison_fx.png"
        fy_plot = plots_dir / "test_comparison_fy.png"
        assert fx_plot.exists(), "Fx plot should be created"
        assert fy_plot.exists(), "Fy plot should be created"
        
        print("✓ Comparison report generation test passed")


def test_report_with_missing_direction():
    """Test report generation when one direction is missing"""
    
    # Create model results with only fx
    model_results = {
        'tmeasy': {
            'fx': ModelResult(
                model_type='tmeasy',
                direction='fx',
                parameters={'mu_x_peak': 1.8, 'k_mu_x': 0.15, 'Kx': 25000.0},
                metrics={'rmse': 150.5, 'tic': 0.08, 'r2': 0.95, 'peak_error': 350.2}
            )
        }
    }
    
    # Create experimental data with only fx
    experimental_data = {
        'fx': {
            'slip_ratio': np.linspace(-0.2, 0.2, 50),
            'force': np.random.randn(50) * 100 + 1000,
            'fz': np.ones(50) * 1200.0
        }
    }
    
    # This should raise ValueError due to missing fy data
    report_data = ComparisonReportData(
        model_results=model_results,
        experimental_data=experimental_data,
        selected_models={'fx': 'tmeasy'},
        selection_rationale="Test",
        tire_name="Test Tire"
    )
    
    with tempfile.TemporaryDirectory() as tmpdir:
        with pytest.raises(ValueError, match="Missing experimental data for fy"):
            generate_comparison_report(report_data, tmpdir, "test_missing")
    
    print("✓ Missing direction validation test passed")


if __name__ == "__main__":
    test_comparison_report_generation()
    test_report_with_missing_direction()
    print("\n✓ All comparison report tests passed!")
