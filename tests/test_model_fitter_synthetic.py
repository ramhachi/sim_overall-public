"""
Test ModelFitter with synthetic data

This test verifies ModelFitter functionality using synthetic data
to avoid long optimization times.
"""

import pytest
import numpy as np
import pandas as pd
from tire_fitting.optimizer import ModelFitter
from tire_fitting.models.tmeasy import TMeasyModel
from tire_fitting.models.magic_formula import MagicFormulaModel


def test_calculate_metrics_tmeasy():
    """Test metrics calculation for TMeasy model"""
    from tire_fitting.data_loader import TireDataLoader
    from tire_fitting.preprocessor import TireDataPreprocessor
    
    fitter = ModelFitter(TireDataLoader(), TireDataPreprocessor())
    
    # Create synthetic data using known TMeasy parameters
    true_params = {
        'mu_x_peak': 1.8,
        'k_mu_x': 0.2,
        'Kx': 25000.0
    }
    
    model = TMeasyModel(fz_nominal=1200.0, params=true_params)
    
    # Generate synthetic data
    slip_data = np.linspace(-0.2, 0.2, 100)
    fz_data = np.full(100, 1200.0)
    force_data = np.array([model.calculate_fx(sr, fz) for sr, fz in zip(slip_data, fz_data)])
    
    # Calculate metrics (should be near perfect since we're using the same model)
    metrics = fitter._calculate_metrics(
        true_params,
        'tmeasy',
        'fx',
        slip_data,
        force_data,
        fz_data
    )
    
    # Verify metrics
    assert 'rmse' in metrics
    assert 'tic' in metrics
    assert 'r2' in metrics
    assert 'peak_error' in metrics
    
    # Should be very small errors (near zero) since we're using the same model
    assert metrics['rmse'] < 1.0  # Less than 1N error
    assert metrics['tic'] < 0.01  # Very small TIC
    assert metrics['r2'] > 0.99   # Very high R²
    
    print(f"\nMetrics with perfect fit:")
    print(f"  RMSE: {metrics['rmse']:.6f} N")
    print(f"  TIC: {metrics['tic']:.6f}")
    print(f"  R²: {metrics['r2']:.6f}")


def test_calculate_metrics_magic_formula():
    """Test metrics calculation for Magic Formula model"""
    from tire_fitting.data_loader import TireDataLoader
    from tire_fitting.preprocessor import TireDataPreprocessor
    
    fitter = ModelFitter(TireDataLoader(), TireDataPreprocessor())
    
    # Create synthetic data using known Magic Formula parameters
    true_params = {
        'pDx1': 2.0,
        'pCx1': 1.5,
        'pEx1': -0.1,
        'pKx1': 20.0,
        'pKx2': -2.0,
        'pKx3': 0.5,
        'fz0': 1200.0
    }
    
    model = MagicFormulaModel(params=true_params)
    
    # Generate synthetic data
    slip_data = np.linspace(-0.2, 0.2, 100)
    fz_data = np.full(100, 1200.0)
    force_data = np.array([model.calculate_fx(sr, fz) for sr, fz in zip(slip_data, fz_data)])
    
    # Calculate metrics
    metrics = fitter._calculate_metrics(
        true_params,
        'magic_formula',
        'fx',
        slip_data,
        force_data,
        fz_data
    )
    
    # Verify metrics
    assert 'rmse' in metrics
    assert 'tic' in metrics
    assert 'r2' in metrics
    assert 'peak_error' in metrics
    
    # Should be very small errors
    assert metrics['rmse'] < 1.0
    assert metrics['tic'] < 0.01
    assert metrics['r2'] > 0.99
    
    print(f"\nMagic Formula metrics with perfect fit:")
    print(f"  RMSE: {metrics['rmse']:.6f} N")
    print(f"  TIC: {metrics['tic']:.6f}")
    print(f"  R²: {metrics['r2']:.6f}")


def test_params_array_to_dict_all_models():
    """Test parameter conversion for all model types and directions"""
    from tire_fitting.data_loader import TireDataLoader
    from tire_fitting.preprocessor import TireDataPreprocessor
    
    fitter = ModelFitter(TireDataLoader(), TireDataPreprocessor())
    
    # Test all combinations
    test_cases = [
        ('tmeasy', 'fx', 3),
        ('tmeasy', 'fy', 4),
        ('magic_formula', 'fx', 6),
        ('magic_formula', 'fy', 6)
    ]
    
    for model_type, direction, expected_count in test_cases:
        bounds = fitter._get_parameter_bounds(model_type, direction)
        params_array = np.array([b[0] + (b[1] - b[0]) / 2 for b in bounds])
        params_dict = fitter._params_array_to_dict(params_array, model_type, direction)
        
        # Verify correct number of parameters (excluding fz0 for MF)
        if model_type == 'magic_formula':
            assert len(params_dict) == expected_count + 1  # +1 for fz0
        else:
            assert len(params_dict) == expected_count
        
        print(f"\n{model_type} {direction}: {list(params_dict.keys())}")


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
