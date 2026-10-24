"""
Integration test for ModelFitter with real data

This test verifies that ModelFitter can fit models using actual tire data.
"""

import pytest
import numpy as np
from pathlib import Path
from tire_fitting.optimizer import ModelFitter
from tire_fitting.data_loader import TireDataLoader
from tire_fitting.preprocessor import TireDataPreprocessor


# Check if data files exist
DRIVEBRAKE_FILE = "data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv"
CORNERING_FILE = "data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv"

has_drivebrake = Path(DRIVEBRAKE_FILE).exists()
has_cornering = Path(CORNERING_FILE).exists()


@pytest.mark.skipif(not has_drivebrake, reason="DriveBrake data file not found")
def test_fit_tmeasy_longitudinal():
    """Test fitting TMeasy model for longitudinal force"""
    data_loader = TireDataLoader()
    preprocessor = TireDataPreprocessor()
    fitter = ModelFitter(data_loader, preprocessor)
    
    # Fit TMeasy for longitudinal force only
    results = fitter.fit_multiple_models(
        model_types=['tmeasy'],
        directions=['fx'],
        drivebrake_file=DRIVEBRAKE_FILE,
        max_workers=1
    )
    
    # Verify results structure
    assert 'tmeasy' in results
    assert 'fx' in results['tmeasy']
    assert 'params' in results['tmeasy']['fx']
    assert 'metrics' in results['tmeasy']['fx']
    
    # Verify parameters
    params = results['tmeasy']['fx']['params']
    assert 'mu_x_peak' in params
    assert 'k_mu_x' in params
    assert 'Kx' in params
    
    # Verify metrics
    metrics = results['tmeasy']['fx']['metrics']
    assert 'rmse' in metrics
    assert 'tic' in metrics
    assert 'r2' in metrics
    assert 'peak_error' in metrics
    
    # Check that metrics are reasonable
    assert metrics['rmse'] > 0
    assert 0 <= metrics['tic'] <= 1
    assert metrics['r2'] <= 1.0
    assert metrics['peak_error'] > 0
    
    print(f"\nTMeasy Longitudinal Fitting Results:")
    print(f"  Parameters: {params}")
    print(f"  RMSE: {metrics['rmse']:.2f} N")
    print(f"  TIC: {metrics['tic']:.4f}")
    print(f"  R²: {metrics['r2']:.4f}")
    print(f"  Peak Error: {metrics['peak_error']:.2f} N")


@pytest.mark.skipif(not has_cornering, reason="Cornering data file not found")
def test_fit_tmeasy_lateral():
    """Test fitting TMeasy model for lateral force"""
    data_loader = TireDataLoader()
    preprocessor = TireDataPreprocessor()
    fitter = ModelFitter(data_loader, preprocessor)
    
    # Fit TMeasy for lateral force only
    results = fitter.fit_multiple_models(
        model_types=['tmeasy'],
        directions=['fy'],
        cornering_file=CORNERING_FILE,
        max_workers=1
    )
    
    # Verify results structure
    assert 'tmeasy' in results
    assert 'fy' in results['tmeasy']
    assert 'params' in results['tmeasy']['fy']
    assert 'metrics' in results['tmeasy']['fy']
    
    # Verify parameters
    params = results['tmeasy']['fy']['params']
    assert 'mu_y_peak' in params
    assert 'k_mu_y' in params
    assert 'Ky' in params
    assert 'C_gamma' in params
    
    # Verify metrics
    metrics = results['tmeasy']['fy']['metrics']
    assert 'rmse' in metrics
    assert 'tic' in metrics
    assert 'r2' in metrics
    assert 'peak_error' in metrics
    
    print(f"\nTMeasy Lateral Fitting Results:")
    print(f"  Parameters: {params}")
    print(f"  RMSE: {metrics['rmse']:.2f} N")
    print(f"  TIC: {metrics['tic']:.4f}")
    print(f"  R²: {metrics['r2']:.4f}")
    print(f"  Peak Error: {metrics['peak_error']:.2f} N")


@pytest.mark.skipif(not has_drivebrake, reason="DriveBrake data file not found")
def test_result_caching():
    """Test that results are cached to avoid redundant computation"""
    data_loader = TireDataLoader()
    preprocessor = TireDataPreprocessor()
    fitter = ModelFitter(data_loader, preprocessor)
    
    # First fit
    results1 = fitter.fit_multiple_models(
        model_types=['tmeasy'],
        directions=['fx'],
        drivebrake_file=DRIVEBRAKE_FILE,
        max_workers=1
    )
    
    # Check cache
    assert 'tmeasy_fx' in fitter.cache
    
    # Second fit (should use cache)
    results2 = fitter.fit_multiple_models(
        model_types=['tmeasy'],
        directions=['fx'],
        drivebrake_file=DRIVEBRAKE_FILE,
        max_workers=1
    )
    
    # Results should be identical
    assert results1['tmeasy']['fx']['params'] == results2['tmeasy']['fx']['params']
    assert results1['tmeasy']['fx']['metrics'] == results2['tmeasy']['fx']['metrics']
    
    print("\nCaching test passed - results are identical")


@pytest.mark.skipif(not (has_drivebrake and has_cornering), 
                    reason="Data files not found")
def test_fit_multiple_models_parallel():
    """Test fitting multiple models in parallel"""
    data_loader = TireDataLoader()
    preprocessor = TireDataPreprocessor()
    fitter = ModelFitter(data_loader, preprocessor)
    
    # Fit both TMeasy and Magic Formula for both directions
    results = fitter.fit_multiple_models(
        model_types=['tmeasy', 'magic_formula'],
        directions=['fx', 'fy'],
        drivebrake_file=DRIVEBRAKE_FILE,
        cornering_file=CORNERING_FILE,
        max_workers=2
    )
    
    # Verify all models were fitted
    assert 'tmeasy' in results
    assert 'magic_formula' in results
    
    # Verify both directions for each model
    for model_type in ['tmeasy', 'magic_formula']:
        assert 'fx' in results[model_type]
        assert 'fy' in results[model_type]
        
        for direction in ['fx', 'fy']:
            assert 'params' in results[model_type][direction]
            assert 'metrics' in results[model_type][direction]
    
    # Print comparison
    print("\n=== Model Comparison ===")
    for direction in ['fx', 'fy']:
        print(f"\n{direction.upper()} Direction:")
        for model_type in ['tmeasy', 'magic_formula']:
            metrics = results[model_type][direction]['metrics']
            print(f"  {model_type}:")
            print(f"    RMSE: {metrics['rmse']:.2f} N")
            print(f"    TIC: {metrics['tic']:.4f}")
            print(f"    R²: {metrics['r2']:.4f}")


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
