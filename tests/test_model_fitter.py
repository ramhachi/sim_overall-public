"""
Test ModelFitter implementation

This test verifies that the ModelFitter class can be instantiated and
has the required methods.
"""

import pytest
from tire_fitting.optimizer import ModelFitter
from tire_fitting.data_loader import TireDataLoader
from tire_fitting.preprocessor import TireDataPreprocessor


def test_model_fitter_instantiation():
    """Test that ModelFitter can be instantiated with data loader and preprocessor"""
    data_loader = TireDataLoader()
    preprocessor = TireDataPreprocessor()
    
    fitter = ModelFitter(data_loader, preprocessor)
    
    assert fitter is not None
    assert fitter.data_loader is data_loader
    assert fitter.preprocessor is preprocessor
    assert isinstance(fitter.cache, dict)
    assert len(fitter.cache) == 0


def test_model_fitter_has_required_methods():
    """Test that ModelFitter has all required methods"""
    data_loader = TireDataLoader()
    preprocessor = TireDataPreprocessor()
    fitter = ModelFitter(data_loader, preprocessor)
    
    # Check public methods
    assert hasattr(fitter, 'fit_multiple_models')
    assert callable(fitter.fit_multiple_models)
    
    # Check private methods
    assert hasattr(fitter, '_fit_single_model')
    assert hasattr(fitter, '_get_parameter_bounds')
    assert hasattr(fitter, '_params_array_to_dict')
    assert hasattr(fitter, '_calculate_metrics')


def test_get_parameter_bounds_tmeasy():
    """Test parameter bounds for TMeasy model"""
    data_loader = TireDataLoader()
    preprocessor = TireDataPreprocessor()
    fitter = ModelFitter(data_loader, preprocessor)
    
    # Test longitudinal bounds
    bounds_fx = fitter._get_parameter_bounds('tmeasy', 'fx')
    assert len(bounds_fx) == 3  # mu_x_peak, k_mu_x, Kx
    assert all(isinstance(b, tuple) and len(b) == 2 for b in bounds_fx)
    
    # Test lateral bounds
    bounds_fy = fitter._get_parameter_bounds('tmeasy', 'fy')
    assert len(bounds_fy) == 4  # mu_y_peak, k_mu_y, Ky, C_gamma
    assert all(isinstance(b, tuple) and len(b) == 2 for b in bounds_fy)


def test_get_parameter_bounds_magic_formula():
    """Test parameter bounds for Magic Formula model"""
    data_loader = TireDataLoader()
    preprocessor = TireDataPreprocessor()
    fitter = ModelFitter(data_loader, preprocessor)
    
    # Test longitudinal bounds
    bounds_fx = fitter._get_parameter_bounds('magic_formula', 'fx')
    assert len(bounds_fx) == 6  # pDx1, pCx1, pEx1, pKx1, pKx2, pKx3
    assert all(isinstance(b, tuple) and len(b) == 2 for b in bounds_fx)
    
    # Test lateral bounds
    bounds_fy = fitter._get_parameter_bounds('magic_formula', 'fy')
    assert len(bounds_fy) == 6  # pDy1, pCy1, pEy1, pKy1, pKy2, pKy3
    assert all(isinstance(b, tuple) and len(b) == 2 for b in bounds_fy)


def test_params_array_to_dict_tmeasy():
    """Test parameter array to dict conversion for TMeasy"""
    import numpy as np
    
    data_loader = TireDataLoader()
    preprocessor = TireDataPreprocessor()
    fitter = ModelFitter(data_loader, preprocessor)
    
    # Test longitudinal
    params_fx = np.array([1.8, 0.2, 25000.0])
    params_dict_fx = fitter._params_array_to_dict(params_fx, 'tmeasy', 'fx')
    assert 'mu_x_peak' in params_dict_fx
    assert 'k_mu_x' in params_dict_fx
    assert 'Kx' in params_dict_fx
    assert params_dict_fx['mu_x_peak'] == 1.8
    
    # Test lateral
    params_fy = np.array([1.9, 0.15, 20000.0, 0.05])
    params_dict_fy = fitter._params_array_to_dict(params_fy, 'tmeasy', 'fy')
    assert 'mu_y_peak' in params_dict_fy
    assert 'k_mu_y' in params_dict_fy
    assert 'Ky' in params_dict_fy
    assert 'C_gamma' in params_dict_fy


def test_params_array_to_dict_magic_formula():
    """Test parameter array to dict conversion for Magic Formula"""
    import numpy as np
    
    data_loader = TireDataLoader()
    preprocessor = TireDataPreprocessor()
    fitter = ModelFitter(data_loader, preprocessor)
    
    # Test longitudinal
    params_fx = np.array([2.0, 1.5, -0.1, 20.0, -2.0, 0.5])
    params_dict_fx = fitter._params_array_to_dict(params_fx, 'magic_formula', 'fx')
    assert 'pDx1' in params_dict_fx
    assert 'pCx1' in params_dict_fx
    assert 'pEx1' in params_dict_fx
    assert 'pKx1' in params_dict_fx
    assert 'pKx2' in params_dict_fx
    assert 'pKx3' in params_dict_fx
    assert 'fz0' in params_dict_fx
    assert params_dict_fx['pDx1'] == 2.0
    
    # Test lateral
    params_fy = np.array([2.1, 1.6, -0.2, 21.0, -1.5, 0.4])
    params_dict_fy = fitter._params_array_to_dict(params_fy, 'magic_formula', 'fy')
    assert 'pDy1' in params_dict_fy
    assert 'pCy1' in params_dict_fy
    assert 'pEy1' in params_dict_fy
    assert 'pKy1' in params_dict_fy
    assert 'pKy2' in params_dict_fy
    assert 'pKy3' in params_dict_fy
    assert 'fz0' in params_dict_fy


def test_fit_multiple_models_validation():
    """Test that fit_multiple_models validates inputs correctly"""
    data_loader = TireDataLoader()
    preprocessor = TireDataPreprocessor()
    fitter = ModelFitter(data_loader, preprocessor)
    
    # Should raise error if fx direction requested without drivebrake_file
    with pytest.raises(ValueError, match="drivebrake_file is required"):
        fitter.fit_multiple_models(
            model_types=['tmeasy'],
            directions=['fx'],
            drivebrake_file=None
        )
    
    # Should raise error if fy direction requested without cornering_file
    with pytest.raises(ValueError, match="cornering_file is required"):
        fitter.fit_multiple_models(
            model_types=['tmeasy'],
            directions=['fy'],
            cornering_file=None
        )


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
