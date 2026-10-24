"""
Property-Based Tests for TMeasy Backward Compatibility

This module tests that the new TMeasyModel implementation (with TireModel interface)
produces identical results to the original implementation.

Property 7: TMeasy Backward Compatibility
Validates: Requirements 2.6, 9.3
"""

import pytest
from hypothesis import given, strategies as st, settings
import numpy as np

# Import both implementations
from tire_fitting.tmeasy_model import TMeasyModel as OriginalTMeasy
from tire_fitting.models.tmeasy import TMeasyModel as NewTMeasy


# Strategy for generating valid tire parameters
@st.composite
def tire_parameters(draw):
    """Generate valid tire model parameters"""
    return {
        'mu_x_peak': draw(st.floats(min_value=0.5, max_value=2.5)),
        'k_mu_x': draw(st.floats(min_value=0.0, max_value=0.5)),
        'Kx': draw(st.floats(min_value=5000.0, max_value=50000.0)),
        'mu_y_peak': draw(st.floats(min_value=0.5, max_value=2.5)),
        'k_mu_y': draw(st.floats(min_value=0.0, max_value=0.5)),
        'Ky': draw(st.floats(min_value=5000.0, max_value=50000.0)),
        'C_gamma': draw(st.floats(min_value=-0.5, max_value=0.5)),
    }


# Strategy for generating valid inputs
@st.composite
def tire_inputs(draw):
    """Generate valid tire force calculation inputs"""
    return {
        'slip_ratio': draw(st.floats(min_value=-0.3, max_value=0.3)),
        'slip_angle': draw(st.floats(min_value=-0.3, max_value=0.3)),  # radians
        'fz': draw(st.floats(min_value=100.0, max_value=2000.0)),
        'camber_angle': draw(st.floats(min_value=-0.1, max_value=0.1)),  # radians
    }


class TestTMeasyBackwardCompatibility:
    """Test backward compatibility between original and new TMeasy implementations"""
    
    @given(params=tire_parameters(), inputs=tire_inputs())
    @settings(max_examples=100, deadline=None)
    def test_longitudinal_force_identical(self, params, inputs):
        """
        Property 7: TMeasy Backward Compatibility - Longitudinal Force
        
        For any valid parameters and inputs, the new TMeasyModel should produce
        longitudinal forces identical to the original implementation within
        numerical precision.
        
        Validates: Requirements 2.6, 9.3
        """
        fz_nominal = 1000.0
        
        # Original implementation
        original = OriginalTMeasy(fz_nominal=fz_nominal)
        fx_original = original.compute_longitudinal_force(
            slip_ratio=inputs['slip_ratio'],
            fz=inputs['fz'],
            mu_x_peak=params['mu_x_peak'],
            k_mu_x=params['k_mu_x'],
            Kx=params['Kx']
        )
        
        # New implementation
        new = NewTMeasy(fz_nominal=fz_nominal, params=params)
        fx_new = new.calculate_fx(
            slip_ratio=inputs['slip_ratio'],
            fz=inputs['fz']
        )
        
        # Assert identical within numerical precision
        assert np.isclose(fx_new, fx_original, rtol=1e-10, atol=1e-10), \
            f"Longitudinal force mismatch: new={fx_new}, original={fx_original}"
    
    @given(params=tire_parameters(), inputs=tire_inputs())
    @settings(max_examples=100, deadline=None)
    def test_lateral_force_identical(self, params, inputs):
        """
        Property 7: TMeasy Backward Compatibility - Lateral Force
        
        For any valid parameters and inputs, the new TMeasyModel should produce
        lateral forces identical to the original implementation within
        numerical precision.
        
        Validates: Requirements 2.6, 9.3
        """
        fz_nominal = 1000.0
        
        # Original implementation
        original = OriginalTMeasy(fz_nominal=fz_nominal)
        fy_original = original.compute_lateral_force(
            slip_angle=inputs['slip_angle'],
            fz=inputs['fz'],
            camber_angle=inputs['camber_angle'],
            mu_y_peak=params['mu_y_peak'],
            k_mu_y=params['k_mu_y'],
            Ky=params['Ky'],
            C_gamma=params['C_gamma']
        )
        
        # New implementation
        new = NewTMeasy(fz_nominal=fz_nominal, params=params)
        fy_new = new.calculate_fy(
            slip_angle=inputs['slip_angle'],
            fz=inputs['fz'],
            camber_angle=inputs['camber_angle']
        )
        
        # Assert identical within numerical precision
        assert np.isclose(fy_new, fy_original, rtol=1e-10, atol=1e-10), \
            f"Lateral force mismatch: new={fy_new}, original={fy_original}"
    
    @given(params=tire_parameters(), inputs=tire_inputs())
    @settings(max_examples=50, deadline=None)
    def test_vectorized_operations_identical(self, params, inputs):
        """
        Property 7: TMeasy Backward Compatibility - Vectorized Operations
        
        For any valid parameters and inputs, vectorized operations should produce
        identical results to scalar operations.
        
        Validates: Requirements 2.6, 9.3
        """
        fz_nominal = 1000.0
        
        # Create arrays of inputs
        n_points = 10
        slip_ratios = np.linspace(-0.2, 0.2, n_points)
        slip_angles = np.linspace(-0.2, 0.2, n_points)
        fz_values = np.full(n_points, inputs['fz'])
        camber_values = np.full(n_points, inputs['camber_angle'])
        
        # Original implementation - vectorized
        original = OriginalTMeasy(fz_nominal=fz_nominal)
        fx_original_vec = original.compute_longitudinal_force(
            slip_ratio=slip_ratios,
            fz=fz_values,
            mu_x_peak=params['mu_x_peak'],
            k_mu_x=params['k_mu_x'],
            Kx=params['Kx']
        )
        
        # New implementation - scalar loop
        new = NewTMeasy(fz_nominal=fz_nominal, params=params)
        fx_new_scalar = np.array([
            new.calculate_fx(sr, fz)
            for sr, fz in zip(slip_ratios, fz_values)
        ])
        
        # Assert vectorized and scalar produce same results
        assert np.allclose(fx_new_scalar, fx_original_vec, rtol=1e-10, atol=1e-10), \
            "Vectorized operations produce different results"
    
    def test_model_type_identification(self):
        """
        Test that new implementation correctly identifies as 'tmeasy'
        
        Validates: Requirements 2.5
        """
        model = NewTMeasy()
        assert model.get_model_type() == "tmeasy"
    
    def test_parameter_get_set_roundtrip(self):
        """
        Test that getting and setting parameters works correctly
        
        Validates: Requirements 2.3, 2.4
        """
        params = {
            'mu_x_peak': 1.5,
            'k_mu_x': 0.1,
            'Kx': 20000.0,
            'mu_y_peak': 1.6,
            'k_mu_y': 0.12,
            'Ky': 18000.0,
            'C_gamma': 0.15
        }
        
        model = NewTMeasy(fz_nominal=1200.0, params=params)
        
        # Get parameters
        retrieved_params = model.get_parameters()
        
        # Check all parameters are present
        assert retrieved_params['fz_nominal'] == 1200.0
        for key, value in params.items():
            assert key in retrieved_params
            assert np.isclose(retrieved_params[key], value)
        
        # Set new parameters
        new_params = {'mu_x_peak': 1.8, 'Kx': 25000.0}
        model.set_parameters(new_params)
        
        # Verify parameters were updated
        updated_params = model.get_parameters()
        assert updated_params['mu_x_peak'] == 1.8
        assert updated_params['Kx'] == 25000.0
        # Other parameters should remain unchanged
        assert updated_params['k_mu_x'] == 0.1
