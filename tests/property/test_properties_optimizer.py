"""
Property-based tests for Tire Parameter Optimizer

These tests use the Hypothesis library to verify universal properties
across many randomly generated inputs for the optimizer.
"""

import pytest
import numpy as np
from hypothesis import given, strategies as st, settings, assume
from tire_fitting.tmeasy_model import TMeasyModel
from tire_fitting.optimizer import TireParameterOptimizer


class TestOptimizerProperties:
    """Property-based tests for TireParameterOptimizer"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.model = TMeasyModel(fz_nominal=1000.0)
        self.optimizer = TireParameterOptimizer(self.model)
    
    @settings(max_examples=20, deadline=None)
    @given(
        # Generate synthetic data with known parameters
        mu_x_peak_true=st.floats(min_value=1.2, max_value=2.0, allow_nan=False, allow_infinity=False),
        k_mu_x_true=st.floats(min_value=0.05, max_value=0.3, allow_nan=False, allow_infinity=False),
        Kx_true=st.floats(min_value=10000.0, max_value=40000.0, allow_nan=False, allow_infinity=False),
        seed=st.integers(min_value=0, max_value=10000)
    )
    def test_property_14_longitudinal_optimizer_returns_valid_parameters(
        self, mu_x_peak_true, k_mu_x_true, Kx_true, seed
    ):
        """
        Property 14: Longitudinal Optimizer Returns Valid Parameters
        
        For any longitudinal fitting operation, the optimizer should return 
        parameters mu_x_peak, k_mu_x, and Kx where 1.0 <= mu_x_peak <= 2.5, 
        0 <= k_mu_x <= 0.5, and 5000 <= Kx <= 50000.
        
        **Validates: Requirements 7.1, 7.2, 7.3, 7.4**
        """
        # Set random seed for reproducibility
        np.random.seed(seed)
        
        # Generate synthetic data with known parameters
        n_points = 50
        sr_data = np.linspace(-0.2, 0.2, n_points)
        fz_data = np.random.uniform(700, 1300, n_points)
        
        # Generate clean force data
        fx_clean = self.model.compute_longitudinal_force(
            sr_data, fz_data, mu_x_peak_true, k_mu_x_true, Kx_true
        )
        
        # Add small noise (5% of mean force magnitude)
        noise_level = 0.05 * np.mean(np.abs(fx_clean))
        fx_data = fx_clean + np.random.normal(0, noise_level, n_points)
        
        # Fit parameters
        result = self.optimizer.fit_longitudinal(sr_data, fx_data, fz_data)
        
        # Property 1: mu_x_peak should be within bounds [1.0, 2.5]
        assert 1.0 <= result['mu_x_peak'] <= 2.5, \
            f"mu_x_peak={result['mu_x_peak']} should be in [1.0, 2.5]"
        
        # Property 2: k_mu_x should be within bounds [0.0, 0.5]
        assert 0.0 <= result['k_mu_x'] <= 0.5, \
            f"k_mu_x={result['k_mu_x']} should be in [0.0, 0.5]"
        
        # Property 3: Kx should be within bounds [5000, 50000]
        assert 5000.0 <= result['Kx'] <= 50000.0, \
            f"Kx={result['Kx']} should be in [5000, 50000]"
        
        # Property 4: Result should contain required keys
        required_keys = ['mu_x_peak', 'k_mu_x', 'Kx', 'rmse', 'r2', 'success', 'message']
        for key in required_keys:
            assert key in result, f"Result should contain key '{key}'"
        
        # Property 5: RMSE should be non-negative
        assert result['rmse'] >= 0, "RMSE should be non-negative"
        
        # Property 6: R² should be between -inf and 1 (typically 0 to 1 for good fits)
        assert result['r2'] <= 1.0, "R² should be <= 1.0"

    @settings(max_examples=20, deadline=None)
    @given(
        # Generate synthetic data with known parameters
        mu_y_peak_true=st.floats(min_value=1.2, max_value=2.0, allow_nan=False, allow_infinity=False),
        k_mu_y_true=st.floats(min_value=0.05, max_value=0.3, allow_nan=False, allow_infinity=False),
        Ky_true=st.floats(min_value=10000.0, max_value=40000.0, allow_nan=False, allow_infinity=False),
        C_gamma_true=st.floats(min_value=0.01, max_value=0.08, allow_nan=False, allow_infinity=False),
        seed=st.integers(min_value=0, max_value=10000)
    )
    def test_property_15_lateral_optimizer_returns_valid_parameters(
        self, mu_y_peak_true, k_mu_y_true, Ky_true, C_gamma_true, seed
    ):
        """
        Property 15: Lateral Optimizer Returns Valid Parameters
        
        For any lateral fitting operation, the optimizer should return 
        parameters mu_y_peak, k_mu_y, Ky, and C_gamma where 1.0 <= mu_y_peak <= 2.5, 
        0 <= k_mu_y <= 0.5, and 5000 <= Ky <= 50000.
        
        **Validates: Requirements 8.1, 8.2, 8.3, 8.4**
        """
        # Set random seed for reproducibility
        np.random.seed(seed)
        
        # Generate synthetic data with known parameters
        n_points = 50
        sa_data = np.linspace(-0.2, 0.2, n_points)
        fz_data = np.random.uniform(700, 1300, n_points)
        camber_data = np.random.uniform(-0.05, 0.05, n_points)
        
        # Generate clean force data
        fy_clean = self.model.compute_lateral_force(
            sa_data, fz_data, camber_data,
            mu_y_peak_true, k_mu_y_true, Ky_true, C_gamma_true
        )
        
        # Add small noise (5% of mean force magnitude)
        noise_level = 0.05 * np.mean(np.abs(fy_clean))
        fy_data = fy_clean + np.random.normal(0, noise_level, n_points)
        
        # Fit parameters
        result = self.optimizer.fit_lateral(sa_data, fy_data, fz_data, camber_data)
        
        # Property 1: mu_y_peak should be within bounds [1.0, 2.5]
        assert 1.0 <= result['mu_y_peak'] <= 2.5, \
            f"mu_y_peak={result['mu_y_peak']} should be in [1.0, 2.5]"
        
        # Property 2: k_mu_y should be within bounds [0.0, 0.5]
        assert 0.0 <= result['k_mu_y'] <= 0.5, \
            f"k_mu_y={result['k_mu_y']} should be in [0.0, 0.5]"
        
        # Property 3: Ky should be within bounds [5000, 50000]
        assert 5000.0 <= result['Ky'] <= 50000.0, \
            f"Ky={result['Ky']} should be in [5000, 50000]"
        
        # Property 4: C_gamma should be within bounds [0.0, 0.15] (updated based on validation analysis)
        assert 0.0 <= result['C_gamma'] <= 0.15, \
            f"C_gamma={result['C_gamma']} should be in [0.0, 0.15]"
        
        # Property 5: Result should contain required keys
        required_keys = ['mu_y_peak', 'k_mu_y', 'Ky', 'C_gamma', 'rmse', 'r2', 'success', 'message']
        for key in required_keys:
            assert key in result, f"Result should contain key '{key}'"
        
        # Property 6: RMSE should be non-negative
        assert result['rmse'] >= 0, "RMSE should be non-negative"
        
        # Property 7: R² should be between -inf and 1 (typically 0 to 1 for good fits)
        assert result['r2'] <= 1.0, "R² should be <= 1.0"
