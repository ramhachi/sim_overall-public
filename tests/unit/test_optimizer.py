"""
Unit tests for TireParameterOptimizer

These tests verify specific examples and edge cases for the optimizer.
"""

import pytest
import numpy as np
from tire_fitting.tmeasy_model import TMeasyModel
from tire_fitting.optimizer import TireParameterOptimizer


class TestTireParameterOptimizer:
    """Unit tests for TireParameterOptimizer"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.model = TMeasyModel(fz_nominal=1000.0)
        self.optimizer = TireParameterOptimizer(self.model)
    
    def test_optimizer_initialization(self):
        """Test that optimizer initializes with correct bounds"""
        # Check longitudinal bounds (updated based on validation analysis)
        assert self.optimizer.longitudinal_bounds['mu_x_peak'] == (1.3, 2.5)
        assert self.optimizer.longitudinal_bounds['k_mu_x'] == (0.05, 0.5)
        assert self.optimizer.longitudinal_bounds['Kx'] == (15000.0, 50000.0)
        
        # Check lateral bounds (updated based on validation analysis)
        assert self.optimizer.lateral_bounds['mu_y_peak'] == (1.3, 2.5)
        assert self.optimizer.lateral_bounds['k_mu_y'] == (0.05, 0.5)
        assert self.optimizer.lateral_bounds['Ky'] == (10000.0, 50000.0)
        assert self.optimizer.lateral_bounds['C_gamma'] == (0.0, 0.15)
    
    def test_fit_longitudinal_with_synthetic_data(self):
        """Test longitudinal fitting with synthetic data"""
        # Generate synthetic data with known parameters
        np.random.seed(42)
        mu_x_peak_true = 1.5
        k_mu_x_true = 0.2
        Kx_true = 20000.0
        
        n_points = 100
        sr_data = np.linspace(-0.2, 0.2, n_points)
        fz_data = np.random.uniform(700, 1300, n_points)
        
        # Generate clean force data
        fx_data = self.model.compute_longitudinal_force(
            sr_data, fz_data, mu_x_peak_true, k_mu_x_true, Kx_true
        )
        
        # Fit parameters
        result = self.optimizer.fit_longitudinal(sr_data, fx_data, fz_data)
        
        # Check that parameters are close to true values (within 10%)
        assert abs(result['mu_x_peak'] - mu_x_peak_true) / mu_x_peak_true < 0.1
        assert abs(result['k_mu_x'] - k_mu_x_true) / (k_mu_x_true + 0.01) < 0.2
        assert abs(result['Kx'] - Kx_true) / Kx_true < 0.1
        
        # Check that RMSE is very small (clean data)
        assert result['rmse'] < 10.0  # Should be near zero for clean data
        
        # Check that R² is very high
        assert result['r2'] > 0.99
        
        # Check that optimization succeeded
        assert result['success']
    
    def test_fit_lateral_with_synthetic_data(self):
        """Test lateral fitting with synthetic data"""
        # Generate synthetic data with known parameters
        np.random.seed(42)
        mu_y_peak_true = 1.6
        k_mu_y_true = 0.15
        Ky_true = 25000.0
        C_gamma_true = 0.05
        
        n_points = 100
        sa_data = np.linspace(-0.2, 0.2, n_points)
        fz_data = np.random.uniform(700, 1300, n_points)
        camber_data = np.random.uniform(-0.05, 0.05, n_points)
        
        # Generate clean force data
        fy_data = self.model.compute_lateral_force(
            sa_data, fz_data, camber_data,
            mu_y_peak_true, k_mu_y_true, Ky_true, C_gamma_true
        )
        
        # Fit parameters
        result = self.optimizer.fit_lateral(sa_data, fy_data, fz_data, camber_data)
        
        # Check that parameters are close to true values (within 10%)
        assert abs(result['mu_y_peak'] - mu_y_peak_true) / mu_y_peak_true < 0.1
        assert abs(result['k_mu_y'] - k_mu_y_true) / (k_mu_y_true + 0.01) < 0.2
        assert abs(result['Ky'] - Ky_true) / Ky_true < 0.1
        assert abs(result['C_gamma'] - C_gamma_true) / (C_gamma_true + 0.01) < 0.2
        
        # Check that RMSE is very small (clean data)
        assert result['rmse'] < 10.0  # Should be near zero for clean data
        
        # Check that R² is very high
        assert result['r2'] > 0.99
        
        # Check that optimization succeeded
        assert result['success']
    
    def test_fit_longitudinal_insufficient_data(self):
        """Test that fitting raises error with insufficient data"""
        # Create data with only 5 points (< 10 required)
        sr_data = np.array([0.0, 0.05, 0.1, 0.15, 0.2])
        fx_data = np.array([0.0, 100.0, 200.0, 300.0, 400.0])
        fz_data = np.array([1000.0, 1000.0, 1000.0, 1000.0, 1000.0])
        
        # Should raise ValueError
        with pytest.raises(ValueError, match="Insufficient data"):
            self.optimizer.fit_longitudinal(sr_data, fx_data, fz_data)
    
    def test_fit_lateral_insufficient_data(self):
        """Test that fitting raises error with insufficient data"""
        # Create data with only 5 points (< 10 required)
        sa_data = np.array([0.0, 0.05, 0.1, 0.15, 0.2])
        fy_data = np.array([0.0, 100.0, 200.0, 300.0, 400.0])
        fz_data = np.array([1000.0, 1000.0, 1000.0, 1000.0, 1000.0])
        camber_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Should raise ValueError
        with pytest.raises(ValueError, match="Insufficient data"):
            self.optimizer.fit_lateral(sa_data, fy_data, fz_data, camber_data)
    
    def test_fit_longitudinal_with_custom_bounds(self):
        """Test longitudinal fitting with custom parameter bounds"""
        # Generate synthetic data
        np.random.seed(42)
        n_points = 50
        sr_data = np.linspace(-0.15, 0.15, n_points)
        fz_data = np.random.uniform(800, 1200, n_points)
        fx_data = self.model.compute_longitudinal_force(
            sr_data, fz_data, mu_x_peak=1.8, k_mu_x=0.1, Kx=15000.0
        )
        
        # Define custom bounds
        custom_bounds = {
            'mu_x_peak': (1.5, 2.0),
            'k_mu_x': (0.0, 0.3),
            'Kx': (10000.0, 30000.0)
        }
        
        # Fit with custom bounds
        result = self.optimizer.fit_longitudinal(sr_data, fx_data, fz_data, bounds=custom_bounds)
        
        # Check that parameters respect custom bounds
        assert 1.5 <= result['mu_x_peak'] <= 2.0
        assert 0.0 <= result['k_mu_x'] <= 0.3
        assert 10000.0 <= result['Kx'] <= 30000.0
    
    def test_fit_lateral_with_custom_bounds(self):
        """Test lateral fitting with custom parameter bounds"""
        # Generate synthetic data
        np.random.seed(42)
        n_points = 50
        sa_data = np.linspace(-0.15, 0.15, n_points)
        fz_data = np.random.uniform(800, 1200, n_points)
        camber_data = np.random.uniform(-0.03, 0.03, n_points)
        fy_data = self.model.compute_lateral_force(
            sa_data, fz_data, camber_data,
            mu_y_peak=1.7, k_mu_y=0.1, Ky=18000.0, C_gamma=0.04
        )
        
        # Define custom bounds
        custom_bounds = {
            'mu_y_peak': (1.5, 2.0),
            'k_mu_y': (0.0, 0.3),
            'Ky': (10000.0, 30000.0),
            'C_gamma': (0.0, 0.08)
        }
        
        # Fit with custom bounds
        result = self.optimizer.fit_lateral(sa_data, fy_data, fz_data, camber_data, bounds=custom_bounds)
        
        # Check that parameters respect custom bounds
        assert 1.5 <= result['mu_y_peak'] <= 2.0
        assert 0.0 <= result['k_mu_y'] <= 0.3
        assert 10000.0 <= result['Ky'] <= 30000.0
        assert 0.0 <= result['C_gamma'] <= 0.08
