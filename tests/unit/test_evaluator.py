"""
Unit tests for FitEvaluator

These tests verify specific examples and edge cases for the evaluator.
"""

import pytest
import numpy as np
from pathlib import Path
import tempfile
import os
from tire_fitting.evaluator import FitEvaluator


class TestFitEvaluator:
    """Unit tests for FitEvaluator"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.evaluator = FitEvaluator()
    
    def test_calculate_metrics_perfect_fit(self):
        """Test metrics calculation with perfect fit (y_true == y_pred)"""
        y_true = np.array([100, 200, 300, 400, 500])
        y_pred = np.array([100, 200, 300, 400, 500])
        
        metrics = self.evaluator.calculate_metrics(y_true, y_pred)
        
        # Perfect fit should have RMSE = 0, R² = 1, TIC = 0
        assert metrics['rmse'] == 0.0
        assert metrics['nrmse'] == 0.0
        assert metrics['r2'] == 1.0
        assert metrics['tic'] == 0.0
    
    def test_calculate_metrics_known_values(self):
        """Test metrics calculation with known values"""
        y_true = np.array([100, 200, 300, 400, 500])
        y_pred = np.array([110, 190, 310, 390, 510])
        
        metrics = self.evaluator.calculate_metrics(y_true, y_pred)
        
        # Manually calculated expected values
        # RMSE = sqrt(mean([10^2, 10^2, 10^2, 10^2, 10^2])) = 10
        assert abs(metrics['rmse'] - 10.0) < 1e-10
        
        # NRMSE = 10 / (500 - 100) = 0.025
        assert abs(metrics['nrmse'] - 0.025) < 1e-10
        
        # R² should be very close to 1 (good fit)
        assert metrics['r2'] > 0.99
        
        # TIC should be very small (good fit)
        assert metrics['tic'] < 0.02  # Adjusted threshold
    
    def test_calculate_peak_friction_error_zero_error(self):
        """Test peak friction error with perfect prediction"""
        force_data = np.array([100, 200, 300, 200, 100])
        fz_data = np.array([1000, 1000, 1000, 1000, 1000])
        force_pred = np.array([100, 200, 300, 200, 100])
        
        error = self.evaluator.calculate_peak_friction_error(
            force_data, fz_data, force_pred
        )
        
        # Perfect prediction should have 0% error
        assert error == 0.0
    
    def test_calculate_peak_friction_error_known_value(self):
        """Test peak friction error with known value"""
        # Peak at index 2: mu = 300/1000 = 0.3
        force_data = np.array([100, 200, 300, 200, 100])
        fz_data = np.array([1000, 1000, 1000, 1000, 1000])
        # Predicted peak: mu = 270/1000 = 0.27
        force_pred = np.array([90, 180, 270, 180, 90])
        
        error = self.evaluator.calculate_peak_friction_error(
            force_data, fz_data, force_pred
        )
        
        # Expected error: |0.3 - 0.27| / 0.3 * 100 = 10%
        assert abs(error - 10.0) < 1e-10
    
    def test_calculate_peak_friction_error_zero_forces(self):
        """Test peak friction error with all zero forces"""
        force_data = np.array([0, 0, 0, 0, 0])
        fz_data = np.array([1000, 1000, 1000, 1000, 1000])
        force_pred = np.array([0, 0, 0, 0, 0])
        
        error = self.evaluator.calculate_peak_friction_error(
            force_data, fz_data, force_pred
        )
        
        # Should return 0 for zero forces
        assert error == 0.0
    
    def test_plot_force_vs_slip_creates_file(self):
        """Test that plot_force_vs_slip creates a PNG file"""
        # Create temporary directory
        with tempfile.TemporaryDirectory() as tmpdir:
            save_path = os.path.join(tmpdir, "test_plot.png")
            
            # Generate sample data
            slip_data = np.linspace(-0.2, 0.2, 50)
            force_data = 1000 * np.sin(slip_data * 5) + np.random.normal(0, 50, 50)
            force_pred = 1000 * np.sin(slip_data * 5)
            
            # Create plot
            self.evaluator.plot_force_vs_slip(
                slip_data, force_data, force_pred,
                xlabel="Slip Ratio [-]",
                ylabel="Longitudinal Force [N]",
                title="Test Plot",
                save_path=save_path
            )
            
            # Check that file was created
            assert os.path.exists(save_path)
            assert os.path.getsize(save_path) > 0
    
    def test_plot_force_vs_slip_with_load_groups(self):
        """Test plot_force_vs_slip with load groups"""
        with tempfile.TemporaryDirectory() as tmpdir:
            save_path = os.path.join(tmpdir, "test_plot_groups.png")
            
            # Generate sample data with load groups
            n_points = 60
            slip_data = np.linspace(-0.2, 0.2, n_points)
            force_data = 1000 * np.sin(slip_data * 5) + np.random.normal(0, 50, n_points)
            force_pred = 1000 * np.sin(slip_data * 5)
            
            # Create load group masks
            load_groups = {
                'Low': np.arange(n_points) < 20,
                'Medium': (np.arange(n_points) >= 20) & (np.arange(n_points) < 40),
                'High': np.arange(n_points) >= 40
            }
            
            # Create plot
            self.evaluator.plot_force_vs_slip(
                slip_data, force_data, force_pred,
                load_groups=load_groups,
                xlabel="Slip Ratio [-]",
                ylabel="Longitudinal Force [N]",
                title="Test Plot with Load Groups",
                save_path=save_path
            )
            
            # Check that file was created
            assert os.path.exists(save_path)
            assert os.path.getsize(save_path) > 0
    
    def test_plot_residuals_creates_file(self):
        """Test that plot_residuals creates a PNG file"""
        with tempfile.TemporaryDirectory() as tmpdir:
            save_path = os.path.join(tmpdir, "test_residuals.png")
            
            # Generate sample data
            y_true = np.random.normal(1000, 200, 100)
            y_pred = y_true + np.random.normal(0, 50, 100)
            
            # Create plot
            self.evaluator.plot_residuals(y_true, y_pred, save_path=save_path)
            
            # Check that file was created
            assert os.path.exists(save_path)
            assert os.path.getsize(save_path) > 0
    
    def test_metrics_with_negative_forces(self):
        """Test metrics calculation with negative forces (braking)"""
        y_true = np.array([-500, -400, -300, -200, -100])
        y_pred = np.array([-490, -410, -290, -210, -90])
        
        metrics = self.evaluator.calculate_metrics(y_true, y_pred)
        
        # Should handle negative values correctly
        assert metrics['rmse'] > 0
        assert 0 <= metrics['nrmse'] <= 1
        assert metrics['r2'] > 0.9  # Good fit
        assert metrics['tic'] < 0.1  # Small error
    
    def test_metrics_with_mixed_sign_forces(self):
        """Test metrics calculation with mixed positive and negative forces"""
        y_true = np.array([-200, -100, 0, 100, 200])
        y_pred = np.array([-190, -110, 10, 90, 210])
        
        metrics = self.evaluator.calculate_metrics(y_true, y_pred)
        
        # Should handle mixed signs correctly
        assert metrics['rmse'] > 0
        assert metrics['nrmse'] > 0
        assert metrics['r2'] > 0.9  # Good fit
        assert metrics['tic'] < 0.1  # Small error
