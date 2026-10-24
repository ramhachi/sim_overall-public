"""
Property-based tests for FitEvaluator

These tests use the Hypothesis library to verify universal properties
across many randomly generated inputs for the evaluation metrics.
"""

import pytest
import numpy as np
from hypothesis import given, strategies as st, settings, assume
from tire_fitting.evaluator import FitEvaluator


class TestFitEvaluatorProperties:
    """Property-based tests for FitEvaluator"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.evaluator = FitEvaluator()
    
    @settings(max_examples=100)
    @given(
        y_true=st.lists(
            st.floats(min_value=-2000.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
            min_size=10,
            max_size=100
        ),
        y_pred=st.lists(
            st.floats(min_value=-2000.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
            min_size=10,
            max_size=100
        )
    )
    def test_property_16_rmse_calculation_formula(self, y_true, y_pred):
        """
        Property 16: RMSE Calculation Formula
        
        For any arrays of measured forces y_true and predicted forces y_pred, 
        the RMSE should equal sqrt(mean((y_true - y_pred)^2)).
        
        **Validates: Requirements 9.1**
        """
        # Ensure arrays have same length
        min_len = min(len(y_true), len(y_pred))
        y_true_array = np.array(y_true[:min_len])
        y_pred_array = np.array(y_pred[:min_len])
        
        # Calculate metrics
        metrics = self.evaluator.calculate_metrics(y_true_array, y_pred_array)
        rmse = metrics['rmse']
        
        # Expected RMSE
        expected_rmse = np.sqrt(np.mean((y_true_array - y_pred_array)**2))
        
        # Property: RMSE should match formula
        assert abs(rmse - expected_rmse) < 1e-8, \
            f"RMSE should equal sqrt(mean((y_true - y_pred)^2))"
    
    @settings(max_examples=100)
    @given(
        y_true=st.lists(
            st.floats(min_value=-2000.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
            min_size=10,
            max_size=100
        ),
        y_pred=st.lists(
            st.floats(min_value=-2000.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
            min_size=10,
            max_size=100
        )
    )
    def test_property_17_nrmse_calculation_formula(self, y_true, y_pred):
        """
        Property 17: NRMSE Calculation Formula
        
        For any RMSE value and measured forces y_true, the NRMSE should equal 
        RMSE / (max(y_true) - min(y_true)).
        
        **Validates: Requirements 9.2**
        """
        # Ensure arrays have same length
        min_len = min(len(y_true), len(y_pred))
        y_true_array = np.array(y_true[:min_len])
        y_pred_array = np.array(y_pred[:min_len])
        
        # Skip if range is too small (would cause division issues)
        y_range = np.max(y_true_array) - np.min(y_true_array)
        assume(y_range > 1.0)
        
        # Calculate metrics
        metrics = self.evaluator.calculate_metrics(y_true_array, y_pred_array)
        rmse = metrics['rmse']
        nrmse = metrics['nrmse']
        
        # Expected NRMSE
        expected_nrmse = rmse / y_range
        
        # Property: NRMSE should match formula
        assert abs(nrmse - expected_nrmse) < 1e-8, \
            f"NRMSE should equal RMSE / (max(y_true) - min(y_true))"
    
    @settings(max_examples=100)
    @given(
        y_true=st.lists(
            st.floats(min_value=-2000.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
            min_size=10,
            max_size=100
        ),
        y_pred=st.lists(
            st.floats(min_value=-2000.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
            min_size=10,
            max_size=100
        )
    )
    def test_property_18_r_squared_calculation_formula(self, y_true, y_pred):
        """
        Property 18: R-Squared Calculation Formula
        
        For any measured forces y_true and predicted forces y_pred, R² should 
        equal 1 - sum((y_true - y_pred)^2) / sum((y_true - mean(y_true))^2).
        
        **Validates: Requirements 9.3**
        """
        # Ensure arrays have same length
        min_len = min(len(y_true), len(y_pred))
        y_true_array = np.array(y_true[:min_len])
        y_pred_array = np.array(y_pred[:min_len])
        
        # Skip if variance is too small
        ss_tot = np.sum((y_true_array - np.mean(y_true_array))**2)
        assume(ss_tot > 1.0)
        
        # Calculate metrics
        metrics = self.evaluator.calculate_metrics(y_true_array, y_pred_array)
        r2 = metrics['r2']
        
        # Expected R²
        ss_res = np.sum((y_true_array - y_pred_array)**2)
        expected_r2 = 1.0 - (ss_res / ss_tot)
        
        # Property: R² should match formula
        assert abs(r2 - expected_r2) < 1e-8, \
            f"R² should equal 1 - sum((y_true - y_pred)^2) / sum((y_true - mean(y_true))^2)"
    
    @settings(max_examples=100)
    @given(
        y_true=st.lists(
            st.floats(min_value=-2000.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
            min_size=10,
            max_size=100
        ),
        y_pred=st.lists(
            st.floats(min_value=-2000.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
            min_size=10,
            max_size=100
        )
    )
    def test_property_19_theil_inequality_coefficient_formula(self, y_true, y_pred):
        """
        Property 19: Theil Inequality Coefficient Formula
        
        For any measured forces y_true and predicted forces y_pred, TIC should 
        equal sqrt(sum((y_true - y_pred)^2)) / (sqrt(sum(y_true^2)) + sqrt(sum(y_pred^2))).
        
        **Validates: Requirements 9.4**
        """
        # Ensure arrays have same length
        min_len = min(len(y_true), len(y_pred))
        y_true_array = np.array(y_true[:min_len])
        y_pred_array = np.array(y_pred[:min_len])
        
        # Skip if denominator would be too small
        denominator = np.sqrt(np.sum(y_true_array**2)) + np.sqrt(np.sum(y_pred_array**2))
        assume(denominator > 1.0)
        
        # Calculate metrics
        metrics = self.evaluator.calculate_metrics(y_true_array, y_pred_array)
        tic = metrics['tic']
        
        # Expected TIC
        numerator = np.sqrt(np.sum((y_true_array - y_pred_array)**2))
        expected_tic = numerator / denominator
        
        # Property: TIC should match formula
        assert abs(tic - expected_tic) < 1e-8, \
            f"TIC should equal sqrt(sum((y_true - y_pred)^2)) / (sqrt(sum(y_true^2)) + sqrt(sum(y_pred^2)))"
    
    @settings(max_examples=100)
    @given(
        force_data=st.lists(
            st.floats(min_value=-2000.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
            min_size=10,
            max_size=100
        ),
        fz_data=st.lists(
            st.floats(min_value=500.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
            min_size=10,
            max_size=100
        ),
        force_pred=st.lists(
            st.floats(min_value=-2000.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
            min_size=10,
            max_size=100
        )
    )
    def test_property_20_peak_friction_error_calculation(self, force_data, fz_data, force_pred):
        """
        Property 20: Peak Friction Error Calculation
        
        For any measured peak friction mu_peak_data and predicted peak friction 
        mu_peak_pred, the peak friction error should equal 
        |mu_peak_data - mu_peak_pred| / mu_peak_data * 100.
        
        **Validates: Requirements 9.5**
        """
        # Ensure arrays have same length
        min_len = min(len(force_data), len(fz_data), len(force_pred))
        force_data_array = np.array(force_data[:min_len])
        fz_data_array = np.array(fz_data[:min_len])
        force_pred_array = np.array(force_pred[:min_len])
        
        # Skip if all forces are zero (undefined peak friction)
        mu_data = np.abs(force_data_array) / fz_data_array
        mu_peak_data = np.max(mu_data)
        assume(mu_peak_data > 0.01)  # Skip near-zero peak friction
        
        # Calculate peak friction error
        error = self.evaluator.calculate_peak_friction_error(
            force_data_array, fz_data_array, force_pred_array
        )
        
        # Manually compute expected error
        mu_pred = np.abs(force_pred_array) / fz_data_array
        
        peak_idx = np.argmax(mu_data)
        mu_peak_pred = mu_pred[peak_idx]
        
        expected_error = np.abs(mu_peak_data - mu_peak_pred) / mu_peak_data * 100.0
        
        # Property: Error should match formula
        assert abs(error - expected_error) < 1e-6, \
            f"Peak friction error should equal |mu_peak_data - mu_peak_pred| / mu_peak_data * 100"
