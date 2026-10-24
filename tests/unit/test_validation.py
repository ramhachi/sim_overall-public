"""
Unit tests for tire model fitting quality validation.

Tests the validation module's ability to check fitting quality against
thresholds and generate validation reports.

Requirements: 7.1, 7.2, 7.3, 7.4, 7.5, 7.6
"""

import pytest
from tire_fitting.validation import (
    ValidationThresholds,
    ValidationStatus,
    MetricValidation,
    DirectionValidation,
    ValidationReport,
    validate_metric,
    validate_direction,
    validate_fitting_quality,
    generate_validation_report_text
)


class TestValidationThresholds:
    """Test ValidationThresholds dataclass"""
    
    def test_default_thresholds(self):
        """Test default threshold values"""
        thresholds = ValidationThresholds()
        assert thresholds.rmse_max == 100.0
        assert thresholds.tic_max == 0.15
        assert thresholds.r2_min == 0.95
    
    def test_custom_thresholds(self):
        """Test custom threshold values"""
        thresholds = ValidationThresholds(
            rmse_max=150.0,
            tic_max=0.20,
            r2_min=0.90
        )
        assert thresholds.rmse_max == 150.0
        assert thresholds.tic_max == 0.20
        assert thresholds.r2_min == 0.90
    
    def test_invalid_rmse_max(self):
        """Test that negative RMSE max raises error"""
        with pytest.raises(ValueError, match="rmse_max must be positive"):
            ValidationThresholds(rmse_max=-10.0)
    
    def test_invalid_tic_max(self):
        """Test that TIC max outside [0,1] raises error"""
        with pytest.raises(ValueError, match="tic_max must be between 0 and 1"):
            ValidationThresholds(tic_max=1.5)
    
    def test_invalid_r2_min(self):
        """Test that R² min outside [0,1] raises error"""
        with pytest.raises(ValueError, match="r2_min must be between 0 and 1"):
            ValidationThresholds(r2_min=-0.1)


class TestValidateMetric:
    """Test validate_metric function"""
    
    def test_less_than_pass(self):
        """Test metric validation passes when value is less than threshold"""
        result = validate_metric("RMSE", 85.0, 100.0, "less_than")
        assert result.status == ValidationStatus.PASS
        assert result.actual_value == 85.0
        assert result.threshold_value == 100.0
        assert "85.0000 < 100.0000" in result.message
    
    def test_less_than_fail(self):
        """Test metric validation fails when value exceeds threshold"""
        result = validate_metric("RMSE", 120.0, 100.0, "less_than")
        assert result.status == ValidationStatus.FAIL
        assert result.actual_value == 120.0
        assert result.threshold_value == 100.0
        assert "exceeds threshold by 20.0000" in result.message
    
    def test_greater_than_pass(self):
        """Test metric validation passes when value is greater than threshold"""
        result = validate_metric("R²", 0.97, 0.95, "greater_than")
        assert result.status == ValidationStatus.PASS
        assert result.actual_value == 0.97
        assert result.threshold_value == 0.95
        assert "0.9700 > 0.9500" in result.message
    
    def test_greater_than_fail(self):
        """Test metric validation fails when value is below threshold"""
        result = validate_metric("R²", 0.92, 0.95, "greater_than")
        assert result.status == ValidationStatus.FAIL
        assert result.actual_value == 0.92
        assert result.threshold_value == 0.95
        assert "below threshold by 0.0300" in result.message
    
    def test_invalid_comparison(self):
        """Test that invalid comparison type raises error"""
        with pytest.raises(ValueError, match="Unknown comparison type"):
            validate_metric("RMSE", 85.0, 100.0, "invalid")


class TestValidateDirection:
    """Test validate_direction function"""
    
    def test_all_metrics_pass(self):
        """Test validation passes when all metrics meet thresholds"""
        metrics = {
            'rmse': 85.0,
            'tic': 0.12,
            'r2': 0.98
        }
        thresholds = ValidationThresholds()
        
        result = validate_direction('fx', 'magic_formula', metrics, thresholds)
        
        assert result.direction == 'fx'
        assert result.model_type == 'magic_formula'
        assert result.overall_status == ValidationStatus.PASS
        assert result.passed
        assert result.rmse_validation.status == ValidationStatus.PASS
        assert result.tic_validation.status == ValidationStatus.PASS
        assert result.r2_validation.status == ValidationStatus.PASS
    
    def test_rmse_fails(self):
        """Test validation fails when RMSE exceeds threshold"""
        metrics = {
            'rmse': 120.0,  # Exceeds 100.0
            'tic': 0.12,
            'r2': 0.98
        }
        thresholds = ValidationThresholds()
        
        result = validate_direction('fx', 'tmeasy', metrics, thresholds)
        
        assert result.overall_status == ValidationStatus.FAIL
        assert not result.passed
        assert result.rmse_validation.status == ValidationStatus.FAIL
        assert result.tic_validation.status == ValidationStatus.PASS
        assert result.r2_validation.status == ValidationStatus.PASS
    
    def test_tic_fails(self):
        """Test validation fails when TIC exceeds threshold"""
        metrics = {
            'rmse': 85.0,
            'tic': 0.20,  # Exceeds 0.15
            'r2': 0.98
        }
        thresholds = ValidationThresholds()
        
        result = validate_direction('fy', 'tmeasy', metrics, thresholds)
        
        assert result.overall_status == ValidationStatus.FAIL
        assert not result.passed
        assert result.rmse_validation.status == ValidationStatus.PASS
        assert result.tic_validation.status == ValidationStatus.FAIL
        assert result.r2_validation.status == ValidationStatus.PASS
    
    def test_r2_fails(self):
        """Test validation fails when R² is below threshold"""
        metrics = {
            'rmse': 85.0,
            'tic': 0.12,
            'r2': 0.92  # Below 0.95
        }
        thresholds = ValidationThresholds()
        
        result = validate_direction('fx', 'magic_formula', metrics, thresholds)
        
        assert result.overall_status == ValidationStatus.FAIL
        assert not result.passed
        assert result.rmse_validation.status == ValidationStatus.PASS
        assert result.tic_validation.status == ValidationStatus.PASS
        assert result.r2_validation.status == ValidationStatus.FAIL
    
    def test_multiple_metrics_fail(self):
        """Test validation fails when multiple metrics fail"""
        metrics = {
            'rmse': 120.0,  # Exceeds 100.0
            'tic': 0.20,    # Exceeds 0.15
            'r2': 0.92      # Below 0.95
        }
        thresholds = ValidationThresholds()
        
        result = validate_direction('fy', 'tmeasy', metrics, thresholds)
        
        assert result.overall_status == ValidationStatus.FAIL
        assert not result.passed
        assert result.rmse_validation.status == ValidationStatus.FAIL
        assert result.tic_validation.status == ValidationStatus.FAIL
        assert result.r2_validation.status == ValidationStatus.FAIL


class TestValidateFittingQuality:
    """Test validate_fitting_quality function"""
    
    def test_both_directions_pass(self):
        """Test validation passes when both directions meet thresholds"""
        fitting_results = {
            'fx': {
                'model_type': 'magic_formula',
                'metrics': {'rmse': 85.0, 'tic': 0.12, 'r2': 0.98}
            },
            'fy': {
                'model_type': 'tmeasy',
                'metrics': {'rmse': 95.0, 'tic': 0.10, 'r2': 0.97}
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        
        assert report.passed
        assert report.overall_status == ValidationStatus.PASS
        assert report.fx_validation.passed
        assert report.fy_validation.passed
        assert "All validations passed" in report.summary
    
    def test_fx_fails(self):
        """Test validation fails when FX fails"""
        fitting_results = {
            'fx': {
                'model_type': 'tmeasy',
                'metrics': {'rmse': 120.0, 'tic': 0.12, 'r2': 0.98}
            },
            'fy': {
                'model_type': 'tmeasy',
                'metrics': {'rmse': 95.0, 'tic': 0.10, 'r2': 0.97}
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        
        assert not report.passed
        assert report.overall_status == ValidationStatus.FAIL
        assert not report.fx_validation.passed
        assert report.fy_validation.passed
        assert "FX" in report.summary
    
    def test_fy_fails(self):
        """Test validation fails when FY fails"""
        fitting_results = {
            'fx': {
                'model_type': 'magic_formula',
                'metrics': {'rmse': 85.0, 'tic': 0.12, 'r2': 0.98}
            },
            'fy': {
                'model_type': 'tmeasy',
                'metrics': {'rmse': 95.0, 'tic': 0.20, 'r2': 0.97}
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        
        assert not report.passed
        assert report.overall_status == ValidationStatus.FAIL
        assert report.fx_validation.passed
        assert not report.fy_validation.passed
        assert "FY" in report.summary
    
    def test_both_directions_fail(self):
        """Test validation fails when both directions fail"""
        fitting_results = {
            'fx': {
                'model_type': 'tmeasy',
                'metrics': {'rmse': 120.0, 'tic': 0.12, 'r2': 0.98}
            },
            'fy': {
                'model_type': 'tmeasy',
                'metrics': {'rmse': 95.0, 'tic': 0.20, 'r2': 0.97}
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        
        assert not report.passed
        assert report.overall_status == ValidationStatus.FAIL
        assert not report.fx_validation.passed
        assert not report.fy_validation.passed
        assert "FX" in report.summary and "FY" in report.summary
    
    def test_only_fx_provided(self):
        """Test validation with only FX results"""
        fitting_results = {
            'fx': {
                'model_type': 'magic_formula',
                'metrics': {'rmse': 85.0, 'tic': 0.12, 'r2': 0.98}
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        
        assert report.passed
        assert report.overall_status == ValidationStatus.PASS
        assert report.fx_validation is not None
        assert report.fy_validation is None
    
    def test_only_fy_provided(self):
        """Test validation with only FY results"""
        fitting_results = {
            'fy': {
                'model_type': 'tmeasy',
                'metrics': {'rmse': 95.0, 'tic': 0.10, 'r2': 0.97}
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        
        assert report.passed
        assert report.overall_status == ValidationStatus.PASS
        assert report.fx_validation is None
        assert report.fy_validation is not None
    
    def test_no_results_provided(self):
        """Test validation with no results"""
        fitting_results = {}
        
        report = validate_fitting_quality(fitting_results)
        
        assert report.overall_status == ValidationStatus.WARNING
        assert "No fitting results" in report.summary
        assert report.fx_validation is None
        assert report.fy_validation is None
    
    def test_custom_thresholds(self):
        """Test validation with custom thresholds"""
        fitting_results = {
            'fx': {
                'model_type': 'magic_formula',
                'metrics': {'rmse': 120.0, 'tic': 0.18, 'r2': 0.92}
            }
        }
        
        # With default thresholds, this should fail
        report_default = validate_fitting_quality(fitting_results)
        assert not report_default.passed
        
        # With relaxed thresholds, this should pass
        relaxed_thresholds = ValidationThresholds(
            rmse_max=150.0,
            tic_max=0.20,
            r2_min=0.90
        )
        report_relaxed = validate_fitting_quality(fitting_results, relaxed_thresholds)
        assert report_relaxed.passed
    
    def test_get_failed_metrics(self):
        """Test getting list of failed metrics"""
        fitting_results = {
            'fx': {
                'model_type': 'tmeasy',
                'metrics': {'rmse': 120.0, 'tic': 0.12, 'r2': 0.92}
            },
            'fy': {
                'model_type': 'tmeasy',
                'metrics': {'rmse': 95.0, 'tic': 0.20, 'r2': 0.97}
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        failed_metrics = report.get_failed_metrics()
        
        assert len(failed_metrics) == 3
        assert any("FX RMSE" in m for m in failed_metrics)
        assert any("FX R²" in m for m in failed_metrics)
        assert any("FY TIC" in m for m in failed_metrics)


class TestGenerateValidationReportText:
    """Test generate_validation_report_text function"""
    
    def test_report_text_format(self):
        """Test that report text is properly formatted"""
        fitting_results = {
            'fx': {
                'model_type': 'magic_formula',
                'metrics': {'rmse': 85.0, 'tic': 0.12, 'r2': 0.98}
            },
            'fy': {
                'model_type': 'tmeasy',
                'metrics': {'rmse': 95.0, 'tic': 0.10, 'r2': 0.97}
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        text = generate_validation_report_text(report)
        
        # Check that key sections are present
        assert "TIRE MODEL FITTING QUALITY VALIDATION REPORT" in text
        assert "Validation Thresholds:" in text
        assert "Longitudinal Force (FX):" in text
        assert "Lateral Force (FY):" in text
        assert "OVERALL STATUS:" in text
        assert "magic_formula" in text
        assert "tmeasy" in text
    
    def test_report_text_with_failures(self):
        """Test that failed metrics are listed in report"""
        fitting_results = {
            'fx': {
                'model_type': 'tmeasy',
                'metrics': {'rmse': 120.0, 'tic': 0.12, 'r2': 0.92}
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        text = generate_validation_report_text(report)
        
        # Check that failed metrics section is present
        assert "Failed Metrics:" in text
        assert "FX RMSE" in text
        assert "FX R²" in text
    
    def test_report_text_all_pass(self):
        """Test that no failed metrics section when all pass"""
        fitting_results = {
            'fx': {
                'model_type': 'magic_formula',
                'metrics': {'rmse': 85.0, 'tic': 0.12, 'r2': 0.98}
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        text = generate_validation_report_text(report)
        
        # Check that failed metrics section is not present
        assert "Failed Metrics:" not in text
        assert "PASS" in text
