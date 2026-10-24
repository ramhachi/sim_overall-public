"""
Integration tests for validation module with actual fitting results.

Tests the validation module's integration with the fitting pipeline
and ensures it correctly validates real fitting results.

Requirements: 7.1, 7.2, 7.3, 7.4, 7.5, 7.6
"""

import pytest
import numpy as np
from pathlib import Path
from tire_fitting.validation import (
    ValidationThresholds,
    ValidationStatus,
    validate_fitting_quality,
    generate_validation_report_text
)
from tire_fitting.optimizer import ModelFitter
from tire_fitting.data_loader import TireDataLoader
from tire_fitting.preprocessor import TireDataPreprocessor


class TestValidationIntegration:
    """Integration tests for validation with real fitting results"""
    
    @pytest.fixture
    def data_paths(self):
        """Get paths to test data files"""
        data_dir = Path("data/tiredata")
        return {
            'drivebrake': data_dir / "DriveBrake-Hoosier18.0×6.0-10-R20.csv",
            'cornering': data_dir / "Cornering-Hoosier18.0×6.0-10-R20.csv"
        }
    
    @pytest.fixture
    def fitting_results_mock(self):
        """Mock fitting results for testing"""
        return {
            'fx': {
                'model_type': 'magic_formula',
                'parameters': {
                    'pDx1': 2.45,
                    'pCx1': 1.65,
                    'pEx1': -0.12,
                    'pKx1': 21.3,
                    'pKx2': -1.2,
                    'pKx3': 0.15,
                    'fz0': 1200.0
                },
                'metrics': {
                    'rmse': 85.3,
                    'tic': 0.12,
                    'r2': 0.9845,
                    'peak_error': 245.7
                }
            },
            'fy': {
                'model_type': 'tmeasy',
                'parameters': {
                    'mu_y_peak': 1.85,
                    'k_mu_y': 0.15,
                    'Ky': 25000.0,
                    'C_gamma': 0.05,
                    'fz_nominal': 1200.0
                },
                'metrics': {
                    'rmse': 119.2,
                    'tic': 0.040,
                    'r2': 0.9937,
                    'peak_error': 312.5
                }
            }
        }
    
    def test_validate_good_fitting_results(self, fitting_results_mock):
        """Test validation passes with good fitting results"""
        # Modify to ensure all metrics pass
        fitting_results_mock['fx']['metrics']['rmse'] = 85.0
        fitting_results_mock['fy']['metrics']['rmse'] = 95.0
        
        report = validate_fitting_quality(fitting_results_mock)
        
        assert report.passed
        assert report.overall_status == ValidationStatus.PASS
        assert report.fx_validation.passed
        assert report.fy_validation.passed
    
    def test_validate_poor_fitting_results(self, fitting_results_mock):
        """Test validation fails with poor fitting results"""
        # Make metrics fail thresholds
        fitting_results_mock['fx']['metrics']['rmse'] = 250.0  # Exceeds 100
        fitting_results_mock['fy']['metrics']['tic'] = 0.25    # Exceeds 0.15
        
        report = validate_fitting_quality(fitting_results_mock)
        
        assert not report.passed
        assert report.overall_status == ValidationStatus.FAIL
        assert not report.fx_validation.passed
        assert not report.fy_validation.passed
    
    def test_validate_with_custom_thresholds(self, fitting_results_mock):
        """Test validation with custom thresholds"""
        # Use strict thresholds
        strict_thresholds = ValidationThresholds(
            rmse_max=80.0,
            tic_max=0.10,
            r2_min= 0.99
        )
        
        report = validate_fitting_quality(fitting_results_mock, strict_thresholds)
        
        # Should fail with strict thresholds
        assert not report.passed
    
    def test_validation_report_generation(self, fitting_results_mock):
        """Test that validation report text is generated correctly"""
        report = validate_fitting_quality(fitting_results_mock)
        text = generate_validation_report_text(report)
        
        # Verify report contains key information
        assert "TIRE MODEL FITTING QUALITY VALIDATION REPORT" in text
        assert "magic_formula" in text
        assert "tmeasy" in text
        assert "85.30 N" in text or "85.3 N" in text
        assert "119.20 N" in text or "119.2 N" in text
    
    def test_validation_with_missing_direction(self, fitting_results_mock):
        """Test validation when one direction is missing"""
        # Remove FY results
        del fitting_results_mock['fy']
        
        report = validate_fitting_quality(fitting_results_mock)
        
        assert report.fx_validation is not None
        assert report.fy_validation is None
        # Overall status depends on FX validation
        if fitting_results_mock['fx']['metrics']['rmse'] < 100.0:
            assert report.overall_status == ValidationStatus.PASS
    
    def test_validation_logs_warnings(self, fitting_results_mock, caplog):
        """Test that validation logs warnings for threshold violations"""
        import logging
        caplog.set_level(logging.WARNING)
        
        # Make FX fail
        fitting_results_mock['fx']['metrics']['rmse'] = 150.0
        
        report = validate_fitting_quality(fitting_results_mock)
        
        # Check that warnings were logged
        assert any("FAILED" in record.message for record in caplog.records)
        assert any("RMSE" in record.message for record in caplog.records)
    
    def test_validation_boundary_cases(self):
        """Test validation at exact threshold boundaries"""
        # Test at exact thresholds
        fitting_results = {
            'fx': {
                'model_type': 'tmeasy',
                'metrics': {
                    'rmse': 100.0,  # Exactly at threshold
                    'tic': 0.15,    # Exactly at threshold
                    'r2': 0.95      # Exactly at threshold
                }
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        
        # At exact threshold, should fail (< not <=)
        assert not report.passed
    
    def test_validation_with_nan_metrics(self):
        """Test validation handles NaN metrics gracefully"""
        fitting_results = {
            'fx': {
                'model_type': 'tmeasy',
                'metrics': {
                    'rmse': float('nan'),
                    'tic': 0.12,
                    'r2': 0.98
                }
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        
        # NaN should fail validation
        assert not report.passed
    
    def test_validation_with_inf_metrics(self):
        """Test validation handles Inf metrics gracefully"""
        fitting_results = {
            'fx': {
                'model_type': 'tmeasy',
                'metrics': {
                    'rmse': float('inf'),
                    'tic': 0.12,
                    'r2': 0.98
                }
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        
        # Inf should fail validation
        assert not report.passed


class TestValidationWithRealData:
    """Test validation with real tire data (if available)"""
    
    @pytest.fixture
    def data_available(self):
        """Check if test data is available"""
        data_dir = Path("data/tiredata")
        drivebrake = data_dir / "DriveBrake-Hoosier18.0×6.0-10-R20.csv"
        cornering = data_dir / "Cornering-Hoosier18.0×6.0-10-R20.csv"
        return drivebrake.exists() and cornering.exists()
    
    @pytest.mark.skipif(
        not Path("data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv").exists(),
        reason="Test data not available"
    )
    def test_validation_with_actual_fitting(self):
        """Test validation with actual fitting results (slow test)"""
        # This is a slow test that actually fits models
        # Skip in normal test runs
        pytest.skip("Slow test - run manually if needed")
        
        # Load data
        data_loader = TireDataLoader()
        preprocessor = TireDataPreprocessor()
        
        drivebrake_path = "data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv"
        cornering_path = "data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv"
        
        # Fit models
        fitter = ModelFitter(data_loader, preprocessor)
        results = fitter.fit_multiple_models(
            model_types=['tmeasy'],
            directions=['fx', 'fy'],
            drivebrake_file=drivebrake_path,
            cornering_file=cornering_path
        )
        
        # Validate results
        fitting_results = {
            'fx': {
                'model_type': 'tmeasy',
                'metrics': results['tmeasy']['fx']['metrics']
            },
            'fy': {
                'model_type': 'tmeasy',
                'metrics': results['tmeasy']['fy']['metrics']
            }
        }
        
        report = validate_fitting_quality(fitting_results)
        
        # Check that report was generated
        assert report is not None
        assert report.fx_validation is not None
        assert report.fy_validation is not None
