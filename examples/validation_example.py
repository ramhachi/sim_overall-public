"""
Example script demonstrating tire model fitting quality validation.

This script shows how to use the validation module to check fitting quality
and generate validation reports.

Requirements: 7.1, 7.2, 7.3, 7.4, 7.5, 7.6
"""

import logging
from tire_fitting.validation import (
    ValidationThresholds,
    validate_fitting_quality,
    generate_validation_report_text
)


# Configure logging to see validation messages
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s: %(message)s'
)


def example_good_fitting():
    """Example with good fitting results that pass validation"""
    print("\n" + "=" * 60)
    print("EXAMPLE 1: Good Fitting Results")
    print("=" * 60 + "\n")
    
    fitting_results = {
        'fx': {
            'model_type': 'magic_formula',
            'metrics': {
                'rmse': 85.3,
                'tic': 0.12,
                'r2': 0.9845,
                'peak_error': 245.7
            }
        },
        'fy': {
            'model_type': 'tmeasy',
            'metrics': {
                'rmse': 95.2,
                'tic': 0.040,
                'r2': 0.9937,
                'peak_error': 312.5
            }
        }
    }
    
    report = validate_fitting_quality(fitting_results)
    
    print("\nValidation Result:")
    print(f"  Overall Status: {report.overall_status.value.upper()}")
    print(f"  FX Passed: {report.fx_validation.passed}")
    print(f"  FY Passed: {report.fy_validation.passed}")
    
    # Generate text report
    text_report = generate_validation_report_text(report)
    print("\n" + text_report)


def example_poor_fitting():
    """Example with poor fitting results that fail validation"""
    print("\n" + "=" * 60)
    print("EXAMPLE 2: Poor Fitting Results")
    print("=" * 60 + "\n")
    
    fitting_results = {
        'fx': {
            'model_type': 'tmeasy',
            'metrics': {
                'rmse': 250.0,  # Exceeds threshold
                'tic': 0.25,    # Exceeds threshold
                'r2': 0.85,     # Below threshold
                'peak_error': 500.0
            }
        },
        'fy': {
            'model_type': 'tmeasy',
            'metrics': {
                'rmse': 180.0,  # Exceeds threshold
                'tic': 0.18,    # Exceeds threshold
                'r2': 0.92,     # Below threshold
                'peak_error': 450.0
            }
        }
    }
    
    report = validate_fitting_quality(fitting_results)
    
    print("\nValidation Result:")
    print(f"  Overall Status: {report.overall_status.value.upper()}")
    print(f"  FX Passed: {report.fx_validation.passed}")
    print(f"  FY Passed: {report.fy_validation.passed}")
    
    # Get failed metrics
    failed_metrics = report.get_failed_metrics()
    print(f"\nFailed Metrics ({len(failed_metrics)}):")
    for metric in failed_metrics:
        print(f"  - {metric}")


def example_custom_thresholds():
    """Example with custom validation thresholds"""
    print("\n" + "=" * 60)
    print("EXAMPLE 3: Custom Validation Thresholds")
    print("=" * 60 + "\n")
    
    fitting_results = {
        'fx': {
            'model_type': 'magic_formula',
            'metrics': {
                'rmse': 120.0,
                'tic': 0.18,
                'r2': 0.92,
                'peak_error': 300.0
            }
        }
    }
    
    # Default thresholds - should fail
    print("With Default Thresholds:")
    report_default = validate_fitting_quality(fitting_results)
    print(f"  Status: {report_default.overall_status.value.upper()}")
    
    # Relaxed thresholds - should pass
    print("\nWith Relaxed Thresholds:")
    relaxed_thresholds = ValidationThresholds(
        rmse_max=150.0,
        tic_max=0.20,
        r2_min=0.90
    )
    report_relaxed = validate_fitting_quality(fitting_results, relaxed_thresholds)
    print(f"  Status: {report_relaxed.overall_status.value.upper()}")
    
    # Strict thresholds - should fail
    print("\nWith Strict Thresholds:")
    strict_thresholds = ValidationThresholds(
        rmse_max=80.0,
        tic_max=0.10,
        r2_min=0.98
    )
    report_strict = validate_fitting_quality(fitting_results, strict_thresholds)
    print(f"  Status: {report_strict.overall_status.value.upper()}")


def example_hybrid_model():
    """Example with hybrid model (different models for FX and FY)"""
    print("\n" + "=" * 60)
    print("EXAMPLE 4: Hybrid Model Validation")
    print("=" * 60 + "\n")
    
    fitting_results = {
        'fx': {
            'model_type': 'magic_formula',
            'metrics': {
                'rmse': 85.3,
                'tic': 0.12,
                'r2': 0.9845,
                'peak_error': 245.7
            }
        },
        'fy': {
            'model_type': 'tmeasy',
            'metrics': {
                'rmse': 119.2,
                'tic': 0.040,
                'r2': 0.9937,
                'peak_error': 312.5
            }
        }
    }
    
    report = validate_fitting_quality(fitting_results)
    
    print(f"FX Model: {report.fx_validation.model_type}")
    print(f"  RMSE: {report.fx_validation.rmse_validation.actual_value:.2f} N "
          f"[{report.fx_validation.rmse_validation.status.value.upper()}]")
    print(f"  TIC:  {report.fx_validation.tic_validation.actual_value:.4f} "
          f"[{report.fx_validation.tic_validation.status.value.upper()}]")
    print(f"  R²:   {report.fx_validation.r2_validation.actual_value:.4f} "
          f"[{report.fx_validation.r2_validation.status.value.upper()}]")
    
    print(f"\nFY Model: {report.fy_validation.model_type}")
    print(f"  RMSE: {report.fy_validation.rmse_validation.actual_value:.2f} N "
          f"[{report.fy_validation.rmse_validation.status.value.upper()}]")
    print(f"  TIC:  {report.fy_validation.tic_validation.actual_value:.4f} "
          f"[{report.fy_validation.tic_validation.status.value.upper()}]")
    print(f"  R²:   {report.fy_validation.r2_validation.actual_value:.4f} "
          f"[{report.fy_validation.r2_validation.status.value.upper()}]")
    
    print(f"\nOverall Status: {report.overall_status.value.upper()}")


if __name__ == "__main__":
    # Run all examples
    example_good_fitting()
    example_poor_fitting()
    example_custom_thresholds()
    example_hybrid_model()
    
    print("\n" + "=" * 60)
    print("All examples completed!")
    print("=" * 60)
