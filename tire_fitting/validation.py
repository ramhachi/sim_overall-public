"""
Tire Model Fitting Quality Validation Module

This module provides validation functions to ensure fitted tire models meet
quality thresholds for RMSE, TIC, and R² metrics. It generates validation
reports with pass/fail status and logs warnings for threshold violations.

Requirements: 7.1, 7.2, 7.3, 7.4, 7.5, 7.6
"""

import logging
from typing import Any, Dict, List, Optional
from dataclasses import dataclass
from enum import Enum


logger = logging.getLogger(__name__)


class ValidationStatus(Enum):
    """Validation status for quality checks"""
    PASS = "pass"
    FAIL = "fail"
    WARNING = "warning"


@dataclass
class ValidationThresholds:
    """Quality thresholds for tire model validation
    
    Requirements: 7.1, 7.2, 7.3, 7.4
    """
    rmse_max: float = 100.0      # Maximum RMSE in Newtons
    rmse_force_scale_fraction_max: float = 0.12
    nrmse_max: Optional[float] = 0.12
    tic_max: float = 0.15        # Maximum Theil Inequality Coefficient
    r2_min: float = 0.95         # Minimum R-squared coefficient
    
    def __post_init__(self):
        """Validate threshold values"""
        if self.rmse_max <= 0:
            raise ValueError("rmse_max must be positive")
        if self.rmse_force_scale_fraction_max <= 0:
            raise ValueError("rmse_force_scale_fraction_max must be positive")
        if self.nrmse_max is not None and self.nrmse_max <= 0:
            raise ValueError("nrmse_max must be positive when provided")
        if not 0 <= self.tic_max <= 1:
            raise ValueError("tic_max must be between 0 and 1")
        if not 0 <= self.r2_min <= 1:
            raise ValueError("r2_min must be between 0 and 1")

    def effective_rmse_max(self, metrics: Dict[str, float]) -> float:
        """Scale absolute RMSE allowance when a force scale is available."""
        force_scale = max(
            float(metrics.get("force_scale", 0.0) or 0.0),
            float(metrics.get("force_range", 0.0) or 0.0),
        )
        if force_scale <= 0.0:
            return self.rmse_max
        return max(self.rmse_max, self.rmse_force_scale_fraction_max * force_scale)


@dataclass
class MetricValidation:
    """Validation result for a single metric"""
    metric_name: str
    actual_value: float
    threshold_value: float
    status: ValidationStatus
    message: str


@dataclass
class DirectionValidation:
    """Validation results for one force direction"""
    direction: str  # 'fx' or 'fy'
    model_type: str
    rmse_validation: MetricValidation
    tic_validation: MetricValidation
    r2_validation: MetricValidation
    overall_status: ValidationStatus
    nrmse_validation: Optional[MetricValidation] = None
    
    @property
    def passed(self) -> bool:
        """Check if all validations passed"""
        return self.overall_status == ValidationStatus.PASS


@dataclass
class ValidationReport:
    """Complete validation report for fitted models
    
    Requirements: 7.5, 7.6
    """
    fx_validation: Optional[DirectionValidation]
    fy_validation: Optional[DirectionValidation]
    thresholds: ValidationThresholds
    overall_status: ValidationStatus
    summary: str
    
    @property
    def passed(self) -> bool:
        """Check if overall validation passed"""
        return self.overall_status == ValidationStatus.PASS
    
    def get_failed_metrics(self) -> List[str]:
        """Get list of failed metric descriptions"""
        failed = []
        
        if self.fx_validation and not self.fx_validation.passed:
            for validation in [
                self.fx_validation.rmse_validation,
                self.fx_validation.tic_validation,
                self.fx_validation.r2_validation,
                self.fx_validation.nrmse_validation,
            ]:
                if validation is not None and validation.status == ValidationStatus.FAIL:
                    failed.append(f"FX {validation.metric_name}: {validation.message}")
        
        if self.fy_validation and not self.fy_validation.passed:
            for validation in [
                self.fy_validation.rmse_validation,
                self.fy_validation.tic_validation,
                self.fy_validation.r2_validation,
                self.fy_validation.nrmse_validation,
            ]:
                if validation is not None and validation.status == ValidationStatus.FAIL:
                    failed.append(f"FY {validation.metric_name}: {validation.message}")
        
        return failed


def validate_metric(
    metric_name: str,
    actual_value: float,
    threshold_value: float,
    comparison: str = "less_than"
) -> MetricValidation:
    """
    Validate a single metric against a threshold.
    
    Args:
        metric_name: Name of the metric (e.g., 'RMSE', 'TIC', 'R²')
        actual_value: Actual metric value from fitting
        threshold_value: Threshold value for validation
        comparison: Type of comparison ('less_than' or 'greater_than')
        
    Returns:
        MetricValidation with status and message
        
    Requirements: 7.1, 7.2, 7.3, 7.4
    """
    if comparison == "less_than":
        passed = actual_value < threshold_value
        if passed:
            message = f"{actual_value:.4f} < {threshold_value:.4f}"
            status = ValidationStatus.PASS
        else:
            excess = actual_value - threshold_value
            message = f"{actual_value:.4f} exceeds threshold by {excess:.4f}"
            status = ValidationStatus.FAIL
    elif comparison == "greater_than":
        passed = actual_value > threshold_value
        if passed:
            message = f"{actual_value:.4f} > {threshold_value:.4f}"
            status = ValidationStatus.PASS
        else:
            deficit = threshold_value - actual_value
            message = f"{actual_value:.4f} below threshold by {deficit:.4f}"
            status = ValidationStatus.FAIL
    else:
        raise ValueError(f"Unknown comparison type: {comparison}")
    
    return MetricValidation(
        metric_name=metric_name,
        actual_value=actual_value,
        threshold_value=threshold_value,
        status=status,
        message=message
    )


def validate_direction(
    direction: str,
    model_type: str,
    metrics: Dict[str, float],
    thresholds: ValidationThresholds
) -> DirectionValidation:
    """
    Validate fitting quality for one force direction.
    
    Args:
        direction: Force direction ('fx' or 'fy')
        model_type: Model type ('tmeasy' or 'magic_formula')
        metrics: Dictionary with 'rmse', 'tic', 'r2' keys
        thresholds: Validation thresholds
        
    Returns:
        DirectionValidation with all metric validations
        
    Requirements: 7.1, 7.2, 7.3, 7.4, 7.5
    """
    # Validate RMSE (Requirement 7.1, 7.2)
    rmse_validation = validate_metric(
        metric_name="RMSE",
        actual_value=metrics['rmse'],
        threshold_value=thresholds.effective_rmse_max(metrics),
        comparison="less_than"
    )

    nrmse_validation = None
    if thresholds.nrmse_max is not None and 'nrmse' in metrics:
        nrmse_validation = validate_metric(
            metric_name="NRMSE",
            actual_value=metrics['nrmse'],
            threshold_value=thresholds.nrmse_max,
            comparison="less_than"
        )
    
    # Validate TIC (Requirement 7.3)
    tic_validation = validate_metric(
        metric_name="TIC",
        actual_value=metrics['tic'],
        threshold_value=thresholds.tic_max,
        comparison="less_than"
    )
    
    # Validate R² (Requirement 7.4)
    r2_validation = validate_metric(
        metric_name="R²",
        actual_value=metrics['r2'],
        threshold_value=thresholds.r2_min,
        comparison="greater_than"
    )
    
    # Determine overall status
    validations = [rmse_validation, tic_validation, r2_validation]
    if nrmse_validation is not None:
        validations.append(nrmse_validation)
    if all(v.status == ValidationStatus.PASS for v in validations):
        overall_status = ValidationStatus.PASS
    else:
        overall_status = ValidationStatus.FAIL
    
    return DirectionValidation(
        direction=direction,
        model_type=model_type,
        rmse_validation=rmse_validation,
        tic_validation=tic_validation,
        r2_validation=r2_validation,
        overall_status=overall_status,
        nrmse_validation=nrmse_validation
    )


def validate_fitting_quality(
    fitting_results: Dict[str, Dict[str, Any]],
    thresholds: Optional[ValidationThresholds] = None
) -> ValidationReport:
    """
    Validate fitting quality for all fitted models.
    
    This function checks that fitted models meet quality thresholds for RMSE,
    TIC, and R² metrics. It generates a comprehensive validation report with
    pass/fail status for each metric and logs warnings for threshold violations.
    
    Args:
        fitting_results: Dictionary with structure:
            {
                'fx': {
                    'model_type': str,
                    'metrics': {'rmse': float, 'tic': float, 'r2': float, ...}
                },
                'fy': {
                    'model_type': str,
                    'metrics': {'rmse': float, 'tic': float, 'r2': float, ...}
                }
            }
        thresholds: Optional custom thresholds. If None, uses defaults.
        
    Returns:
        ValidationReport with validation results for all directions
        
    Requirements: 7.1, 7.2, 7.3, 7.4, 7.5, 7.6
    
    Example:
        >>> results = {
        ...     'fx': {
        ...         'model_type': 'magic_formula',
        ...         'metrics': {'rmse': 85.3, 'tic': 0.12, 'r2': 0.9845}
        ...     },
        ...     'fy': {
        ...         'model_type': 'tmeasy',
        ...         'metrics': {'rmse': 119.2, 'tic': 0.040, 'r2': 0.9937}
        ...     }
        ... }
        >>> report = validate_fitting_quality(results)
        >>> print(report.passed)
        False  # FX RMSE is 85.3 < 100, but FY RMSE is 119.2 > 100
    """
    if thresholds is None:
        thresholds = ValidationThresholds()
    
    logger.info("=" * 60)
    logger.info("FITTING QUALITY VALIDATION")
    logger.info("=" * 60)
    logger.info(
        f"Thresholds: RMSE < max({thresholds.rmse_max} N, "
        f"{thresholds.rmse_force_scale_fraction_max:.1%} force scale), "
        f"NRMSE < {thresholds.nrmse_max}, "
        f"TIC < {thresholds.tic_max}, R² > {thresholds.r2_min}"
    )
    logger.info("")
    
    # Validate longitudinal force (fx)
    fx_validation = None
    if 'fx' in fitting_results:
        fx_data = fitting_results['fx']
        fx_validation = validate_direction(
            direction='fx',
            model_type=fx_data['model_type'],
            metrics=fx_data['metrics'],
            thresholds=thresholds
        )
        
        logger.info("Longitudinal Force (FX) Validation:")
        logger.info(f"  Model: {fx_validation.model_type}")
        logger.info(f"  RMSE: {fx_validation.rmse_validation.message} "
                   f"[{fx_validation.rmse_validation.status.value.upper()}]")
        if fx_validation.nrmse_validation is not None:
            logger.info(f"  NRMSE:{fx_validation.nrmse_validation.message} "
                       f"[{fx_validation.nrmse_validation.status.value.upper()}]")
        logger.info(f"  TIC:  {fx_validation.tic_validation.message} "
                   f"[{fx_validation.tic_validation.status.value.upper()}]")
        logger.info(f"  R²:   {fx_validation.r2_validation.message} "
                   f"[{fx_validation.r2_validation.status.value.upper()}]")
        logger.info(f"  Overall: {fx_validation.overall_status.value.upper()}")
        logger.info("")
        
        # Log warnings for failures (Requirement 7.6)
        if not fx_validation.passed:
            logger.warning(f"Longitudinal force validation FAILED for {fx_validation.model_type}")
            for validation in [fx_validation.rmse_validation, 
                             fx_validation.tic_validation,
                             fx_validation.r2_validation,
                             fx_validation.nrmse_validation]:
                if validation is not None and validation.status == ValidationStatus.FAIL:
                    logger.warning(f"  {validation.metric_name}: {validation.message}")
    
    # Validate lateral force (fy)
    fy_validation = None
    if 'fy' in fitting_results:
        fy_data = fitting_results['fy']
        fy_validation = validate_direction(
            direction='fy',
            model_type=fy_data['model_type'],
            metrics=fy_data['metrics'],
            thresholds=thresholds
        )
        
        logger.info("Lateral Force (FY) Validation:")
        logger.info(f"  Model: {fy_validation.model_type}")
        logger.info(f"  RMSE: {fy_validation.rmse_validation.message} "
                   f"[{fy_validation.rmse_validation.status.value.upper()}]")
        if fy_validation.nrmse_validation is not None:
            logger.info(f"  NRMSE:{fy_validation.nrmse_validation.message} "
                       f"[{fy_validation.nrmse_validation.status.value.upper()}]")
        logger.info(f"  TIC:  {fy_validation.tic_validation.message} "
                   f"[{fy_validation.tic_validation.status.value.upper()}]")
        logger.info(f"  R²:   {fy_validation.r2_validation.message} "
                   f"[{fy_validation.r2_validation.status.value.upper()}]")
        logger.info(f"  Overall: {fy_validation.overall_status.value.upper()}")
        logger.info("")
        
        # Log warnings for failures (Requirement 7.6)
        if not fy_validation.passed:
            logger.warning(f"Lateral force validation FAILED for {fy_validation.model_type}")
            for validation in [fy_validation.rmse_validation,
                             fy_validation.tic_validation,
                             fy_validation.r2_validation,
                             fy_validation.nrmse_validation]:
                if validation is not None and validation.status == ValidationStatus.FAIL:
                    logger.warning(f"  {validation.metric_name}: {validation.message}")
    
    # Determine overall status
    if fx_validation is None and fy_validation is None:
        overall_status = ValidationStatus.WARNING
        summary = "No fitting results to validate"
    elif fx_validation and fy_validation:
        if fx_validation.passed and fy_validation.passed:
            overall_status = ValidationStatus.PASS
            summary = "All validations passed"
        else:
            overall_status = ValidationStatus.FAIL
            failed_directions = []
            if not fx_validation.passed:
                failed_directions.append("FX")
            if not fy_validation.passed:
                failed_directions.append("FY")
            summary = f"Validation failed for: {', '.join(failed_directions)}"
    elif fx_validation:
        overall_status = fx_validation.overall_status
        summary = f"FX validation: {overall_status.value}"
    else:  # fy_validation only
        overall_status = fy_validation.overall_status
        summary = f"FY validation: {overall_status.value}"
    
    logger.info("=" * 60)
    logger.info(f"OVERALL VALIDATION: {overall_status.value.upper()}")
    logger.info(f"Summary: {summary}")
    logger.info("=" * 60)
    logger.info("")
    
    report = ValidationReport(
        fx_validation=fx_validation,
        fy_validation=fy_validation,
        thresholds=thresholds,
        overall_status=overall_status,
        summary=summary
    )
    
    return report


def generate_validation_report_text(report: ValidationReport) -> str:
    """
    Generate a text-based validation report.
    
    Args:
        report: ValidationReport to format
        
    Returns:
        Formatted text report
        
    Requirements: 7.6
    """
    lines = []
    lines.append("=" * 60)
    lines.append("TIRE MODEL FITTING QUALITY VALIDATION REPORT")
    lines.append("=" * 60)
    lines.append("")
    
    # Thresholds
    lines.append("Validation Thresholds:")
    lines.append(
        f"  RMSE < max({report.thresholds.rmse_max} N, "
        f"{report.thresholds.rmse_force_scale_fraction_max:.1%} force scale)"
    )
    if report.thresholds.nrmse_max is not None:
        lines.append(f"  NRMSE < {report.thresholds.nrmse_max}")
    lines.append(f"  TIC  < {report.thresholds.tic_max}")
    lines.append(f"  R²   > {report.thresholds.r2_min}")
    lines.append("")
    
    # Longitudinal force
    if report.fx_validation:
        fx = report.fx_validation
        lines.append("Longitudinal Force (FX):")
        lines.append(f"  Model Type: {fx.model_type}")
        lines.append(f"  RMSE: {fx.rmse_validation.actual_value:.2f} N "
                    f"[{fx.rmse_validation.status.value.upper()}]")
        if fx.nrmse_validation is not None:
            lines.append(f"  NRMSE: {fx.nrmse_validation.actual_value:.4f} "
                        f"[{fx.nrmse_validation.status.value.upper()}]")
        lines.append(f"  TIC:  {fx.tic_validation.actual_value:.4f} "
                    f"[{fx.tic_validation.status.value.upper()}]")
        lines.append(f"  R²:   {fx.r2_validation.actual_value:.4f} "
                    f"[{fx.r2_validation.status.value.upper()}]")
        lines.append(f"  Status: {fx.overall_status.value.upper()}")
        lines.append("")
    
    # Lateral force
    if report.fy_validation:
        fy = report.fy_validation
        lines.append("Lateral Force (FY):")
        lines.append(f"  Model Type: {fy.model_type}")
        lines.append(f"  RMSE: {fy.rmse_validation.actual_value:.2f} N "
                    f"[{fy.rmse_validation.status.value.upper()}]")
        if fy.nrmse_validation is not None:
            lines.append(f"  NRMSE: {fy.nrmse_validation.actual_value:.4f} "
                        f"[{fy.nrmse_validation.status.value.upper()}]")
        lines.append(f"  TIC:  {fy.tic_validation.actual_value:.4f} "
                    f"[{fy.tic_validation.status.value.upper()}]")
        lines.append(f"  R²:   {fy.r2_validation.actual_value:.4f} "
                    f"[{fy.r2_validation.status.value.upper()}]")
        lines.append(f"  Status: {fy.overall_status.value.upper()}")
        lines.append("")
    
    # Overall summary
    lines.append("=" * 60)
    lines.append(f"OVERALL STATUS: {report.overall_status.value.upper()}")
    lines.append(f"Summary: {report.summary}")
    
    # Failed metrics
    failed_metrics = report.get_failed_metrics()
    if failed_metrics:
        lines.append("")
        lines.append("Failed Metrics:")
        for metric in failed_metrics:
            lines.append(f"  - {metric}")
    
    lines.append("=" * 60)
    
    return "\n".join(lines)
