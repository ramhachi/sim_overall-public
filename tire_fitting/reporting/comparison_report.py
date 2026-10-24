"""
Comparison Report Generator

This module provides functions for generating comprehensive comparison reports
between different tire models (TMeasy and Magic Formula). Reports include:
- Fitting metrics comparison (RMSE, TIC, R², peak error)
- Model selection recommendations
- Force-slip curve visualizations
- Markdown-formatted output with embedded plots

Requirements: 4.5, 12.1, 12.2, 12.3, 12.4, 12.5, 12.6
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from datetime import datetime
import logging

from tire_fitting.models.base import TireModel
from tire_fitting.models.tmeasy import TMeasyModel
from tire_fitting.models.magic_formula import MagicFormulaModel


@dataclass
class ModelResult:
    """Results for a single model and force direction
    
    Attributes:
        model_type: Model type ('tmeasy' or 'magic_formula')
        direction: Force direction ('fx' or 'fy')
        parameters: Fitted model parameters
        metrics: Fitting quality metrics (rmse, tic, r2, peak_error)
    """
    model_type: str
    direction: str
    parameters: Dict[str, float]
    metrics: Dict[str, float]


@dataclass
class ComparisonReportData:
    """Data for generating comparison report
    
    Attributes:
        model_results: Dictionary mapping model_type -> direction -> ModelResult
        experimental_data: Dictionary with experimental data for plotting
            - 'fx': {'slip_ratio': array, 'force': array, 'fz': array}
            - 'fy': {'slip_angle': array, 'force': array, 'fz': array}
        selected_models: Dictionary with selected model for each direction
            - 'fx': model_type
            - 'fy': model_type
        selection_rationale: Explanation of model selection
        tire_name: Tire identifier
    """
    model_results: Dict[str, Dict[str, ModelResult]]
    experimental_data: Dict[str, Dict[str, np.ndarray]]
    selected_models: Dict[str, str]
    selection_rationale: str
    tire_name: str = "Unknown"


def generate_comparison_report(
    report_data: ComparisonReportData,
    output_dir: str,
    report_name: str = "model_comparison_report"
) -> str:
    """Generate comprehensive comparison report for tire models
    
    Creates a Markdown report with:
    - Executive summary with model selection recommendations
    - Detailed metrics comparison table
    - Force-slip curve visualizations for all models
    - Analysis of performance differences
    
    Args:
        report_data: Comparison data including model results and experimental data
        output_dir: Directory to save report and plots
        report_name: Base name for report file (without extension)
        
    Returns:
        Path to generated report file
        
    Raises:
        ValueError: If report_data is invalid or incomplete
        PermissionError: If output directory is not writable
        
    Requirements: 4.5, 12.1, 12.2, 12.3, 12.4, 12.5, 12.6
    """
    logger = logging.getLogger(__name__)
    logger.info(f"Generating comparison report: {report_name}")
    
    # Validate report data
    _validate_report_data(report_data)
    
    # Create output directory
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Create plots subdirectory
    plots_dir = output_path / "plots"
    plots_dir.mkdir(exist_ok=True)
    
    # Generate visualizations (Requirement 12.4)
    plot_files = _generate_visualizations(
        report_data,
        plots_dir,
        report_name
    )
    
    # Build report content (Requirements 12.1, 12.2, 12.3, 12.5)
    report_content = _build_report_content(
        report_data,
        plot_files,
        plots_dir.name
    )
    
    # Write report file (Requirement 12.6)
    report_file = output_path / f"{report_name}.md"
    try:
        with open(report_file, 'w') as f:
            f.write(report_content)
    except PermissionError as e:
        raise PermissionError(
            f"Cannot write to output directory: permission denied"
        ) from e
    
    logger.info(f"Report generated: {report_file}")
    return str(report_file)


def _validate_report_data(report_data: ComparisonReportData) -> None:
    """Validate comparison report data
    
    Args:
        report_data: Data to validate
        
    Raises:
        ValueError: If data is invalid or incomplete
    """
    # Check model results exist
    if not report_data.model_results:
        raise ValueError("No model results provided")
    
    # Check experimental data exists
    if not report_data.experimental_data:
        raise ValueError("No experimental data provided")
    
    # Check selected models exist
    if not report_data.selected_models:
        raise ValueError("No selected models provided")
    
    # Validate each direction has data
    for direction in ['fx', 'fy']:
        if direction not in report_data.experimental_data:
            raise ValueError(f"Missing experimental data for {direction}")
        
        exp_data = report_data.experimental_data[direction]
        required_keys = ['slip_ratio' if direction == 'fx' else 'slip_angle', 'force', 'fz']
        for key in required_keys:
            if key not in exp_data:
                raise ValueError(f"Missing '{key}' in experimental data for {direction}")


def _generate_visualizations(
    report_data: ComparisonReportData,
    plots_dir: Path,
    report_name: str
) -> Dict[str, str]:
    """Generate force-slip curve visualizations
    
    Creates plots comparing model predictions against experimental data
    for both longitudinal and lateral forces.
    
    Args:
        report_data: Comparison data
        plots_dir: Directory to save plots
        report_name: Base name for plot files
        
    Returns:
        Dictionary mapping direction -> plot filename
        
    Requirements: 12.4
    """
    logger = logging.getLogger(__name__)
    plot_files = {}
    
    # Generate longitudinal force plot
    if 'fx' in report_data.experimental_data:
        logger.info("Generating longitudinal force plot")
        fx_plot = _plot_longitudinal_comparison(
            report_data,
            plots_dir,
            f"{report_name}_fx"
        )
        plot_files['fx'] = fx_plot
    
    # Generate lateral force plot
    if 'fy' in report_data.experimental_data:
        logger.info("Generating lateral force plot")
        fy_plot = _plot_lateral_comparison(
            report_data,
            plots_dir,
            f"{report_name}_fy"
        )
        plot_files['fy'] = fy_plot
    
    return plot_files


def _plot_longitudinal_comparison(
    report_data: ComparisonReportData,
    plots_dir: Path,
    filename: str
) -> str:
    """Plot longitudinal force comparison
    
    Args:
        report_data: Comparison data
        plots_dir: Directory to save plot
        filename: Base filename (without extension)
        
    Returns:
        Plot filename
    """
    exp_data = report_data.experimental_data['fx']
    slip_ratio = exp_data['slip_ratio']
    force_exp = exp_data['force']
    fz = exp_data['fz']
    
    # Create figure
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Plot experimental data
    ax.scatter(slip_ratio, force_exp, alpha=0.3, s=10, c='gray', label='Experimental Data')
    
    # Plot model predictions
    colors = {'tmeasy': 'blue', 'magic_formula': 'red'}
    linestyles = {'tmeasy': '-', 'magic_formula': '--'}
    
    for model_type in report_data.model_results:
        if 'fx' not in report_data.model_results[model_type]:
            continue
        
        result = report_data.model_results[model_type]['fx']
        
        # Create model instance
        model = _create_model_instance(result.model_type, result.parameters)
        
        # Generate predictions on a grid
        sr_grid = np.linspace(slip_ratio.min(), slip_ratio.max(), 200)
        fz_mean = np.mean(fz)
        fx_pred = np.array([model.calculate_fx(sr, fz_mean) for sr in sr_grid])
        
        # Plot model curve
        label = f"{model_type.upper()} (RMSE: {result.metrics['rmse']:.1f}N)"
        ax.plot(sr_grid, fx_pred, 
               color=colors.get(model_type, 'black'),
               linestyle=linestyles.get(model_type, '-'),
               linewidth=2,
               label=label)
    
    # Formatting
    ax.set_xlabel('Slip Ratio [-]', fontsize=12)
    ax.set_ylabel('Longitudinal Force [N]', fontsize=12)
    ax.set_title(f'Longitudinal Force Comparison\n{report_data.tire_name}', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)
    
    # Save plot
    plot_file = plots_dir / f"{filename}.png"
    plt.tight_layout()
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    plt.close()
    
    return plot_file.name


def _plot_lateral_comparison(
    report_data: ComparisonReportData,
    plots_dir: Path,
    filename: str
) -> str:
    """Plot lateral force comparison
    
    Args:
        report_data: Comparison data
        plots_dir: Directory to save plot
        filename: Base filename (without extension)
        
    Returns:
        Plot filename
    """
    exp_data = report_data.experimental_data['fy']
    slip_angle = exp_data['slip_angle']
    force_exp = exp_data['force']
    fz = exp_data['fz']
    
    # Create figure
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Plot experimental data
    ax.scatter(np.rad2deg(slip_angle), force_exp, alpha=0.3, s=10, c='gray', label='Experimental Data')
    
    # Plot model predictions
    colors = {'tmeasy': 'blue', 'magic_formula': 'red'}
    linestyles = {'tmeasy': '-', 'magic_formula': '--'}
    
    for model_type in report_data.model_results:
        if 'fy' not in report_data.model_results[model_type]:
            continue
        
        result = report_data.model_results[model_type]['fy']
        
        # Create model instance
        model = _create_model_instance(result.model_type, result.parameters)
        
        # Generate predictions on a grid
        sa_grid = np.linspace(slip_angle.min(), slip_angle.max(), 200)
        fz_mean = np.mean(fz)
        fy_pred = np.array([model.calculate_fy(sa, fz_mean) for sa in sa_grid])
        
        # Plot model curve
        label = f"{model_type.upper()} (RMSE: {result.metrics['rmse']:.1f}N)"
        ax.plot(np.rad2deg(sa_grid), fy_pred,
               color=colors.get(model_type, 'black'),
               linestyle=linestyles.get(model_type, '-'),
               linewidth=2,
               label=label)
    
    # Formatting
    ax.set_xlabel('Slip Angle [deg]', fontsize=12)
    ax.set_ylabel('Lateral Force [N]', fontsize=12)
    ax.set_title(f'Lateral Force Comparison\n{report_data.tire_name}', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)
    
    # Save plot
    plot_file = plots_dir / f"{filename}.png"
    plt.tight_layout()
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    plt.close()
    
    return plot_file.name


def _create_model_instance(model_type: str, parameters: Dict[str, float]) -> TireModel:
    """Create tire model instance from parameters
    
    Args:
        model_type: Model type ('tmeasy' or 'magic_formula')
        parameters: Model parameters
        
    Returns:
        TireModel instance
    """
    if model_type == 'tmeasy':
        fz_nominal = parameters.get('fz_nominal', 1200.0)
        return TMeasyModel(fz_nominal=fz_nominal, params=parameters)
    elif model_type == 'magic_formula':
        return MagicFormulaModel(params=parameters)
    else:
        raise ValueError(f"Unknown model type: {model_type}")


def _build_report_content(
    report_data: ComparisonReportData,
    plot_files: Dict[str, str],
    plots_subdir: str
) -> str:
    """Build Markdown report content
    
    Args:
        report_data: Comparison data
        plot_files: Dictionary mapping direction -> plot filename
        plots_subdir: Subdirectory name for plots
        
    Returns:
        Markdown-formatted report content
        
    Requirements: 12.1, 12.2, 12.3, 12.5, 12.6
    """
    lines = []
    
    # Header
    lines.append("# Tire Model Comparison Report")
    lines.append("")
    lines.append(f"**Tire:** {report_data.tire_name}")
    lines.append(f"**Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("")
    lines.append("---")
    lines.append("")
    
    # Executive Summary (Requirement 4.5, 12.5)
    lines.append("## Executive Summary")
    lines.append("")
    lines.append("### Model Selection")
    lines.append("")
    lines.append(f"**Longitudinal Force (Fx):** {report_data.selected_models.get('fx', 'N/A').upper()}")
    lines.append(f"**Lateral Force (Fy):** {report_data.selected_models.get('fy', 'N/A').upper()}")
    lines.append("")
    lines.append("### Rationale")
    lines.append("")
    lines.append(report_data.selection_rationale)
    lines.append("")
    lines.append("---")
    lines.append("")
    
    # Metrics Comparison (Requirements 12.1, 12.2, 12.3)
    lines.append("## Fitting Metrics Comparison")
    lines.append("")
    
    # Longitudinal force metrics
    if any('fx' in results for results in report_data.model_results.values()):
        lines.append("### Longitudinal Force (Fx)")
        lines.append("")
        lines.extend(_build_metrics_table('fx', report_data))
        lines.append("")
        lines.extend(_build_performance_analysis('fx', report_data))
        lines.append("")
    
    # Lateral force metrics
    if any('fy' in results for results in report_data.model_results.values()):
        lines.append("### Lateral Force (Fy)")
        lines.append("")
        lines.extend(_build_metrics_table('fy', report_data))
        lines.append("")
        lines.extend(_build_performance_analysis('fy', report_data))
        lines.append("")
    
    lines.append("---")
    lines.append("")
    
    # Visualizations (Requirement 12.4)
    lines.append("## Force-Slip Curve Visualizations")
    lines.append("")
    
    if 'fx' in plot_files:
        lines.append("### Longitudinal Force")
        lines.append("")
        lines.append(f"![Longitudinal Force Comparison]({plots_subdir}/{plot_files['fx']})")
        lines.append("")
    
    if 'fy' in plot_files:
        lines.append("### Lateral Force")
        lines.append("")
        lines.append(f"![Lateral Force Comparison]({plots_subdir}/{plot_files['fy']})")
        lines.append("")
    
    lines.append("---")
    lines.append("")
    
    # Model Parameters
    lines.append("## Fitted Model Parameters")
    lines.append("")
    lines.extend(_build_parameters_section(report_data))
    lines.append("")
    
    # Footer
    lines.append("---")
    lines.append("")
    lines.append("*Report generated by Hybrid Tire Model Fitting System*")
    lines.append("")
    
    return "\n".join(lines)


def _build_metrics_table(direction: str, report_data: ComparisonReportData) -> List[str]:
    """Build metrics comparison table for a force direction
    
    Args:
        direction: Force direction ('fx' or 'fy')
        report_data: Comparison data
        
    Returns:
        List of Markdown table lines
        
    Requirements: 12.1, 12.2, 12.3
    """
    lines = []
    
    # Table header
    lines.append("| Model | RMSE [N] | TIC [-] | R² [-] | Peak Error [N] |")
    lines.append("|-------|----------|---------|--------|----------------|")
    
    # Table rows
    for model_type in sorted(report_data.model_results.keys()):
        if direction not in report_data.model_results[model_type]:
            continue
        
        result = report_data.model_results[model_type][direction]
        metrics = result.metrics
        
        # Highlight selected model (Requirement 12.5)
        model_name = model_type.upper()
        if report_data.selected_models.get(direction) == model_type:
            model_name = f"**{model_name}** ✓"
        
        lines.append(
            f"| {model_name} | "
            f"{metrics['rmse']:.2f} | "
            f"{metrics['tic']:.4f} | "
            f"{metrics['r2']:.4f} | "
            f"{metrics['peak_error']:.2f} |"
        )
    
    return lines


def _build_performance_analysis(direction: str, report_data: ComparisonReportData) -> List[str]:
    """Build performance analysis for a force direction
    
    Analyzes which model performs better and by how much.
    
    Args:
        direction: Force direction ('fx' or 'fy')
        report_data: Comparison data
        
    Returns:
        List of analysis text lines
        
    Requirements: 12.5
    """
    lines = []
    
    # Get all results for this direction
    results = []
    for model_type in report_data.model_results:
        if direction in report_data.model_results[model_type]:
            result = report_data.model_results[model_type][direction]
            results.append((model_type, result))
    
    if len(results) < 2:
        return lines
    
    # Sort by RMSE
    results.sort(key=lambda x: x[1].metrics['rmse'])
    best_model, best_result = results[0]
    second_model, second_result = results[1]
    
    # Calculate improvement
    rmse_improvement = second_result.metrics['rmse'] - best_result.metrics['rmse']
    rmse_improvement_pct = (rmse_improvement / second_result.metrics['rmse']) * 100
    
    # Build analysis text
    lines.append("**Performance Analysis:**")
    lines.append("")
    lines.append(
        f"- **{best_model.upper()}** achieves the lowest RMSE of "
        f"**{best_result.metrics['rmse']:.2f} N**, which is "
        f"**{rmse_improvement:.2f} N ({rmse_improvement_pct:.1f}%)** better than "
        f"{second_model.upper()} ({second_result.metrics['rmse']:.2f} N)."
    )
    
    # TIC comparison
    tic_diff = abs(best_result.metrics['tic'] - second_result.metrics['tic'])
    if tic_diff > 0.01:
        if best_result.metrics['tic'] < second_result.metrics['tic']:
            lines.append(
                f"- **{best_model.upper()}** also has a lower TIC "
                f"({best_result.metrics['tic']:.4f} vs {second_result.metrics['tic']:.4f}), "
                f"indicating better overall prediction quality."
            )
        else:
            lines.append(
                f"- However, **{second_model.upper()}** has a lower TIC "
                f"({second_result.metrics['tic']:.4f} vs {best_result.metrics['tic']:.4f})."
            )
    
    # R² comparison
    r2_diff = abs(best_result.metrics['r2'] - second_result.metrics['r2'])
    if r2_diff > 0.01:
        if best_result.metrics['r2'] > second_result.metrics['r2']:
            lines.append(
                f"- **{best_model.upper()}** has a higher R² "
                f"({best_result.metrics['r2']:.4f} vs {second_result.metrics['r2']:.4f}), "
                f"explaining more variance in the data."
            )
        else:
            lines.append(
                f"- However, **{second_model.upper()}** has a higher R² "
                f"({second_result.metrics['r2']:.4f} vs {best_result.metrics['r2']:.4f})."
            )
    
    return lines


def _build_parameters_section(report_data: ComparisonReportData) -> List[str]:
    """Build parameters section showing fitted values
    
    Args:
        report_data: Comparison data
        
    Returns:
        List of parameter section lines
    """
    lines = []
    
    for model_type in sorted(report_data.model_results.keys()):
        lines.append(f"### {model_type.upper()}")
        lines.append("")
        
        for direction in ['fx', 'fy']:
            if direction not in report_data.model_results[model_type]:
                continue
            
            result = report_data.model_results[model_type][direction]
            direction_name = "Longitudinal" if direction == 'fx' else "Lateral"
            
            lines.append(f"**{direction_name} Force Parameters:**")
            lines.append("")
            lines.append("```")
            for param_name, param_value in sorted(result.parameters.items()):
                if isinstance(param_value, float):
                    lines.append(f"{param_name:15s} = {param_value:.6f}")
                else:
                    lines.append(f"{param_name:15s} = {param_value}")
            lines.append("```")
            lines.append("")
    
    return lines
