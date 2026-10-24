#!/usr/bin/env python3
"""
Main Script for Hybrid Tire Model Fitting

This script provides a complete pipeline for fitting hybrid tire models
(TMeasy and Magic Formula) from FSAE Tire Test Consortium (TTC) experimental data.

Pipeline stages:
1. Data loading (DriveBrake and Cornering CSV files)
2. Data preprocessing (filtering, grouping, outlier removal)
3. Model fitting (multiple models in parallel)
4. Model selection (automatic or manual)
5. Validation (quality thresholds)
6. Comparison reporting (metrics and visualizations)
7. Export (YAML and C++ header files)

Usage:
    python tire_fitting/main_hybrid.py --drivebrake <path> --cornering <path> --output <dir> --tire-name <name>
    python tire_fitting/main_hybrid.py --drivebrake <path> --cornering <path> --output <dir> --tire-name <name> --model-long magic_formula --model-lat tmeasy
"""

import argparse
import logging
import sys
from pathlib import Path
import yaml

from tire_fitting import (
    TireDataLoader,
    TireDataPreprocessor
)

# Import hybrid model components
from tire_fitting.model_selector import ModelSelector, SelectionMode
from tire_fitting.optimizer import ModelFitter
from tire_fitting.exporters.yaml_exporter import (
    export_hybrid_config,
    HybridConfigData,
    ModelConfig,
    FittingMetrics
)
from tire_fitting.exporters.cpp_exporter import export_cpp_header
from tire_fitting.reporting.comparison_report import (
    generate_comparison_report,
    ComparisonReportData,
    ModelResult
)
from tire_fitting.units import ttc_lateral_angles_to_radians
from tire_fitting.validation import validate_fitting_quality, ValidationThresholds


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


def load_config_file(config_path: str) -> dict:
    """Load and validate configuration file.
    
    Args:
        config_path: Path to YAML configuration file
        
    Returns:
        Dictionary containing configuration values
        
    Raises:
        FileNotFoundError: If config file doesn't exist
        ValueError: If config file is invalid or contains invalid values
    """
    config_file = Path(config_path)
    
    if not config_file.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")
    
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise ValueError(f"Invalid YAML in configuration file: {e}")
    
    if config is None:
        config = {}
    
    # Validate model preferences if present
    valid_models = ['tmeasy', 'magic_formula', 'auto']
    
    if 'model_long' in config:
        if config['model_long'] not in valid_models:
            raise ValueError(
                f"Invalid model_long in config: {config['model_long']}. "
                f"Must be one of: {valid_models}"
            )
    
    if 'model_lat' in config:
        if config['model_lat'] not in valid_models:
            raise ValueError(
                f"Invalid model_lat in config: {config['model_lat']}. "
                f"Must be one of: {valid_models}"
            )
    
    return config


def merge_config_with_args(args: argparse.Namespace) -> argparse.Namespace:
    """Merge configuration file with command-line arguments.
    
    Command-line arguments take precedence over configuration file values.
    
    Args:
        args: Parsed command-line arguments
        
    Returns:
        Updated arguments with config file values where CLI args not specified
        
    Raises:
        FileNotFoundError: If specified config file doesn't exist
        ValueError: If config file is invalid
    """
    if args.config is None:
        return args
    
    logger.info(f"Loading configuration from: {args.config}")
    
    try:
        config = load_config_file(args.config)
    except FileNotFoundError as e:
        logger.error(f"Configuration file not found: {e}")
        raise
    except ValueError as e:
        logger.error(f"Invalid configuration file: {e}")
        raise
    
    # Track which values came from config vs CLI
    config_applied = []
    cli_override = []
    
    # Apply config values only if CLI argument is at default value
    if 'model_long' in config and args.model_long == 'auto':
        if config['model_long'] != 'auto':
            args.model_long = config['model_long']
            config_applied.append(f"model_long={config['model_long']}")
    elif args.model_long != 'auto':
        cli_override.append(f"model_long={args.model_long}")
    
    if 'model_lat' in config and args.model_lat == 'auto':
        if config['model_lat'] != 'auto':
            args.model_lat = config['model_lat']
            config_applied.append(f"model_lat={config['model_lat']}")
    elif args.model_lat != 'auto':
        cli_override.append(f"model_lat={args.model_lat}")
    
    # Log what was applied
    if config_applied:
        logger.info(f"  Applied from config: {', '.join(config_applied)}")
    if cli_override:
        logger.info(f"  CLI overrides: {', '.join(cli_override)}")
    
    return args


def parse_arguments():
    """
    Parse command line arguments.
    
    Returns:
        argparse.Namespace: Parsed arguments
    """
    parser = argparse.ArgumentParser(
        description='Fit hybrid tire model parameters from TTC experimental data',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Required arguments
    parser.add_argument(
        '--drivebrake',
        type=str,
        required=True,
        help='Path to DriveBrake CSV file (longitudinal force data)'
    )
    
    parser.add_argument(
        '--cornering',
        type=str,
        required=True,
        help='Path to Cornering CSV file (lateral force data)'
    )
    
    parser.add_argument(
        '--output',
        type=str,
        required=True,
        help='Output directory for results (YAML, C++ header, plots)'
    )
    
    parser.add_argument(
        '--tire-name',
        type=str,
        required=True,
        help='Tire model name/identifier (e.g., "Hoosier 18.0x6.0-10 R20")'
    )
    
    # Optional arguments
    parser.add_argument(
        '--effective-radius',
        type=float,
        default=0.2286,
        help='Tire effective radius in meters'
    )
    
    parser.add_argument(
        '--nominal-load',
        type=float,
        default=1000.0,
        help='Nominal vertical load in Newtons'
    )
    
    parser.add_argument(
        '--remove-outliers',
        action='store_true',
        help='Enable IQR-based outlier removal'
    )
    
    # Model selection arguments
    parser.add_argument(
        '--model-long',
        type=str,
        choices=['tmeasy', 'magic_formula', 'auto'],
        default='auto',
        help='Model type for longitudinal force (default: auto)'
    )
    
    parser.add_argument(
        '--model-lat',
        type=str,
        choices=['tmeasy', 'magic_formula', 'auto'],
        default='auto',
        help='Model type for lateral force (default: auto)'
    )
    
    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='Path to YAML configuration file for model preferences'
    )

    parser.add_argument(
        '--allow-validation-fail',
        action='store_true',
        help='Export results even when fitting quality thresholds fail'
    )
    
    return parser.parse_args()


def main():
    """
    Main execution function for hybrid tire fitting pipeline.
    
    Implements the complete hybrid tire model pipeline:
    1. Load data
    2. Preprocess data
    3. Fit multiple models (TMeasy and Magic Formula)
    4. Select best models per force direction
    5. Validate fitting quality
    6. Generate comparison reports
    7. Export outputs (YAML and C++ header)
    
    Returns:
        int: Exit code (0 for success, 1 for failure)
    """
    # Parse command line arguments
    args = parse_arguments()
    
    # Merge with configuration file if provided
    try:
        args = merge_config_with_args(args)
    except (FileNotFoundError, ValueError) as e:
        logger.error(f"Configuration error: {e}")
        return 1
    
    logger.info("=" * 80)
    logger.info("Hybrid Tire Model Fitting Pipeline")
    logger.info("=" * 80)
    logger.info(f"Tire: {args.tire_name}")
    logger.info(f"DriveBrake data: {args.drivebrake}")
    logger.info(f"Cornering data: {args.cornering}")
    logger.info(f"Output directory: {args.output}")
    logger.info(f"Model selection: Longitudinal={args.model_long}, Lateral={args.model_lat}")
    logger.info("=" * 80)
    
    try:
        # ===================================================================
        # Stage 1: Data Loading
        # ===================================================================
        logger.info("\n[Stage 1/7] Loading data...")
        
        loader = TireDataLoader()
        
        # Load DriveBrake data
        logger.info(f"  Loading DriveBrake data from: {args.drivebrake}")
        try:
            drivebrake_df = loader.load_drivebrake_data(args.drivebrake)
            logger.info(f"  ✓ Loaded {len(drivebrake_df)} DriveBrake data points")
        except FileNotFoundError:
            logger.error(f"  ✗ DriveBrake file not found: {args.drivebrake}")
            logger.error("  Please verify the file path and try again.")
            raise
        except ValueError as e:
            logger.error(f"  ✗ Invalid DriveBrake file: {e}")
            logger.error("  Please ensure the file contains all required columns.")
            raise
        
        # Load Cornering data
        logger.info(f"  Loading Cornering data from: {args.cornering}")
        try:
            cornering_df = loader.load_cornering_data(args.cornering)
            logger.info(f"  ✓ Loaded {len(cornering_df)} Cornering data points")
        except FileNotFoundError:
            logger.error(f"  ✗ Cornering file not found: {args.cornering}")
            logger.error("  Please verify the file path and try again.")
            raise
        except ValueError as e:
            logger.error(f"  ✗ Invalid Cornering file: {e}")
            logger.error("  Please ensure the file contains all required columns.")
            raise
        
        # ===================================================================
        # Stage 2: Data Preprocessing
        # ===================================================================
        logger.info("\n[Stage 2/7] Preprocessing data...")
        
        preprocessor = TireDataPreprocessor()
        
        # Filter DriveBrake data (remove only missing values)
        logger.info("  Filtering DriveBrake data (removing missing values only)...")
        
        required_cols_drivebrake = ['SR', 'FX', 'FZ', 'SA']
        drivebrake_before = len(drivebrake_df)
        drivebrake_filtered = drivebrake_df.dropna(subset=required_cols_drivebrake).copy()
        drivebrake_after_na = len(drivebrake_filtered)
        
        # Only filter out invalid vertical loads (FZ <= 0)
        drivebrake_filtered = drivebrake_filtered[drivebrake_filtered['FZ'] > 0].copy()
        drivebrake_after_fz = len(drivebrake_filtered)
        
        if len(drivebrake_filtered) == 0:
            logger.error("  ✗ No DriveBrake data remains after filtering")
            raise ValueError("No data remains after applying filters to DriveBrake data")
        
        logger.info(f"  ✓ {len(drivebrake_filtered)} points after filtering "
                   f"({100 * len(drivebrake_filtered) / drivebrake_before:.1f}% retained)")
        logger.info(f"    Removed {drivebrake_before - drivebrake_after_na} rows with missing values")
        logger.info(f"    Removed {drivebrake_after_na - drivebrake_after_fz} rows with FZ <= 0")
        
        # Filter Cornering data (remove only missing values)
        logger.info("  Filtering Cornering data (removing missing values only)...")
        
        required_cols_cornering = ['SA', 'FY', 'FZ', 'IA']
        cornering_before = len(cornering_df)
        cornering_filtered = cornering_df.dropna(subset=required_cols_cornering).copy()
        cornering_after_na = len(cornering_filtered)
        
        # Only filter out invalid vertical loads (FZ <= 0)
        cornering_filtered = cornering_filtered[cornering_filtered['FZ'] > 0].copy()
        cornering_after_fz = len(cornering_filtered)
        
        if len(cornering_filtered) == 0:
            logger.error("  ✗ No Cornering data remains after filtering")
            raise ValueError("No data remains after applying filters to Cornering data")
        
        logger.info(f"  ✓ {len(cornering_filtered)} points after filtering "
                   f"({100 * len(cornering_filtered) / cornering_before:.1f}% retained)")
        logger.info(f"    Removed {cornering_before - cornering_after_na} rows with missing values")
        logger.info(f"    Removed {cornering_after_na - cornering_after_fz} rows with FZ <= 0")
        
        # Remove outliers if requested
        if args.remove_outliers:
            logger.info("  Removing outliers using IQR method...")
            drivebrake_filtered = preprocessor.remove_outliers_iqr(drivebrake_filtered, 'FX')
            cornering_filtered = preprocessor.remove_outliers_iqr(cornering_filtered, 'FY')
            logger.info(f"  ✓ {len(drivebrake_filtered)} DriveBrake, {len(cornering_filtered)} Cornering points after outlier removal")
        
        # ===================================================================
        # Stage 3: Model Fitting (Multiple Models)
        # ===================================================================
        logger.info("\n[Stage 3/7] Fitting tire models...")
        
        # Prepare data for fitting
        sr_data = drivebrake_filtered['SR'].values
        fx_data = drivebrake_filtered['FX'].values
        fz_long_data = drivebrake_filtered['FZ'].values
        
        sa_data, camber_data = ttc_lateral_angles_to_radians(cornering_filtered)
        fy_data = cornering_filtered['FY'].values
        fz_lat_data = cornering_filtered['FZ'].values
        
        # Initialize ModelFitter
        model_fitter = ModelFitter(loader, preprocessor)
        
        # Determine which models to fit based on CLI arguments
        models_to_fit = set()
        if args.model_long == 'auto':
            models_to_fit.update(['tmeasy', 'magic_formula'])
        else:
            models_to_fit.add(args.model_long)
        
        if args.model_lat == 'auto':
            models_to_fit.update(['tmeasy', 'magic_formula'])
        else:
            models_to_fit.add(args.model_lat)
        
        logger.info(f"  Fitting models: {', '.join(sorted(models_to_fit))}")
        
        # Fit all models
        fitting_results = model_fitter.fit_multiple_models(
            model_types=list(models_to_fit),
            directions=['fx', 'fy'],
            drivebrake_file=args.drivebrake,
            cornering_file=args.cornering
        )
        
        logger.info("  ✓ Model fitting complete")
        
        # Log fitting results
        for model_type in sorted(fitting_results.keys()):
            logger.info(f"\n  {model_type.upper()} Results:")
            for direction in ['fx', 'fy']:
                if direction in fitting_results[model_type]:
                    result = fitting_results[model_type][direction]
                    metrics = result['metrics']
                    logger.info(f"    {direction.upper()}: RMSE={metrics['rmse']:.2f}N, "
                               f"TIC={metrics['tic']:.4f}, R²={metrics['r2']:.4f}")
        
        # ===================================================================
        # Stage 4: Model Selection
        # ===================================================================
        logger.info("\n[Stage 4/7] Selecting optimal models...")
        
        # Determine selection mode
        if args.model_long == 'auto' and args.model_lat == 'auto':
            selection_mode = SelectionMode.AUTO
        elif args.model_long != 'auto' and args.model_lat != 'auto':
            selection_mode = SelectionMode.MANUAL
        else:
            selection_mode = SelectionMode.HYBRID
        
        logger.info(f"  Selection mode: {selection_mode.value}")
        
        # Initialize model selector
        selector = ModelSelector(selection_mode)
        
        # Prepare fitting results for selector
        selector_results = {}
        for model_type, directions in fitting_results.items():
            selector_results[model_type] = {}
            for direction, result in directions.items():
                selector_results[model_type][direction] = result['metrics']
        
        # Select models
        manual_fx = args.model_long if args.model_long != 'auto' else None
        manual_fy = args.model_lat if args.model_lat != 'auto' else None
        
        selected_fx, selected_fy = selector.select_models(
            selector_results,
            manual_fx=manual_fx,
            manual_fy=manual_fy
        )
        
        logger.info(f"  ✓ Selected models: FX={selected_fx}, FY={selected_fy}")
        
        # Build selection rationale
        fx_metrics = fitting_results[selected_fx]['fx']['metrics']
        fy_metrics = fitting_results[selected_fy]['fy']['metrics']
        
        selection_rationale = (
            f"{selected_fx.upper()} selected for longitudinal force "
            f"(RMSE: {fx_metrics['rmse']:.1f}N, TIC: {fx_metrics['tic']:.3f}, R²: {fx_metrics['r2']:.4f}). "
            f"{selected_fy.upper()} selected for lateral force "
            f"(RMSE: {fy_metrics['rmse']:.1f}N, TIC: {fy_metrics['tic']:.3f}, R²: {fy_metrics['r2']:.4f})."
        )
        
        # ===================================================================
        # Stage 5: Validation
        # ===================================================================
        logger.info("\n[Stage 5/7] Validating fitting quality...")
        
        # Prepare validation data
        validation_data = {
            'fx': {
                'model_type': selected_fx,
                'metrics': fx_metrics
            },
            'fy': {
                'model_type': selected_fy,
                'metrics': fy_metrics
            }
        }
        
        # Validate fitting quality
        validation_report = validate_fitting_quality(
            validation_data,
            thresholds=ValidationThresholds()
        )
        
        if validation_report.passed:
            logger.info("  ✓ All quality thresholds met")
        else:
            logger.error("  ✗ Some quality thresholds not met")
            for failed_metric in validation_report.get_failed_metrics():
                logger.error(f"    {failed_metric}")
            if not args.allow_validation_fail:
                raise ValueError(
                    "Tire fitting quality thresholds failed. "
                    "Use --allow-validation-fail only for diagnostic exports."
                )
        
        # ===================================================================
        # Stage 6: Generate Comparison Report
        # ===================================================================
        logger.info("\n[Stage 6/7] Generating comparison report...")
        
        output_dir = Path(args.output)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Prepare comparison report data
        model_results = {}
        for model_type in fitting_results:
            model_results[model_type] = {}
            for direction in ['fx', 'fy']:
                if direction in fitting_results[model_type]:
                    result = fitting_results[model_type][direction]
                    model_results[model_type][direction] = ModelResult(
                        model_type=model_type,
                        direction=direction,
                        parameters=result['params'],
                        metrics=result['metrics']
                    )
        
        experimental_data = {
            'fx': {
                'slip_ratio': sr_data,
                'force': fx_data,
                'fz': fz_long_data
            },
            'fy': {
                'slip_angle': sa_data,
                'force': fy_data,
                'fz': fz_lat_data
            }
        }
        
        comparison_data = ComparisonReportData(
            model_results=model_results,
            experimental_data=experimental_data,
            selected_models={'fx': selected_fx, 'fy': selected_fy},
            selection_rationale=selection_rationale,
            tire_name=args.tire_name
        )
        
        # Generate report
        report_dir = output_dir / "comparison_reports"
        report_path = generate_comparison_report(
            comparison_data,
            str(report_dir),
            report_name="hybrid_model_comparison"
        )
        
        logger.info(f"  ✓ Comparison report generated: {report_path}")
        
        # ===================================================================
        # Stage 7: Export Results
        # ===================================================================
        logger.info("\n[Stage 7/7] Exporting results...")
        
        # Prepare export data
        fx_result = fitting_results[selected_fx]['fx']
        fy_result = fitting_results[selected_fy]['fy']
        
        fx_config = ModelConfig(
            model_type=selected_fx,
            parameters=fx_result['params'],
            metrics=FittingMetrics(
                rmse=fx_result['metrics']['rmse'],
                tic=fx_result['metrics']['tic'],
                r2=fx_result['metrics']['r2'],
                peak_error=fx_result['metrics'].get('peak_error', 0.0)
            )
        )
        
        fy_config = ModelConfig(
            model_type=selected_fy,
            parameters=fy_result['params'],
            metrics=FittingMetrics(
                rmse=fy_result['metrics']['rmse'],
                tic=fy_result['metrics']['tic'],
                r2=fy_result['metrics']['r2'],
                peak_error=fy_result['metrics'].get('peak_error', 0.0)
            )
        )
        
        hybrid_config = HybridConfigData(
            fx_config=fx_config,
            fy_config=fy_config,
            selection_mode=selection_mode.value,
            selection_rationale=selection_rationale,
            tire_name=args.tire_name,
            nominal_load=args.nominal_load,
            nominal_pressure=12.0,
            data_points_fx=len(drivebrake_filtered),
            data_points_fy=len(cornering_filtered),
            optimization_method="differential_evolution",
            convergence_iterations=None
        )
        
        # Export YAML
        yaml_path = output_dir / "tire_params" / f"{args.tire_name.replace(' ', '_').lower()}.yaml"
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        
        try:
            export_hybrid_config(hybrid_config, str(yaml_path))
            logger.info(f"  ✓ Exported YAML: {yaml_path.name}")
        except PermissionError:
            logger.error("  ✗ Cannot write YAML file: permission denied")
            raise
        except ValueError as e:
            logger.error(f"  ✗ Cannot export invalid parameters: {e}")
            raise
        
        # Export C++ header
        hpp_path = output_dir / "tire_params" / f"{args.tire_name.replace(' ', '_').lower()}.hpp"
        
        try:
            export_cpp_header(
                fx_model_type=selected_fx,
                fx_parameters=fx_result['params'],
                fy_model_type=selected_fy,
                fy_parameters=fy_result['params'],
                tire_name=args.tire_name,
                filepath=str(hpp_path)
            )
            logger.info(f"  ✓ Exported C++ header: {hpp_path.name}")
        except PermissionError:
            logger.error("  ✗ Cannot write C++ header: permission denied")
            raise
        except ValueError as e:
            logger.error(f"  ✗ Cannot export invalid parameters: {e}")
            raise
        
        # ===================================================================
        # Summary
        # ===================================================================
        logger.info("\n" + "=" * 80)
        logger.info("Fitting Complete!")
        logger.info("=" * 80)
        logger.info("Selected Models:")
        logger.info(f"  Longitudinal: {selected_fx.upper()}")
        logger.info(f"  Lateral: {selected_fy.upper()}")
        logger.info(f"\nValidation Status: {validation_report.overall_status.value.upper()}")
        logger.info(f"\nOutput directory: {output_dir}")
        logger.info(f"  - tire_params/{yaml_path.name}")
        logger.info(f"  - tire_params/{hpp_path.name}")
        logger.info("  - comparison_reports/hybrid_model_comparison.md")
        logger.info("  - comparison_reports/plots/")
        logger.info("=" * 80)
        
        return 0
        
    except FileNotFoundError as e:
        logger.error(f"\n✗ File not found: {e}")
        logger.error("Please check that the input file paths are correct.")
        return 1
        
    except ValueError as e:
        logger.error(f"\n✗ Invalid data: {e}")
        logger.error("Please check that the input files contain valid data.")
        return 1
        
    except PermissionError as e:
        logger.error(f"\n✗ Permission denied: {e}")
        logger.error("Please check that you have write permissions for the output directory.")
        return 1
        
    except Exception as e:
        logger.error(f"\n✗ Unexpected error: {e}")
        logger.exception("Full traceback:")
        return 1


if __name__ == '__main__':
    sys.exit(main())
