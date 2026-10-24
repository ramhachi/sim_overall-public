"""
YAML Exporter for Hybrid Tire Model Configurations

This module provides functions for exporting and importing hybrid tire model
configurations to/from YAML format. It supports both hybrid configurations
(different models for Fx and Fy) and legacy TMeasy-only configurations.

Requirements: 6.1, 6.2, 6.4, 6.5, 6.6, 9.1, 9.2, 11.1, 11.2, 11.3, 11.4, 11.5, 11.6
"""

import yaml
from typing import Dict, Optional, Any
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, asdict
import numpy as np


@dataclass
class FittingMetrics:
    """Metrics for evaluating model fit quality
    
    Attributes:
        rmse: Root Mean Square Error (N)
        tic: Theil Inequality Coefficient (dimensionless)
        r2: R-squared coefficient (dimensionless)
        peak_error: Maximum absolute error (N)
    """
    rmse: float
    tic: float
    r2: float
    peak_error: float


@dataclass
class ModelConfig:
    """Configuration for a single tire model (Fx or Fy)
    
    Attributes:
        model_type: Model type ('tmeasy' or 'magic_formula')
        parameters: Dictionary of model parameters
        metrics: Fitting quality metrics
    """
    model_type: str
    parameters: Dict[str, float]
    metrics: FittingMetrics


@dataclass
class HybridConfigData:
    """Complete hybrid tire model configuration
    
    Attributes:
        fx_config: Configuration for longitudinal force model
        fy_config: Configuration for lateral force model
        selection_mode: Selection mode used ('auto', 'manual', or 'hybrid')
        selection_rationale: Explanation of why these models were selected
        tire_name: Tire identifier
        nominal_load: Nominal vertical load (N)
        nominal_pressure: Nominal tire pressure (psi)
        data_points_fx: Number of data points used for Fx fitting
        data_points_fy: Number of data points used for Fy fitting
        optimization_method: Optimization algorithm used
        convergence_iterations: Number of iterations to convergence
    """
    fx_config: ModelConfig
    fy_config: ModelConfig
    selection_mode: str
    selection_rationale: str
    tire_name: str = "Unknown"
    nominal_load: float = 1200.0
    nominal_pressure: float = 12.0
    data_points_fx: Optional[int] = None
    data_points_fy: Optional[int] = None
    optimization_method: str = "differential_evolution"
    convergence_iterations: Optional[int] = None


def _convert_numpy_types(obj: Any) -> Any:
    """Convert numpy types to native Python types for YAML serialization
    
    Args:
        obj: Object to convert (can be dict, list, numpy type, or primitive)
        
    Returns:
        Object with numpy types converted to Python types
    """
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, (np.integer, np.floating)):
        return float(obj)
    elif isinstance(obj, dict):
        return {k: _convert_numpy_types(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [_convert_numpy_types(item) for item in obj]
    else:
        return obj


def _validate_config_data(config: HybridConfigData) -> None:
    """Validate hybrid configuration data
    
    Args:
        config: Configuration to validate
        
    Raises:
        ValueError: If configuration is invalid
    """
    # Validate model types
    valid_types = ['tmeasy', 'magic_formula']
    if config.fx_config.model_type not in valid_types:
        raise ValueError(
            f"Invalid fx model type: {config.fx_config.model_type}. "
            f"Must be one of {valid_types}"
        )
    if config.fy_config.model_type not in valid_types:
        raise ValueError(
            f"Invalid fy model type: {config.fy_config.model_type}. "
            f"Must be one of {valid_types}"
        )
    
    # Validate selection mode
    valid_modes = ['auto', 'manual', 'hybrid']
    if config.selection_mode not in valid_modes:
        raise ValueError(
            f"Invalid selection mode: {config.selection_mode}. "
            f"Must be one of {valid_modes}"
        )
    
    # Validate parameters are not NaN or Inf
    for direction, model_config in [('fx', config.fx_config), ('fy', config.fy_config)]:
        for param_name, param_value in model_config.parameters.items():
            if not isinstance(param_value, (int, float)):
                continue
            if not np.isfinite(param_value):
                raise ValueError(
                    f"Invalid parameter {param_name} in {direction} model: {param_value}"
                )


def export_hybrid_config(
    config: HybridConfigData,
    filepath: str
) -> None:
    """Export hybrid tire model configuration to YAML file
    
    Creates a YAML file with the following structure:
    - model_config: Selection mode and rationale
    - longitudinal_force: Model type, parameters, and metrics for Fx
    - lateral_force: Model type, parameters, and metrics for Fy
    - tire_info: Tire specifications
    - fitting_info: Fitting metadata and statistics
    
    The YAML file includes comments with parameter units and descriptions
    for human readability.
    
    Args:
        config: Hybrid configuration data to export
        filepath: Output file path (will create parent directories)
        
    Raises:
        ValueError: If configuration data is invalid
        PermissionError: If output directory is not writable
        
    Requirements: 6.1, 6.2, 6.4, 11.1, 11.2, 11.3, 11.5
    """
    # Validate configuration
    _validate_config_data(config)
    
    # Convert numpy types
    config_dict = asdict(config)
    config_dict = _convert_numpy_types(config_dict)
    
    # Create output directory if needed
    output_path = Path(filepath)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Build YAML structure
    yaml_data = {
        # Model configuration section (Requirement 6.4)
        'model_config': {
            'selection_mode': config.selection_mode,
            'selection_rationale': config.selection_rationale
        },
        
        # Longitudinal force section (Requirements 6.1, 11.1, 11.2)
        'longitudinal_force': {
            'model_type': config.fx_config.model_type,
            'parameters': config.fx_config.parameters,
            'metrics': {
                'rmse': config.fx_config.metrics.rmse,
                'tic': config.fx_config.metrics.tic,
                'r2': config.fx_config.metrics.r2,
                'peak_error': config.fx_config.metrics.peak_error
            }
        },
        
        # Lateral force section (Requirements 6.1, 11.1, 11.3)
        'lateral_force': {
            'model_type': config.fy_config.model_type,
            'parameters': config.fy_config.parameters,
            'metrics': {
                'rmse': config.fy_config.metrics.rmse,
                'tic': config.fy_config.metrics.tic,
                'r2': config.fy_config.metrics.r2,
                'peak_error': config.fy_config.metrics.peak_error
            }
        },
        
        # Tire information section (Requirement 11.5)
        'tire_info': {
            'name': config.tire_name,
            'nominal_load': config.nominal_load,
            'nominal_pressure': config.nominal_pressure
        },
        
        # Fitting information section (Requirements 6.2, 11.5)
        'fitting_info': {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'data_points_fx': config.data_points_fx,
            'data_points_fy': config.data_points_fy,
            'optimization_method': config.optimization_method,
            'convergence_iterations': config.convergence_iterations
        }
    }
    
    # Write YAML file with comments
    try:
        with open(filepath, 'w') as f:
            # Write header comment
            f.write("# Hybrid Tire Model Configuration\n")
            f.write(f"# Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("#\n")
            f.write("# This file contains fitted parameters for a hybrid tire model\n")
            f.write("# that may use different models for longitudinal and lateral forces.\n")
            f.write("#\n")
            f.write("# Units:\n")
            f.write("#   - Forces: Newtons [N]\n")
            f.write("#   - Loads: Newtons [N]\n")
            f.write("#   - Pressure: PSI\n")
            f.write("#   - Angles: Radians [rad]\n")
            f.write("#   - Friction coefficients: Dimensionless [-]\n")
            f.write("#   - Stiffness: Newtons [N]\n")
            f.write("#\n\n")
            
            # Write model_config section
            f.write("# Model Selection Configuration\n")
            yaml.dump({'model_config': yaml_data['model_config']}, f, 
                     default_flow_style=False, sort_keys=False)
            f.write("\n")
            
            # Write longitudinal_force section with parameter descriptions
            f.write("# Longitudinal Force Model\n")
            f.write(f"# Model Type: {config.fx_config.model_type}\n")
            if config.fx_config.model_type == 'magic_formula':
                f.write("# Parameters (Magic Formula MF5.2):\n")
                f.write("#   pDx1: Peak force coefficient [-]\n")
                f.write("#   pCx1: Shape factor [-]\n")
                f.write("#   pEx1: Curvature factor [-]\n")
                f.write("#   pKx1: Stiffness coefficient [-]\n")
                f.write("#   pKx2: Load sensitivity for stiffness [-]\n")
                f.write("#   pKx3: Exponential load factor [-]\n")
                f.write("#   fz0: Nominal vertical load [N]\n")
            else:  # tmeasy
                f.write("# Parameters (TMeasy):\n")
                f.write("#   mu_x_peak: Peak longitudinal friction coefficient [-]\n")
                f.write("#   k_mu_x: Longitudinal load sensitivity [-]\n")
                f.write("#   Kx: Longitudinal stiffness parameter [N]\n")
            yaml.dump({'longitudinal_force': yaml_data['longitudinal_force']}, f,
                     default_flow_style=False, sort_keys=False)
            f.write("\n")
            
            # Write lateral_force section with parameter descriptions
            f.write("# Lateral Force Model\n")
            f.write(f"# Model Type: {config.fy_config.model_type}\n")
            if config.fy_config.model_type == 'magic_formula':
                f.write("# Parameters (Magic Formula MF5.2):\n")
                f.write("#   pDy1: Peak force coefficient [-]\n")
                f.write("#   pCy1: Shape factor [-]\n")
                f.write("#   pEy1: Curvature factor [-]\n")
                f.write("#   pKy1: Stiffness coefficient [-]\n")
                f.write("#   pKy2: Load sensitivity for stiffness [-]\n")
                f.write("#   pKy3: Exponential load factor [-]\n")
                f.write("#   fz0: Nominal vertical load [N]\n")
            else:  # tmeasy
                f.write("# Parameters (TMeasy):\n")
                f.write("#   mu_y_peak: Peak lateral friction coefficient [-]\n")
                f.write("#   k_mu_y: Lateral load sensitivity [-]\n")
                f.write("#   Ky: Lateral stiffness parameter [N]\n")
                f.write("#   C_gamma: Camber thrust coefficient [1/rad]\n")
            yaml.dump({'lateral_force': yaml_data['lateral_force']}, f,
                     default_flow_style=False, sort_keys=False)
            f.write("\n")
            
            # Write tire_info section
            f.write("# Tire Information\n")
            yaml.dump({'tire_info': yaml_data['tire_info']}, f,
                     default_flow_style=False, sort_keys=False)
            f.write("\n")
            
            # Write fitting_info section
            f.write("# Fitting Information\n")
            yaml.dump({'fitting_info': yaml_data['fitting_info']}, f,
                     default_flow_style=False, sort_keys=False)
            
    except PermissionError as e:
        raise PermissionError(
            f"Cannot write to output directory: permission denied"
        ) from e


def import_hybrid_config(filepath: str) -> HybridConfigData:
    """Import hybrid tire model configuration from YAML file
    
    Loads a hybrid configuration from YAML format. Supports both:
    - New hybrid format with model_type fields
    - Legacy TMeasy-only format (defaults to TMeasy for both directions)
    
    Args:
        filepath: Path to YAML configuration file
        
    Returns:
        HybridConfigData object containing the loaded configuration
        
    Raises:
        FileNotFoundError: If file does not exist
        yaml.YAMLError: If file has invalid YAML syntax
        ValueError: If required fields are missing or invalid
        
    Requirements: 6.6, 9.1, 9.2, 11.4
    """
    # Check file exists
    file_path = Path(filepath)
    if not file_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {filepath}")
    
    # Load YAML file
    try:
        with open(filepath, 'r') as f:
            yaml_data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise yaml.YAMLError(f"Invalid YAML syntax in {filepath}: {e}") from e
    
    if yaml_data is None:
        raise ValueError(f"Empty YAML file: {filepath}")
    
    # Detect format: hybrid or legacy TMeasy
    is_hybrid_format = (
        'longitudinal_force' in yaml_data and 
        'lateral_force' in yaml_data and
        'model_type' in yaml_data.get('longitudinal_force', {})
    )
    
    if is_hybrid_format:
        # Parse hybrid format (Requirement 6.6, 11.4)
        return _parse_hybrid_format(yaml_data, filepath)
    else:
        # Parse legacy TMeasy format (Requirements 9.1, 9.2)
        return _parse_legacy_format(yaml_data, filepath)


def _parse_hybrid_format(yaml_data: Dict, filepath: str) -> HybridConfigData:
    """Parse hybrid format YAML configuration
    
    Args:
        yaml_data: Parsed YAML data
        filepath: File path (for error messages)
        
    Returns:
        HybridConfigData object
        
    Raises:
        ValueError: If required fields are missing
    """
    # Validate required sections
    required_sections = ['longitudinal_force', 'lateral_force']
    missing = [s for s in required_sections if s not in yaml_data]
    if missing:
        raise ValueError(
            f"Missing required sections in {filepath}: {missing}"
        )
    
    # Parse longitudinal force configuration
    fx_data = yaml_data['longitudinal_force']
    if 'model_type' not in fx_data:
        raise ValueError(f"Missing 'model_type' in longitudinal_force section")
    if 'parameters' not in fx_data:
        raise ValueError(f"Missing 'parameters' in longitudinal_force section")
    
    fx_metrics = fx_data.get('metrics', {})
    fx_config = ModelConfig(
        model_type=fx_data['model_type'],
        parameters=fx_data['parameters'],
        metrics=FittingMetrics(
            rmse=fx_metrics.get('rmse', 0.0),
            tic=fx_metrics.get('tic', 0.0),
            r2=fx_metrics.get('r2', 0.0),
            peak_error=fx_metrics.get('peak_error', 0.0)
        )
    )
    
    # Parse lateral force configuration
    fy_data = yaml_data['lateral_force']
    if 'model_type' not in fy_data:
        raise ValueError(f"Missing 'model_type' in lateral_force section")
    if 'parameters' not in fy_data:
        raise ValueError(f"Missing 'parameters' in lateral_force section")
    
    fy_metrics = fy_data.get('metrics', {})
    fy_config = ModelConfig(
        model_type=fy_data['model_type'],
        parameters=fy_data['parameters'],
        metrics=FittingMetrics(
            rmse=fy_metrics.get('rmse', 0.0),
            tic=fy_metrics.get('tic', 0.0),
            r2=fy_metrics.get('r2', 0.0),
            peak_error=fy_metrics.get('peak_error', 0.0)
        )
    )
    
    # Parse model configuration
    model_config = yaml_data.get('model_config', {})
    selection_mode = model_config.get('selection_mode', 'manual')
    selection_rationale = model_config.get('selection_rationale', 'Loaded from file')
    
    # Parse tire info
    tire_info = yaml_data.get('tire_info', {})
    tire_name = tire_info.get('name', 'Unknown')
    nominal_load = tire_info.get('nominal_load', 1200.0)
    nominal_pressure = tire_info.get('nominal_pressure', 12.0)
    
    # Parse fitting info
    fitting_info = yaml_data.get('fitting_info', {})
    data_points_fx = fitting_info.get('data_points_fx')
    data_points_fy = fitting_info.get('data_points_fy')
    optimization_method = fitting_info.get('optimization_method', 'differential_evolution')
    convergence_iterations = fitting_info.get('convergence_iterations')
    
    return HybridConfigData(
        fx_config=fx_config,
        fy_config=fy_config,
        selection_mode=selection_mode,
        selection_rationale=selection_rationale,
        tire_name=tire_name,
        nominal_load=nominal_load,
        nominal_pressure=nominal_pressure,
        data_points_fx=data_points_fx,
        data_points_fy=data_points_fy,
        optimization_method=optimization_method,
        convergence_iterations=convergence_iterations
    )


def _parse_legacy_format(yaml_data: Dict, filepath: str) -> HybridConfigData:
    """Parse legacy TMeasy format YAML configuration
    
    Converts legacy TMeasy-only format to hybrid format by defaulting
    both force directions to TMeasy model.
    
    Args:
        yaml_data: Parsed YAML data
        filepath: File path (for error messages)
        
    Returns:
        HybridConfigData object with TMeasy for both directions
        
    Raises:
        ValueError: If required fields are missing
        
    Requirements: 9.1, 9.2
    """
    # Legacy format has 'longitudinal' and 'lateral' sections (not 'longitudinal_force')
    # Extract parameters from legacy format
    fx_params = {}
    fy_params = {}
    
    if 'longitudinal' in yaml_data:
        long_data = yaml_data['longitudinal']
        fx_params = {
            'mu_x_peak': long_data.get('mu_peak', 1.0),
            'k_mu_x': long_data.get('mu_sensitivity', 0.0),
            'Kx': long_data.get('Kx', 10000.0)
        }
    
    if 'lateral' in yaml_data:
        lat_data = yaml_data['lateral']
        fy_params = {
            'mu_y_peak': lat_data.get('mu_peak', 1.0),
            'k_mu_y': lat_data.get('mu_sensitivity', 0.0),
            'Ky': lat_data.get('Ky', 10000.0)
        }
        # Add camber parameter if present
        if 'camber' in yaml_data:
            fy_params['C_gamma'] = yaml_data['camber'].get('thrust_coefficient', 0.0)
    
    # Extract metrics from fitting_info if available
    fitting_info = yaml_data.get('fitting_info', {})
    fx_metrics = FittingMetrics(
        rmse=fitting_info.get('rmse_fx', 0.0),
        tic=fitting_info.get('tic_fx', 0.0),
        r2=fitting_info.get('r2_fx', 0.0),
        peak_error=0.0  # Not available in legacy format
    )
    fy_metrics = FittingMetrics(
        rmse=fitting_info.get('rmse_fy', 0.0),
        tic=fitting_info.get('tic_fy', 0.0),
        r2=fitting_info.get('r2_fy', 0.0),
        peak_error=0.0  # Not available in legacy format
    )
    
    # Create model configs with TMeasy as default (Requirement 9.1)
    fx_config = ModelConfig(
        model_type='tmeasy',
        parameters=fx_params,
        metrics=fx_metrics
    )
    fy_config = ModelConfig(
        model_type='tmeasy',
        parameters=fy_params,
        metrics=fy_metrics
    )
    
    # Extract tire info
    tire_name = yaml_data.get('tire_name', 'Unknown')
    nominal_load = yaml_data.get('nominal_load', 1200.0)
    nominal_pressure = 12.0  # Not in legacy format
    
    return HybridConfigData(
        fx_config=fx_config,
        fy_config=fy_config,
        selection_mode='manual',
        selection_rationale='Loaded from legacy TMeasy configuration',
        tire_name=tire_name,
        nominal_load=nominal_load,
        nominal_pressure=nominal_pressure,
        data_points_fx=None,
        data_points_fy=None,
        optimization_method='differential_evolution',
        convergence_iterations=None
    )
