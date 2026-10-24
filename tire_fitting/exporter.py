"""
Parameter Exporter Module

This module provides the ParameterExporter class for exporting fitted
tire model parameters to YAML and C++ header files.
"""

import yaml
import logging
from typing import Dict, Optional
from pathlib import Path
from datetime import datetime


class ParameterExporter:
    """
    Exporter for fitted tire model parameters.
    
    This class provides methods for:
    - Exporting parameters to YAML format for general use
    - Exporting parameters to C++ header files for simulator integration
    
    The exporter ensures that fitted parameters are properly formatted
    and documented for downstream use in simulations and analysis.
    """
    
    def __init__(self):
        """Initialize the exporter with logging."""
        self.logger = logging.getLogger(__name__)
    
    def export_yaml(self,
                   params: Dict,
                   metrics: Dict,
                   filepath: str,
                   tire_name: str = "Unknown",
                   effective_radius: float = 0.2286,
                   nominal_load: float = 1000.0,
                   data_source: str = "FSAE TTC"):
        """
        Export fitted parameters to YAML file.
        
        Creates a YAML file with the following structure:
        - tire_model: Model type (TMeasy)
        - tire_name: Tire identifier
        - effective_radius: Tire effective radius [m]
        - nominal_load: Nominal vertical load [N]
        - longitudinal: Longitudinal parameters section
        - lateral: Lateral parameters section
        - camber: Camber parameters section
        - fitting_info: Fitting metadata and metrics
        
        Args:
            params: Dictionary containing fitted parameters
                   Expected keys: mu_x_peak, k_mu_x, Kx, mu_y_peak, 
                                 k_mu_y, Ky, C_gamma
            metrics: Dictionary containing evaluation metrics
                    Expected keys: rmse_fx, rmse_fy, r2_fx, r2_fy, etc.
            filepath: Output file path (will create parent directories)
            tire_name: Tire model name/identifier
            effective_radius: Tire effective radius [m]
            nominal_load: Nominal vertical load [N]
            data_source: Source of experimental data
            
        Raises:
            PermissionError: If output directory is not writable
            ValueError: If parameters contain NaN or infinite values
            
        Notes:
            - Implements Requirements 11.1, 11.2, 11.3, 11.4, 11.5
            - YAML format is human-readable and machine-parseable
            - Includes metadata for traceability
            - Relaxation lengths are computed from stiffness parameters
        """
        self.logger.info(f"Exporting parameters to YAML: {filepath}")
        
        # Validate parameters
        for key, value in params.items():
            if not isinstance(value, (int, float)):
                continue
            if not (abs(value) < float('inf')):
                self.logger.error(f"Invalid parameter value: {key} = {value}")
                raise ValueError(f"Cannot export invalid parameters: {key} is {value}")
        
        self.logger.debug(f"  Tire: {tire_name}")
        self.logger.debug(f"  Parameters: {list(params.keys())}")
        self.logger.debug(f"  Metrics: {list(metrics.keys())}")
        
        # Convert numpy types to native Python types
        import numpy as np
        def convert_numpy(obj):
            """Convert numpy types to native Python types for YAML serialization."""
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, (np.integer, np.floating)):
                return float(obj)
            elif isinstance(obj, dict):
                return {k: convert_numpy(v) for k, v in obj.items()}
            elif isinstance(obj, (list, tuple)):
                return [convert_numpy(item) for item in obj]
            else:
                return obj
        
        # Convert all parameters and metrics
        params = convert_numpy(params)
        metrics = convert_numpy(metrics)
        
        # Create output directory if needed
        output_path = Path(filepath)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.logger.debug(f"  Output directory: {output_path.parent}")
        
        # Compute relaxation lengths (reference values)
        # Lx = Kx / Cx, assuming Cx ~ 100000 N for typical FSAE tire
        # These are approximate values for reference
        Lx = params.get('Kx', 20000) / 100000.0
        Ly = params.get('Ky', 20000) / 100000.0
        
        # Build YAML structure (Requirements 11.1, 11.2, 11.3, 11.4)
        yaml_data = {
            'tire_model': 'TMeasy',
            'tire_name': tire_name,
            'effective_radius': effective_radius,
            'nominal_load': nominal_load,
            
            # Longitudinal parameters section (Requirement 11.2)
            'longitudinal': {
                'mu_peak': params.get('mu_x_peak', 1.5),
                'mu_sensitivity': params.get('k_mu_x', 0.1),
                'Kx': params.get('Kx', 20000.0),
                'relaxation_length': Lx
            },
            
            # Lateral parameters section (Requirement 11.3)
            'lateral': {
                'mu_peak': params.get('mu_y_peak', 1.5),
                'mu_sensitivity': params.get('k_mu_y', 0.1),
                'Ky': params.get('Ky', 20000.0),
                'relaxation_length': Ly
            },
            
            # Camber parameters section (Requirement 11.4)
            'camber': {
                'thrust_coefficient': params.get('C_gamma', 0.0)
            },
            
            # Fitting information section (Requirement 11.5)
            'fitting_info': {
                'data_source': data_source,
                'fit_date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'rmse_fx': metrics.get('rmse_fx', 0.0),
                'rmse_fy': metrics.get('rmse_fy', 0.0),
                'r2_fx': metrics.get('r2_fx', 0.0),
                'r2_fy': metrics.get('r2_fy', 0.0),
                'nrmse_fx': metrics.get('nrmse_fx', 0.0),
                'nrmse_fy': metrics.get('nrmse_fy', 0.0),
                'tic_fx': metrics.get('tic_fx', 0.0),
                'tic_fy': metrics.get('tic_fy', 0.0)
            }
        }
        
        # Write YAML file
        try:
            with open(filepath, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)
            self.logger.info(f"Successfully exported YAML file: {output_path.name}")
        except PermissionError as e:
            self.logger.error(f"Permission denied writing YAML file: {filepath}")
            raise PermissionError(f"Cannot write to output directory: permission denied") from e
    
    def export_cpp_header(self,
                         params: Dict,
                         tire_name: str,
                         filepath: str,
                         effective_radius: float = 0.2286,
                         nominal_load: float = 1000.0):
        """
        Export fitted parameters to C++ header file.
        
        Generates a C++ header file with:
        - Include guards
        - fsae::tire namespace
        - Struct with static constexpr double members for all parameters
        - Comments with tire name, generation date, and units
        
        Args:
            params: Dictionary containing fitted parameters
                   Expected keys: mu_x_peak, k_mu_x, Kx, mu_y_peak,
                                 k_mu_y, Ky, C_gamma
            tire_name: Tire model name (used for struct name)
            filepath: Output file path (should end with .h or .hpp)
            effective_radius: Tire effective radius [m]
            nominal_load: Nominal vertical load [N]
            
        Raises:
            PermissionError: If output directory is not writable
            ValueError: If parameters contain NaN or infinite values
            
        Notes:
            - Implements Requirements 12.1, 12.2, 12.3, 12.4, 12.5
            - Uses static constexpr for compile-time constants
            - Includes detailed comments for each parameter
            - Follows C++ naming conventions
        """
        self.logger.info(f"Exporting parameters to C++ header: {filepath}")
        
        # Validate parameters
        for key, value in params.items():
            if not isinstance(value, (int, float)):
                continue
            if not (abs(value) < float('inf')):
                self.logger.error(f"Invalid parameter value: {key} = {value}")
                raise ValueError(f"Cannot export invalid parameters: {key} is {value}")
        
        self.logger.debug(f"  Tire: {tire_name}")
        self.logger.debug(f"  Parameters: {list(params.keys())}")
        
        # Create output directory if needed
        output_path = Path(filepath)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.logger.debug(f"  Output directory: {output_path.parent}")
        
        # Generate struct name from tire name (replace spaces/special chars with underscores)
        struct_name = tire_name.replace(' ', '_').replace('-', '_').replace('.', '_')
        struct_name = ''.join(c if c.isalnum() or c == '_' else '_' for c in struct_name)
        self.logger.debug(f"  Struct name: {struct_name}")
        
        # Generate include guard name
        guard_name = f"FSAE_TIRE_{struct_name.upper()}_H"
        
        # Get current date for header comment
        current_date = datetime.now().strftime('%Y-%m-%d')
        
        # Build C++ header content
        header_lines = []
        
        # Include guard start (Requirement 12.1)
        header_lines.append(f"#ifndef {guard_name}")
        header_lines.append(f"#define {guard_name}")
        header_lines.append("")
        
        # File header comment (Requirement 12.3)
        header_lines.append("/**")
        header_lines.append(f" * @file {output_path.name}")
        header_lines.append(f" * @brief TMeasy tire model parameters for {tire_name}")
        header_lines.append(f" * @date Generated on {current_date}")
        header_lines.append(" *")
        header_lines.append(" * This file contains fitted parameters for the TMeasy tire model.")
        header_lines.append(" * Parameters were obtained from experimental data fitting.")
        header_lines.append(" *")
        header_lines.append(" * Units:")
        header_lines.append(" *   - Forces: Newtons [N]")
        header_lines.append(" *   - Lengths: Meters [m]")
        header_lines.append(" *   - Angles: Radians [rad]")
        header_lines.append(" *   - Friction coefficients: Dimensionless [-]")
        header_lines.append(" */")
        header_lines.append("")
        
        # Namespace start (Requirement 12.4)
        header_lines.append("namespace fsae {")
        header_lines.append("namespace tire {")
        header_lines.append("")
        
        # Struct definition (Requirement 12.2)
        header_lines.append(f"struct {struct_name} {{")
        header_lines.append("    // Basic tire properties")
        header_lines.append(f"    static constexpr double effective_radius = {effective_radius};  ///< Effective tire radius [m]")
        header_lines.append(f"    static constexpr double nominal_load = {nominal_load};  ///< Nominal vertical load [N]")
        header_lines.append("")
        
        # Longitudinal parameters
        header_lines.append("    // Longitudinal force parameters")
        header_lines.append(f"    static constexpr double mu_x_peak = {params.get('mu_x_peak', 1.5)};  ///< Peak longitudinal friction coefficient [-]")
        header_lines.append(f"    static constexpr double k_mu_x = {params.get('k_mu_x', 0.1)};  ///< Longitudinal load sensitivity [-]")
        header_lines.append(f"    static constexpr double Kx = {params.get('Kx', 20000.0)};  ///< Longitudinal stiffness parameter [N]")
        header_lines.append("")
        
        # Lateral parameters
        header_lines.append("    // Lateral force parameters")
        header_lines.append(f"    static constexpr double mu_y_peak = {params.get('mu_y_peak', 1.5)};  ///< Peak lateral friction coefficient [-]")
        header_lines.append(f"    static constexpr double k_mu_y = {params.get('k_mu_y', 0.1)};  ///< Lateral load sensitivity [-]")
        header_lines.append(f"    static constexpr double Ky = {params.get('Ky', 20000.0)};  ///< Lateral stiffness parameter [N]")
        header_lines.append("")
        
        # Camber parameters
        header_lines.append("    // Camber parameters")
        header_lines.append(f"    static constexpr double C_gamma = {params.get('C_gamma', 0.0)};  ///< Camber thrust coefficient [1/rad]")
        header_lines.append("};")
        header_lines.append("")
        
        # Namespace end
        header_lines.append("}  // namespace tire")
        header_lines.append("}  // namespace fsae")
        header_lines.append("")
        
        # Include guard end
        header_lines.append(f"#endif  // {guard_name}")
        header_lines.append("")
        
        # Write to file
        try:
            with open(filepath, 'w') as f:
                f.write('\n'.join(header_lines))
            self.logger.info(f"Successfully exported C++ header file: {output_path.name}")
        except PermissionError as e:
            self.logger.error(f"Permission denied writing C++ header file: {filepath}")
            raise PermissionError(f"Cannot write to output directory: permission denied") from e
