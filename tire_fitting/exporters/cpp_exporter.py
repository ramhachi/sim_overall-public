"""
C++ Header Exporter for Hybrid Tire Model Configurations

This module provides functions for exporting hybrid tire model configurations
to C++ header format. It generates model-specific parameter structs and a
hybrid model configuration struct that can be directly included in C++ code.

Requirements: 6.3
"""

from typing import Dict, Optional
from pathlib import Path
from datetime import datetime
import re


def _sanitize_name(name: str) -> str:
    """Sanitize tire name for use in C++ identifiers
    
    Converts tire name to valid C++ identifier by:
    - Converting to uppercase
    - Replacing spaces and special characters with underscores
    - Removing consecutive underscores
    
    Args:
        name: Tire name to sanitize
        
    Returns:
        Sanitized name suitable for C++ identifiers
        
    Example:
        "Hoosier 18.0x6.0-10 R20" -> "HOOSIER_18_0X6_0_10_R20"
    """
    # Convert to uppercase
    sanitized = name.upper()
    
    # Replace special characters with underscores
    sanitized = re.sub(r'[^A-Z0-9_]', '_', sanitized)
    
    # Remove consecutive underscores
    sanitized = re.sub(r'_+', '_', sanitized)
    
    # Remove leading/trailing underscores
    sanitized = sanitized.strip('_')
    
    return sanitized


def _generate_tmeasy_fx_struct(parameters: Dict[str, float], indent: str = "    ") -> str:
    """Generate TMeasy longitudinal force parameter struct
    
    Args:
        parameters: Dictionary of TMeasy Fx parameters
        indent: Indentation string for struct members
        
    Returns:
        C++ struct definition as string
    """
    lines = [
        "// TMeasy parameters for longitudinal force",
        "struct TMeasyFxParams {",
    ]
    
    # Add parameters with default values
    if 'mu_x_peak' in parameters:
        lines.append(f"{indent}double mu_x_peak = {parameters['mu_x_peak']:.6f};")
    if 'k_mu_x' in parameters:
        lines.append(f"{indent}double k_mu_x = {parameters['k_mu_x']:.6f};")
    if 'Kx' in parameters:
        lines.append(f"{indent}double Kx = {parameters['Kx']:.6f};")
    
    lines.append("};")
    
    return "\n".join(lines)


def _generate_tmeasy_fy_struct(parameters: Dict[str, float], indent: str = "    ") -> str:
    """Generate TMeasy lateral force parameter struct
    
    Args:
        parameters: Dictionary of TMeasy Fy parameters
        indent: Indentation string for struct members
        
    Returns:
        C++ struct definition as string
    """
    lines = [
        "// TMeasy parameters for lateral force",
        "struct TMeasyFyParams {",
    ]
    
    # Add parameters with default values
    if 'mu_y_peak' in parameters:
        lines.append(f"{indent}double mu_y_peak = {parameters['mu_y_peak']:.6f};")
    if 'k_mu_y' in parameters:
        lines.append(f"{indent}double k_mu_y = {parameters['k_mu_y']:.6f};")
    if 'Ky' in parameters:
        lines.append(f"{indent}double Ky = {parameters['Ky']:.6f};")
    if 'C_gamma' in parameters:
        lines.append(f"{indent}double C_gamma = {parameters['C_gamma']:.6f};")
    
    lines.append("};")
    
    return "\n".join(lines)


def _generate_magic_formula_fx_struct(parameters: Dict[str, float], indent: str = "    ") -> str:
    """Generate Magic Formula longitudinal force parameter struct
    
    Args:
        parameters: Dictionary of Magic Formula Fx parameters
        indent: Indentation string for struct members
        
    Returns:
        C++ struct definition as string
    """
    lines = [
        "// Magic Formula parameters for longitudinal force",
        "struct MagicFormulaFxParams {",
    ]
    
    # Add parameters with default values
    if 'pDx1' in parameters:
        lines.append(f"{indent}double pDx1 = {parameters['pDx1']:.6f};")
    if 'pCx1' in parameters:
        lines.append(f"{indent}double pCx1 = {parameters['pCx1']:.6f};")
    if 'pEx1' in parameters:
        lines.append(f"{indent}double pEx1 = {parameters['pEx1']:.6f};")
    if 'pKx1' in parameters:
        lines.append(f"{indent}double pKx1 = {parameters['pKx1']:.6f};")
    if 'pKx2' in parameters:
        lines.append(f"{indent}double pKx2 = {parameters['pKx2']:.6f};")
    if 'pKx3' in parameters:
        lines.append(f"{indent}double pKx3 = {parameters['pKx3']:.6f};")
    if 'fz0' in parameters:
        lines.append(f"{indent}double fz0 = {parameters['fz0']:.6f};")
    
    lines.append("};")
    
    return "\n".join(lines)


def _generate_magic_formula_fy_struct(parameters: Dict[str, float], indent: str = "    ") -> str:
    """Generate Magic Formula lateral force parameter struct
    
    Args:
        parameters: Dictionary of Magic Formula Fy parameters
        indent: Indentation string for struct members
        
    Returns:
        C++ struct definition as string
    """
    lines = [
        "// Magic Formula parameters for lateral force",
        "struct MagicFormulaFyParams {",
    ]
    
    # Add parameters with default values
    if 'pDy1' in parameters:
        lines.append(f"{indent}double pDy1 = {parameters['pDy1']:.6f};")
    if 'pCy1' in parameters:
        lines.append(f"{indent}double pCy1 = {parameters['pCy1']:.6f};")
    if 'pEy1' in parameters:
        lines.append(f"{indent}double pEy1 = {parameters['pEy1']:.6f};")
    if 'pKy1' in parameters:
        lines.append(f"{indent}double pKy1 = {parameters['pKy1']:.6f};")
    if 'pKy2' in parameters:
        lines.append(f"{indent}double pKy2 = {parameters['pKy2']:.6f};")
    if 'pKy3' in parameters:
        lines.append(f"{indent}double pKy3 = {parameters['pKy3']:.6f};")
    if 'fz0' in parameters:
        lines.append(f"{indent}double fz0 = {parameters['fz0']:.6f};")
    
    lines.append("};")
    
    return "\n".join(lines)


def _generate_hybrid_model_struct(
    fx_model_type: str,
    fx_params_struct: str,
    fy_model_type: str,
    fy_params_struct: str,
    indent: str = "    "
) -> str:
    """Generate hybrid tire model configuration struct
    
    Args:
        fx_model_type: Model type for longitudinal force ('tmeasy' or 'magic_formula')
        fx_params_struct: Name of Fx parameter struct
        fy_model_type: Model type for lateral force ('tmeasy' or 'magic_formula')
        fy_params_struct: Name of Fy parameter struct
        indent: Indentation string for struct members
        
    Returns:
        C++ struct definition as string
    """
    # Convert model types to enum values
    fx_enum = "ModelType::MAGIC_FORMULA" if fx_model_type == "magic_formula" else "ModelType::TMEASY"
    fy_enum = "ModelType::MAGIC_FORMULA" if fy_model_type == "magic_formula" else "ModelType::TMEASY"
    
    lines = [
        "// Hybrid model configuration",
        "struct HybridTireModel {",
        f"{indent}ModelType fx_model_type = {fx_enum};",
        f"{indent}{fx_params_struct} fx_params;",
        "",
        f"{indent}ModelType fy_model_type = {fy_enum};",
        f"{indent}{fy_params_struct} fy_params;",
        "};",
    ]
    
    return "\n".join(lines)


def export_cpp_header(
    fx_model_type: str,
    fx_parameters: Dict[str, float],
    fy_model_type: str,
    fy_parameters: Dict[str, float],
    tire_name: str,
    filepath: str,
    namespace: str = "tire_model"
) -> None:
    """Export hybrid tire model configuration to C++ header file
    
    Generates a C++ header file with:
    - Header guards based on tire name
    - ModelType enum (TMEASY, MAGIC_FORMULA)
    - Model-specific parameter structs for Fx and Fy
    - HybridTireModel struct combining both models
    - Namespace wrapping
    
    Args:
        fx_model_type: Model type for longitudinal force ('tmeasy' or 'magic_formula')
        fx_parameters: Dictionary of Fx model parameters
        fy_model_type: Model type for lateral force ('tmeasy' or 'magic_formula')
        fy_parameters: Dictionary of Fy model parameters
        tire_name: Tire identifier (used for header guard)
        filepath: Output file path (will create parent directories)
        namespace: C++ namespace for generated code (default: 'tire_model')
        
    Raises:
        ValueError: If model types are invalid
        PermissionError: If output directory is not writable
        
    Requirements: 6.3
    
    Example:
        >>> export_cpp_header(
        ...     fx_model_type='magic_formula',
        ...     fx_parameters={'pDx1': 2.45, 'pCx1': 1.65, ...},
        ...     fy_model_type='tmeasy',
        ...     fy_parameters={'mu_y_peak': 1.85, 'Ky': 10000.0, ...},
        ...     tire_name='Hoosier 18.0x6.0-10 R20',
        ...     filepath='output/tire_params.h'
        ... )
    """
    # Validate model types
    valid_types = ['tmeasy', 'magic_formula']
    if fx_model_type not in valid_types:
        raise ValueError(
            f"Invalid fx_model_type: {fx_model_type}. Must be one of {valid_types}"
        )
    if fy_model_type not in valid_types:
        raise ValueError(
            f"Invalid fy_model_type: {fy_model_type}. Must be one of {valid_types}"
        )
    
    # Create output directory if needed
    output_path = Path(filepath)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Generate header guard name
    sanitized_name = _sanitize_name(tire_name)
    header_guard = f"TIRE_PARAMS_{sanitized_name}_H"
    
    # Generate timestamp
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    # Generate parameter structs based on model types
    fx_struct_name = ""
    fx_struct_code = ""
    
    if fx_model_type == "tmeasy":
        fx_struct_name = "TMeasyFxParams"
        fx_struct_code = _generate_tmeasy_fx_struct(fx_parameters)
    else:  # magic_formula
        fx_struct_name = "MagicFormulaFxParams"
        fx_struct_code = _generate_magic_formula_fx_struct(fx_parameters)
    
    fy_struct_name = ""
    fy_struct_code = ""
    
    if fy_model_type == "tmeasy":
        fy_struct_name = "TMeasyFyParams"
        fy_struct_code = _generate_tmeasy_fy_struct(fy_parameters)
    else:  # magic_formula
        fy_struct_name = "MagicFormulaFyParams"
        fy_struct_code = _generate_magic_formula_fy_struct(fy_parameters)
    
    # Generate hybrid model struct
    hybrid_struct_code = _generate_hybrid_model_struct(
        fx_model_type, fx_struct_name,
        fy_model_type, fy_struct_name
    )
    
    # Build complete header file
    header_lines = [
        "// Generated tire model parameters",
        f"// Timestamp: {timestamp}",
        f"// Tire: {tire_name}",
        f"// Longitudinal model: {fx_model_type}",
        f"// Lateral model: {fy_model_type}",
        "",
        f"#ifndef {header_guard}",
        f"#define {header_guard}",
        "",
        f"namespace {namespace} {{",
        "",
        "// Model type enumeration",
        "enum class ModelType {",
        "    TMEASY,",
        "    MAGIC_FORMULA",
        "};",
        "",
        fx_struct_code,
        "",
        fy_struct_code,
        "",
        hybrid_struct_code,
        "",
        f"}} // namespace {namespace}",
        "",
        f"#endif // {header_guard}",
        ""  # Final newline
    ]
    
    # Write header file
    try:
        with open(filepath, 'w') as f:
            f.write("\n".join(header_lines))
    except PermissionError as e:
        raise PermissionError(
            f"Cannot write to output directory: permission denied"
        ) from e


def export_cpp_header_from_yaml_data(
    yaml_data: Dict,
    filepath: str,
    namespace: str = "tire_model"
) -> None:
    """Export C++ header from parsed YAML configuration data
    
    Convenience function that extracts model types and parameters from
    YAML data structure and calls export_cpp_header.
    
    Args:
        yaml_data: Parsed YAML configuration data (from yaml.safe_load)
        filepath: Output file path
        namespace: C++ namespace for generated code (default: 'tire_model')
        
    Raises:
        ValueError: If required fields are missing from YAML data
        PermissionError: If output directory is not writable
        
    Requirements: 6.3
    """
    # Extract longitudinal force configuration
    if 'longitudinal_force' not in yaml_data:
        raise ValueError("Missing 'longitudinal_force' section in YAML data")
    
    fx_data = yaml_data['longitudinal_force']
    fx_model_type = fx_data.get('model_type', 'tmeasy')
    fx_parameters = fx_data.get('parameters', {})
    
    # Extract lateral force configuration
    if 'lateral_force' not in yaml_data:
        raise ValueError("Missing 'lateral_force' section in YAML data")
    
    fy_data = yaml_data['lateral_force']
    fy_model_type = fy_data.get('model_type', 'tmeasy')
    fy_parameters = fy_data.get('parameters', {})
    
    # Extract tire name
    tire_info = yaml_data.get('tire_info', {})
    tire_name = tire_info.get('name', 'Unknown')
    
    # Export header
    export_cpp_header(
        fx_model_type=fx_model_type,
        fx_parameters=fx_parameters,
        fy_model_type=fy_model_type,
        fy_parameters=fy_parameters,
        tire_name=tire_name,
        filepath=filepath,
        namespace=namespace
    )
