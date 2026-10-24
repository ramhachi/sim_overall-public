#!/usr/bin/env python3
"""
Demonstration of C++ Header Export for Hybrid Tire Models

This script demonstrates how to export tire model parameters to C++ header
format for use in vehicle simulation code. It shows the complete workflow
from creating a hybrid configuration to generating a production-ready header.

Usage:
    python3 examples/cpp_export_demo.py
"""

import os
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from tire_fitting.exporters import (
    export_hybrid_config,
    export_cpp_header,
    HybridConfigData,
    ModelConfig,
    FittingMetrics
)


def main():
    """Demonstrate C++ header export functionality"""
    
    print("="*70)
    print("C++ Header Export Demonstration")
    print("="*70)
    print()
    
    # Create output directory
    output_dir = Path("output/cpp_demo")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Define tire parameters (example from design document)
    print("Step 1: Define tire model parameters")
    print("-" * 70)
    
    # Magic Formula parameters for longitudinal force
    fx_params = {
        'pDx1': 2.45,    # Peak force coefficient
        'pCx1': 1.65,    # Shape factor
        'pEx1': -0.12,   # Curvature factor
        'pKx1': 21.3,    # Stiffness coefficient
        'pKx2': -1.2,    # Load sensitivity
        'pKx3': 0.15,    # Exponential load factor
        'fz0': 1200.0    # Nominal load (N)
    }
    print(f"  Longitudinal (Magic Formula): {len(fx_params)} parameters")
    
    # TMeasy parameters for lateral force
    fy_params = {
        'mu_y_peak': 1.85,   # Peak lateral friction coefficient
        'k_mu_y': 0.1,       # Lateral load sensitivity
        'Ky': 10000.0,       # Lateral stiffness (N)
        'C_gamma': 0.05      # Camber thrust coefficient (1/rad)
    }
    print(f"  Lateral (TMeasy): {len(fy_params)} parameters")
    print()
    
    # Create hybrid configuration
    print("Step 2: Create hybrid configuration")
    print("-" * 70)
    
    config = HybridConfigData(
        fx_config=ModelConfig(
            model_type='magic_formula',
            parameters=fx_params,
            metrics=FittingMetrics(
                rmse=85.3,
                tic=0.12,
                r2=0.9845,
                peak_error=245.7
            )
        ),
        fy_config=ModelConfig(
            model_type='tmeasy',
            parameters=fy_params,
            metrics=FittingMetrics(
                rmse=119.2,
                tic=0.040,
                r2=0.9937,
                peak_error=312.5
            )
        ),
        selection_mode='auto',
        selection_rationale='Magic Formula selected for FX (RMSE: 85N vs 2547N), TMeasy for FY (RMSE: 119N vs 125N)',
        tire_name='Hoosier 18.0x6.0-10 R20',
        nominal_load=1200.0,
        nominal_pressure=12.0,
        data_points_fx=75860,
        data_points_fy=61032,
        optimization_method='differential_evolution',
        convergence_iterations=847
    )
    print(f"  Selection mode: {config.selection_mode}")
    print(f"  Tire: {config.tire_name}")
    print(f"  FX model: {config.fx_config.model_type} (RMSE: {config.fx_config.metrics.rmse:.1f} N)")
    print(f"  FY model: {config.fy_config.model_type} (RMSE: {config.fy_config.metrics.rmse:.1f} N)")
    print()
    
    # Export to YAML
    print("Step 3: Export to YAML configuration")
    print("-" * 70)
    yaml_path = output_dir / "hoosier_tire_config.yaml"
    export_hybrid_config(config, str(yaml_path))
    print(f"  ✓ YAML exported to: {yaml_path}")
    print(f"  ✓ File size: {yaml_path.stat().st_size} bytes")
    print()
    
    # Export to C++ header
    print("Step 4: Export to C++ header")
    print("-" * 70)
    cpp_path = output_dir / "hoosier_tire_params.h"
    export_cpp_header(
        fx_model_type=config.fx_config.model_type,
        fx_parameters=config.fx_config.parameters,
        fy_model_type=config.fy_config.model_type,
        fy_parameters=config.fy_config.parameters,
        tire_name=config.tire_name,
        filepath=str(cpp_path),
        namespace='fsae_tire'
    )
    print(f"  ✓ C++ header exported to: {cpp_path}")
    print(f"  ✓ File size: {cpp_path.stat().st_size} bytes")
    print()
    
    # Display C++ header content
    print("Step 5: Generated C++ header content")
    print("="*70)
    with open(cpp_path, 'r') as f:
        cpp_content = f.read()
    print(cpp_content)
    print("="*70)
    print()
    
    # Usage instructions
    print("Step 6: How to use in C++ code")
    print("-" * 70)
    print("""
To use this header in your C++ vehicle simulation:

1. Include the header:
   #include "hoosier_tire_params.h"

2. Instantiate the model:
   fsae_tire::HybridTireModel tire_model;

3. Access parameters:
   if (tire_model.fx_model_type == fsae_tire::ModelType::MAGIC_FORMULA) {
       double peak_coeff = tire_model.fx_params.pDx1;
       // Use Magic Formula for longitudinal force
   }
   
   if (tire_model.fy_model_type == fsae_tire::ModelType::TMEASY) {
       double mu_peak = tire_model.fy_params.mu_y_peak;
       // Use TMeasy for lateral force
   }

4. The header is self-contained and requires no external dependencies.
    """)
    print()
    
    print("="*70)
    print("✓ Demonstration complete!")
    print(f"✓ Output files in: {output_dir}")
    print("="*70)


if __name__ == '__main__':
    main()
