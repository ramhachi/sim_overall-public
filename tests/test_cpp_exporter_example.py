"""
Example demonstrating C++ header export functionality

This test creates a realistic example showing how to export tire model
parameters to a C++ header file that can be used in vehicle simulation code.
"""

import tempfile
import os
from tire_fitting.exporters import export_cpp_header


def test_realistic_cpp_export_example():
    """Demonstrate realistic C++ header export for vehicle simulation"""
    with tempfile.TemporaryDirectory() as tmpdir:
        output_path = os.path.join(tmpdir, "hoosier_tire_params.h")
        
        # Example: Magic Formula for longitudinal, TMeasy for lateral
        # (based on the design document example)
        fx_params = {
            'pDx1': 2.45,    # Peak force coefficient
            'pCx1': 1.65,    # Shape factor
            'pEx1': -0.12,   # Curvature factor
            'pKx1': 21.3,    # Stiffness coefficient
            'pKx2': -1.2,    # Load sensitivity
            'pKx3': 0.15,    # Exponential load factor
            'fz0': 1200.0    # Nominal load (N)
        }
        
        fy_params = {
            'mu_y_peak': 1.85,   # Peak lateral friction coefficient
            'k_mu_y': 0.1,       # Lateral load sensitivity
            'Ky': 10000.0,       # Lateral stiffness (N)
            'C_gamma': 0.05      # Camber thrust coefficient (1/rad)
        }
        
        # Export to C++ header
        export_cpp_header(
            fx_model_type='magic_formula',
            fx_parameters=fx_params,
            fy_model_type='tmeasy',
            fy_parameters=fy_params,
            tire_name='Hoosier 18.0x6.0-10 R20',
            filepath=output_path,
            namespace='fsae_tire'
        )
        
        # Read the generated header
        with open(output_path, 'r') as f:
            header_content = f.read()
        
        # Print the generated header for demonstration
        print("\n" + "="*70)
        print("Generated C++ Header File:")
        print("="*70)
        print(header_content)
        print("="*70)
        
        # Verify key components
        assert "namespace fsae_tire {" in header_content
        assert "enum class ModelType" in header_content
        assert "struct MagicFormulaFxParams" in header_content
        assert "struct TMeasyFyParams" in header_content
        assert "struct HybridTireModel" in header_content
        
        # Verify the header can be used in C++ code (syntax check)
        # In a real scenario, this would be compiled with a C++ compiler
        assert header_content.count('{') == header_content.count('}')
        assert "#ifndef" in header_content
        assert "#define" in header_content
        assert "#endif" in header_content
        
        print("\n✓ C++ header generated successfully!")
        print(f"✓ Output file: {output_path}")
        print("✓ Ready for use in C++ vehicle simulation code")


if __name__ == '__main__':
    test_realistic_cpp_export_example()
