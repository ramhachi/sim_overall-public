"""
Integration test for C++ header export

Tests the complete workflow of exporting a C++ header from a YAML configuration.
"""

import pytest
import tempfile
import os
import yaml
from tire_fitting.exporters import (
    export_hybrid_config,
    export_cpp_header_from_yaml_data,
    HybridConfigData,
    ModelConfig,
    FittingMetrics
)


def test_yaml_to_cpp_export_workflow():
    """Test complete workflow: create YAML config, then export to C++ header"""
    with tempfile.TemporaryDirectory() as tmpdir:
        yaml_path = os.path.join(tmpdir, "tire_config.yaml")
        cpp_path = os.path.join(tmpdir, "tire_params.h")
        
        # Create hybrid configuration
        fx_config = ModelConfig(
            model_type='magic_formula',
            parameters={
                'pDx1': 2.45,
                'pCx1': 1.65,
                'pEx1': -0.12,
                'pKx1': 21.3,
                'pKx2': -1.2,
                'pKx3': 0.15,
                'fz0': 1200.0
            },
            metrics=FittingMetrics(
                rmse=85.3,
                tic=0.12,
                r2=0.9845,
                peak_error=245.7
            )
        )
        
        fy_config = ModelConfig(
            model_type='tmeasy',
            parameters={
                'mu_y_peak': 1.85,
                'k_mu_y': 0.1,
                'Ky': 10000.0,
                'C_gamma': 0.05
            },
            metrics=FittingMetrics(
                rmse=119.2,
                tic=0.040,
                r2=0.9937,
                peak_error=312.5
            )
        )
        
        config = HybridConfigData(
            fx_config=fx_config,
            fy_config=fy_config,
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
        
        # Export to YAML
        export_hybrid_config(config, yaml_path)
        
        # Verify YAML was created
        assert os.path.exists(yaml_path)
        
        # Load YAML and export to C++
        with open(yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
        
        export_cpp_header_from_yaml_data(yaml_data, cpp_path)
        
        # Verify C++ header was created
        assert os.path.exists(cpp_path)
        
        # Read and verify C++ header content
        with open(cpp_path, 'r') as f:
            cpp_content = f.read()
        
        # Verify header structure
        assert "#ifndef TIRE_PARAMS_HOOSIER_18_0X6_0_10_R20_H" in cpp_content
        assert "#define TIRE_PARAMS_HOOSIER_18_0X6_0_10_R20_H" in cpp_content
        assert "namespace tire_model {" in cpp_content
        
        # Verify enum
        assert "enum class ModelType {" in cpp_content
        assert "TMEASY," in cpp_content
        assert "MAGIC_FORMULA" in cpp_content
        
        # Verify structs
        assert "struct MagicFormulaFxParams {" in cpp_content
        assert "struct TMeasyFyParams {" in cpp_content
        assert "struct HybridTireModel {" in cpp_content
        
        # Verify parameters from Magic Formula
        assert "double pDx1 = 2.450000" in cpp_content
        assert "double pCx1 = 1.650000" in cpp_content
        assert "double pEx1 = -0.120000" in cpp_content
        assert "double pKx1 = 21.300000" in cpp_content
        assert "double pKx2 = -1.200000" in cpp_content
        assert "double pKx3 = 0.150000" in cpp_content
        assert "double fz0 = 1200.000000" in cpp_content
        
        # Verify parameters from TMeasy
        assert "double mu_y_peak = 1.850000" in cpp_content
        assert "double k_mu_y = 0.100000" in cpp_content
        assert "double Ky = 10000.000000" in cpp_content
        assert "double C_gamma = 0.050000" in cpp_content
        
        # Verify hybrid model configuration
        assert "ModelType fx_model_type = ModelType::MAGIC_FORMULA" in cpp_content
        assert "MagicFormulaFxParams fx_params" in cpp_content
        assert "ModelType fy_model_type = ModelType::TMEASY" in cpp_content
        assert "TMeasyFyParams fy_params" in cpp_content
        
        # Verify closing
        assert "} // namespace tire_model" in cpp_content
        assert "#endif // TIRE_PARAMS_HOOSIER_18_0X6_0_10_R20_H" in cpp_content


def test_cpp_header_syntax_validity():
    """Test that generated C++ header has valid syntax structure"""
    with tempfile.TemporaryDirectory() as tmpdir:
        cpp_path = os.path.join(tmpdir, "tire_params.h")
        
        yaml_data = {
            'longitudinal_force': {
                'model_type': 'tmeasy',
                'parameters': {
                    'mu_x_peak': 1.5,
                    'k_mu_x': 0.1,
                    'Kx': 10000.0
                }
            },
            'lateral_force': {
                'model_type': 'tmeasy',
                'parameters': {
                    'mu_y_peak': 1.8,
                    'k_mu_y': 0.2,
                    'Ky': 12000.0
                }
            },
            'tire_info': {
                'name': 'Test Tire'
            }
        }
        
        export_cpp_header_from_yaml_data(yaml_data, cpp_path)
        
        with open(cpp_path, 'r') as f:
            content = f.read()
        
        # Check for balanced braces
        assert content.count('{') == content.count('}')
        
        # Check for balanced header guards
        assert content.count('#ifndef') == 1
        assert content.count('#define') == 1
        assert content.count('#endif') == 1
        
        # Check namespace is properly closed
        assert 'namespace tire_model {' in content
        assert '} // namespace tire_model' in content
        
        # Check all structs are properly closed
        struct_count = content.count('struct ')
        # Note: enum also has }; so we need to account for that
        # We have: enum (1) + structs (3 for TMeasy-only case) = 4 total };
        closing_brace_semicolon = content.count('};')
        # Verify we have at least as many closing braces as structs
        assert closing_brace_semicolon >= struct_count
        
        # Check enum is properly formatted
        assert 'enum class ModelType {' in content
        assert content.count('TMEASY,') >= 1
        assert content.count('MAGIC_FORMULA') >= 1


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
