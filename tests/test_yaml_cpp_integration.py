"""
Integration test verifying YAML and C++ exporters work together

This test demonstrates the complete workflow:
1. Create hybrid configuration
2. Export to YAML
3. Load YAML
4. Export to C++ header
5. Verify both outputs are consistent
"""

import pytest
import tempfile
import os
import yaml
from tire_fitting.exporters import (
    export_hybrid_config,
    import_hybrid_config,
    export_cpp_header_from_yaml_data,
    HybridConfigData,
    ModelConfig,
    FittingMetrics
)


def test_yaml_cpp_round_trip():
    """Test complete round-trip: config -> YAML -> C++ header"""
    with tempfile.TemporaryDirectory() as tmpdir:
        yaml_path = os.path.join(tmpdir, "config.yaml")
        cpp_path = os.path.join(tmpdir, "params.h")
        
        # Create hybrid configuration
        config = HybridConfigData(
            fx_config=ModelConfig(
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
                metrics=FittingMetrics(rmse=85.3, tic=0.12, r2=0.9845, peak_error=245.7)
            ),
            fy_config=ModelConfig(
                model_type='tmeasy',
                parameters={
                    'mu_y_peak': 1.85,
                    'k_mu_y': 0.1,
                    'Ky': 10000.0,
                    'C_gamma': 0.05
                },
                metrics=FittingMetrics(rmse=119.2, tic=0.040, r2=0.9937, peak_error=312.5)
            ),
            selection_mode='auto',
            selection_rationale='Magic Formula for FX, TMeasy for FY',
            tire_name='Test Tire',
            nominal_load=1200.0,
            nominal_pressure=12.0
        )
        
        # Export to YAML
        export_hybrid_config(config, yaml_path)
        assert os.path.exists(yaml_path)
        
        # Load YAML
        with open(yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
        
        # Verify YAML structure
        assert 'longitudinal_force' in yaml_data
        assert 'lateral_force' in yaml_data
        assert yaml_data['longitudinal_force']['model_type'] == 'magic_formula'
        assert yaml_data['lateral_force']['model_type'] == 'tmeasy'
        
        # Export to C++ header
        export_cpp_header_from_yaml_data(yaml_data, cpp_path)
        assert os.path.exists(cpp_path)
        
        # Read C++ header
        with open(cpp_path, 'r') as f:
            cpp_content = f.read()
        
        # Verify C++ header contains correct model types
        assert 'ModelType fx_model_type = ModelType::MAGIC_FORMULA' in cpp_content
        assert 'ModelType fy_model_type = ModelType::TMEASY' in cpp_content
        
        # Verify parameters are present in C++ header
        assert 'double pDx1 = 2.450000' in cpp_content
        assert 'double pCx1 = 1.650000' in cpp_content
        assert 'double mu_y_peak = 1.850000' in cpp_content
        assert 'double Ky = 10000.000000' in cpp_content
        
        # Verify both files are consistent
        # Load YAML again and verify it matches original config
        loaded_config = import_hybrid_config(yaml_path)
        assert loaded_config.fx_config.model_type == config.fx_config.model_type
        assert loaded_config.fy_config.model_type == config.fy_config.model_type
        assert loaded_config.fx_config.parameters['pDx1'] == config.fx_config.parameters['pDx1']
        assert loaded_config.fy_config.parameters['mu_y_peak'] == config.fy_config.parameters['mu_y_peak']


def test_both_exporters_with_tmeasy_only():
    """Test both exporters with TMeasy-only configuration"""
    with tempfile.TemporaryDirectory() as tmpdir:
        yaml_path = os.path.join(tmpdir, "tmeasy_config.yaml")
        cpp_path = os.path.join(tmpdir, "tmeasy_params.h")
        
        # Create TMeasy-only configuration
        config = HybridConfigData(
            fx_config=ModelConfig(
                model_type='tmeasy',
                parameters={'mu_x_peak': 1.5, 'k_mu_x': 0.1, 'Kx': 10000.0},
                metrics=FittingMetrics(rmse=100.0, tic=0.05, r2=0.98, peak_error=200.0)
            ),
            fy_config=ModelConfig(
                model_type='tmeasy',
                parameters={'mu_y_peak': 1.8, 'k_mu_y': 0.2, 'Ky': 12000.0},
                metrics=FittingMetrics(rmse=120.0, tic=0.06, r2=0.97, peak_error=250.0)
            ),
            selection_mode='manual',
            selection_rationale='TMeasy for both directions',
            tire_name='TMeasy Test'
        )
        
        # Export to both formats
        export_hybrid_config(config, yaml_path)
        
        with open(yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
        
        export_cpp_header_from_yaml_data(yaml_data, cpp_path)
        
        # Verify both files exist
        assert os.path.exists(yaml_path)
        assert os.path.exists(cpp_path)
        
        # Verify C++ header uses TMeasy for both
        with open(cpp_path, 'r') as f:
            cpp_content = f.read()
        
        assert 'ModelType fx_model_type = ModelType::TMEASY' in cpp_content
        assert 'ModelType fy_model_type = ModelType::TMEASY' in cpp_content
        assert 'struct TMeasyFxParams' in cpp_content
        assert 'struct TMeasyFyParams' in cpp_content


def test_both_exporters_with_magic_formula_only():
    """Test both exporters with Magic Formula-only configuration"""
    with tempfile.TemporaryDirectory() as tmpdir:
        yaml_path = os.path.join(tmpdir, "mf_config.yaml")
        cpp_path = os.path.join(tmpdir, "mf_params.h")
        
        # Create Magic Formula-only configuration
        mf_fx_params = {
            'pDx1': 2.45, 'pCx1': 1.65, 'pEx1': -0.12,
            'pKx1': 21.3, 'pKx2': -1.2, 'pKx3': 0.15, 'fz0': 1200.0
        }
        mf_fy_params = {
            'pDy1': 2.5, 'pCy1': 1.7, 'pEy1': -0.1,
            'pKy1': 22.0, 'pKy2': -1.5, 'pKy3': 0.2, 'fz0': 1200.0
        }
        
        config = HybridConfigData(
            fx_config=ModelConfig(
                model_type='magic_formula',
                parameters=mf_fx_params,
                metrics=FittingMetrics(rmse=80.0, tic=0.04, r2=0.99, peak_error=180.0)
            ),
            fy_config=ModelConfig(
                model_type='magic_formula',
                parameters=mf_fy_params,
                metrics=FittingMetrics(rmse=90.0, tic=0.045, r2=0.985, peak_error=190.0)
            ),
            selection_mode='auto',
            selection_rationale='Magic Formula for both directions',
            tire_name='MF Test'
        )
        
        # Export to both formats
        export_hybrid_config(config, yaml_path)
        
        with open(yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
        
        export_cpp_header_from_yaml_data(yaml_data, cpp_path)
        
        # Verify both files exist
        assert os.path.exists(yaml_path)
        assert os.path.exists(cpp_path)
        
        # Verify C++ header uses Magic Formula for both
        with open(cpp_path, 'r') as f:
            cpp_content = f.read()
        
        assert 'ModelType fx_model_type = ModelType::MAGIC_FORMULA' in cpp_content
        assert 'ModelType fy_model_type = ModelType::MAGIC_FORMULA' in cpp_content
        assert 'struct MagicFormulaFxParams' in cpp_content
        assert 'struct MagicFormulaFyParams' in cpp_content


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
