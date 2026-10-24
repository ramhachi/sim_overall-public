"""
Unit tests for C++ header exporter

Tests the C++ header generation functionality for hybrid tire models.
"""

import pytest
import tempfile
import os
from pathlib import Path
from tire_fitting.exporters.cpp_exporter import (
    export_cpp_header,
    export_cpp_header_from_yaml_data,
    _sanitize_name,
    _generate_tmeasy_fx_struct,
    _generate_tmeasy_fy_struct,
    _generate_magic_formula_fx_struct,
    _generate_magic_formula_fy_struct,
    _generate_hybrid_model_struct
)


class TestSanitizeName:
    """Test tire name sanitization for C++ identifiers"""
    
    def test_basic_sanitization(self):
        """Test basic name sanitization"""
        assert _sanitize_name("Hoosier 18.0x6.0-10 R20") == "HOOSIER_18_0X6_0_10_R20"
    
    def test_special_characters(self):
        """Test removal of special characters"""
        assert _sanitize_name("Test-Tire (2024)") == "TEST_TIRE_2024"
    
    def test_consecutive_underscores(self):
        """Test removal of consecutive underscores"""
        assert _sanitize_name("Test  Tire") == "TEST_TIRE"
    
    def test_leading_trailing_underscores(self):
        """Test removal of leading/trailing underscores"""
        assert _sanitize_name(" Test Tire ") == "TEST_TIRE"


class TestStructGeneration:
    """Test parameter struct generation"""
    
    def test_tmeasy_fx_struct(self):
        """Test TMeasy Fx struct generation"""
        params = {
            'mu_x_peak': 1.5,
            'k_mu_x': 0.1,
            'Kx': 10000.0
        }
        struct = _generate_tmeasy_fx_struct(params)
        
        assert "struct TMeasyFxParams" in struct
        assert "double mu_x_peak = 1.500000" in struct
        assert "double k_mu_x = 0.100000" in struct
        assert "double Kx = 10000.000000" in struct
    
    def test_tmeasy_fy_struct(self):
        """Test TMeasy Fy struct generation"""
        params = {
            'mu_y_peak': 1.8,
            'k_mu_y': 0.2,
            'Ky': 12000.0,
            'C_gamma': 0.05
        }
        struct = _generate_tmeasy_fy_struct(params)
        
        assert "struct TMeasyFyParams" in struct
        assert "double mu_y_peak = 1.800000" in struct
        assert "double k_mu_y = 0.200000" in struct
        assert "double Ky = 12000.000000" in struct
        assert "double C_gamma = 0.050000" in struct
    
    def test_magic_formula_fx_struct(self):
        """Test Magic Formula Fx struct generation"""
        params = {
            'pDx1': 2.45,
            'pCx1': 1.65,
            'pEx1': -0.12,
            'pKx1': 21.3,
            'pKx2': -1.2,
            'pKx3': 0.15,
            'fz0': 1200.0
        }
        struct = _generate_magic_formula_fx_struct(params)
        
        assert "struct MagicFormulaFxParams" in struct
        assert "double pDx1 = 2.450000" in struct
        assert "double pCx1 = 1.650000" in struct
        assert "double pEx1 = -0.120000" in struct
        assert "double pKx1 = 21.300000" in struct
        assert "double pKx2 = -1.200000" in struct
        assert "double pKx3 = 0.150000" in struct
        assert "double fz0 = 1200.000000" in struct
    
    def test_magic_formula_fy_struct(self):
        """Test Magic Formula Fy struct generation"""
        params = {
            'pDy1': 2.5,
            'pCy1': 1.7,
            'pEy1': -0.1,
            'pKy1': 22.0,
            'pKy2': -1.5,
            'pKy3': 0.2,
            'fz0': 1200.0
        }
        struct = _generate_magic_formula_fy_struct(params)
        
        assert "struct MagicFormulaFyParams" in struct
        assert "double pDy1 = 2.500000" in struct
        assert "double pCy1 = 1.700000" in struct
        assert "double pEy1 = -0.100000" in struct
        assert "double pKy1 = 22.000000" in struct
        assert "double pKy2 = -1.500000" in struct
        assert "double pKy3 = 0.200000" in struct
        assert "double fz0 = 1200.000000" in struct
    
    def test_hybrid_model_struct(self):
        """Test hybrid model struct generation"""
        struct = _generate_hybrid_model_struct(
            'magic_formula', 'MagicFormulaFxParams',
            'tmeasy', 'TMeasyFyParams'
        )
        
        assert "struct HybridTireModel" in struct
        assert "ModelType fx_model_type = ModelType::MAGIC_FORMULA" in struct
        assert "MagicFormulaFxParams fx_params" in struct
        assert "ModelType fy_model_type = ModelType::TMEASY" in struct
        assert "TMeasyFyParams fy_params" in struct


class TestCppHeaderExport:
    """Test complete C++ header export"""
    
    def test_export_hybrid_magic_tmeasy(self):
        """Test export with Magic Formula for Fx and TMeasy for Fy"""
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "tire_params.h")
            
            fx_params = {
                'pDx1': 2.45,
                'pCx1': 1.65,
                'pEx1': -0.12,
                'pKx1': 21.3,
                'pKx2': -1.2,
                'pKx3': 0.15,
                'fz0': 1200.0
            }
            
            fy_params = {
                'mu_y_peak': 1.85,
                'k_mu_y': 0.1,
                'Ky': 10000.0,
                'C_gamma': 0.05
            }
            
            export_cpp_header(
                fx_model_type='magic_formula',
                fx_parameters=fx_params,
                fy_model_type='tmeasy',
                fy_parameters=fy_params,
                tire_name='Hoosier 18.0x6.0-10 R20',
                filepath=filepath
            )
            
            # Verify file was created
            assert os.path.exists(filepath)
            
            # Read and verify content
            with open(filepath, 'r') as f:
                content = f.read()
            
            # Check header guard
            assert "#ifndef TIRE_PARAMS_HOOSIER_18_0X6_0_10_R20_H" in content
            assert "#define TIRE_PARAMS_HOOSIER_18_0X6_0_10_R20_H" in content
            
            # Check namespace
            assert "namespace tire_model {" in content
            assert "} // namespace tire_model" in content
            
            # Check enum
            assert "enum class ModelType {" in content
            assert "TMEASY," in content
            assert "MAGIC_FORMULA" in content
            
            # Check structs
            assert "struct MagicFormulaFxParams {" in content
            assert "struct TMeasyFyParams {" in content
            assert "struct HybridTireModel {" in content
            
            # Check parameters
            assert "double pDx1 = 2.450000" in content
            assert "double mu_y_peak = 1.850000" in content
            
            # Check hybrid model configuration
            assert "ModelType fx_model_type = ModelType::MAGIC_FORMULA" in content
            assert "ModelType fy_model_type = ModelType::TMEASY" in content
    
    def test_export_tmeasy_only(self):
        """Test export with TMeasy for both directions"""
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "tire_params.h")
            
            fx_params = {
                'mu_x_peak': 1.5,
                'k_mu_x': 0.1,
                'Kx': 10000.0
            }
            
            fy_params = {
                'mu_y_peak': 1.8,
                'k_mu_y': 0.2,
                'Ky': 12000.0
            }
            
            export_cpp_header(
                fx_model_type='tmeasy',
                fx_parameters=fx_params,
                fy_model_type='tmeasy',
                fy_parameters=fy_params,
                tire_name='Test Tire',
                filepath=filepath
            )
            
            # Verify file was created
            assert os.path.exists(filepath)
            
            # Read and verify content
            with open(filepath, 'r') as f:
                content = f.read()
            
            # Check structs
            assert "struct TMeasyFxParams {" in content
            assert "struct TMeasyFyParams {" in content
            
            # Check hybrid model uses TMeasy for both
            assert "ModelType fx_model_type = ModelType::TMEASY" in content
            assert "ModelType fy_model_type = ModelType::TMEASY" in content
    
    def test_export_magic_formula_only(self):
        """Test export with Magic Formula for both directions"""
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "tire_params.h")
            
            fx_params = {
                'pDx1': 2.45,
                'pCx1': 1.65,
                'pEx1': -0.12,
                'pKx1': 21.3,
                'pKx2': -1.2,
                'pKx3': 0.15,
                'fz0': 1200.0
            }
            
            fy_params = {
                'pDy1': 2.5,
                'pCy1': 1.7,
                'pEy1': -0.1,
                'pKy1': 22.0,
                'pKy2': -1.5,
                'pKy3': 0.2,
                'fz0': 1200.0
            }
            
            export_cpp_header(
                fx_model_type='magic_formula',
                fx_parameters=fx_params,
                fy_model_type='magic_formula',
                fy_parameters=fy_params,
                tire_name='Test Tire',
                filepath=filepath
            )
            
            # Verify file was created
            assert os.path.exists(filepath)
            
            # Read and verify content
            with open(filepath, 'r') as f:
                content = f.read()
            
            # Check structs
            assert "struct MagicFormulaFxParams {" in content
            assert "struct MagicFormulaFyParams {" in content
            
            # Check hybrid model uses Magic Formula for both
            assert "ModelType fx_model_type = ModelType::MAGIC_FORMULA" in content
            assert "ModelType fy_model_type = ModelType::MAGIC_FORMULA" in content
    
    def test_invalid_model_type(self):
        """Test error handling for invalid model types"""
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "tire_params.h")
            
            with pytest.raises(ValueError, match="Invalid fx_model_type"):
                export_cpp_header(
                    fx_model_type='invalid_model',
                    fx_parameters={},
                    fy_model_type='tmeasy',
                    fy_parameters={},
                    tire_name='Test',
                    filepath=filepath
                )
            
            with pytest.raises(ValueError, match="Invalid fy_model_type"):
                export_cpp_header(
                    fx_model_type='tmeasy',
                    fx_parameters={},
                    fy_model_type='invalid_model',
                    fy_parameters={},
                    tire_name='Test',
                    filepath=filepath
                )
    
    def test_custom_namespace(self):
        """Test export with custom namespace"""
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "tire_params.h")
            
            export_cpp_header(
                fx_model_type='tmeasy',
                fx_parameters={'mu_x_peak': 1.5, 'Kx': 10000.0},
                fy_model_type='tmeasy',
                fy_parameters={'mu_y_peak': 1.8, 'Ky': 12000.0},
                tire_name='Test',
                filepath=filepath,
                namespace='custom_namespace'
            )
            
            with open(filepath, 'r') as f:
                content = f.read()
            
            assert "namespace custom_namespace {" in content
            assert "} // namespace custom_namespace" in content


class TestExportFromYamlData:
    """Test C++ export from YAML data"""
    
    def test_export_from_yaml_data(self):
        """Test export from YAML data structure"""
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "tire_params.h")
            
            yaml_data = {
                'longitudinal_force': {
                    'model_type': 'magic_formula',
                    'parameters': {
                        'pDx1': 2.45,
                        'pCx1': 1.65,
                        'pEx1': -0.12,
                        'pKx1': 21.3,
                        'pKx2': -1.2,
                        'pKx3': 0.15,
                        'fz0': 1200.0
                    }
                },
                'lateral_force': {
                    'model_type': 'tmeasy',
                    'parameters': {
                        'mu_y_peak': 1.85,
                        'k_mu_y': 0.1,
                        'Ky': 10000.0
                    }
                },
                'tire_info': {
                    'name': 'Hoosier 18.0x6.0-10 R20'
                }
            }
            
            export_cpp_header_from_yaml_data(yaml_data, filepath)
            
            # Verify file was created
            assert os.path.exists(filepath)
            
            # Read and verify content
            with open(filepath, 'r') as f:
                content = f.read()
            
            assert "struct MagicFormulaFxParams {" in content
            assert "struct TMeasyFyParams {" in content
            assert "double pDx1 = 2.450000" in content
            assert "double mu_y_peak = 1.850000" in content
    
    def test_missing_sections(self):
        """Test error handling for missing YAML sections"""
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "tire_params.h")
            
            # Missing longitudinal_force
            yaml_data = {
                'lateral_force': {
                    'model_type': 'tmeasy',
                    'parameters': {}
                }
            }
            
            with pytest.raises(ValueError, match="Missing 'longitudinal_force'"):
                export_cpp_header_from_yaml_data(yaml_data, filepath)
            
            # Missing lateral_force
            yaml_data = {
                'longitudinal_force': {
                    'model_type': 'tmeasy',
                    'parameters': {}
                }
            }
            
            with pytest.raises(ValueError, match="Missing 'lateral_force'"):
                export_cpp_header_from_yaml_data(yaml_data, filepath)
