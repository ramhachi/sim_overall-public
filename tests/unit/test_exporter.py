"""
Unit tests for ParameterExporter

These tests verify specific examples and edge cases for parameter export.
"""

import pytest
import yaml
import tempfile
from pathlib import Path
from tire_fitting.exporter import ParameterExporter


class TestParameterExporter:
    """Unit tests for ParameterExporter class"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.exporter = ParameterExporter()
        
        # Sample parameters
        self.params = {
            'mu_x_peak': 1.5,
            'k_mu_x': 0.1,
            'Kx': 20000.0,
            'mu_y_peak': 1.6,
            'k_mu_y': 0.12,
            'Ky': 22000.0,
            'C_gamma': 0.05
        }
        
        # Sample metrics
        self.metrics = {
            'rmse_fx': 50.0,
            'rmse_fy': 45.0,
            'r2_fx': 0.95,
            'r2_fy': 0.96,
            'nrmse_fx': 0.05,
            'nrmse_fy': 0.04,
            'tic_fx': 0.08,
            'tic_fy': 0.07
        }
    
    def test_export_yaml_creates_valid_file(self):
        """Test that YAML export creates a valid, parseable file"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_path = f.name
        
        try:
            # Export to YAML
            self.exporter.export_yaml(
                params=self.params,
                metrics=self.metrics,
                filepath=temp_path,
                tire_name="Hoosier_18x6_R20",
                effective_radius=0.2286,
                nominal_load=1000.0
            )
            
            # Verify file exists
            assert Path(temp_path).exists()
            
            # Verify file is valid YAML
            with open(temp_path, 'r') as f:
                data = yaml.safe_load(f)
            
            # Verify basic structure
            assert data['tire_model'] == 'TMeasy'
            assert data['tire_name'] == 'Hoosier_18x6_R20'
            assert data['longitudinal']['mu_peak'] == 1.5
            assert data['lateral']['Ky'] == 22000.0
            
        finally:
            Path(temp_path).unlink()
    
    def test_export_yaml_with_invalid_params_raises_error(self):
        """Test that exporting NaN parameters raises ValueError"""
        invalid_params = self.params.copy()
        invalid_params['mu_x_peak'] = float('nan')
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_path = f.name
        
        try:
            with pytest.raises(ValueError, match="Cannot export invalid parameters"):
                self.exporter.export_yaml(
                    params=invalid_params,
                    metrics=self.metrics,
                    filepath=temp_path
                )
        finally:
            # Clean up if file was created
            if Path(temp_path).exists():
                Path(temp_path).unlink()
    
    def test_export_cpp_header_creates_valid_file(self):
        """Test that C++ header export creates a valid file"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.hpp', delete=False) as f:
            temp_path = f.name
        
        try:
            # Export to C++ header
            self.exporter.export_cpp_header(
                params=self.params,
                tire_name="Hoosier_18x6_R20",
                filepath=temp_path,
                effective_radius=0.2286,
                nominal_load=1000.0
            )
            
            # Verify file exists
            assert Path(temp_path).exists()
            
            # Read and verify content
            with open(temp_path, 'r') as f:
                content = f.read()
            
            # Verify include guards
            assert '#ifndef FSAE_TIRE_HOOSIER_18X6_R20_H' in content
            assert '#define FSAE_TIRE_HOOSIER_18X6_R20_H' in content
            assert '#endif' in content
            
            # Verify namespace
            assert 'namespace fsae' in content
            assert 'namespace tire' in content
            
            # Verify struct and parameters
            assert 'struct Hoosier_18x6_R20' in content
            assert 'static constexpr double mu_x_peak = 1.5' in content
            assert 'static constexpr double Ky = 22000.0' in content
            
        finally:
            Path(temp_path).unlink()
    
    def test_export_cpp_header_with_special_chars_in_name(self):
        """Test that special characters in tire name are handled correctly"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.hpp', delete=False) as f:
            temp_path = f.name
        
        try:
            # Export with special characters in name
            self.exporter.export_cpp_header(
                params=self.params,
                tire_name="Test-Tire.Model 2024",
                filepath=temp_path
            )
            
            # Read content
            with open(temp_path, 'r') as f:
                content = f.read()
            
            # Verify special chars are replaced with underscores
            assert 'struct Test_Tire_Model_2024' in content
            assert 'FSAE_TIRE_TEST_TIRE_MODEL_2024_H' in content
            
        finally:
            Path(temp_path).unlink()
    
    def test_export_cpp_header_with_invalid_params_raises_error(self):
        """Test that exporting infinite parameters raises ValueError"""
        invalid_params = self.params.copy()
        invalid_params['Kx'] = float('inf')
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.hpp', delete=False) as f:
            temp_path = f.name
        
        try:
            with pytest.raises(ValueError, match="Cannot export invalid parameters"):
                self.exporter.export_cpp_header(
                    params=invalid_params,
                    tire_name="Test",
                    filepath=temp_path
                )
        finally:
            # Clean up if file was created
            if Path(temp_path).exists():
                Path(temp_path).unlink()
    
    def test_yaml_contains_all_required_sections(self):
        """Test that YAML output contains all required sections"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_path = f.name
        
        try:
            self.exporter.export_yaml(
                params=self.params,
                metrics=self.metrics,
                filepath=temp_path
            )
            
            with open(temp_path, 'r') as f:
                data = yaml.safe_load(f)
            
            # Verify all required sections exist
            assert 'tire_model' in data
            assert 'tire_name' in data
            assert 'effective_radius' in data
            assert 'nominal_load' in data
            assert 'longitudinal' in data
            assert 'lateral' in data
            assert 'camber' in data
            assert 'fitting_info' in data
            
            # Verify longitudinal section
            assert 'mu_peak' in data['longitudinal']
            assert 'mu_sensitivity' in data['longitudinal']
            assert 'Kx' in data['longitudinal']
            assert 'relaxation_length' in data['longitudinal']
            
            # Verify lateral section
            assert 'mu_peak' in data['lateral']
            assert 'mu_sensitivity' in data['lateral']
            assert 'Ky' in data['lateral']
            assert 'relaxation_length' in data['lateral']
            
            # Verify camber section
            assert 'thrust_coefficient' in data['camber']
            
            # Verify fitting_info section
            assert 'data_source' in data['fitting_info']
            assert 'fit_date' in data['fitting_info']
            assert 'rmse_fx' in data['fitting_info']
            assert 'rmse_fy' in data['fitting_info']
            assert 'r2_fx' in data['fitting_info']
            assert 'r2_fy' in data['fitting_info']
            
        finally:
            Path(temp_path).unlink()
    
    def test_cpp_header_contains_all_parameters(self):
        """Test that C++ header contains all required parameters"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.hpp', delete=False) as f:
            temp_path = f.name
        
        try:
            self.exporter.export_cpp_header(
                params=self.params,
                tire_name="Test",
                filepath=temp_path
            )
            
            with open(temp_path, 'r') as f:
                content = f.read()
            
            # Verify all parameters are present
            assert 'effective_radius' in content
            assert 'nominal_load' in content
            assert 'mu_x_peak' in content
            assert 'k_mu_x' in content
            assert 'Kx' in content
            assert 'mu_y_peak' in content
            assert 'k_mu_y' in content
            assert 'Ky' in content
            assert 'C_gamma' in content
            
        finally:
            Path(temp_path).unlink()
