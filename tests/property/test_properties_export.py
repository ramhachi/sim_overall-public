"""
Property-based tests for parameter export functionality

These tests use the Hypothesis library to verify universal properties
across many randomly generated inputs.
"""

import pytest
import yaml
import tempfile
import re
from pathlib import Path
from hypothesis import given, strategies as st, settings
from tire_fitting.exporter import ParameterExporter


class TestExportProperties:
    """Property-based tests for ParameterExporter"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.exporter = ParameterExporter()
    
    @settings(max_examples=100)
    @given(
        mu_x_peak=st.floats(min_value=1.0, max_value=2.5, allow_nan=False, allow_infinity=False),
        k_mu_x=st.floats(min_value=0.0, max_value=0.5, allow_nan=False, allow_infinity=False),
        Kx=st.floats(min_value=5000.0, max_value=50000.0, allow_nan=False, allow_infinity=False),
        mu_y_peak=st.floats(min_value=1.0, max_value=2.5, allow_nan=False, allow_infinity=False),
        k_mu_y=st.floats(min_value=0.0, max_value=0.5, allow_nan=False, allow_infinity=False),
        Ky=st.floats(min_value=5000.0, max_value=50000.0, allow_nan=False, allow_infinity=False),
        C_gamma=st.floats(min_value=0.0, max_value=0.1, allow_nan=False, allow_infinity=False),
        rmse_fx=st.floats(min_value=0.0, max_value=200.0, allow_nan=False, allow_infinity=False),
        rmse_fy=st.floats(min_value=0.0, max_value=200.0, allow_nan=False, allow_infinity=False),
        r2_fx=st.floats(min_value=0.0, max_value=1.0, allow_nan=False, allow_infinity=False),
        r2_fy=st.floats(min_value=0.0, max_value=1.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_21_yaml_parameter_serialization_round_trip(
        self, mu_x_peak, k_mu_x, Kx, mu_y_peak, k_mu_y, Ky, C_gamma,
        rmse_fx, rmse_fy, r2_fx, r2_fy
    ):
        """
        Property 21: YAML Parameter Serialization Round Trip
        
        For any fitted parameters object, exporting to YAML and then parsing 
        the YAML file should produce a data structure containing all required 
        fields (tire_model, tire_name, effective_radius, nominal_load, 
        longitudinal section, lateral section, camber section, fitting_info 
        section) with their correct values.
        
        **Validates: Requirements 11.1, 11.2, 11.3, 11.4, 11.5**
        """
        # Create parameter dictionary
        params = {
            'mu_x_peak': mu_x_peak,
            'k_mu_x': k_mu_x,
            'Kx': Kx,
            'mu_y_peak': mu_y_peak,
            'k_mu_y': k_mu_y,
            'Ky': Ky,
            'C_gamma': C_gamma
        }
        
        # Create metrics dictionary
        metrics = {
            'rmse_fx': rmse_fx,
            'rmse_fy': rmse_fy,
            'r2_fx': r2_fx,
            'r2_fy': r2_fy,
            'nrmse_fx': 0.05,
            'nrmse_fy': 0.05,
            'tic_fx': 0.1,
            'tic_fy': 0.1
        }
        
        # Create temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_path = f.name
        
        try:
            # Export to YAML
            self.exporter.export_yaml(
                params=params,
                metrics=metrics,
                filepath=temp_path,
                tire_name="Test_Tire",
                effective_radius=0.2286,
                nominal_load=1000.0,
                data_source="Test Data"
            )
            
            # Load the YAML file
            with open(temp_path, 'r') as f:
                loaded = yaml.safe_load(f)
            
            # Property: tire_model field should exist (Requirement 11.1)
            assert 'tire_model' in loaded, \
                "YAML should contain tire_model field"
            assert loaded['tire_model'] == 'TMeasy', \
                "tire_model should be 'TMeasy'"
            
            # Property: tire_name field should exist (Requirement 11.1)
            assert 'tire_name' in loaded, \
                "YAML should contain tire_name field"
            assert loaded['tire_name'] == 'Test_Tire', \
                "tire_name should match input"
            
            # Property: effective_radius field should exist (Requirement 11.1)
            assert 'effective_radius' in loaded, \
                "YAML should contain effective_radius field"
            assert abs(loaded['effective_radius'] - 0.2286) < 1e-6, \
                "effective_radius should match input"
            
            # Property: nominal_load field should exist (Requirement 11.1)
            assert 'nominal_load' in loaded, \
                "YAML should contain nominal_load field"
            assert abs(loaded['nominal_load'] - 1000.0) < 1e-6, \
                "nominal_load should match input"
            
            # Property: longitudinal section should exist (Requirement 11.2)
            assert 'longitudinal' in loaded, \
                "YAML should contain longitudinal section"
            
            long_section = loaded['longitudinal']
            assert 'mu_peak' in long_section, \
                "Longitudinal section should contain mu_peak"
            assert abs(long_section['mu_peak'] - mu_x_peak) < 1e-6, \
                "Longitudinal mu_peak should match input"
            
            assert 'mu_sensitivity' in long_section, \
                "Longitudinal section should contain mu_sensitivity"
            assert abs(long_section['mu_sensitivity'] - k_mu_x) < 1e-6, \
                "Longitudinal mu_sensitivity should match input"
            
            assert 'Kx' in long_section, \
                "Longitudinal section should contain Kx"
            assert abs(long_section['Kx'] - Kx) < 1e-6, \
                "Longitudinal Kx should match input"
            
            assert 'relaxation_length' in long_section, \
                "Longitudinal section should contain relaxation_length"
            
            # Property: lateral section should exist (Requirement 11.3)
            assert 'lateral' in loaded, \
                "YAML should contain lateral section"
            
            lat_section = loaded['lateral']
            assert 'mu_peak' in lat_section, \
                "Lateral section should contain mu_peak"
            assert abs(lat_section['mu_peak'] - mu_y_peak) < 1e-6, \
                "Lateral mu_peak should match input"
            
            assert 'mu_sensitivity' in lat_section, \
                "Lateral section should contain mu_sensitivity"
            assert abs(lat_section['mu_sensitivity'] - k_mu_y) < 1e-6, \
                "Lateral mu_sensitivity should match input"
            
            assert 'Ky' in lat_section, \
                "Lateral section should contain Ky"
            assert abs(lat_section['Ky'] - Ky) < 1e-6, \
                "Lateral Ky should match input"
            
            assert 'relaxation_length' in lat_section, \
                "Lateral section should contain relaxation_length"
            
            # Property: camber section should exist (Requirement 11.4)
            assert 'camber' in loaded, \
                "YAML should contain camber section"
            
            camber_section = loaded['camber']
            assert 'thrust_coefficient' in camber_section, \
                "Camber section should contain thrust_coefficient"
            assert abs(camber_section['thrust_coefficient'] - C_gamma) < 1e-6, \
                "Camber thrust_coefficient should match input"
            
            # Property: fitting_info section should exist (Requirement 11.5)
            assert 'fitting_info' in loaded, \
                "YAML should contain fitting_info section"
            
            fit_info = loaded['fitting_info']
            assert 'data_source' in fit_info, \
                "fitting_info should contain data_source"
            assert fit_info['data_source'] == 'Test Data', \
                "data_source should match input"
            
            assert 'fit_date' in fit_info, \
                "fitting_info should contain fit_date"
            
            assert 'rmse_fx' in fit_info, \
                "fitting_info should contain rmse_fx"
            assert abs(fit_info['rmse_fx'] - rmse_fx) < 1e-6, \
                "rmse_fx should match input"
            
            assert 'rmse_fy' in fit_info, \
                "fitting_info should contain rmse_fy"
            assert abs(fit_info['rmse_fy'] - rmse_fy) < 1e-6, \
                "rmse_fy should match input"
            
            assert 'r2_fx' in fit_info, \
                "fitting_info should contain r2_fx"
            assert abs(fit_info['r2_fx'] - r2_fx) < 1e-6, \
                "r2_fx should match input"
            
            assert 'r2_fy' in fit_info, \
                "fitting_info should contain r2_fy"
            assert abs(fit_info['r2_fy'] - r2_fy) < 1e-6, \
                "r2_fy should match input"
            
        finally:
            # Clean up temporary file
            Path(temp_path).unlink()
    
    @settings(max_examples=50)
    @given(
        mu_x_peak=st.floats(min_value=1.0, max_value=2.5, allow_nan=False, allow_infinity=False),
        k_mu_x=st.floats(min_value=0.0, max_value=0.5, allow_nan=False, allow_infinity=False),
        Kx=st.floats(min_value=5000.0, max_value=50000.0, allow_nan=False, allow_infinity=False),
        mu_y_peak=st.floats(min_value=1.0, max_value=2.5, allow_nan=False, allow_infinity=False),
        k_mu_y=st.floats(min_value=0.0, max_value=0.5, allow_nan=False, allow_infinity=False),
        Ky=st.floats(min_value=5000.0, max_value=50000.0, allow_nan=False, allow_infinity=False),
        C_gamma=st.floats(min_value=0.0, max_value=0.1, allow_nan=False, allow_infinity=False)
    )
    def test_property_22_cpp_header_structure_correctness(
        self, mu_x_peak, k_mu_x, Kx, mu_y_peak, k_mu_y, Ky, C_gamma
    ):
        """
        Property 22: C++ Header Structure Correctness
        
        For any fitted parameters, the generated C++ header should contain 
        include guards, a struct with all parameters as static constexpr double 
        members, comments with tire name and units, and the struct should be 
        within the fsae::tire namespace.
        
        **Validates: Requirements 12.2, 12.3, 12.4**
        """
        # Create parameter dictionary
        params = {
            'mu_x_peak': mu_x_peak,
            'k_mu_x': k_mu_x,
            'Kx': Kx,
            'mu_y_peak': mu_y_peak,
            'k_mu_y': k_mu_y,
            'Ky': Ky,
            'C_gamma': C_gamma
        }
        
        # Create temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.hpp', delete=False) as f:
            temp_path = f.name
        
        try:
            # Export to C++ header
            self.exporter.export_cpp_header(
                params=params,
                tire_name="Test_Tire_Model",
                filepath=temp_path,
                effective_radius=0.2286,
                nominal_load=1000.0
            )
            
            # Read the generated header file
            with open(temp_path, 'r') as f:
                header_content = f.read()
            
            # Property: Should contain include guards (Requirement 12.1)
            assert '#ifndef' in header_content, \
                "Header should contain #ifndef include guard"
            assert '#define' in header_content, \
                "Header should contain #define include guard"
            assert '#endif' in header_content, \
                "Header should contain #endif include guard"
            
            # Extract guard name
            ifndef_match = re.search(r'#ifndef\s+(\w+)', header_content)
            assert ifndef_match is not None, \
                "Should find include guard name"
            guard_name = ifndef_match.group(1)
            
            # Property: Guard should appear in both #ifndef and #define
            assert f'#ifndef {guard_name}' in header_content, \
                "Guard name should appear in #ifndef"
            assert f'#define {guard_name}' in header_content, \
                "Guard name should appear in #define"
            assert f'#endif  // {guard_name}' in header_content, \
                "Guard name should appear in #endif comment"
            
            # Property: Should be in fsae::tire namespace (Requirement 12.4)
            assert 'namespace fsae' in header_content, \
                "Header should contain fsae namespace"
            assert 'namespace tire' in header_content, \
                "Header should contain tire namespace"
            assert '}  // namespace tire' in header_content, \
                "Header should close tire namespace"
            assert '}  // namespace fsae' in header_content, \
                "Header should close fsae namespace"
            
            # Property: Should contain struct definition (Requirement 12.2)
            assert 'struct' in header_content, \
                "Header should contain struct definition"
            
            # Property: Should contain static constexpr double members (Requirement 12.2)
            assert 'static constexpr double' in header_content, \
                "Header should contain static constexpr double members"
            
            # Property: All parameters should be present as members
            assert f'mu_x_peak = {mu_x_peak}' in header_content, \
                "Header should contain mu_x_peak parameter"
            assert f'k_mu_x = {k_mu_x}' in header_content, \
                "Header should contain k_mu_x parameter"
            assert f'Kx = {Kx}' in header_content, \
                "Header should contain Kx parameter"
            assert f'mu_y_peak = {mu_y_peak}' in header_content, \
                "Header should contain mu_y_peak parameter"
            assert f'k_mu_y = {k_mu_y}' in header_content, \
                "Header should contain k_mu_y parameter"
            assert f'Ky = {Ky}' in header_content, \
                "Header should contain Ky parameter"
            assert f'C_gamma = {C_gamma}' in header_content, \
                "Header should contain C_gamma parameter"
            
            # Property: Should contain comments with units (Requirement 12.3)
            assert '///' in header_content or '//' in header_content, \
                "Header should contain comments"
            assert '[N]' in header_content, \
                "Header should document force units"
            assert '[m]' in header_content, \
                "Header should document length units"
            assert '[-]' in header_content, \
                "Header should document dimensionless units"
            
            # Property: Should contain file header comment (Requirement 12.3)
            assert '@file' in header_content or 'TMeasy' in header_content, \
                "Header should contain file documentation"
            assert 'Generated on' in header_content or '@date' in header_content, \
                "Header should contain generation date"
            
            # Property: Should contain tire name in comments (Requirement 12.3)
            assert 'Test_Tire_Model' in header_content, \
                "Header should contain tire name"
            
        finally:
            # Clean up temporary file
            Path(temp_path).unlink()
