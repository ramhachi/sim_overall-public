"""
Unit tests for HybridTireModel

These tests verify that the HybridTireModel correctly delegates force
calculations to the appropriate sub-models and maintains the unified
TireModel interface.

Requirements: 3.1, 3.2
"""

import pytest
import numpy as np
from tire_fitting.models import TMeasyModel, MagicFormulaModel, HybridTireModel, TireModel


class TestHybridTireModel:
    """Unit tests for HybridTireModel delegation behavior"""
    
    def setup_method(self):
        """Set up test fixtures with sample models"""
        # Create TMeasy model for lateral forces
        self.tmeasy_params = {
            'mu_x_peak': 1.0,
            'k_mu_x': 0.0,
            'Kx': 10000.0,
            'mu_y_peak': 1.0,
            'k_mu_y': 0.0,
            'Ky': 10000.0,
            'C_gamma': 0.0
        }
        self.tmeasy = TMeasyModel(fz_nominal=1000.0, params=self.tmeasy_params)
        
        # Create Magic Formula model for longitudinal forces
        self.mf_params = {
            'pDx1': 2.0,
            'pCx1': 1.5,
            'pEx1': -0.1,
            'pKx1': 20.0,
            'pKx2': -1.0,
            'pKx3': 0.1,
            'pDy1': 2.0,
            'pCy1': 1.5,
            'pEy1': -0.1,
            'pKy1': 20.0,
            'pKy2': -1.0,
            'pKy3': 0.1,
            'fz0': 1000.0
        }
        self.mf = MagicFormulaModel(params=self.mf_params)
    
    def test_initialization_with_valid_models(self):
        """Test that HybridTireModel initializes with valid TireModel instances"""
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        
        assert hybrid.fx_model is self.mf
        assert hybrid.fy_model is self.tmeasy
    
    def test_initialization_rejects_non_tire_models(self):
        """Test that HybridTireModel rejects objects that don't implement TireModel"""
        with pytest.raises(TypeError, match="fx_model must implement TireModel interface"):
            HybridTireModel(fx_model="not a model", fy_model=self.tmeasy)
        
        with pytest.raises(TypeError, match="fy_model must implement TireModel interface"):
            HybridTireModel(fx_model=self.mf, fy_model=42)
    
    def test_calculate_fx_delegates_to_fx_model(self):
        """Test that calculate_fx calls are delegated to fx_model"""
        # Create hybrid with MF for fx, TMeasy for fy
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        
        # Calculate force using hybrid
        slip_ratio = 0.1
        fz = 1000.0
        fx_hybrid = hybrid.calculate_fx(slip_ratio, fz)
        
        # Calculate force directly from MF model
        fx_direct = self.mf.calculate_fx(slip_ratio, fz)
        
        # Should be identical (delegated to same model)
        assert fx_hybrid == fx_direct
    
    def test_calculate_fy_delegates_to_fy_model(self):
        """Test that calculate_fy calls are delegated to fy_model"""
        # Create hybrid with MF for fx, TMeasy for fy
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        
        # Calculate force using hybrid
        slip_angle = 0.05
        fz = 1000.0
        fy_hybrid = hybrid.calculate_fy(slip_angle, fz)
        
        # Calculate force directly from TMeasy model
        fy_direct = self.tmeasy.calculate_fy(slip_angle, fz)
        
        # Should be identical (delegated to same model)
        assert fy_hybrid == fy_direct
    
    def test_fx_and_fy_use_different_models(self):
        """Test that fx and fy can use different models independently"""
        # Create hybrid with MF for fx, TMeasy for fy
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        
        # Calculate both forces
        slip_ratio = 0.1
        slip_angle = 0.05
        fz = 1000.0
        
        fx_hybrid = hybrid.calculate_fx(slip_ratio, fz)
        fy_hybrid = hybrid.calculate_fy(slip_angle, fz)
        
        # Verify fx comes from MF
        fx_mf = self.mf.calculate_fx(slip_ratio, fz)
        assert fx_hybrid == fx_mf
        
        # Verify fy comes from TMeasy
        fy_tmeasy = self.tmeasy.calculate_fy(slip_angle, fz)
        assert fy_hybrid == fy_tmeasy
        
        # Verify they're different from what the other model would produce
        fx_tmeasy = self.tmeasy.calculate_fx(slip_ratio, fz)
        fy_mf = self.mf.calculate_fy(slip_angle, fz)
        
        # Forces should differ (different models)
        assert fx_hybrid != fx_tmeasy
        assert fy_hybrid != fy_mf
    
    def test_get_model_type_returns_hybrid_description(self):
        """Test that get_model_type returns correct hybrid description"""
        # Test MF for fx, TMeasy for fy
        hybrid1 = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        model_type1 = hybrid1.get_model_type()
        
        assert "hybrid" in model_type1
        assert "magic_formula" in model_type1
        assert "tmeasy" in model_type1
        assert model_type1 == "hybrid(fx=magic_formula, fy=tmeasy)"
        
        # Test TMeasy for fx, MF for fy (reversed)
        hybrid2 = HybridTireModel(fx_model=self.tmeasy, fy_model=self.mf)
        model_type2 = hybrid2.get_model_type()
        
        assert "hybrid" in model_type2
        assert "magic_formula" in model_type2
        assert "tmeasy" in model_type2
        assert model_type2 == "hybrid(fx=tmeasy, fy=magic_formula)"
        
        # Different hybrids should have different type strings
        assert model_type1 != model_type2
    
    def test_get_model_type_with_same_model_both_directions(self):
        """Test get_model_type when same model is used for both directions"""
        # Use TMeasy for both
        hybrid = HybridTireModel(fx_model=self.tmeasy, fy_model=self.tmeasy)
        model_type = hybrid.get_model_type()
        
        assert model_type == "hybrid(fx=tmeasy, fy=tmeasy)"
    
    def test_get_parameters_returns_both_models(self):
        """Test that get_parameters returns parameters for both sub-models"""
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        params = hybrid.get_parameters()
        
        # Should have all required keys
        assert 'fx_model' in params
        assert 'fx_params' in params
        assert 'fy_model' in params
        assert 'fy_params' in params
        
        # Model types should be correct
        assert params['fx_model'] == 'magic_formula'
        assert params['fy_model'] == 'tmeasy'
        
        # Parameters should match the sub-models
        assert params['fx_params'] == self.mf.get_parameters()
        assert params['fy_params'] == self.tmeasy.get_parameters()
    
    def test_set_parameters_updates_fx_model(self):
        """Test that set_parameters updates fx_model parameters"""
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        
        # Update fx parameters
        new_fx_params = {'pDx1': 2.5, 'pCx1': 1.8}
        hybrid.set_parameters({'fx_params': new_fx_params})
        
        # Verify fx_model was updated
        updated_params = hybrid.fx_model.get_parameters()
        assert updated_params['pDx1'] == 2.5
        assert updated_params['pCx1'] == 1.8
        
        # Verify fy_model was not affected
        fy_params = hybrid.fy_model.get_parameters()
        assert fy_params == self.tmeasy.get_parameters()
    
    def test_set_parameters_updates_fy_model(self):
        """Test that set_parameters updates fy_model parameters"""
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        
        # Update fy parameters
        new_fy_params = {'mu_y_peak': 1.2, 'Ky': 12000.0}
        hybrid.set_parameters({'fy_params': new_fy_params})
        
        # Verify fy_model was updated
        updated_params = hybrid.fy_model.get_parameters()
        assert updated_params['mu_y_peak'] == 1.2
        assert updated_params['Ky'] == 12000.0
        
        # Verify fx_model was not affected
        fx_params = hybrid.fx_model.get_parameters()
        assert fx_params == self.mf.get_parameters()
    
    def test_set_parameters_updates_both_models(self):
        """Test that set_parameters can update both models simultaneously"""
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        
        # Update both
        new_params = {
            'fx_params': {'pDx1': 2.5},
            'fy_params': {'mu_y_peak': 1.2}
        }
        hybrid.set_parameters(new_params)
        
        # Verify both were updated
        fx_params = hybrid.fx_model.get_parameters()
        fy_params = hybrid.fy_model.get_parameters()
        
        assert fx_params['pDx1'] == 2.5
        assert fy_params['mu_y_peak'] == 1.2
    
    def test_set_parameters_with_empty_dict(self):
        """Test that set_parameters handles empty dict gracefully"""
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        
        # Get original parameters
        original_fx = hybrid.fx_model.get_parameters()
        original_fy = hybrid.fy_model.get_parameters()
        
        # Set with empty dict (should do nothing)
        hybrid.set_parameters({})
        
        # Verify nothing changed
        assert hybrid.fx_model.get_parameters() == original_fx
        assert hybrid.fy_model.get_parameters() == original_fy
    
    def test_calculate_forces_convenience_method(self):
        """Test that calculate_forces method works correctly"""
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        
        slip_ratio = 0.1
        slip_angle = 0.05
        fz = 1000.0
        
        # Use convenience method
        forces = hybrid.calculate_forces(slip_ratio, slip_angle, fz)
        
        # Verify both forces are calculated correctly
        fx_expected = self.mf.calculate_fx(slip_ratio, fz)
        fy_expected = self.tmeasy.calculate_fy(slip_angle, fz)
        
        assert forces.fx == fx_expected
        assert forces.fy == fy_expected
    
    def test_hybrid_implements_tire_model_interface(self):
        """Test that HybridTireModel properly implements TireModel interface"""
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        
        # Should be instance of TireModel
        assert isinstance(hybrid, TireModel)
        
        # Should have all required methods
        assert hasattr(hybrid, 'calculate_fx')
        assert hasattr(hybrid, 'calculate_fy')
        assert hasattr(hybrid, 'get_model_type')
        assert hasattr(hybrid, 'get_parameters')
        assert hasattr(hybrid, 'set_parameters')
        assert hasattr(hybrid, 'calculate_forces')
        
        # Methods should be callable
        assert callable(hybrid.calculate_fx)
        assert callable(hybrid.calculate_fy)
        assert callable(hybrid.get_model_type)
        assert callable(hybrid.get_parameters)
        assert callable(hybrid.set_parameters)
        assert callable(hybrid.calculate_forces)
    
    def test_kwargs_passed_to_sub_models(self):
        """Test that kwargs are passed through to sub-models"""
        # Create TMeasy with non-zero camber coefficient
        tmeasy_with_camber = TMeasyModel(
            fz_nominal=1000.0,
            params={
                'mu_y_peak': 1.0,
                'k_mu_y': 0.0,
                'Ky': 10000.0,
                'C_gamma': 0.05  # Non-zero camber coefficient
            }
        )
        hybrid = HybridTireModel(fx_model=self.tmeasy, fy_model=tmeasy_with_camber)
        
        # TMeasy supports camber_angle in calculate_fy
        slip_angle = 0.05
        fz = 1000.0
        camber_angle = 0.02
        
        # Calculate with camber through hybrid
        fy_hybrid = hybrid.calculate_fy(slip_angle, fz, camber_angle=camber_angle)
        
        # Calculate directly with camber
        fy_direct = tmeasy_with_camber.calculate_fy(slip_angle, fz, camber_angle=camber_angle)
        
        # Should be identical
        assert fy_hybrid == fy_direct
        
        # Should differ from calculation without camber (when C_gamma != 0)
        fy_no_camber = hybrid.calculate_fy(slip_angle, fz)
        assert abs(fy_hybrid - fy_no_camber) > 0.1  # Should have noticeable difference
    
    def test_parameter_isolation_between_models(self):
        """Test that parameter changes in one model don't affect the other"""
        # Create two separate TMeasy instances
        tmeasy1 = TMeasyModel(fz_nominal=1000.0, params=self.tmeasy_params.copy())
        tmeasy2 = TMeasyModel(fz_nominal=1000.0, params=self.tmeasy_params.copy())
        
        hybrid = HybridTireModel(fx_model=tmeasy1, fy_model=tmeasy2)
        
        # Modify fx_model parameters
        hybrid.set_parameters({'fx_params': {'mu_x_peak': 1.5}})
        
        # Verify fx_model changed
        assert hybrid.fx_model.get_parameters()['mu_x_peak'] == 1.5
        
        # Verify fy_model unchanged
        assert hybrid.fy_model.get_parameters()['mu_x_peak'] == 1.0
    
    def test_forces_with_different_vertical_loads(self):
        """Test that hybrid model handles different vertical loads correctly"""
        hybrid = HybridTireModel(fx_model=self.mf, fy_model=self.tmeasy)
        
        slip_ratio = 0.1
        slip_angle = 0.05
        
        # Test with different loads
        loads = [500.0, 1000.0, 1500.0, 2000.0]
        
        for fz in loads:
            fx = hybrid.calculate_fx(slip_ratio, fz)
            fy = hybrid.calculate_fy(slip_angle, fz)
            
            # Forces should be non-zero
            assert fx != 0.0
            assert fy != 0.0
            
            # Forces should match direct calculations
            assert fx == self.mf.calculate_fx(slip_ratio, fz)
            assert fy == self.tmeasy.calculate_fy(slip_angle, fz)
