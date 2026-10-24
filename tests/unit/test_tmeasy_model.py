"""
Unit tests for TMeasy model

These tests verify specific examples and edge cases for the TMeasy tire model.
"""

import pytest
import numpy as np
from tire_fitting.tmeasy_model import TMeasyModel


class TestTMeasyModel:
    """Unit tests for TMeasyModel"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.model = TMeasyModel(fz_nominal=1000.0)
    
    def test_initialization(self):
        """Test model initialization"""
        model = TMeasyModel(fz_nominal=1200.0)
        assert model.fz_nominal == 1200.0
    
    def test_load_dependent_friction_at_nominal_load(self):
        """Test that friction equals mu_peak at nominal load"""
        fz = np.array([1000.0])
        mu_peak = 1.5
        k_mu = 0.2
        
        mu = self.model._compute_load_dependent_friction(fz, mu_peak, k_mu)
        
        # At nominal load, mu should equal mu_peak
        assert np.abs(mu[0] - mu_peak) < 1e-10
    
    def test_load_dependent_friction_above_nominal(self):
        """Test that friction decreases above nominal load"""
        fz = np.array([1500.0])  # 50% above nominal
        mu_peak = 1.5
        k_mu = 0.2
        
        mu = self.model._compute_load_dependent_friction(fz, mu_peak, k_mu)
        
        # Expected: mu = 1.5 * (1 - 0.2 * (1500 - 1000) / 1000)
        #              = 1.5 * (1 - 0.2 * 0.5)
        #              = 1.5 * 0.9 = 1.35
        expected_mu = 1.35
        assert np.abs(mu[0] - expected_mu) < 1e-10
    
    def test_load_dependent_friction_below_nominal(self):
        """Test that friction increases below nominal load"""
        fz = np.array([500.0])  # 50% below nominal
        mu_peak = 1.5
        k_mu = 0.2
        
        mu = self.model._compute_load_dependent_friction(fz, mu_peak, k_mu)
        
        # Expected: mu = 1.5 * (1 - 0.2 * (500 - 1000) / 1000)
        #              = 1.5 * (1 - 0.2 * (-0.5))
        #              = 1.5 * 1.1 = 1.65
        expected_mu = 1.65
        assert np.abs(mu[0] - expected_mu) < 1e-10
    
    def test_saturation_function_at_zero(self):
        """Test saturation function at s = 0"""
        s = np.array([0.0])
        mu_eff = self.model._compute_saturation_function(s)
        
        # At s = 0: mu_eff = 0 * (2 - 0) = 0
        assert np.abs(mu_eff[0] - 0.0) < 1e-10
    
    def test_saturation_function_at_half(self):
        """Test saturation function at s = 0.5"""
        s = np.array([0.5])
        mu_eff = self.model._compute_saturation_function(s)
        
        # At s = 0.5: mu_eff = 0.5 * (2 - 0.5) = 0.5 * 1.5 = 0.75
        expected = 0.75
        assert np.abs(mu_eff[0] - expected) < 1e-10
    
    def test_saturation_function_at_one(self):
        """Test saturation function at s = 1"""
        s = np.array([1.0])
        mu_eff = self.model._compute_saturation_function(s)
        
        # At s = 1: mu_eff = 1 * (2 - 1) = 1
        assert np.abs(mu_eff[0] - 1.0) < 1e-10
    
    def test_saturation_function_above_one(self):
        """Test saturation function at s > 1"""
        s = np.array([2.0])
        mu_eff = self.model._compute_saturation_function(s)
        
        # At s >= 1: mu_eff = 1
        assert np.abs(mu_eff[0] - 1.0) < 1e-10
    
    def test_longitudinal_force_zero_slip(self):
        """Test longitudinal force at zero slip ratio"""
        slip_ratio = 0.0
        fz = 1000.0
        mu_x_peak = 1.5
        k_mu_x = 0.2
        Kx = 20000.0
        
        Fx = self.model.compute_longitudinal_force(
            slip_ratio, fz, mu_x_peak, k_mu_x, Kx
        )
        
        # At zero slip, force should be zero
        assert np.abs(Fx) < 1e-10
    
    def test_longitudinal_force_zero_vertical_load(self):
        """Test longitudinal force at zero vertical load"""
        slip_ratio = 0.1
        fz = 0.0
        mu_x_peak = 1.5
        k_mu_x = 0.2
        Kx = 20000.0
        
        Fx = self.model.compute_longitudinal_force(
            slip_ratio, fz, mu_x_peak, k_mu_x, Kx
        )
        
        # At zero vertical load, force should be zero
        assert np.abs(Fx) < 1e-10
    
    def test_longitudinal_force_sign_convention(self):
        """Test longitudinal force sign convention"""
        fz = 1000.0
        mu_x_peak = 1.5
        k_mu_x = 0.0  # No load dependency for simplicity
        Kx = 20000.0
        
        # Positive slip ratio (driving)
        Fx_positive = self.model.compute_longitudinal_force(
            0.1, fz, mu_x_peak, k_mu_x, Kx
        )
        
        # Negative slip ratio (braking)
        Fx_negative = self.model.compute_longitudinal_force(
            -0.1, fz, mu_x_peak, k_mu_x, Kx
        )
        
        # Forces should have opposite signs
        assert Fx_positive * Fx_negative < 0
        
        # Magnitudes should be equal (symmetric model)
        assert np.abs(np.abs(Fx_positive) - np.abs(Fx_negative)) < 1e-6
    
    def test_longitudinal_force_saturation(self):
        """Test that longitudinal force saturates at high slip"""
        fz = 1000.0
        mu_x_peak = 1.5
        k_mu_x = 0.0
        Kx = 20000.0
        
        # Compute forces at increasing slip ratios
        # Use slip ratios that span from linear to saturated region
        # For saturation to occur, we need xi_bar = Kx * SR / (mu_x * Fz) >= 1
        # So SR >= mu_x * Fz / Kx = 1.5 * 1000 / 20000 = 0.075
        slip_ratios = [0.01, 0.05, 0.1, 0.2]
        forces = []
        for sr in slip_ratios:
            Fx = self.model.compute_longitudinal_force(
                sr, fz, mu_x_peak, k_mu_x, Kx
            )
            forces.append(np.abs(Fx))
        
        # Force should increase monotonically
        assert forces[1] > forces[0]
        assert forces[2] > forces[1]
        # At high slip, force should saturate (may be equal or slightly increasing)
        assert forces[3] >= forces[2]
        
        # Force should saturate (rate of increase should decrease)
        rate_1 = forces[1] - forces[0]
        rate_2 = forces[2] - forces[1]
        rate_3 = forces[3] - forces[2]
        
        # Rate should decrease as we approach saturation
        assert rate_2 < rate_1  # Decreasing rate of increase
        # At full saturation, rate_3 may be zero or very small
        assert rate_3 <= rate_2
    
    def test_lateral_force_zero_slip_angle(self):
        """Test lateral force at zero slip angle"""
        slip_angle = 0.0
        fz = 1000.0
        camber_angle = 0.0
        mu_y_peak = 1.5
        k_mu_y = 0.2
        Ky = 20000.0
        C_gamma = 0.05
        
        Fy = self.model.compute_lateral_force(
            slip_angle, fz, camber_angle, mu_y_peak, k_mu_y, Ky, C_gamma
        )
        
        # At zero slip angle and zero camber, force should be zero
        assert np.abs(Fy) < 1e-10
    
    def test_lateral_force_zero_vertical_load(self):
        """Test lateral force at zero vertical load"""
        slip_angle = 0.1
        fz = 0.0
        camber_angle = 0.0
        mu_y_peak = 1.5
        k_mu_y = 0.2
        Ky = 20000.0
        C_gamma = 0.05
        
        Fy = self.model.compute_lateral_force(
            slip_angle, fz, camber_angle, mu_y_peak, k_mu_y, Ky, C_gamma
        )
        
        # At zero vertical load, force should be zero
        assert np.abs(Fy) < 1e-10
    
    def test_lateral_force_with_camber_only(self):
        """Test lateral force with only camber (no slip angle)"""
        slip_angle = 0.0
        fz = 1000.0
        camber_angle = 0.05  # 5 degrees in radians ≈ 0.087 rad
        mu_y_peak = 1.5
        k_mu_y = 0.2
        Ky = 20000.0
        C_gamma = 0.05
        
        Fy = self.model.compute_lateral_force(
            slip_angle, fz, camber_angle, mu_y_peak, k_mu_y, Ky, C_gamma
        )
        
        # Expected camber thrust: C_gamma * camber * Fz
        expected_camber_thrust = C_gamma * camber_angle * fz
        
        # Force should equal camber thrust (no slip angle contribution)
        assert np.abs(Fy - expected_camber_thrust) < 1e-6
    
    def test_lateral_force_sign_convention(self):
        """Test lateral force sign convention"""
        fz = 1000.0
        camber_angle = 0.0
        mu_y_peak = 1.5
        k_mu_y = 0.0
        Ky = 20000.0
        C_gamma = 0.0
        
        # Positive slip angle
        Fy_positive = self.model.compute_lateral_force(
            0.1, fz, camber_angle, mu_y_peak, k_mu_y, Ky, C_gamma
        )
        
        # Negative slip angle
        Fy_negative = self.model.compute_lateral_force(
            -0.1, fz, camber_angle, mu_y_peak, k_mu_y, Ky, C_gamma
        )
        
        # Forces should have opposite signs
        assert Fy_positive * Fy_negative < 0
        
        # Magnitudes should be equal (symmetric model)
        assert np.abs(np.abs(Fy_positive) - np.abs(Fy_negative)) < 1e-6
    
    def test_lateral_force_saturation(self):
        """Test that lateral force saturates at high slip angle"""
        fz = 1000.0
        camber_angle = 0.0
        mu_y_peak = 1.5
        k_mu_y = 0.0
        Ky = 20000.0
        C_gamma = 0.0
        
        # Compute forces at increasing slip angles
        # Use slip angles that span from linear to saturated region
        # For saturation to occur, we need eta_bar = Ky * tan(SA) / (mu_y * Fz) >= 1
        # So tan(SA) >= mu_y * Fz / Ky = 1.5 * 1000 / 20000 = 0.075
        # SA >= arctan(0.075) ≈ 0.075 rad
        slip_angles = [0.01, 0.05, 0.1, 0.2]
        forces = []
        for sa in slip_angles:
            Fy = self.model.compute_lateral_force(
                sa, fz, camber_angle, mu_y_peak, k_mu_y, Ky, C_gamma
            )
            forces.append(np.abs(Fy))
        
        # Force should increase monotonically
        assert forces[1] > forces[0]
        assert forces[2] > forces[1]
        # At high slip, force should saturate (may be equal or slightly increasing)
        assert forces[3] >= forces[2]
        
        # Force should saturate (rate of increase should decrease)
        rate_1 = forces[1] - forces[0]
        rate_2 = forces[2] - forces[1]
        rate_3 = forces[3] - forces[2]
        
        # Rate should decrease as we approach saturation
        assert rate_2 < rate_1  # Decreasing rate of increase
        # At full saturation, rate_3 may be zero or very small
        assert rate_3 <= rate_2
    
    def test_array_input_handling(self):
        """Test that model handles array inputs correctly"""
        slip_ratios = np.array([0.0, 0.05, 0.1, 0.15])
        fz = np.array([800.0, 1000.0, 1200.0, 1400.0])
        mu_x_peak = 1.5
        k_mu_x = 0.2
        Kx = 20000.0
        
        Fx = self.model.compute_longitudinal_force(
            slip_ratios, fz, mu_x_peak, k_mu_x, Kx
        )
        
        # Should return array of same length
        assert len(Fx) == len(slip_ratios)
        
        # First element should be near zero (zero slip)
        assert np.abs(Fx[0]) < 1e-6
        
        # Other elements should be non-zero
        assert np.all(np.abs(Fx[1:]) > 0)
    
    def test_consistency_between_longitudinal_and_lateral(self):
        """Test that longitudinal and lateral models use same saturation function"""
        # Create conditions with same normalized deformation
        fz = 1000.0
        mu = 1.5
        K = 20000.0
        
        # For normalized deformation = 0.5
        deformation = 0.5
        
        # Longitudinal: xi_bar = K * SR / (mu * Fz)
        slip_ratio = deformation * mu * fz / K
        
        # Lateral: eta_bar = K * tan(SA) / (mu * Fz)
        slip_angle = np.arctan(deformation * mu * fz / K)
        
        # Compute forces
        Fx = self.model.compute_longitudinal_force(
            slip_ratio, fz, mu_x_peak=mu, k_mu_x=0.0, Kx=K
        )
        Fy = self.model.compute_lateral_force(
            slip_angle, fz, camber_angle=0.0,
            mu_y_peak=mu, k_mu_y=0.0, Ky=K, C_gamma=0.0
        )
        
        # Magnitudes should be equal (same saturation function)
        assert np.abs(np.abs(Fx) - np.abs(Fy)) < 1e-6
