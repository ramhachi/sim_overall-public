"""
Property-based tests for TMeasy model

These tests use the Hypothesis library to verify universal properties
across many randomly generated inputs for the TMeasy tire model.
"""

import pytest
import numpy as np
from hypothesis import given, strategies as st, settings, assume
from tire_fitting.tmeasy_model import TMeasyModel


class TestTMeasyModelProperties:
    """Property-based tests for TMeasyModel"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.model = TMeasyModel(fz_nominal=1000.0)
    
    @settings(max_examples=100)
    @given(
        slip_ratio=st.floats(min_value=-0.3, max_value=0.3, allow_nan=False, allow_infinity=False),
        fz=st.floats(min_value=500.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
        Kx=st.floats(min_value=5000.0, max_value=50000.0, allow_nan=False, allow_infinity=False),
        mu_x=st.floats(min_value=1.0, max_value=2.5, allow_nan=False, allow_infinity=False)
    )
    def test_property_7_longitudinal_normalized_deformation_formula(self, slip_ratio, fz, Kx, mu_x):
        """
        Property 7: Longitudinal Normalized Deformation Formula
        
        For any slip ratio SR, vertical load Fz, and parameters (Kx, mu_x), 
        the TMeasy model should calculate normalized deformation xi_bar as 
        (Kx * SR) / (mu_x * Fz).
        
        **Validates: Requirements 5.1**
        """
        # Compute expected normalized deformation
        expected_xi_bar = (Kx * slip_ratio) / (mu_x * fz)
        
        # We verify this property by checking the force calculation components
        # The model computes: xi_bar = Kx * SR / (mu_x * Fz)
        # Then: s = |xi_bar|, mu_eff = saturation(s), Fx = -mu_eff * mu_x * Fz * sign(xi_bar)
        
        # Use k_mu_x = 0 to avoid load dependency
        Fx = self.model.compute_longitudinal_force(
            slip_ratio, fz, mu_x_peak=mu_x, k_mu_x=0.0, Kx=Kx
        )
        
        # Manually compute using the same formula
        xi_bar_manual = (Kx * slip_ratio) / (mu_x * fz)
        s_manual = abs(xi_bar_manual)
        mu_eff_manual = self.model._compute_saturation_function(s_manual)
        sign_xi_manual = xi_bar_manual / s_manual if s_manual > 1e-10 else 0.0
        Fx_manual = -mu_eff_manual * mu_x * fz * sign_xi_manual
        
        # Property: Force computed with manual xi_bar should match model output
        assert abs(Fx - Fx_manual) < 1e-6, \
            f"Force calculation should use normalized deformation formula xi_bar = Kx * SR / (mu_x * Fz)"
    
    @settings(max_examples=100)
    @given(
        s=st.floats(min_value=0.0, max_value=3.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_8_saturation_function_piecewise_correctness(self, s):
        """
        Property 8: Saturation Function Piecewise Correctness
        
        For any normalized deformation magnitude s, the effective friction 
        should equal s * (2 - s) when s < 1, and should equal 1 when s >= 1.
        
        **Validates: Requirements 5.2, 5.3**
        """
        mu_eff = self.model._compute_saturation_function(s)
        
        if s < 1.0:
            # Property: mu_eff = s * (2 - s) for s < 1
            expected = s * (2.0 - s)
            assert abs(mu_eff - expected) < 1e-10, \
                f"Saturation function should equal s*(2-s) for s < 1"
        else:
            # Property: mu_eff = 1 for s >= 1
            assert abs(mu_eff - 1.0) < 1e-10, \
                f"Saturation function should equal 1 for s >= 1"
    
    @settings(max_examples=100)
    @given(
        slip_ratio=st.floats(min_value=-0.3, max_value=0.3, allow_nan=False, allow_infinity=False),
        fz=st.floats(min_value=500.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
        mu_x_peak=st.floats(min_value=1.0, max_value=2.5, allow_nan=False, allow_infinity=False),
        k_mu_x=st.floats(min_value=0.0, max_value=0.5, allow_nan=False, allow_infinity=False),
        Kx=st.floats(min_value=5000.0, max_value=50000.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_9_longitudinal_force_calculation_formula(self, slip_ratio, fz, mu_x_peak, k_mu_x, Kx):
        """
        Property 9: Longitudinal Force Calculation Formula
        
        For any effective friction mu_eff, friction coefficient mu_x, vertical 
        load Fz, and normalized deformation xi_bar with magnitude s, the 
        longitudinal force should equal -mu_eff * mu_x * Fz * (xi_bar / s).
        
        **Validates: Requirements 5.4**
        """
        # Skip zero slip ratio (undefined sign)
        assume(abs(slip_ratio) > 1e-6)
        
        # Compute force
        Fx = self.model.compute_longitudinal_force(
            slip_ratio, fz, mu_x_peak, k_mu_x, Kx
        )
        
        # Manually compute components
        mu_x = self.model._compute_load_dependent_friction(fz, mu_x_peak, k_mu_x)
        xi_bar = Kx * slip_ratio / (mu_x * fz)
        s = abs(xi_bar)
        mu_eff = self.model._compute_saturation_function(s)
        sign_xi = xi_bar / s if s > 1e-10 else 0.0
        
        # Expected force
        expected_Fx = -mu_eff * mu_x * fz * sign_xi
        
        # Property: Force should match formula
        assert abs(Fx - expected_Fx) < 1e-6, \
            f"Longitudinal force should match formula: -mu_eff * mu_x * Fz * sign(xi_bar)"
    
    @settings(max_examples=100)
    @given(
        fz=st.floats(min_value=500.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
        mu_peak=st.floats(min_value=1.0, max_value=2.5, allow_nan=False, allow_infinity=False),
        k_mu=st.floats(min_value=0.0, max_value=0.5, allow_nan=False, allow_infinity=False)
    )
    def test_property_10_load_dependent_friction_coefficient_formula(self, fz, mu_peak, k_mu):
        """
        Property 10: Load-Dependent Friction Coefficient Formula
        
        For any vertical load Fz, peak friction mu_peak, load sensitivity k_mu, 
        and nominal load Fz_nom, the load-dependent friction coefficient should 
        equal mu_peak * (1 - k_mu * (Fz - Fz_nom) / Fz_nom).
        
        **Validates: Requirements 5.5, 6.3**
        """
        fz_nom = self.model.fz_nominal
        
        # Compute friction coefficient
        mu = self.model._compute_load_dependent_friction(fz, mu_peak, k_mu)
        
        # Expected value
        expected_mu = mu_peak * (1.0 - k_mu * (fz - fz_nom) / fz_nom)
        
        # Property: Should match formula
        assert abs(mu - expected_mu) < 1e-10, \
            f"Load-dependent friction should match formula: mu_peak * (1 - k_mu * (Fz - Fz_nom) / Fz_nom)"
    
    @settings(max_examples=100)
    @given(
        slip_angle=st.floats(min_value=-0.3, max_value=0.3, allow_nan=False, allow_infinity=False),
        fz=st.floats(min_value=500.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
        Ky=st.floats(min_value=5000.0, max_value=50000.0, allow_nan=False, allow_infinity=False),
        mu_y=st.floats(min_value=1.0, max_value=2.5, allow_nan=False, allow_infinity=False)
    )
    def test_property_11_lateral_normalized_deformation_formula(self, slip_angle, fz, Ky, mu_y):
        """
        Property 11: Lateral Normalized Deformation Formula
        
        For any slip angle SA, vertical load Fz, and parameters (Ky, mu_y), 
        the TMeasy model should calculate normalized deformation eta_bar as 
        (Ky * tan(SA)) / (mu_y * Fz).
        
        **Validates: Requirements 6.1**
        """
        # Compute expected normalized deformation
        expected_eta_bar = (Ky * np.tan(slip_angle)) / (mu_y * fz)
        
        # We verify this property by checking the force calculation components
        # The model computes: eta_bar = Ky * tan(SA) / (mu_y * Fz)
        # Then: s = |eta_bar|, mu_eff = saturation(s), Fy = -mu_eff * mu_y * Fz * sign(eta_bar) + camber_thrust
        
        # Use k_mu_y = 0 and C_gamma = 0 to simplify
        Fy = self.model.compute_lateral_force(
            slip_angle, fz, camber_angle=0.0,
            mu_y_peak=mu_y, k_mu_y=0.0, Ky=Ky, C_gamma=0.0
        )
        
        # Manually compute using the same formula
        eta_bar_manual = (Ky * np.tan(slip_angle)) / (mu_y * fz)
        s_manual = abs(eta_bar_manual)
        mu_eff_manual = self.model._compute_saturation_function(s_manual)
        sign_eta_manual = eta_bar_manual / s_manual if s_manual > 1e-10 else 0.0
        Fy_manual = -mu_eff_manual * mu_y * fz * sign_eta_manual
        
        # Property: Force computed with manual eta_bar should match model output
        assert abs(Fy - Fy_manual) < 1e-6, \
            f"Force calculation should use normalized deformation formula eta_bar = Ky * tan(SA) / (mu_y * Fz)"
    
    @settings(max_examples=100)
    @given(
        slip_angle=st.floats(min_value=-0.2, max_value=0.2, allow_nan=False, allow_infinity=False),
        fz=st.floats(min_value=500.0, max_value=2000.0, allow_nan=False, allow_infinity=False),
        camber_angle=st.floats(min_value=-0.1, max_value=0.1, allow_nan=False, allow_infinity=False),
        mu_y_peak=st.floats(min_value=1.0, max_value=2.5, allow_nan=False, allow_infinity=False),
        k_mu_y=st.floats(min_value=0.0, max_value=0.5, allow_nan=False, allow_infinity=False),
        Ky=st.floats(min_value=5000.0, max_value=50000.0, allow_nan=False, allow_infinity=False),
        C_gamma=st.floats(min_value=0.0, max_value=0.1, allow_nan=False, allow_infinity=False)
    )
    def test_property_12_camber_thrust_addition(self, slip_angle, fz, camber_angle, mu_y_peak, k_mu_y, Ky, C_gamma):
        """
        Property 12: Camber Thrust Addition
        
        For any lateral force calculation with camber angle gamma, vertical 
        load Fz, and camber coefficient C_gamma, the camber contribution should 
        equal C_gamma * gamma * Fz and should be added to the base lateral force.
        
        **Validates: Requirements 6.2**
        """
        # Compute force with camber
        Fy_with_camber = self.model.compute_lateral_force(
            slip_angle, fz, camber_angle,
            mu_y_peak, k_mu_y, Ky, C_gamma
        )
        
        # Compute force without camber
        Fy_without_camber = self.model.compute_lateral_force(
            slip_angle, fz, 0.0,
            mu_y_peak, k_mu_y, Ky, 0.0
        )
        
        # Expected camber thrust
        expected_camber_thrust = C_gamma * camber_angle * fz
        
        # Property: Difference should equal camber thrust
        actual_camber_contribution = Fy_with_camber - Fy_without_camber
        assert abs(actual_camber_contribution - expected_camber_thrust) < 1e-6, \
            f"Camber contribution should equal C_gamma * camber * Fz"
    
    @settings(max_examples=100)
    @given(
        deformation=st.floats(min_value=0.0, max_value=3.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_13_saturation_function_consistency(self, deformation):
        """
        Property 13: Saturation Function Consistency
        
        For any normalized deformation (longitudinal or lateral), both 
        longitudinal and lateral force calculations should use the identical 
        saturation function mu_eff = s * (2 - s) for s < 1, else 1.
        
        **Validates: Requirements 6.4**
        """
        # Compute saturation function
        mu_eff = self.model._compute_saturation_function(deformation)
        
        # Expected value based on piecewise function
        if deformation < 1.0:
            expected = deformation * (2.0 - deformation)
        else:
            expected = 1.0
        
        # Property: Should match expected value
        assert abs(mu_eff - expected) < 1e-10, \
            f"Saturation function should be consistent for all force calculations"
        
        # Additional check: Verify it's the same function used in both directions
        # Create conditions with same normalized deformation for both directions
        fz = 1000.0
        mu = 1.5
        K = 20000.0
        
        # For longitudinal
        slip_ratio = deformation * mu * fz / K
        Fx = self.model.compute_longitudinal_force(
            slip_ratio, fz, mu_x_peak=mu, k_mu_x=0.0, Kx=K
        )
        
        # For lateral
        slip_angle = np.arctan(deformation * mu * fz / K)
        Fy = self.model.compute_lateral_force(
            slip_angle, fz, camber_angle=0.0,
            mu_y_peak=mu, k_mu_y=0.0, Ky=K, C_gamma=0.0
        )
        
        # Property: Magnitudes should be equal (same saturation function)
        # Allow some tolerance due to numerical precision
        if abs(deformation) > 1e-6:  # Skip near-zero deformations
            assert abs(abs(Fx) - abs(Fy)) / (abs(Fx) + 1e-10) < 1e-4, \
                f"Longitudinal and lateral forces should use same saturation function"
