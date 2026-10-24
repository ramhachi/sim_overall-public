"""
TMeasy Model Module

This module provides the TMeasyModel class for computing tire forces
using the TMeasy (Tire Model Easy) semi-physical tire model.
"""

import numpy as np
from typing import Union


class TMeasyModel:
    """
    TMeasy tire model for steady-state force calculations.
    
    This class implements the TMeasy semi-physical tire model, which provides
    a simpler alternative to Pacejka Magic Formula with fewer parameters and
    no low-speed singularities. The model computes longitudinal and lateral
    forces based on slip ratio, slip angle, vertical load, and camber angle.
    
    Key features:
    - Load-dependent friction coefficients
    - Saturation function for realistic force behavior
    - Camber thrust for lateral forces
    - No singularities at low speeds
    """
    
    def __init__(self, fz_nominal: float = 1000.0):
        """
        Initialize TMeasy model.
        
        Args:
            fz_nominal: Nominal (reference) vertical load [N] (default: 1000.0)
                       Used for load-dependent friction coefficient calculations
        """
        self.fz_nominal = fz_nominal
    
    def _compute_load_dependent_friction(self,
                                        fz: Union[float, np.ndarray],
                                        mu_peak: float,
                                        k_mu: float) -> Union[float, np.ndarray]:
        """
        Compute load-dependent friction coefficient.
        
        The friction coefficient decreases with increasing vertical load
        according to: mu(Fz) = mu_peak * (1 - k_mu * (Fz - Fz_nom) / Fz_nom)
        
        Args:
            fz: Vertical load [N]
            mu_peak: Peak friction coefficient at nominal load [-]
            k_mu: Load sensitivity parameter [-]
            
        Returns:
            Load-dependent friction coefficient [-]
            
        Notes:
            - Implements Requirements 5.5, 6.3
            - k_mu typically ranges from 0 to 0.5
            - Higher k_mu means more sensitivity to load changes
        """
        # Ensure fz is array for vectorized operations
        fz_array = np.atleast_1d(fz)
        
        # Calculate load-dependent friction coefficient
        # mu(Fz) = mu_peak * (1 - k_mu * (Fz - Fz_nom) / Fz_nom)
        mu = mu_peak * (1.0 - k_mu * (fz_array - self.fz_nominal) / self.fz_nominal)
        
        # Return scalar if input was scalar
        return mu.item() if np.isscalar(fz) else mu
    
    def _compute_saturation_function(self, s: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        """
        Compute saturation function for effective friction.
        
        The saturation function provides smooth transition from linear to
        saturated behavior:
        - For s < 1: mu_eff = s * (2 - s)  (parabolic)
        - For s >= 1: mu_eff = 1           (saturated)
        
        Args:
            s: Normalized deformation magnitude [-]
            
        Returns:
            Effective friction coefficient [-]
            
        Notes:
            - Implements Requirements 5.2, 5.3, 6.4
            - Ensures smooth force saturation
            - Same function used for both longitudinal and lateral
        """
        # Ensure s is array for vectorized operations
        s_array = np.atleast_1d(s)
        
        # Compute saturation function
        mu_eff = np.where(s_array < 1.0,
                         s_array * (2.0 - s_array),  # Parabolic region
                         1.0)                         # Saturated region
        
        # Return scalar if input was scalar
        return mu_eff.item() if np.isscalar(s) else mu_eff
    
    def compute_longitudinal_force(self,
                                   slip_ratio: Union[float, np.ndarray],
                                   fz: Union[float, np.ndarray],
                                   mu_x_peak: float,
                                   k_mu_x: float,
                                   Kx: float) -> Union[float, np.ndarray]:
        """
        Compute longitudinal (tractive/braking) force.
        
        The longitudinal force is computed using:
        1. Load-dependent friction coefficient: mu_x(Fz)
        2. Normalized deformation: xi_bar = Kx * SR / (mu_x * Fz)
        3. Saturation function: mu_eff(|xi_bar|)
        4. Force: Fx = -mu_eff * mu_x * Fz * sign(xi_bar)
        
        Args:
            slip_ratio: Longitudinal slip ratio [-]
                       Positive for acceleration, negative for braking
            fz: Vertical load [N]
            mu_x_peak: Peak longitudinal friction coefficient [-]
            k_mu_x: Longitudinal load sensitivity [-]
            Kx: Longitudinal stiffness parameter Cx*Lx [N]
            
        Returns:
            Longitudinal force Fx [N]
            Positive for acceleration, negative for braking
            
        Notes:
            - Implements Requirements 5.1, 5.2, 5.3, 5.4, 5.5
            - Handles zero vertical load by returning zero force
            - Vectorized for efficient batch computation
        """
        # Ensure inputs are arrays
        sr_array = np.atleast_1d(slip_ratio)
        fz_array = np.atleast_1d(fz)
        
        # Handle zero vertical load
        if np.any(fz_array == 0):
            # Return zero force for zero load
            fx = np.zeros_like(sr_array, dtype=float)
            return fx.item() if np.isscalar(slip_ratio) else fx
        
        # 1. Compute load-dependent friction coefficient (Requirement 5.5)
        mu_x = self._compute_load_dependent_friction(fz_array, mu_x_peak, k_mu_x)
        
        # 2. Compute normalized deformation (Requirement 5.1)
        # xi_bar = Kx * SR / (mu_x * Fz)
        xi_bar = Kx * sr_array / (mu_x * fz_array)
        
        # 3. Compute magnitude and saturation function (Requirements 5.2, 5.3)
        s = np.abs(xi_bar)
        mu_eff = self._compute_saturation_function(s)
        
        # 4. Compute longitudinal force (Requirement 5.4)
        # Fx = -mu_eff * mu_x * Fz * (xi_bar / s)
        # Note: xi_bar / s = sign(xi_bar)
        # Handle s = 0 case (no slip)
        with np.errstate(divide='ignore', invalid='ignore'):
            sign_xi = np.where(s > 1e-10, xi_bar / s, 0.0)
        
        fx = -mu_eff * mu_x * fz_array * sign_xi
        
        # Return scalar if input was scalar
        return fx.item() if np.isscalar(slip_ratio) else fx
    
    def compute_lateral_force(self,
                             slip_angle: Union[float, np.ndarray],
                             fz: Union[float, np.ndarray],
                             camber_angle: Union[float, np.ndarray],
                             mu_y_peak: float,
                             k_mu_y: float,
                             Ky: float,
                             C_gamma: float = 0.0) -> Union[float, np.ndarray]:
        """
        Compute lateral (cornering) force.
        
        The lateral force is computed using:
        1. Load-dependent friction coefficient: mu_y(Fz)
        2. Normalized deformation: eta_bar = Ky * tan(SA) / (mu_y * Fz)
        3. Saturation function: mu_eff(|eta_bar|)
        4. Base force: Fy_base = -mu_eff * mu_y * Fz * sign(eta_bar)
        5. Camber thrust: Fy_camber = C_gamma * camber * Fz
        6. Total force: Fy = Fy_base + Fy_camber
        
        Args:
            slip_angle: Lateral slip angle [rad]
                       Positive for left turn (right lateral force)
            fz: Vertical load [N]
            camber_angle: Camber angle [rad]
                         Positive for top of tire leaning outward
            mu_y_peak: Peak lateral friction coefficient [-]
            k_mu_y: Lateral load sensitivity [-]
            Ky: Lateral stiffness parameter Cy*Ly [N]
            C_gamma: Camber thrust coefficient [1/rad] (default: 0.0)
            
        Returns:
            Lateral force Fy [N]
            
        Notes:
            - Implements Requirements 6.1, 6.2, 6.3, 6.4
            - Handles zero vertical load by returning zero force
            - Uses same saturation function as longitudinal force
            - Vectorized for efficient batch computation
        """
        # Ensure inputs are arrays
        sa_array = np.atleast_1d(slip_angle)
        fz_array = np.atleast_1d(fz)
        gamma_array = np.atleast_1d(camber_angle)
        
        # Handle zero vertical load
        if np.any(fz_array == 0):
            # Return zero force for zero load
            fy = np.zeros_like(sa_array, dtype=float)
            return fy.item() if np.isscalar(slip_angle) else fy
        
        # 1. Compute load-dependent friction coefficient (Requirement 6.3)
        mu_y = self._compute_load_dependent_friction(fz_array, mu_y_peak, k_mu_y)
        
        # 2. Compute normalized deformation (Requirement 6.1)
        # eta_bar = Ky * tan(SA) / (mu_y * Fz)
        eta_bar = Ky * np.tan(sa_array) / (mu_y * fz_array)
        
        # 3. Compute magnitude and saturation function (Requirement 6.4)
        s = np.abs(eta_bar)
        mu_eff = self._compute_saturation_function(s)
        
        # 4. Compute base lateral force
        # Fy_base = -mu_eff * mu_y * Fz * (eta_bar / s)
        # Note: eta_bar / s = sign(eta_bar)
        # Handle s = 0 case (no slip)
        with np.errstate(divide='ignore', invalid='ignore'):
            sign_eta = np.where(s > 1e-10, eta_bar / s, 0.0)
        
        fy_base = -mu_eff * mu_y * fz_array * sign_eta
        
        # 5. Add camber thrust (Requirement 6.2)
        # Fy_camber = C_gamma * camber * Fz
        fy_camber = C_gamma * gamma_array * fz_array
        
        # 6. Total lateral force
        fy = fy_base + fy_camber
        
        # Return scalar if input was scalar
        return fy.item() if np.isscalar(slip_angle) else fy
