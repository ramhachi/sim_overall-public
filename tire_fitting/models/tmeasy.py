"""
TMeasy Model Implementation

This module provides the TMeasyModel class implementing the TireModel interface
for computing tire forces using the TMeasy (Tire Model Easy) semi-physical model.
"""

import numpy as np
from typing import Dict, Union
from tire_fitting.models.base import TireModel


class TMeasyModel(TireModel):
    """
    TMeasy tire model for steady-state force calculations.
    
    This class implements the TMeasy semi-physical tire model with the unified
    TireModel interface. The model provides a simpler alternative to Pacejka
    Magic Formula with fewer parameters and no low-speed singularities.
    
    Key features:
    - Load-dependent friction coefficients
    - Saturation function for realistic force behavior
    - Camber thrust for lateral forces
    - No singularities at low speeds
    
    Requirements: 2.6, 9.3
    """
    
    def __init__(self, fz_nominal: float = 1000.0, params: Dict[str, float] = None):
        """
        Initialize TMeasy model.
        
        Args:
            fz_nominal: Nominal (reference) vertical load [N] (default: 1000.0)
            params: Optional dictionary of model parameters
        """
        self.fz_nominal = fz_nominal
        self.params = params if params is not None else {}
    
    def calculate_fx(self, slip_ratio: float, fz: float, **kwargs) -> float:
        """Calculate longitudinal force
        
        Args:
            slip_ratio: Longitudinal slip ratio (dimensionless)
            fz: Vertical load (N)
            **kwargs: Ignored for TMeasy
            
        Returns:
            Longitudinal force (N)
        """
        self.validate_inputs(fz)
        
        # Get parameters
        mu_x_peak = self.params.get('mu_x_peak', 1.0)
        k_mu_x = self.params.get('k_mu_x', 0.0)
        Kx = self.params.get('Kx', 10000.0)
        
        return self.compute_longitudinal_force(slip_ratio, fz, mu_x_peak, k_mu_x, Kx)
    
    def calculate_fy(self, slip_angle: float, fz: float, **kwargs) -> float:
        """Calculate lateral force
        
        Args:
            slip_angle: Slip angle (radians)
            fz: Vertical load (N)
            **kwargs: May include 'camber_angle' (radians)
            
        Returns:
            Lateral force (N)
        """
        self.validate_inputs(fz)
        
        # Get parameters
        mu_y_peak = self.params.get('mu_y_peak', 1.0)
        k_mu_y = self.params.get('k_mu_y', 0.0)
        Ky = self.params.get('Ky', 10000.0)
        C_gamma = self.params.get('C_gamma', 0.0)
        
        # Get camber angle from kwargs if provided
        camber_angle = kwargs.get('camber_angle', 0.0)
        
        return self.compute_lateral_force(
            slip_angle, fz, camber_angle, mu_y_peak, k_mu_y, Ky, C_gamma
        )
    
    def get_model_type(self) -> str:
        """Return model type identifier"""
        return "tmeasy"
    
    def get_parameters(self) -> Dict[str, float]:
        """Return current model parameters"""
        return {
            'fz_nominal': self.fz_nominal,
            **self.params.copy()
        }
    
    def set_parameters(self, params: Dict[str, float]) -> None:
        """Set model parameters
        
        Args:
            params: Dictionary of parameters to update
        """
        if 'fz_nominal' in params:
            self.fz_nominal = params['fz_nominal']
        
        # Update other parameters
        for key, value in params.items():
            if key != 'fz_nominal':
                self.params[key] = value
    
    # ========================================================================
    # Original TMeasy implementation methods (preserved for backward compatibility)
    # ========================================================================
    
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
        """
        fz_array = np.atleast_1d(fz)
        mu = mu_peak * (1.0 - k_mu * (fz_array - self.fz_nominal) / self.fz_nominal)
        return mu.item() if np.isscalar(fz) else mu
    
    def _compute_saturation_function(self, s: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        """
        Compute saturation function for effective friction.
        
        Args:
            s: Normalized deformation magnitude [-]
            
        Returns:
            Effective friction coefficient [-]
        """
        s_array = np.atleast_1d(s)
        mu_eff = np.where(s_array < 1.0,
                         s_array * (2.0 - s_array),
                         1.0)
        return mu_eff.item() if np.isscalar(s) else mu_eff
    
    def compute_longitudinal_force(self,
                                   slip_ratio: Union[float, np.ndarray],
                                   fz: Union[float, np.ndarray],
                                   mu_x_peak: float,
                                   k_mu_x: float,
                                   Kx: float) -> Union[float, np.ndarray]:
        """
        Compute longitudinal (tractive/braking) force.
        
        Args:
            slip_ratio: Longitudinal slip ratio [-]
            fz: Vertical load [N]
            mu_x_peak: Peak longitudinal friction coefficient [-]
            k_mu_x: Longitudinal load sensitivity [-]
            Kx: Longitudinal stiffness parameter [N]
            
        Returns:
            Longitudinal force Fx [N]
        """
        sr_array = np.atleast_1d(slip_ratio)
        fz_array = np.atleast_1d(fz)
        
        if np.any(fz_array == 0):
            fx = np.zeros_like(sr_array, dtype=float)
            return fx.item() if np.isscalar(slip_ratio) else fx
        
        mu_x = self._compute_load_dependent_friction(fz_array, mu_x_peak, k_mu_x)
        xi_bar = Kx * sr_array / (mu_x * fz_array)
        s = np.abs(xi_bar)
        mu_eff = self._compute_saturation_function(s)
        
        with np.errstate(divide='ignore', invalid='ignore'):
            sign_xi = np.where(s > 1e-10, xi_bar / s, 0.0)
        
        fx = -mu_eff * mu_x * fz_array * sign_xi
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
        
        Args:
            slip_angle: Lateral slip angle [rad]
            fz: Vertical load [N]
            camber_angle: Camber angle [rad]
            mu_y_peak: Peak lateral friction coefficient [-]
            k_mu_y: Lateral load sensitivity [-]
            Ky: Lateral stiffness parameter [N]
            C_gamma: Camber thrust coefficient [1/rad]
            
        Returns:
            Lateral force Fy [N]
        """
        sa_array = np.atleast_1d(slip_angle)
        fz_array = np.atleast_1d(fz)
        gamma_array = np.atleast_1d(camber_angle)
        
        if np.any(fz_array == 0):
            fy = np.zeros_like(sa_array, dtype=float)
            return fy.item() if np.isscalar(slip_angle) else fy
        
        mu_y = self._compute_load_dependent_friction(fz_array, mu_y_peak, k_mu_y)
        eta_bar = Ky * np.tan(sa_array) / (mu_y * fz_array)
        s = np.abs(eta_bar)
        mu_eff = self._compute_saturation_function(s)
        
        with np.errstate(divide='ignore', invalid='ignore'):
            sign_eta = np.where(s > 1e-10, eta_bar / s, 0.0)
        
        fy_base = -mu_eff * mu_y * fz_array * sign_eta
        fy_camber = C_gamma * gamma_array * fz_array
        fy = fy_base + fy_camber
        
        return fy.item() if np.isscalar(slip_angle) else fy
