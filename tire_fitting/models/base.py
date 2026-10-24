"""
Base Tire Model Interface

This module defines the abstract base class for all tire models,
providing a unified interface for force calculation and parameter management.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict
import numpy as np


@dataclass
class TireForces:
    """Container for tire force outputs
    
    Attributes:
        fx: Longitudinal force (N)
        fy: Lateral force (N)
    """
    fx: float
    fy: float


class TireModel(ABC):
    """Abstract base class for all tire models
    
    This class defines the interface that all tire models must implement,
    ensuring consistent behavior across different model types (TMeasy,
    Magic Formula, etc.).
    
    The interface supports:
    - Independent calculation of longitudinal and lateral forces
    - Parameter management (get/set)
    - Model type identification
    
    Requirements: 2.1, 2.3, 2.4
    """
    
    @abstractmethod
    def calculate_fx(self, slip_ratio: float, fz: float, **kwargs) -> float:
        """Calculate longitudinal force
        
        Args:
            slip_ratio: Longitudinal slip ratio (dimensionless)
                       Positive for acceleration, negative for braking
            fz: Vertical load (N), must be positive
            **kwargs: Additional model-specific parameters (e.g., camber angle)
            
        Returns:
            Longitudinal force (N)
            Positive for forward force, negative for braking force
            
        Raises:
            ValueError: If fz is non-positive or inputs are invalid
        """
        pass
    
    @abstractmethod
    def calculate_fy(self, slip_angle: float, fz: float, **kwargs) -> float:
        """Calculate lateral force
        
        Args:
            slip_angle: Slip angle (radians)
                       Positive for left turn, negative for right turn
            fz: Vertical load (N), must be positive
            **kwargs: Additional model-specific parameters (e.g., camber angle)
            
        Returns:
            Lateral force (N)
            Positive for left force, negative for right force
            
        Raises:
            ValueError: If fz is non-positive or inputs are invalid
        """
        pass
    
    @abstractmethod
    def get_model_type(self) -> str:
        """Return model type identifier
        
        Returns:
            Model type string (e.g., 'tmeasy', 'magic_formula', 'hybrid')
        """
        pass
    
    @abstractmethod
    def get_parameters(self) -> Dict[str, float]:
        """Return current model parameters
        
        Returns:
            Dictionary mapping parameter names to values
            The structure depends on the specific model type
        """
        pass
    
    @abstractmethod
    def set_parameters(self, params: Dict[str, float]) -> None:
        """Set model parameters
        
        Args:
            params: Dictionary mapping parameter names to values
                   Only parameters present in the dict will be updated
                   
        Raises:
            ValueError: If parameter names are invalid or values are out of bounds
        """
        pass
    
    def calculate_forces(
        self, 
        slip_ratio: float, 
        slip_angle: float, 
        fz: float,
        **kwargs
    ) -> TireForces:
        """Calculate both longitudinal and lateral forces
        
        This is a convenience method that calls calculate_fx and calculate_fy.
        
        Args:
            slip_ratio: Longitudinal slip ratio (dimensionless)
            slip_angle: Slip angle (radians)
            fz: Vertical load (N)
            **kwargs: Additional model-specific parameters
            
        Returns:
            TireForces object containing fx and fy
        """
        fx = self.calculate_fx(slip_ratio, fz, **kwargs)
        fy = self.calculate_fy(slip_angle, fz, **kwargs)
        return TireForces(fx=fx, fy=fy)
    
    def validate_inputs(self, fz: float, param_name: str = "fz") -> None:
        """Validate common input parameters
        
        Args:
            fz: Vertical load to validate
            param_name: Name of parameter for error messages
            
        Raises:
            ValueError: If fz is non-positive or NaN/Inf
        """
        if not np.isfinite(fz):
            raise ValueError(f"{param_name} must be finite, got {fz}")
        if fz <= 0:
            raise ValueError(f"{param_name} must be positive, got {fz}")
