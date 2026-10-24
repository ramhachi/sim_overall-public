"""
Hybrid Tire Model

This module provides the HybridTireModel class that allows using different
tire models for longitudinal and lateral force directions independently.
This enables optimal model selection per force direction.

For example, you might use Magic Formula for longitudinal forces (better
for complex braking/acceleration behavior) and TMeasy for lateral forces
(simpler and equally accurate for cornering).
"""

from typing import Dict
from tire_fitting.models.base import TireModel


class HybridTireModel(TireModel):
    """Hybrid tire model supporting different models per force direction
    
    This class wraps two separate tire models - one for longitudinal force
    calculation and one for lateral force calculation. This allows independent
    model selection per force direction to achieve optimal accuracy.
    
    The hybrid model delegates force calculations to the appropriate sub-model
    while maintaining the unified TireModel interface.
    
    Example:
        >>> from tire_fitting.models import TMeasyModel, MagicFormulaModel, HybridTireModel
        >>> 
        >>> # Create sub-models
        >>> mf_model = MagicFormulaModel(params={...})  # For longitudinal
        >>> tmeasy_model = TMeasyModel(params={...})    # For lateral
        >>> 
        >>> # Create hybrid model
        >>> hybrid = HybridTireModel(fx_model=mf_model, fy_model=tmeasy_model)
        >>> 
        >>> # Use like any other tire model
        >>> fx = hybrid.calculate_fx(slip_ratio=0.1, fz=1000.0)
        >>> fy = hybrid.calculate_fy(slip_angle=0.05, fz=1000.0)
    
    Requirements: 3.1, 3.2
    """
    
    def __init__(self, fx_model: TireModel, fy_model: TireModel):
        """Initialize hybrid model with separate models for each force direction
        
        Args:
            fx_model: Tire model to use for longitudinal force calculation
                     (e.g., TMeasyModel or MagicFormulaModel)
            fy_model: Tire model to use for lateral force calculation
                     (e.g., TMeasyModel or MagicFormulaModel)
        
        Raises:
            TypeError: If fx_model or fy_model don't implement TireModel interface
        """
        # Validate that both models implement TireModel interface
        if not isinstance(fx_model, TireModel):
            raise TypeError(
                f"fx_model must implement TireModel interface, got {type(fx_model)}"
            )
        if not isinstance(fy_model, TireModel):
            raise TypeError(
                f"fy_model must implement TireModel interface, got {type(fy_model)}"
            )
        
        self.fx_model = fx_model
        self.fy_model = fy_model
    
    def calculate_fx(self, slip_ratio: float, fz: float, **kwargs) -> float:
        """Calculate longitudinal force using the fx_model
        
        Delegates the calculation to the longitudinal force model specified
        during initialization.
        
        Args:
            slip_ratio: Longitudinal slip ratio (dimensionless)
            fz: Vertical load (N)
            **kwargs: Additional model-specific parameters
            
        Returns:
            Longitudinal force (N)
        """
        return self.fx_model.calculate_fx(slip_ratio, fz, **kwargs)
    
    def calculate_fy(self, slip_angle: float, fz: float, **kwargs) -> float:
        """Calculate lateral force using the fy_model
        
        Delegates the calculation to the lateral force model specified
        during initialization.
        
        Args:
            slip_angle: Slip angle (radians)
            fz: Vertical load (N)
            **kwargs: Additional model-specific parameters
            
        Returns:
            Lateral force (N)
        """
        return self.fy_model.calculate_fy(slip_angle, fz, **kwargs)
    
    def get_model_type(self) -> str:
        """Return hybrid model type description
        
        Returns a string describing which models are used for each force
        direction in the format: "hybrid(fx=<model>, fy=<model>)"
        
        Returns:
            Hybrid model type string (e.g., "hybrid(fx=magic_formula, fy=tmeasy)")
        """
        fx_type = self.fx_model.get_model_type()
        fy_type = self.fy_model.get_model_type()
        return f"hybrid(fx={fx_type}, fy={fy_type})"
    
    def get_parameters(self) -> Dict[str, Dict[str, float]]:
        """Return parameters for both sub-models
        
        Returns a nested dictionary containing the model types and parameters
        for both longitudinal and lateral force models.
        
        Returns:
            Dictionary with structure:
            {
                'fx_model': <model_type_string>,
                'fx_params': {<parameter_dict>},
                'fy_model': <model_type_string>,
                'fy_params': {<parameter_dict>}
            }
        """
        return {
            'fx_model': self.fx_model.get_model_type(),
            'fx_params': self.fx_model.get_parameters(),
            'fy_model': self.fy_model.get_model_type(),
            'fy_params': self.fy_model.get_parameters()
        }
    
    def set_parameters(self, params: Dict) -> None:
        """Set parameters for sub-models
        
        Updates parameters for the longitudinal and/or lateral force models.
        The params dictionary should contain 'fx_params' and/or 'fy_params'
        keys with nested parameter dictionaries.
        
        Args:
            params: Dictionary containing parameter updates
                   Expected structure:
                   {
                       'fx_params': {<fx_model_parameters>},  # Optional
                       'fy_params': {<fy_model_parameters>}   # Optional
                   }
        
        Example:
            >>> hybrid.set_parameters({
            ...     'fx_params': {'pDx1': 2.5, 'pCx1': 1.7},
            ...     'fy_params': {'mu_y_peak': 1.1, 'Ky': 12000.0}
            ... })
        """
        # Update fx model parameters if provided
        if 'fx_params' in params:
            self.fx_model.set_parameters(params['fx_params'])
        
        # Update fy model parameters if provided
        if 'fy_params' in params:
            self.fy_model.set_parameters(params['fy_params'])
