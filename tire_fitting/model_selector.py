"""Model selector for choosing optimal tire models based on fitting metrics.

This module implements the model selection logic for the hybrid tire model system,
supporting automatic, manual, and hybrid selection modes.
"""

from enum import Enum
from typing import Dict, Tuple, Optional
import logging


class SelectionMode(Enum):
    """Model selection modes"""
    AUTO = "auto"
    MANUAL = "manual"
    HYBRID = "hybrid"


class ModelSelector:
    """Selects optimal tire models based on fitting metrics.
    
    The ModelSelector supports three selection modes:
    - AUTO: Automatically selects the best model for each force direction based on
            a balanced score using normalized error, TIC, R², and peak error
    - MANUAL: Uses user-specified models for both force directions
    - HYBRID: Combines automatic and manual selection (auto-selects for unspecified
              directions, uses manual specification for others)
    
    Selection criteria:
    A weighted score is minimized for each direction. The score favors low
    normalized RMSE, low TIC, high R², and low peak error fraction so that a model
    with a slightly better absolute RMSE cannot hide poor vehicle-dynamics behavior
    in high-load or peak-force regions.
    
    Attributes:
        mode: The selection mode (AUTO, MANUAL, or HYBRID)
        logger: Logger instance for recording selection decisions
    """
    
    def __init__(self, mode: SelectionMode):
        """Initialize the model selector.
        
        Args:
            mode: The selection mode to use
        """
        self.mode = mode
        self.logger = logging.getLogger(__name__)
    
    def select_models(
        self,
        fitting_results: Dict[str, Dict[str, Dict[str, float]]],
        manual_fx: Optional[str] = None,
        manual_fy: Optional[str] = None
    ) -> Tuple[str, str]:
        """Select best models for longitudinal and lateral forces.
        
        Args:
            fitting_results: Dict mapping model_name -> {direction -> metrics}
                Example: {
                    'tmeasy': {
                        'fx': {'rmse': 2547, 'tic': 0.45, 'r2': 0.75},
                        'fy': {'rmse': 119, 'tic': 0.04, 'r2': 0.99}
                    },
                    'magic_formula': {
                        'fx': {'rmse': 85, 'tic': 0.12, 'r2': 0.98},
                        'fy': {'rmse': 125, 'tic': 0.05, 'r2': 0.99}
                    }
                }
            manual_fx: Manually specified model for longitudinal force (optional)
            manual_fy: Manually specified model for lateral force (optional)
            
        Returns:
            Tuple of (fx_model_name, fy_model_name)
            
        Raises:
            ValueError: If manual mode is used without specifying both models,
                       or if no models are available for a required direction
        """
        if self.mode == SelectionMode.MANUAL:
            return self._manual_selection(manual_fx, manual_fy)
        elif self.mode == SelectionMode.AUTO:
            return self._auto_selection(fitting_results)
        else:  # HYBRID
            return self._hybrid_selection(fitting_results, manual_fx, manual_fy)
    
    def _auto_selection(self, results: Dict[str, Dict[str, Dict[str, float]]]) -> Tuple[str, str]:
        """Automatically select best model for each direction.
        
        Args:
            results: Fitting results for all models and directions
            
        Returns:
            Tuple of (fx_model_name, fy_model_name)
            
        Raises:
            ValueError: If no models are available for a required direction
        """
        fx_model = self._select_best_for_direction(results, 'fx')
        fy_model = self._select_best_for_direction(results, 'fy')
        
        self.logger.info(f"Auto-selected models: FX={fx_model}, FY={fy_model}")
        return fx_model, fy_model
    
    def _select_best_for_direction(
        self, 
        results: Dict[str, Dict[str, Dict[str, float]]], 
        direction: str
    ) -> str:
        """Select best model for a specific force direction.
        
        Selection criteria (in order):
        Selects the model with the lowest balanced score. Falls back to relative
        RMSE/peak-error normalization when older metrics do not include NRMSE or
        force-scale metadata.
        
        Args:
            results: Fitting results for all models
            direction: Force direction ('fx' or 'fy')
            
        Returns:
            Name of the best model for this direction
            
        Raises:
            ValueError: If no models are available for this direction
        """
        # Collect candidates with metrics for this direction
        candidates = []
        for model_name, metrics_by_direction in results.items():
            if direction in metrics_by_direction:
                candidates.append((model_name, metrics_by_direction[direction]))
        
        if not candidates:
            raise ValueError(
                f"No models available for direction '{direction}'. "
                f"At least one model must be fitted for each force direction."
            )
        
        score_by_model = self._score_candidates(candidates)
        candidates.sort(
            key=lambda item: (
                score_by_model[item[0]],
                item[1].get('rmse', float('inf')),
                item[1].get('tic', float('inf')),
                -item[1].get('r2', -float('inf')),
            )
        )
        best_model, best_metrics = candidates[0]
        
        self.logger.info(
            f"Selected {best_model} for {direction}: "
            f"score={score_by_model[best_model]:.4f}, "
            f"RMSE={best_metrics['rmse']:.1f}N, "
            f"TIC={best_metrics['tic']:.3f}, "
            f"R²={best_metrics['r2']:.4f}"
        )
        return best_model

    def _score_candidates(self, candidates: list[tuple[str, Dict[str, float]]]) -> Dict[str, float]:
        """Compute balanced model-selection scores for one force direction."""
        max_rmse = max(abs(metrics.get('rmse', 0.0)) for _, metrics in candidates) or 1.0
        max_peak_error = (
            max(abs(metrics.get('peak_error', 0.0)) for _, metrics in candidates) or 1.0
        )

        scores = {}
        for model_name, metrics in candidates:
            rmse = abs(metrics.get('rmse', max_rmse))
            force_scale = max(
                abs(metrics.get('force_range', 0.0)),
                abs(metrics.get('force_scale', 0.0)),
                1.0,
            )
            nrmse = metrics.get('nrmse')
            if nrmse is None:
                nrmse = rmse / force_scale if force_scale > 1.0 else rmse / max_rmse

            peak_fraction = metrics.get('peak_error_fraction')
            if peak_fraction is None:
                peak_error = abs(metrics.get('peak_error', 0.0))
                peak_fraction = (
                    peak_error / force_scale if force_scale > 1.0 else peak_error / max_peak_error
                )

            tic = max(metrics.get('tic', 1.0), 0.0)
            r2_error = max(0.0, 1.0 - metrics.get('r2', 0.0))

            scores[model_name] = (
                0.45 * float(nrmse)
                + 0.25 * float(tic)
                + 0.20 * float(r2_error)
                + 0.10 * float(peak_fraction)
            )

        return scores
    
    def _manual_selection(self, fx: Optional[str], fy: Optional[str]) -> Tuple[str, str]:
        """Use manually specified models.
        
        Args:
            fx: Model name for longitudinal force
            fy: Model name for lateral force
            
        Returns:
            Tuple of (fx_model_name, fy_model_name)
            
        Raises:
            ValueError: If either model is not specified
        """
        if not fx or not fy:
            raise ValueError(
                "Manual mode requires both fx and fy models to be specified. "
                f"Got fx={fx}, fy={fy}"
            )
        
        self.logger.info(f"Manual selection: FX={fx}, FY={fy}")
        return fx, fy
    
    def _hybrid_selection(
        self, 
        results: Dict[str, Dict[str, Dict[str, float]]], 
        manual_fx: Optional[str], 
        manual_fy: Optional[str]
    ) -> Tuple[str, str]:
        """Hybrid: auto-select for unspecified directions.
        
        Args:
            results: Fitting results for all models
            manual_fx: Manually specified model for longitudinal force (optional)
            manual_fy: Manually specified model for lateral force (optional)
            
        Returns:
            Tuple of (fx_model_name, fy_model_name)
            
        Raises:
            ValueError: If no models are available for auto-selected directions
        """
        fx = manual_fx if manual_fx else self._select_best_for_direction(results, 'fx')
        fy = manual_fy if manual_fy else self._select_best_for_direction(results, 'fy')
        
        self.logger.info(
            f"Hybrid selection: FX={fx} ({'manual' if manual_fx else 'auto'}), "
            f"FY={fy} ({'manual' if manual_fy else 'auto'})"
        )
        return fx, fy
