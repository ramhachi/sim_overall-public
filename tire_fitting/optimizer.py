"""
Tire Parameter Optimizer Module

This module provides the TireParameterOptimizer class for fitting TMeasy
tire model parameters from experimental data using two-stage optimization.

It also provides the ModelFitter class for fitting multiple tire models
(TMeasy and Magic Formula) with parallel execution and result caching.
"""

import numpy as np
from typing import Dict, Tuple, Optional, List
from scipy.optimize import differential_evolution, least_squares
from concurrent.futures import ProcessPoolExecutor
import logging
import time
from tire_fitting.tmeasy_model import TMeasyModel
from tire_fitting.models.tmeasy import TMeasyModel as TMeasyModelNew
from tire_fitting.models.magic_formula import MagicFormulaModel
from tire_fitting.units import ttc_lateral_angles_to_radians


class TireParameterOptimizer:
    """
    Optimizer for fitting TMeasy tire model parameters.
    
    This class implements a two-stage optimization strategy:
    1. Global search using differential evolution
    2. Local refinement using Levenberg-Marquardt
    
    The optimizer fits longitudinal parameters (mu_x_peak, k_mu_x, Kx) from
    DriveBrake data and lateral parameters (mu_y_peak, k_mu_y, Ky, C_gamma)
    from Cornering data.
    """
    
    def __init__(self, model: TMeasyModel):
        """
        Initialize optimizer with TMeasy model.
        
        Args:
            model: TMeasyModel instance for force calculations
        """
        self.model = model
        
        # Default parameter bounds (Requirements 7.2, 7.3, 7.4, 8.2, 8.3, 8.4)
        # Updated bounds based on validation analysis (2026-01-24)
        self.longitudinal_bounds = {
            'mu_x_peak': (1.3, 2.5),    # Peak longitudinal friction coefficient (raised lower bound)
            'k_mu_x': (0.05, 0.5),      # Longitudinal load sensitivity (avoid zero)
            'Kx': (15000.0, 50000.0)    # Longitudinal stiffness parameter (raised lower bound)
        }
        
        self.lateral_bounds = {
            'mu_y_peak': (1.3, 2.5),    # Peak lateral friction coefficient (raised lower bound)
            'k_mu_y': (0.05, 0.5),      # Lateral load sensitivity (avoid zero)
            'Ky': (10000.0, 50000.0),   # Lateral stiffness parameter (raised lower bound)
            'C_gamma': (0.0, 0.15)      # Camber thrust coefficient (raised upper bound)
        }

    def fit_longitudinal(self,
                        sr_data: np.ndarray,
                        fx_data: np.ndarray,
                        fz_data: np.ndarray,
                        bounds: Optional[Dict[str, Tuple[float, float]]] = None) -> Dict:
        """
        Fit longitudinal tire parameters from DriveBrake data.
        
        Uses two-stage optimization:
        1. Differential evolution for global search
        2. Levenberg-Marquardt for local refinement
        
        The objective function minimizes normalized residual sum of squares:
        sum((F_data - F_model) / Fz)^2
        
        Args:
            sr_data: Slip ratio data [-]
            fx_data: Longitudinal force data [N]
            fz_data: Vertical load data [N]
            bounds: Optional custom parameter bounds
                   If None, uses default bounds from __init__
                   
        Returns:
            Dictionary containing:
                - mu_x_peak: Fitted peak longitudinal friction coefficient
                - k_mu_x: Fitted longitudinal load sensitivity
                - Kx: Fitted longitudinal stiffness parameter
                - rmse: Root mean square error [N]
                - r2: R-squared coefficient of determination
                - success: Whether optimization converged
                - message: Optimization status message
                
        Raises:
            ValueError: If insufficient data points (< 10)
            
        Notes:
            - Implements Requirements 7.1, 7.2, 7.3, 7.4
            - Normalized residuals prevent bias toward high loads
            - Differential evolution explores parameter space globally
            - Levenberg-Marquardt refines solution locally
        """
        logger = logging.getLogger(__name__)
        logger.info("Starting longitudinal parameter fitting")
        logger.debug(f"  Data points: {len(sr_data)}")
        logger.debug(f"  SR range: [{sr_data.min():.4f}, {sr_data.max():.4f}]")
        logger.debug(f"  FX range: [{fx_data.min():.1f}, {fx_data.max():.1f}] N")
        logger.debug(f"  FZ range: [{fz_data.min():.1f}, {fz_data.max():.1f}] N")
        
        # Validate input data
        if len(sr_data) < 10:
            logger.error(f"Insufficient data for fitting: only {len(sr_data)} points available")
            raise ValueError(f"Insufficient data for fitting: only {len(sr_data)} points available, need at least 10")
        
        # Use default bounds if not provided
        if bounds is None:
            bounds = self.longitudinal_bounds
        
        logger.debug("Parameter bounds:")
        for param_name, (lower, upper) in bounds.items():
            logger.debug(f"  {param_name}: [{lower}, {upper}]")
        
        # Convert bounds dict to list format for scipy
        bounds_list = [
            bounds['mu_x_peak'],
            bounds['k_mu_x'],
            bounds['Kx']
        ]
        
        # Define objective function for absolute residuals
        # NOTE: Changed from normalized residuals to absolute residuals (2026-01-24)
        # Reason: Normalized residuals underestimate errors at high loads/slips
        def objective(params):
            """Compute absolute residual sum of squares."""
            mu_x_peak, k_mu_x, Kx = params
            
            # Compute predicted forces
            fx_pred = self.model.compute_longitudinal_force(
                sr_data, fz_data, mu_x_peak, k_mu_x, Kx
            )
            
            # Compute absolute residuals (no normalization)
            residuals = fx_data - fx_pred
            
            # Return sum of squared residuals
            return np.sum(residuals**2)
        
        # Stage 1: Global search with differential evolution
        result_de = differential_evolution(
            objective,
            bounds_list,
            strategy='best1bin',
            maxiter=1000,
            popsize=15,
            tol=1e-7,
            seed=42,
            workers=1
        )
        
        # Stage 2: Local refinement with Levenberg-Marquardt
        # Define residual function for least_squares
        # NOTE: Changed to absolute residuals (2026-01-24)
        def residual_func(params):
            """Compute residual vector for least squares."""
            mu_x_peak, k_mu_x, Kx = params
            
            # Compute predicted forces
            fx_pred = self.model.compute_longitudinal_force(
                sr_data, fz_data, mu_x_peak, k_mu_x, Kx
            )
            
            # Return absolute residuals (no normalization)
            return fx_data - fx_pred
        
        # Use DE result as initial guess
        x0 = result_de.x
        
        # Set bounds for least_squares
        bounds_lower = [b[0] for b in bounds_list]
        bounds_upper = [b[1] for b in bounds_list]
        
        result_lm = least_squares(
            residual_func,
            x0,
            bounds=(bounds_lower, bounds_upper),
            method='trf',  # Trust Region Reflective (handles bounds)
            ftol=1e-8,
            xtol=1e-8,
            gtol=1e-8,
            max_nfev=1000
        )
        
        # Extract fitted parameters
        mu_x_peak, k_mu_x, Kx = result_lm.x
        
        logger.debug(f"  LM converged: {result_lm.success}")
        logger.debug(f"  LM iterations: {result_lm.nfev}")
        logger.debug(f"  LM cost: {result_lm.cost:.2f}")
        logger.debug(f"  Final parameters: mu_x_peak={mu_x_peak:.4f}, k_mu_x={k_mu_x:.4f}, Kx={Kx:.1f}")
        
        # Compute final predictions and metrics
        fx_pred = self.model.compute_longitudinal_force(
            sr_data, fz_data, mu_x_peak, k_mu_x, Kx
        )
        
        # Calculate RMSE
        rmse = np.sqrt(np.mean((fx_data - fx_pred)**2))
        
        # Calculate R-squared
        ss_res = np.sum((fx_data - fx_pred)**2)
        ss_tot = np.sum((fx_data - np.mean(fx_data))**2)
        r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
        
        logger.info(f"Longitudinal fitting complete: RMSE={rmse:.2f}N, R²={r2:.4f}, Success={result_lm.success}")
        if not result_lm.success:
            logger.warning(f"  Optimization did not converge: {result_lm.message}")
        
        # Return results
        return {
            'mu_x_peak': mu_x_peak,
            'k_mu_x': k_mu_x,
            'Kx': Kx,
            'rmse': rmse,
            'r2': r2,
            'success': result_lm.success,
            'message': result_lm.message
        }

    def fit_lateral(self,
                   sa_data: np.ndarray,
                   fy_data: np.ndarray,
                   fz_data: np.ndarray,
                   camber_data: np.ndarray,
                   bounds: Optional[Dict[str, Tuple[float, float]]] = None) -> Dict:
        """
        Fit lateral tire parameters from Cornering data.
        
        Uses two-stage optimization:
        1. Differential evolution for global search
        2. Levenberg-Marquardt for local refinement
        
        The objective function minimizes normalized residual sum of squares:
        sum((F_data - F_model) / Fz)^2
        
        Args:
            sa_data: Slip angle data [rad]
            fy_data: Lateral force data [N]
            fz_data: Vertical load data [N]
            camber_data: Camber angle data [rad]
            bounds: Optional custom parameter bounds
                   If None, uses default bounds from __init__
                   
        Returns:
            Dictionary containing:
                - mu_y_peak: Fitted peak lateral friction coefficient
                - k_mu_y: Fitted lateral load sensitivity
                - Ky: Fitted lateral stiffness parameter
                - C_gamma: Fitted camber thrust coefficient
                - rmse: Root mean square error [N]
                - r2: R-squared coefficient of determination
                - success: Whether optimization converged
                - message: Optimization status message
                
        Raises:
            ValueError: If insufficient data points (< 10)
            
        Notes:
            - Implements Requirements 8.1, 8.2, 8.3, 8.4
            - Uses same optimization strategy as longitudinal fitting
            - Normalized residuals prevent bias toward high loads
            - Should be performed after longitudinal fitting
        """
        logger = logging.getLogger(__name__)
        logger.info("Starting lateral parameter fitting")
        logger.debug(f"  Data points: {len(sa_data)}")
        logger.debug(f"  SA range: [{np.rad2deg(sa_data.min()):.2f}, {np.rad2deg(sa_data.max()):.2f}] deg")
        logger.debug(f"  FY range: [{fy_data.min():.1f}, {fy_data.max():.1f}] N")
        logger.debug(f"  FZ range: [{fz_data.min():.1f}, {fz_data.max():.1f}] N")
        
        # Validate input data
        if len(sa_data) < 10:
            logger.error(f"Insufficient data for fitting: only {len(sa_data)} points available")
            raise ValueError(f"Insufficient data for fitting: only {len(sa_data)} points available, need at least 10")
        
        # Use default bounds if not provided
        if bounds is None:
            bounds = self.lateral_bounds
        
        logger.debug("Parameter bounds:")
        for param_name, (lower, upper) in bounds.items():
            logger.debug(f"  {param_name}: [{lower}, {upper}]")
        
        # Convert bounds dict to list format for scipy
        bounds_list = [
            bounds['mu_y_peak'],
            bounds['k_mu_y'],
            bounds['Ky'],
            bounds['C_gamma']
        ]
        
        # Define objective function for absolute residuals
        # NOTE: Changed from normalized residuals to absolute residuals (2026-01-24)
        # Reason: Normalized residuals underestimate errors at high loads/slips
        def objective(params):
            """Compute absolute residual sum of squares."""
            mu_y_peak, k_mu_y, Ky, C_gamma = params
            
            # Compute predicted forces
            fy_pred = self.model.compute_lateral_force(
                sa_data, fz_data, camber_data,
                mu_y_peak, k_mu_y, Ky, C_gamma
            )
            
            # Compute absolute residuals (no normalization)
            residuals = fy_data - fy_pred
            
            # Return sum of squared residuals
            return np.sum(residuals**2)
        
        # Stage 1: Global search with differential evolution
        result_de = differential_evolution(
            objective,
            bounds_list,
            strategy='best1bin',
            maxiter=1000,
            popsize=15,
            tol=1e-7,
            seed=42,
            workers=1
        )
        
        # Stage 2: Local refinement with Levenberg-Marquardt
        # Define residual function for least_squares
        # NOTE: Changed to absolute residuals (2026-01-24)
        def residual_func(params):
            """Compute residual vector for least squares."""
            mu_y_peak, k_mu_y, Ky, C_gamma = params
            
            # Compute predicted forces
            fy_pred = self.model.compute_lateral_force(
                sa_data, fz_data, camber_data,
                mu_y_peak, k_mu_y, Ky, C_gamma
            )
            
            # Return absolute residuals (no normalization)
            return fy_data - fy_pred
        
        # Use DE result as initial guess
        x0 = result_de.x
        
        # Set bounds for least_squares
        bounds_lower = [b[0] for b in bounds_list]
        bounds_upper = [b[1] for b in bounds_list]
        
        result_lm = least_squares(
            residual_func,
            x0,
            bounds=(bounds_lower, bounds_upper),
            method='trf',  # Trust Region Reflective (handles bounds)
            ftol=1e-8,
            xtol=1e-8,
            gtol=1e-8,
            max_nfev=1000
        )
        
        # Extract fitted parameters
        mu_y_peak, k_mu_y, Ky, C_gamma = result_lm.x
        
        logger.debug(f"  LM converged: {result_lm.success}")
        logger.debug(f"  LM iterations: {result_lm.nfev}")
        logger.debug(f"  LM cost: {result_lm.cost:.2f}")
        logger.debug(f"  Final parameters: mu_y_peak={mu_y_peak:.4f}, k_mu_y={k_mu_y:.4f}, Ky={Ky:.1f}, C_gamma={C_gamma:.4f}")
        
        # Compute final predictions and metrics
        fy_pred = self.model.compute_lateral_force(
            sa_data, fz_data, camber_data,
            mu_y_peak, k_mu_y, Ky, C_gamma
        )
        
        # Calculate RMSE
        rmse = np.sqrt(np.mean((fy_data - fy_pred)**2))
        
        # Calculate R-squared
        ss_res = np.sum((fy_data - fy_pred)**2)
        ss_tot = np.sum((fy_data - np.mean(fy_data))**2)
        r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
        
        logger.info(f"Lateral fitting complete: RMSE={rmse:.2f}N, R²={r2:.4f}, Success={result_lm.success}")
        if not result_lm.success:
            logger.warning(f"  Optimization did not converge: {result_lm.message}")
        
        # Return results
        return {
            'mu_y_peak': mu_y_peak,
            'k_mu_y': k_mu_y,
            'Ky': Ky,
            'C_gamma': C_gamma,
            'rmse': rmse,
            'r2': r2,
            'success': result_lm.success,
            'message': result_lm.message
        }



class ModelFitter:
    """
    Fits multiple tire models to experimental data with parallel execution.
    
    This class supports fitting both TMeasy and Magic Formula models to
    tire test data, with automatic model comparison and selection based
    on fitting metrics (RMSE, TIC, R²).
    
    Features:
    - Parallel model fitting using ProcessPoolExecutor
    - Result caching to avoid redundant optimization
    - Comprehensive metrics calculation (RMSE, TIC, R², peak error)
    - Model-specific parameter bounds
    
    Requirements: 5.1, 5.2, 5.3, 5.4, 5.5
    """
    
    def __init__(self, data_loader, preprocessor):
        """
        Initialize ModelFitter with data loader and preprocessor.
        
        Args:
            data_loader: TireDataLoader instance for loading experimental data
            preprocessor: TireDataPreprocessor instance for filtering data
        """
        self.data_loader = data_loader
        self.preprocessor = preprocessor
        self.cache = {}  # Cache fitting results: {model_type}_{direction} -> (params, metrics)
        self.logger = logging.getLogger(__name__)
    
    def fit_multiple_models(
        self,
        model_types: List[str],
        directions: List[str] = ['fx', 'fy'],
        drivebrake_file: Optional[str] = None,
        cornering_file: Optional[str] = None,
        max_workers: Optional[int] = None
    ) -> Dict[str, Dict[str, Dict]]:
        """
        Fit multiple models in parallel.
        
        This method fits all specified model types for all specified force
        directions using parallel execution. Results are cached to avoid
        redundant computation.
        
        Args:
            model_types: List of model types to fit (e.g., ['tmeasy', 'magic_formula'])
            directions: Force directions to fit (e.g., ['fx', 'fy'])
            drivebrake_file: Path to DriveBrake CSV file (required for 'fx')
            cornering_file: Path to Cornering CSV file (required for 'fy')
            max_workers: Maximum number of parallel workers (None = use all CPUs)
            
        Returns:
            Nested dict: {model_type: {direction: {'params': {...}, 'metrics': {...}}}}
            
        Example:
            >>> fitter = ModelFitter(data_loader, preprocessor)
            >>> results = fitter.fit_multiple_models(
            ...     model_types=['tmeasy', 'magic_formula'],
            ...     directions=['fx', 'fy'],
            ...     drivebrake_file='data/drivebrake.csv',
            ...     cornering_file='data/cornering.csv'
            ... )
            >>> print(results['tmeasy']['fx']['metrics']['rmse'])
            
        Requirements: 5.1, 5.3, 5.4
        """
        results = {}
        
        # Validate inputs
        if 'fx' in directions and drivebrake_file is None:
            raise ValueError("drivebrake_file is required for 'fx' direction")
        if 'fy' in directions and cornering_file is None:
            raise ValueError("cornering_file is required for 'fy' direction")
        
        # Parallel fitting using process pool
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            futures = {}
            
            for model_type in model_types:
                for direction in directions:
                    # Check cache first (Requirement 5.4)
                    cache_key = f"{model_type}_{direction}"
                    if cache_key in self.cache:
                        self.logger.info(f"Using cached results for {model_type} {direction}")
                        if model_type not in results:
                            results[model_type] = {}
                        params, metrics = self.cache[cache_key]
                        results[model_type][direction] = {
                            'params': params,
                            'metrics': metrics
                        }
                        continue
                    
                    # Submit fitting task
                    data_file = drivebrake_file if direction == 'fx' else cornering_file
                    future = executor.submit(
                        self._fit_single_model,
                        model_type,
                        direction,
                        data_file
                    )
                    futures[future] = (model_type, direction)
            
            # Collect results
            for future in futures:
                model_type, direction = futures[future]
                try:
                    params, metrics = future.result()
                    
                    if model_type not in results:
                        results[model_type] = {}
                    results[model_type][direction] = {
                        'params': params,
                        'metrics': metrics
                    }
                    
                    # Cache result (Requirement 5.4)
                    cache_key = f"{model_type}_{direction}"
                    self.cache[cache_key] = (params, metrics)
                    
                    self.logger.info(
                        f"Fitted {model_type} for {direction}: "
                        f"RMSE={metrics['rmse']:.1f}N, TIC={metrics['tic']:.4f}, R²={metrics['r2']:.4f}"
                    )
                    
                except Exception as e:
                    self.logger.error(f"Failed to fit {model_type} for {direction}: {str(e)}")
                    # Continue with other models even if one fails
        
        return results
    
    def _fit_single_model(
        self, 
        model_type: str, 
        direction: str,
        data_file: str
    ) -> Tuple[Dict[str, float], Dict[str, float]]:
        """
        Fit a single model for one force direction.
        
        This method loads data, preprocesses it, sets up the optimization
        problem, and runs differential evolution to find optimal parameters.
        
        Args:
            model_type: Model type ('tmeasy' or 'magic_formula')
            direction: Force direction ('fx' or 'fy')
            data_file: Path to data file (DriveBrake or Cornering)
            
        Returns:
            Tuple of (optimal_params, metrics)
            
        Requirements: 5.2, 5.5
        """
        # Load and preprocess data
        if direction == 'fx':
            data = self.data_loader.load_drivebrake_data(data_file)
        else:
            data = self.data_loader.load_cornering_data(data_file)
        
        # Apply minimal filtering: only remove missing values and invalid loads
        # Do NOT apply strict filtering conditions to maximize training data
        if direction == 'fx':
            required_cols = ['SR', 'FX', 'FZ', 'SA']
        else:
            required_cols = ['SA', 'FY', 'FZ', 'IA']
        
        data = data.dropna(subset=required_cols).copy()
        data = data[data['FZ'] > 0].copy()
        
        self.logger.info(f"Loaded {len(data)} data points for {model_type} {direction} (minimal filtering)")
        
        # Get parameter bounds for this model and direction
        bounds = self._get_parameter_bounds(model_type, direction)
        
        # Extract data arrays
        if direction == 'fx':
            slip_data = data['SR'].values
            force_data = data['FX'].values
            fz_data = data['FZ'].values
            extra_data = None
            # Calculate average tire temperature if available
            temperature_data = None
            if all(col in data.columns for col in ['TSTI', 'TSTC', 'TSTO']):
                temperature_data = (data['TSTI'].values + data['TSTC'].values + data['TSTO'].values) / 3.0
                self.logger.info(f"Using temperature data: mean={np.mean(temperature_data):.1f}°C, "
                               f"range=[{np.min(temperature_data):.1f}, {np.max(temperature_data):.1f}]°C")
        else:  # fy
            slip_data, extra_data = ttc_lateral_angles_to_radians(data)
            force_data = data['FY'].values
            fz_data = data['FZ'].values
            temperature_data = None  # No temperature for lateral (for now)
        
        # Define objective function
        def objective(params_array):
            """Compute sum of squared residuals."""
            params_dict = self._params_array_to_dict(params_array, model_type, direction)
            
            # Create model instance
            if model_type == 'tmeasy':
                # TMeasy needs fz_nominal
                fz_nominal = 1200.0  # Default nominal load
                model = TMeasyModelNew(fz_nominal=fz_nominal, params=params_dict)
            elif model_type == 'magic_formula':
                # Add nominal load if not present
                if 'fz0' not in params_dict:
                    params_dict['fz0'] = 1200.0  # Default nominal load
                model = MagicFormulaModel(params=params_dict)
            else:
                raise ValueError(f"Unknown model type: {model_type}")
            
            # Calculate predictions
            predictions = np.zeros_like(force_data)
            for i in range(len(slip_data)):
                try:
                    if direction == 'fx':
                        # Pass temperature if available
                        kwargs = {}
                        if temperature_data is not None:
                            kwargs['temperature'] = temperature_data[i]
                        predictions[i] = model.calculate_fx(slip_data[i], fz_data[i], **kwargs)
                    else:  # fy
                        camber = extra_data[i] if extra_data is not None else 0.0
                        predictions[i] = model.calculate_fy(slip_data[i], fz_data[i], camber_angle=camber)
                except Exception:
                    # If calculation fails, return large error
                    predictions[i] = 0.0
            
            # Compute residuals
            residuals = force_data - predictions
            return np.sum(residuals**2)
        
        # Progress tracking
        iteration_count = [0]  # Use list to allow modification in nested function
        start_time = time.time()
        
        def progress_callback(xk, convergence):
            """Callback to report optimization progress"""
            iteration_count[0] += 1
            if iteration_count[0] % 50 == 0:  # Report every 50 iterations
                elapsed = time.time() - start_time
                self.logger.info(
                    f"  {model_type} {direction}: iteration {iteration_count[0]}, "
                    f"convergence={convergence:.6f}, elapsed={elapsed:.1f}s"
                )
            return False  # Continue optimization
        
        # Run optimization using differential evolution
        self.logger.info(f"Starting optimization for {model_type} {direction} with {len(force_data)} data points...")
        result = differential_evolution(
            objective,
            bounds,
            strategy='best1bin',
            maxiter=1000,
            popsize=15,
            tol=1e-7,
            seed=42,
            workers=1,  # Use single worker per model (parallel at model level)
            callback=progress_callback,
            disp=False
        )
        
        self.logger.info(
            f"Optimization complete for {model_type} {direction}: "
            f"{iteration_count[0]} iterations, success={result.success}"
        )
        
        # Convert optimal parameters to dict
        optimal_params = self._params_array_to_dict(result.x, model_type, direction)
        
        # Calculate final metrics
        metrics = self._calculate_metrics(
            optimal_params, 
            model_type, 
            direction, 
            slip_data, 
            force_data, 
            fz_data,
            extra_data,
            temperature_data
        )
        
        return optimal_params, metrics
    
    def _get_parameter_bounds(
        self, 
        model_type: str, 
        direction: str
    ) -> List[Tuple[float, float]]:
        """
        Get parameter bounds for optimization.
        
        Returns bounds appropriate for the specified model type and force
        direction. Bounds are based on physical constraints and literature
        values for tire models.
        
        Args:
            model_type: Model type ('tmeasy' or 'magic_formula')
            direction: Force direction ('fx' or 'fy')
            
        Returns:
            List of (min, max) tuples for each parameter
            
        Requirements: 1.5, 5.5
        """
        if model_type == 'tmeasy':
            if direction == 'fx':
                # TMeasy longitudinal parameters
                return [
                    (1.3, 2.5),      # mu_x_peak
                    (0.05, 0.5),     # k_mu_x
                    (15000.0, 50000.0)  # Kx
                ]
            else:  # fy
                # TMeasy lateral parameters
                return [
                    (1.3, 2.5),      # mu_y_peak
                    (0.05, 0.5),     # k_mu_y
                    (10000.0, 50000.0),  # Ky
                    (0.0, 0.15)      # C_gamma
                ]
        
        elif model_type == 'magic_formula':
            if direction == 'fx':
                # Magic Formula longitudinal parameters
                return [
                    (0.5, 3.0),      # pDx1 (peak force coefficient)
                    (1.0, 2.5),      # pCx1 (shape factor)
                    (-1.0, 1.0),     # pEx1 (curvature factor)
                    (5.0, 50.0),     # pKx1 (stiffness coefficient)
                    (-5.0, 5.0),     # pKx2 (load sensitivity)
                    (-1.0, 1.0),     # pKx3 (exponential load factor)
                ]
            else:  # fy
                # Magic Formula lateral parameters
                return [
                    (0.5, 3.0),      # pDy1 (peak force coefficient)
                    (1.0, 2.5),      # pCy1 (shape factor)
                    (-1.0, 1.0),     # pEy1 (curvature factor)
                    (5.0, 50.0),     # pKy1 (stiffness coefficient)
                    (-5.0, 5.0),     # pKy2 (load sensitivity)
                    (-1.0, 1.0)      # pKy3 (exponential load factor)
                ]
        
        else:
            raise ValueError(f"Unknown model type: {model_type}")
    
    def _params_array_to_dict(
        self, 
        params_array: np.ndarray, 
        model_type: str, 
        direction: str
    ) -> Dict[str, float]:
        """
        Convert parameter array to dictionary.
        
        Args:
            params_array: Array of parameter values from optimizer
            model_type: Model type ('tmeasy' or 'magic_formula')
            direction: Force direction ('fx' or 'fy')
            
        Returns:
            Dictionary mapping parameter names to values
        """
        if model_type == 'tmeasy':
            if direction == 'fx':
                return {
                    'mu_x_peak': params_array[0],
                    'k_mu_x': params_array[1],
                    'Kx': params_array[2]
                }
            else:  # fy
                return {
                    'mu_y_peak': params_array[0],
                    'k_mu_y': params_array[1],
                    'Ky': params_array[2],
                    'C_gamma': params_array[3]
                }
        
        elif model_type == 'magic_formula':
            if direction == 'fx':
                params = {
                    'pDx1': params_array[0],
                    'pCx1': params_array[1],
                    'pEx1': params_array[2],
                    'pKx1': params_array[3],
                    'pKx2': params_array[4],
                    'pKx3': params_array[5],
                    'fz0': 1200.0,  # Default nominal load
                }
                if len(params_array) > 6:
                    params['pDx2_temp'] = params_array[6]
                    params['T_ref'] = 60.0
                return params
            else:  # fy
                return {
                    'pDy1': params_array[0],
                    'pCy1': params_array[1],
                    'pEy1': params_array[2],
                    'pKy1': params_array[3],
                    'pKy2': params_array[4],
                    'pKy3': params_array[5],
                    'fz0': 1200.0  # Default nominal load
                }
        
        else:
            raise ValueError(f"Unknown model type: {model_type}")
    
    def _calculate_metrics(
        self,
        params: Dict[str, float],
        model_type: str,
        direction: str,
        slip_data: np.ndarray,
        force_data: np.ndarray,
        fz_data: np.ndarray,
        extra_data: Optional[np.ndarray] = None,
        temperature_data: Optional[np.ndarray] = None
    ) -> Dict[str, float]:
        """
        Calculate fitting metrics (RMSE, TIC, R², peak error).
        
        Args:
            params: Model parameters
            model_type: Model type ('tmeasy' or 'magic_formula')
            direction: Force direction ('fx' or 'fy')
            slip_data: Slip ratio or slip angle data
            force_data: Measured force data
            fz_data: Vertical load data
            extra_data: Additional data (e.g., camber angle for lateral)
            temperature_data: Tire temperature data (°C), optional
            
        Returns:
            Dictionary containing:
                - rmse: Root Mean Square Error (N)
                - tic: Theil Inequality Coefficient
                - r2: R-squared coefficient
                - peak_error: Maximum absolute error (N)
                
        Requirements: 5.2
        """
        # Create model instance
        if model_type == 'tmeasy':
            # TMeasy needs fz_nominal
            fz_nominal = params.get('fz_nominal', 1200.0)
            model = TMeasyModelNew(fz_nominal=fz_nominal, params=params)
        elif model_type == 'magic_formula':
            model = MagicFormulaModel(params=params)
        else:
            raise ValueError(f"Unknown model type: {model_type}")
        
        # Calculate predictions
        predictions = np.zeros_like(force_data)
        for i in range(len(slip_data)):
            try:
                if direction == 'fx':
                    # Pass temperature if available
                    kwargs = {}
                    if temperature_data is not None:
                        kwargs['temperature'] = temperature_data[i]
                    predictions[i] = model.calculate_fx(slip_data[i], fz_data[i], **kwargs)
                else:  # fy
                    camber = extra_data[i] if extra_data is not None else 0.0
                    predictions[i] = model.calculate_fy(slip_data[i], fz_data[i], camber_angle=camber)
            except Exception:
                predictions[i] = 0.0
        
        # Calculate RMSE
        rmse = np.sqrt(np.mean((force_data - predictions)**2))
        force_range = float(np.max(force_data) - np.min(force_data))
        force_scale = float(max(np.max(np.abs(force_data)), 1.0))
        nrmse = float(rmse / force_range) if force_range > 0.0 else 0.0
        
        # Calculate TIC (Theil Inequality Coefficient)
        numerator = np.sqrt(np.mean((predictions - force_data)**2))
        denominator = np.sqrt(np.mean(predictions**2)) + np.sqrt(np.mean(force_data**2))
        tic = numerator / denominator if denominator > 0 else 0.0
        
        # Calculate R²
        ss_res = np.sum((force_data - predictions)**2)
        ss_tot = np.sum((force_data - np.mean(force_data))**2)
        r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
        
        # Calculate peak error
        peak_error = np.max(np.abs(predictions - force_data))
        peak_error_fraction = float(peak_error / force_scale)
        
        return {
            'rmse': float(rmse),
            'nrmse': nrmse,
            'tic': float(tic),
            'r2': float(r2),
            'peak_error': float(peak_error),
            'peak_error_fraction': peak_error_fraction,
            'force_range': force_range,
            'force_scale': force_scale,
        }
