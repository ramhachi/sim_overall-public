# Design Document: Hybrid Tire Model Fitting System

## Overview

The hybrid tire model fitting system extends the existing tire fitting pipeline to support multiple tire models (TMeasy and Pacejka Magic Formula) with independent selection per force direction. This design addresses the structural limitations of TMeasy for longitudinal force prediction (RMSE 2547 N) while maintaining its excellent lateral force performance (RMSE 119 N).

The system introduces a unified model interface that abstracts model-specific implementations, an automatic model selection mechanism based on fitting metrics, and enhanced output formats that support hybrid configurations. The architecture maintains backward compatibility with existing TMeasy implementations while enabling seamless integration of the Magic Formula model.

Key design principles:
- **Modularity**: Each tire model is implemented as a separate, self-contained module
- **Extensibility**: New tire models can be added without modifying existing code
- **Transparency**: Model selection decisions are logged with clear justification
- **Performance**: Parallel fitting enables efficient model comparison
- **Compatibility**: Existing code continues to work without modification

## Architecture

The system follows a layered architecture with clear separation of concerns:

```
┌─────────────────────────────────────────────────────────┐
│                  Command-Line Interface                  │
│              (main.py with enhanced flags)               │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│                   Model Selector                         │
│        (Auto/Manual/Hybrid mode selection logic)         │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│                 Unified Model Interface                  │
│           (Abstract base class for all models)           │
└─────────────┬──────────────────────┬────────────────────┘
              │                      │
    ┌─────────▼─────────┐  ┌────────▼──────────┐
    │   TMeasy Model    │  │  Magic Formula    │
    │  (existing impl)  │  │   (new impl)      │
    └─────────┬─────────┘  └────────┬──────────┘
              │                      │
              └──────────┬───────────┘
                         │
              ┌──────────▼──────────┐
              │  Fitting Pipeline   │
              │  (enhanced optimizer)│
              └──────────┬──────────┘
                         │
              ┌──────────▼──────────┐
              │   Output Exporters  │
              │  (YAML, C++ header) │
              └─────────────────────┘
```


### Component Interaction Flow

1. **Initialization**: CLI parses model selection flags and loads configuration
2. **Data Loading**: Existing data loader reads experimental tire data (unchanged)
3. **Model Selection**: Model Selector determines which models to fit based on mode
4. **Parallel Fitting**: Fitting Pipeline fits all candidate models concurrently
5. **Comparison**: Model Selector compares fitting metrics and selects best models
6. **Export**: Output Exporters generate YAML and C++ files with model metadata

## Components and Interfaces

### 1. Unified Model Interface (Abstract Base Class)

```python
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple
import numpy as np

@dataclass
class TireForces:
    """Container for tire force outputs"""
    fx: float  # Longitudinal force (N)
    fy: float  # Lateral force (N)

class TireModel(ABC):
    """Abstract base class for all tire models"""
    
    @abstractmethod
    def calculate_fx(self, slip_ratio: float, fz: float) -> float:
        """Calculate longitudinal force
        
        Args:
            slip_ratio: Longitudinal slip ratio (dimensionless)
            fz: Vertical load (N)
            
        Returns:
            Longitudinal force (N)
        """
        pass
    
    @abstractmethod
    def calculate_fy(self, slip_angle: float, fz: float) -> float:
        """Calculate lateral force
        
        Args:
            slip_angle: Slip angle (radians)
            fz: Vertical load (N)
            
        Returns:
            Lateral force (N)
        """
        pass
    
    @abstractmethod
    def get_model_type(self) -> str:
        """Return model type identifier"""
        pass
    
    @abstractmethod
    def get_parameters(self) -> dict:
        """Return current model parameters"""
        pass
    
    @abstractmethod
    def set_parameters(self, params: dict) -> None:
        """Set model parameters"""
        pass
```


### 2. TMeasy Model Implementation

The existing TMeasy model will be refactored to implement the TireModel interface:

```python
class TMeasyModel(TireModel):
    """TMeasy tire model implementation"""
    
    def __init__(self, params: dict):
        self.params = params
        # Longitudinal parameters: dfx0, fxm, fxs, sxm, sxs
        # Lateral parameters: dfy0, fym, fys, sym, sys
    
    def calculate_fx(self, slip_ratio: float, fz: float) -> float:
        """TMeasy longitudinal force using tanh function"""
        sx = slip_ratio
        dfx0 = self.params['dfx0']
        fxm = self.params['fxm']
        fxs = self.params['fxs']
        sxm = self.params['sxm']
        sxs = self.params['sxs']
        
        # TMeasy equation (existing implementation)
        fx = dfx0 * fz * np.tanh(sx / sxm) + \
             (fxm * fz - dfx0 * fz) * np.tanh(sx / sxs)
        return fx
    
    def calculate_fy(self, slip_angle: float, fz: float) -> float:
        """TMeasy lateral force using tanh function"""
        # Similar structure to calculate_fx
        pass
    
    def get_model_type(self) -> str:
        return "tmeasy"
    
    def get_parameters(self) -> dict:
        return self.params.copy()
    
    def set_parameters(self, params: dict) -> None:
        self.params.update(params)
```


### 3. Magic Formula Model Implementation

The Pacejka Magic Formula (MF5.2) implementation:

```python
class MagicFormulaModel(TireModel):
    """Pacejka Magic Formula tire model (MF5.2)"""
    
    def __init__(self, params: dict):
        self.params = params
        # Longitudinal: pDx1, pCx1, pEx1, pKx1, pKx2, pKx3
        # Lateral: pDy1, pCy1, pEy1, pKy1, pKy2, pKy3
        # Load dependency: fz0 (nominal load)
    
    def calculate_fx(self, slip_ratio: float, fz: float) -> float:
        """Magic Formula longitudinal force
        
        MF5.2 Pure Longitudinal Force equation:
        Fx0 = D * sin(C * arctan(B * kappa - E * (B * kappa - arctan(B * kappa))))
        
        Where:
        - D = peak value (pDx1 * fz)
        - C = shape factor (pCx1)
        - B = stiffness factor (calculated from pKx parameters)
        - E = curvature factor (pEx1)
        - kappa = slip_ratio
        """
        fz0 = self.params['fz0']
        dfz = (fz - fz0) / fz0  # Normalized load variation
        
        # Peak factor D
        D = self.params['pDx1'] * fz
        
        # Shape factor C
        C = self.params['pCx1']
        
        # Stiffness factor B
        K = fz * (self.params['pKx1'] + self.params['pKx2'] * dfz) * \
            np.exp(self.params['pKx3'] * dfz)
        B = K / (C * D + 1e-6)  # Avoid division by zero
        
        # Curvature factor E
        E = self.params['pEx1']
        
        # Magic Formula equation
        x = B * slip_ratio
        fx = D * np.sin(C * np.arctan(x - E * (x - np.arctan(x))))
        
        return fx
    
    def calculate_fy(self, slip_angle: float, fz: float) -> float:
        """Magic Formula lateral force
        
        Similar structure to calculate_fx but with lateral parameters
        """
        fz0 = self.params['fz0']
        dfz = (fz - fz0) / fz0
        
        # Peak factor D
        D = self.params['pDy1'] * fz
        
        # Shape factor C
        C = self.params['pCy1']
        
        # Stiffness factor B
        K = fz * (self.params['pKy1'] + self.params['pKy2'] * dfz) * \
            np.exp(self.params['pKy3'] * dfz)
        B = K / (C * D + 1e-6)
        
        # Curvature factor E
        E = self.params['pEy1']
        
        # Magic Formula equation
        x = B * slip_angle
        fy = D * np.sin(C * np.arctan(x - E * (x - np.arctan(x))))
        
        return fy
    
    def get_model_type(self) -> str:
        return "magic_formula"
    
    def get_parameters(self) -> dict:
        return self.params.copy()
    
    def set_parameters(self, params: dict) -> None:
        self.params.update(params)
```

**Parameter Bounds** (based on tire dynamics literature):
- pDx1, pDy1: [0.5, 3.0] (peak force coefficient)
- pCx1, pCy1: [1.0, 2.5] (shape factor)
- pEx1, pEy1: [-1.0, 1.0] (curvature factor)
- pKx1, pKy1: [5.0, 50.0] (stiffness coefficient)
- pKx2, pKy2: [-5.0, 5.0] (load sensitivity)
- pKx3, pKy3: [-1.0, 1.0] (exponential load factor)


### 4. Hybrid Model Wrapper

The hybrid model allows different models for longitudinal and lateral forces:

```python
class HybridTireModel(TireModel):
    """Hybrid tire model supporting different models per force direction"""
    
    def __init__(self, fx_model: TireModel, fy_model: TireModel):
        self.fx_model = fx_model
        self.fy_model = fy_model
    
    def calculate_fx(self, slip_ratio: float, fz: float) -> float:
        return self.fx_model.calculate_fx(slip_ratio, fz)
    
    def calculate_fy(self, slip_angle: float, fz: float) -> float:
        return self.fy_model.calculate_fy(slip_angle, fz)
    
    def get_model_type(self) -> str:
        return f"hybrid(fx={self.fx_model.get_model_type()}, fy={self.fy_model.get_model_type()})"
    
    def get_parameters(self) -> dict:
        return {
            'fx_model': self.fx_model.get_model_type(),
            'fx_params': self.fx_model.get_parameters(),
            'fy_model': self.fy_model.get_model_type(),
            'fy_params': self.fy_model.get_parameters()
        }
    
    def set_parameters(self, params: dict) -> None:
        if 'fx_params' in params:
            self.fx_model.set_parameters(params['fx_params'])
        if 'fy_params' in params:
            self.fy_model.set_parameters(params['fy_params'])
```


### 5. Model Selector

The model selector implements the auto/manual/hybrid selection logic:

```python
from enum import Enum
from typing import Dict, Tuple, Optional
import logging

class SelectionMode(Enum):
    AUTO = "auto"
    MANUAL = "manual"
    HYBRID = "hybrid"

class ModelSelector:
    """Selects optimal tire models based on fitting metrics"""
    
    def __init__(self, mode: SelectionMode):
        self.mode = mode
        self.logger = logging.getLogger(__name__)
    
    def select_models(
        self,
        fitting_results: Dict[str, Dict[str, float]],
        manual_fx: Optional[str] = None,
        manual_fy: Optional[str] = None
    ) -> Tuple[str, str]:
        """Select best models for longitudinal and lateral forces
        
        Args:
            fitting_results: Dict mapping model_name -> {direction -> metrics}
                Example: {
                    'tmeasy': {'fx': {'rmse': 2547, 'tic': 0.45, 'r2': 0.75},
                               'fy': {'rmse': 119, 'tic': 0.04, 'r2': 0.99}},
                    'magic_formula': {'fx': {'rmse': 85, 'tic': 0.12, 'r2': 0.98},
                                      'fy': {'rmse': 125, 'tic': 0.05, 'r2': 0.99}}
                }
            manual_fx: Manually specified model for longitudinal force
            manual_fy: Manually specified model for lateral force
            
        Returns:
            Tuple of (fx_model_name, fy_model_name)
        """
        if self.mode == SelectionMode.MANUAL:
            return self._manual_selection(manual_fx, manual_fy)
        elif self.mode == SelectionMode.AUTO:
            return self._auto_selection(fitting_results)
        else:  # HYBRID
            return self._hybrid_selection(fitting_results, manual_fx, manual_fy)
    
    def _auto_selection(self, results: Dict) -> Tuple[str, str]:
        """Automatically select best model for each direction"""
        fx_model = self._select_best_for_direction(results, 'fx')
        fy_model = self._select_best_for_direction(results, 'fy')
        
        self.logger.info(f"Auto-selected models: FX={fx_model}, FY={fy_model}")
        return fx_model, fy_model
    
    def _select_best_for_direction(self, results: Dict, direction: str) -> str:
        """Select best model for a specific force direction
        
        Selection criteria (in order):
        1. Lowest RMSE
        2. If RMSE within 5%, use lowest TIC
        3. If TIC within 5%, use highest R²
        """
        candidates = []
        for model_name, metrics in results.items():
            if direction in metrics:
                candidates.append((model_name, metrics[direction]))
        
        if not candidates:
            raise ValueError(f"No models available for direction {direction}")
        
        # Sort by RMSE
        candidates.sort(key=lambda x: x[1]['rmse'])
        best_model, best_metrics = candidates[0]
        
        # Check if there's a close competitor
        if len(candidates) > 1:
            second_model, second_metrics = candidates[1]
            rmse_diff_pct = abs(best_metrics['rmse'] - second_metrics['rmse']) / best_metrics['rmse']
            
            if rmse_diff_pct < 0.05:  # Within 5%
                # Use TIC as tiebreaker
                if second_metrics['tic'] < best_metrics['tic']:
                    self.logger.info(f"RMSE similar, selected {second_model} based on TIC")
                    return second_model
        
        self.logger.info(
            f"Selected {best_model} for {direction}: "
            f"RMSE={best_metrics['rmse']:.1f}, TIC={best_metrics['tic']:.3f}, R²={best_metrics['r2']:.4f}"
        )
        return best_model
    
    def _manual_selection(self, fx: str, fy: str) -> Tuple[str, str]:
        """Use manually specified models"""
        if not fx or not fy:
            raise ValueError("Manual mode requires both fx and fy models to be specified")
        self.logger.info(f"Manual selection: FX={fx}, FY={fy}")
        return fx, fy
    
    def _hybrid_selection(
        self, 
        results: Dict, 
        manual_fx: Optional[str], 
        manual_fy: Optional[str]
    ) -> Tuple[str, str]:
        """Hybrid: auto-select for unspecified directions"""
        fx = manual_fx if manual_fx else self._select_best_for_direction(results, 'fx')
        fy = manual_fy if manual_fy else self._select_best_for_direction(results, 'fy')
        self.logger.info(f"Hybrid selection: FX={fx}, FY={fy}")
        return fx, fy
```


### 6. Enhanced Fitting Pipeline

The fitting pipeline is extended to support multiple models:

```python
from typing import List, Dict
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.optimize import differential_evolution

class ModelFitter:
    """Fits tire models to experimental data"""
    
    def __init__(self, data_loader, preprocessor):
        self.data_loader = data_loader
        self.preprocessor = preprocessor
        self.cache = {}  # Cache fitting results
    
    def fit_multiple_models(
        self,
        model_types: List[str],
        directions: List[str] = ['fx', 'fy']
    ) -> Dict[str, Dict[str, Dict]]:
        """Fit multiple models in parallel
        
        Args:
            model_types: List of model types to fit (e.g., ['tmeasy', 'magic_formula'])
            directions: Force directions to fit (e.g., ['fx', 'fy'])
            
        Returns:
            Nested dict: {model_type: {direction: {params, metrics}}}
        """
        results = {}
        
        # Parallel fitting using process pool
        with ProcessPoolExecutor() as executor:
            futures = {}
            for model_type in model_types:
                for direction in directions:
                    cache_key = f"{model_type}_{direction}"
                    if cache_key in self.cache:
                        continue
                    
                    future = executor.submit(
                        self._fit_single_model,
                        model_type,
                        direction
                    )
                    futures[future] = (model_type, direction)
            
            # Collect results
            for future in futures:
                model_type, direction = futures[future]
                params, metrics = future.result()
                
                if model_type not in results:
                    results[model_type] = {}
                results[model_type][direction] = {
                    'params': params,
                    'metrics': metrics
                }
                
                # Cache result
                cache_key = f"{model_type}_{direction}"
                self.cache[cache_key] = (params, metrics)
        
        return results
    
    def _fit_single_model(self, model_type: str, direction: str) -> Tuple[dict, dict]:
        """Fit a single model for one force direction"""
        # Load appropriate data
        if direction == 'fx':
            data = self.data_loader.load_drivebrake_data()
        else:
            data = self.data_loader.load_cornering_data()
        
        # Preprocess
        data = self.preprocessor.filter_data(data)
        
        # Get model-specific bounds and initial guess
        bounds = self._get_parameter_bounds(model_type, direction)
        
        # Define objective function
        def objective(params):
            model = self._create_model(model_type, params, direction)
            predictions = self._predict(model, data, direction)
            rmse = np.sqrt(np.mean((predictions - data['force'])**2))
            return rmse
        
        # Optimize using differential evolution
        result = differential_evolution(
            objective,
            bounds,
            maxiter=1000,
            popsize=15,
            seed=42,
            workers=-1  # Use all CPU cores
        )
        
        # Calculate final metrics
        optimal_params = self._params_array_to_dict(result.x, model_type, direction)
        metrics = self._calculate_metrics(optimal_params, model_type, direction, data)
        
        return optimal_params, metrics
    
    def _get_parameter_bounds(self, model_type: str, direction: str) -> List[Tuple]:
        """Get parameter bounds for optimization"""
        if model_type == 'tmeasy':
            if direction == 'fx':
                return [
                    (0.1, 5.0),   # dfx0
                    (0.5, 3.0),   # fxm
                    (0.5, 3.0),   # fxs
                    (0.01, 0.5),  # sxm
                    (0.01, 0.5)   # sxs
                ]
            else:  # fy
                return [
                    (0.1, 5.0),   # dfy0
                    (0.5, 3.0),   # fym
                    (0.5, 3.0),   # fys
                    (0.01, 0.5),  # sym
                    (0.01, 0.5)   # sys
                ]
        elif model_type == 'magic_formula':
            if direction == 'fx':
                return [
                    (0.5, 3.0),   # pDx1
                    (1.0, 2.5),   # pCx1
                    (-1.0, 1.0),  # pEx1
                    (5.0, 50.0),  # pKx1
                    (-5.0, 5.0),  # pKx2
                    (-1.0, 1.0)   # pKx3
                ]
            else:  # fy
                return [
                    (0.5, 3.0),   # pDy1
                    (1.0, 2.5),   # pCy1
                    (-1.0, 1.0),  # pEy1
                    (5.0, 50.0),  # pKy1
                    (-5.0, 5.0),  # pKy2
                    (-1.0, 1.0)   # pKy3
                ]
    
    def _calculate_metrics(self, params: dict, model_type: str, direction: str, data) -> dict:
        """Calculate RMSE, TIC, R² for fitted model"""
        model = self._create_model(model_type, params, direction)
        predictions = self._predict(model, data, direction)
        actual = data['force']
        
        # RMSE
        rmse = np.sqrt(np.mean((predictions - actual)**2))
        
        # TIC (Theil Inequality Coefficient)
        numerator = np.sqrt(np.mean((predictions - actual)**2))
        denominator = np.sqrt(np.mean(predictions**2)) + np.sqrt(np.mean(actual**2))
        tic = numerator / denominator
        
        # R²
        ss_res = np.sum((actual - predictions)**2)
        ss_tot = np.sum((actual - np.mean(actual))**2)
        r2 = 1 - (ss_res / ss_tot)
        
        # Peak error
        peak_error = np.max(np.abs(predictions - actual))
        
        return {
            'rmse': rmse,
            'tic': tic,
            'r2': r2,
            'peak_error': peak_error
        }
```


## Data Models

### Fitting Results Structure

```python
@dataclass
class FittingMetrics:
    """Metrics for evaluating model fit quality"""
    rmse: float          # Root Mean Square Error (N)
    tic: float           # Theil Inequality Coefficient
    r2: float            # R-squared coefficient
    peak_error: float    # Maximum absolute error (N)

@dataclass
class ModelFittingResult:
    """Complete fitting result for one model and direction"""
    model_type: str                    # 'tmeasy' or 'magic_formula'
    direction: str                     # 'fx' or 'fy'
    parameters: Dict[str, float]       # Fitted parameter values
    metrics: FittingMetrics            # Fitting quality metrics
    convergence_info: Dict             # Optimization convergence details

@dataclass
class HybridModelConfig:
    """Configuration for hybrid tire model"""
    fx_model_type: str                 # Model type for longitudinal force
    fx_parameters: Dict[str, float]    # Parameters for fx model
    fy_model_type: str                 # Model type for lateral force
    fy_parameters: Dict[str, float]    # Parameters for fy model
    selection_mode: str                # 'auto', 'manual', or 'hybrid'
    selection_rationale: str           # Why these models were selected
```

### YAML Output Format

```yaml
# Hybrid tire model configuration
model_config:
  selection_mode: auto
  selection_rationale: "Magic Formula selected for FX (RMSE: 85N vs 2547N), TMeasy for FY (RMSE: 119N vs 125N)"

longitudinal_force:
  model_type: magic_formula
  parameters:
    pDx1: 2.45        # Peak force coefficient
    pCx1: 1.65        # Shape factor
    pEx1: -0.12       # Curvature factor
    pKx1: 21.3        # Stiffness coefficient
    pKx2: -1.2        # Load sensitivity
    pKx3: 0.15        # Exponential load factor
    fz0: 1200.0       # Nominal load (N)
  metrics:
    rmse: 85.3
    tic: 0.12
    r2: 0.9845
    peak_error: 245.7

lateral_force:
  model_type: tmeasy
  parameters:
    dfy0: 1.85
    fym: 2.42
    fys: 2.38
    sym: 0.089
    sys: 0.087
  metrics:
    rmse: 119.2
    tic: 0.040
    r2: 0.9937
    peak_error: 312.5

# Metadata
tire_info:
  name: "Hoosier 18.0x6.0-10 R20"
  nominal_load: 1200.0
  nominal_pressure: 12.0
  
fitting_info:
  timestamp: "2024-01-15T10:30:00"
  data_points_fx: 75860
  data_points_fy: 61032
  optimization_method: "differential_evolution"
  convergence_iterations: 847
```

### C++ Header Output Format

```cpp
// Generated tire model parameters
// Timestamp: 2024-01-15T10:30:00

#ifndef TIRE_PARAMS_HOOSIER_18X6_R20_H
#define TIRE_PARAMS_HOOSIER_18X6_R20_H

namespace tire_model {

// Model type enumeration
enum class ModelType {
    TMEASY,
    MAGIC_FORMULA
};

// Magic Formula parameters for longitudinal force
struct MagicFormulaFxParams {
    double pDx1 = 2.45;
    double pCx1 = 1.65;
    double pEx1 = -0.12;
    double pKx1 = 21.3;
    double pKx2 = -1.2;
    double pKx3 = 0.15;
    double fz0 = 1200.0;
};

// TMeasy parameters for lateral force
struct TMeasyFyParams {
    double dfy0 = 1.85;
    double fym = 2.42;
    double fys = 2.38;
    double sym = 0.089;
    double sys = 0.087;
};

// Hybrid model configuration
struct HybridTireModel {
    ModelType fx_model_type = ModelType::MAGIC_FORMULA;
    MagicFormulaFxParams fx_params;
    
    ModelType fy_model_type = ModelType::TMEASY;
    TMeasyFyParams fy_params;
};

} // namespace tire_model

#endif // TIRE_PARAMS_HOOSIER_18X6_R20_H
```


## Correctness Properties

A property is a characteristic or behavior that should hold true across all valid executions of a system—essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.

### Property 1: Magic Formula Force Continuity

*For any* Magic Formula model with valid parameters, and for any two inputs (slip, fz) that differ by a small amount (epsilon < 0.01), the force outputs should differ by a proportionally small amount, demonstrating continuity of the force function.

**Validates: Requirements 1.1, 1.2, 1.6**

### Property 2: Magic Formula Parameter Sensitivity

*For any* Magic Formula model, changing each parameter (B, C, D, E) individually should produce the expected effect on force output: D affects peak force magnitude, C affects shape, B affects stiffness, and E affects curvature.

**Validates: Requirements 1.3**

### Property 3: Load Dependency Behavior

*For any* Magic Formula model with load dependency parameters, increasing vertical load should increase the peak force magnitude and stiffness factor according to the load dependency equations.

**Validates: Requirements 1.4**

### Property 4: Parameter Bounds Enforcement

*For any* fitted model parameters, all parameter values must fall within the physically meaningful bounds specified for that model type (TMeasy or Magic Formula).

**Validates: Requirements 1.5, 5.5**

### Property 5: Unified Interface Consistency

*For any* tire model implementing the TireModel interface, the model must provide calculate_fx, calculate_fy, get_model_type, get_parameters, and set_parameters methods with consistent signatures.

**Validates: Requirements 2.1, 2.3, 2.4**

### Property 6: Model Type Query Accuracy

*For any* instantiated tire model, querying the model type should return the type that was specified during instantiation (tmeasy, magic_formula, or hybrid).

**Validates: Requirements 2.5**

### Property 7: TMeasy Backward Compatibility

*For any* input data and TMeasy parameters from the existing implementation, the new Hybrid_System using TMeasy-only mode should produce force outputs identical to the original TMeasy implementation (within numerical precision).

**Validates: Requirements 2.6, 9.3**

### Property 8: Auto Mode Selection Optimality

*For any* set of fitting results with different RMSE values, the Model_Selector in Auto mode should select the model with the lowest RMSE for each force direction, unless RMSE values are within 5%, in which case it should use TIC as the tiebreaker.

**Validates: Requirements 3.4, 4.3, 4.4**

### Property 9: Model Availability Validation

*For any* model selection request, if no models are available for a required force direction, the Model_Selector should raise a validation error indicating which direction lacks models.

**Validates: Requirements 3.6**

### Property 10: Fitting Result Caching

*For any* model type and force direction, if fitting is requested twice with the same data and parameters, the second request should return cached results without re-running optimization.

**Validates: Requirements 5.4**

### Property 11: Output Format Completeness

*For any* fitted hybrid model, the YAML output should contain model_type fields, all parameters for each fitted model, fitting metrics, and model selection metadata; and the C++ output should contain model-specific parameter structs.

**Validates: Requirements 6.1, 6.2, 6.3, 6.4, 11.1, 11.2, 11.3**

### Property 12: Configuration Round-Trip Consistency

*For any* hybrid model configuration, serializing to YAML then deserializing should produce an equivalent configuration with the same model types, parameters, and metadata.

**Validates: Requirements 6.6, 11.6**

### Property 13: Backward Compatible File Loading

*For any* existing TMeasy parameter YAML file from the previous implementation, the Hybrid_System should successfully load and parse the file, defaulting to TMeasy model type.

**Validates: Requirements 6.5, 9.2**

### Property 14: Comparison Report Completeness

*For any* auto mode execution that fits multiple models, the generated comparison report should include RMSE, TIC, R², and peak error for all fitted models, along with an indication of which model performs better for each force direction.

**Validates: Requirements 4.5, 12.1, 12.2, 12.3, 12.5**

### Property 15: CLI Flag Precedence

*For any* execution where both command-line flags and configuration file specify model preferences, the command-line flag values should take precedence over configuration file values.

**Validates: Requirements 8.6**

### Property 16: Logging Completeness

*For any* fitting execution, the logs should contain: which models are being fitted, fitting metrics for each model, model selection decisions with rationale, parameter values and bounds during optimization, and detailed error messages when errors occur.

**Validates: Requirements 10.1, 10.2, 10.3, 10.4, 10.5**

### Property 17: Data Pipeline Preservation

*For any* experimental tire data, the data loading and preprocessing pipeline should produce identical filtered datasets in the Hybrid_System as in the original TMeasy-only implementation.

**Validates: Requirements 9.4**


## Error Handling

### Model Instantiation Errors

**Invalid Model Type**:
- **Condition**: User specifies an unsupported model type
- **Response**: Raise `ValueError` with message listing valid model types
- **Recovery**: None - user must provide valid model type

**Missing Required Parameters**:
- **Condition**: Model instantiation attempted without required parameters
- **Response**: Raise `ValueError` with message listing missing parameters
- **Recovery**: None - user must provide all required parameters

### Fitting Pipeline Errors

**Optimization Non-Convergence**:
- **Condition**: Differential evolution fails to converge within iteration limit
- **Response**: Log warning with convergence diagnostics, return best result found
- **Recovery**: User can increase iteration limit or adjust bounds

**Parameter Bound Violations**:
- **Condition**: Optimizer produces parameters outside specified bounds
- **Response**: Clip parameters to bounds, log warning
- **Recovery**: Automatic - clipped parameters are used

**Insufficient Data**:
- **Condition**: Experimental data has fewer than 100 points after filtering
- **Response**: Raise `ValueError` with message about data quantity
- **Recovery**: None - user must provide more data or adjust filtering

**Numerical Instability**:
- **Condition**: Force calculation produces NaN or Inf values
- **Response**: Log error with input values, raise `RuntimeError`
- **Recovery**: None - indicates invalid parameters or data

### Model Selection Errors

**No Models Available**:
- **Condition**: Model selector cannot find any fitted models for a direction
- **Response**: Raise `ValueError` indicating which direction lacks models
- **Recovery**: None - user must fit at least one model

**Manual Mode Missing Specification**:
- **Condition**: Manual mode selected but model types not specified
- **Response**: Raise `ValueError` requesting model type specification
- **Recovery**: None - user must specify model types

**Metric Calculation Failure**:
- **Condition**: Cannot calculate RMSE/TIC/R² due to data issues
- **Response**: Log error, exclude model from selection
- **Recovery**: Automatic - other models can still be selected

### File I/O Errors

**YAML Parse Error**:
- **Condition**: Configuration file has invalid YAML syntax
- **Response**: Raise `yaml.YAMLError` with line number and description
- **Recovery**: None - user must fix YAML syntax

**Missing Required Fields**:
- **Condition**: YAML file missing required model parameters
- **Response**: Raise `ValueError` listing missing fields
- **Recovery**: None - user must provide complete configuration

**File Write Permission Error**:
- **Condition**: Cannot write output files due to permissions
- **Response**: Raise `IOError` with file path and permission details
- **Recovery**: None - user must fix file permissions

### Validation Errors

**Quality Threshold Violation**:
- **Condition**: Fitted model fails to meet RMSE/TIC/R² thresholds
- **Response**: Log warning with actual vs. required metrics, continue execution
- **Recovery**: Automatic - model is still usable but flagged as low quality

**Data Range Mismatch**:
- **Condition**: Input data outside expected ranges (e.g., slip ratio > 1.0)
- **Response**: Log warning, clip values to expected ranges
- **Recovery**: Automatic - clipped values are used


## Testing Strategy

The testing strategy employs a dual approach combining unit tests for specific scenarios and property-based tests for universal correctness guarantees.

### Unit Testing Approach

Unit tests focus on:
- **Specific examples**: Known input-output pairs for model validation
- **Edge cases**: Boundary conditions (zero slip, maximum load, etc.)
- **Error conditions**: Invalid inputs, missing parameters, convergence failures
- **Integration points**: Component interactions (model selector + fitter, exporter + models)

**Key Unit Test Suites**:

1. **Model Implementation Tests** (`test_models.py`)
   - TMeasy force calculations with known parameters
   - Magic Formula force calculations with known parameters
   - Hybrid model delegation to correct sub-models
   - Model type queries return correct values

2. **Model Selector Tests** (`test_model_selector.py`)
   - Auto mode selects lowest RMSE model
   - Manual mode respects user specification
   - Hybrid mode combines auto and manual correctly
   - Error handling for missing models

3. **Fitting Pipeline Tests** (`test_fitting.py`)
   - Single model fitting completes successfully
   - Parallel fitting of multiple models
   - Caching prevents redundant optimization
   - Convergence failure handling

4. **I/O Tests** (`test_io.py`)
   - YAML export includes all required fields
   - C++ header generation produces valid code
   - YAML import handles legacy TMeasy files
   - Error handling for malformed files

5. **CLI Tests** (`test_cli.py`)
   - Flag parsing for model selection
   - Configuration file loading
   - CLI flag precedence over config file
   - Help text and error messages

### Property-Based Testing Approach

Property tests verify universal properties across randomized inputs using the Hypothesis library. Each property test runs a minimum of 100 iterations with randomly generated inputs.

**Configuration**:
```python
from hypothesis import given, settings, strategies as st

# Custom strategies for tire model testing
@st.composite
def tire_parameters(draw, model_type='tmeasy'):
    """Generate valid tire model parameters"""
    if model_type == 'tmeasy':
        return {
            'dfx0': draw(st.floats(0.1, 5.0)),
            'fxm': draw(st.floats(0.5, 3.0)),
            'fxs': draw(st.floats(0.5, 3.0)),
            'sxm': draw(st.floats(0.01, 0.5)),
            'sxs': draw(st.floats(0.01, 0.5))
        }
    elif model_type == 'magic_formula':
        return {
            'pDx1': draw(st.floats(0.5, 3.0)),
            'pCx1': draw(st.floats(1.0, 2.5)),
            'pEx1': draw(st.floats(-1.0, 1.0)),
            'pKx1': draw(st.floats(5.0, 50.0)),
            'pKx2': draw(st.floats(-5.0, 5.0)),
            'pKx3': draw(st.floats(-1.0, 1.0)),
            'fz0': draw(st.floats(800.0, 1500.0))
        }

@st.composite
def tire_inputs(draw):
    """Generate valid tire model inputs"""
    return {
        'slip': draw(st.floats(-0.3, 0.3)),
        'fz': draw(st.floats(500.0, 2000.0))
    }
```

**Property Test Suites**:

1. **Model Continuity Properties** (`test_properties_continuity.py`)
   ```python
   @given(params=tire_parameters('magic_formula'), 
          input1=tire_inputs(), 
          epsilon=st.floats(0.001, 0.01))
   @settings(max_examples=100)
   def test_magic_formula_continuity(params, input1, epsilon):
       """Feature: hybrid-tire-model, Property 1: Magic Formula Force Continuity"""
       model = MagicFormulaModel(params)
       
       fx1 = model.calculate_fx(input1['slip'], input1['fz'])
       fx2 = model.calculate_fx(input1['slip'] + epsilon, input1['fz'])
       
       # Force should change proportionally to input change
       force_change = abs(fx2 - fx1)
       # Allow for reasonable derivative (max ~10000 N per unit slip)
       assert force_change < epsilon * 10000
   ```

2. **Parameter Bounds Properties** (`test_properties_bounds.py`)
   ```python
   @given(model_type=st.sampled_from(['tmeasy', 'magic_formula']))
   @settings(max_examples=100)
   def test_fitted_parameters_within_bounds(model_type):
       """Feature: hybrid-tire-model, Property 4: Parameter Bounds Enforcement"""
       # Mock fitting result
       fitter = ModelFitter(mock_data_loader, mock_preprocessor)
       params, metrics = fitter._fit_single_model(model_type, 'fx')
       
       bounds = fitter._get_parameter_bounds(model_type, 'fx')
       param_values = list(params.values())
       
       # All parameters must be within bounds
       for value, (lower, upper) in zip(param_values, bounds):
           assert lower <= value <= upper
   ```

3. **Interface Consistency Properties** (`test_properties_interface.py`)
   ```python
   @given(model_type=st.sampled_from(['tmeasy', 'magic_formula']))
   @settings(max_examples=100)
   def test_unified_interface_consistency(model_type):
       """Feature: hybrid-tire-model, Property 5: Unified Interface Consistency"""
       params = generate_valid_params(model_type)
       model = create_model(model_type, params)
       
       # All required methods must exist and be callable
       assert hasattr(model, 'calculate_fx')
       assert hasattr(model, 'calculate_fy')
       assert hasattr(model, 'get_model_type')
       assert hasattr(model, 'get_parameters')
       assert hasattr(model, 'set_parameters')
       
       # Methods should be callable
       assert callable(model.calculate_fx)
       assert callable(model.calculate_fy)
   ```

4. **Serialization Round-Trip Properties** (`test_properties_serialization.py`)
   ```python
   @given(fx_model=st.sampled_from(['tmeasy', 'magic_formula']),
          fy_model=st.sampled_from(['tmeasy', 'magic_formula']))
   @settings(max_examples=100)
   def test_configuration_round_trip(fx_model, fy_model):
       """Feature: hybrid-tire-model, Property 12: Configuration Round-Trip Consistency"""
       # Create hybrid configuration
       config = HybridModelConfig(
           fx_model_type=fx_model,
           fx_parameters=generate_valid_params(fx_model),
           fy_model_type=fy_model,
           fy_parameters=generate_valid_params(fy_model),
           selection_mode='manual',
           selection_rationale='Test configuration'
       )
       
       # Serialize to YAML
       yaml_str = serialize_to_yaml(config)
       
       # Deserialize back
       config2 = deserialize_from_yaml(yaml_str)
       
       # Should be equivalent
       assert config.fx_model_type == config2.fx_model_type
       assert config.fy_model_type == config2.fy_model_type
       assert config.fx_parameters == config2.fx_parameters
       assert config.fy_parameters == config2.fy_parameters
   ```

5. **Model Selection Properties** (`test_properties_selection.py`)
   ```python
   @given(rmse_diff=st.floats(0.0, 0.2))
   @settings(max_examples=100)
   def test_auto_mode_selection_optimality(rmse_diff):
       """Feature: hybrid-tire-model, Property 8: Auto Mode Selection Optimality"""
       # Create mock fitting results
       results = {
           'tmeasy': {'fx': {'rmse': 100.0, 'tic': 0.15, 'r2': 0.95}},
           'magic_formula': {'fx': {'rmse': 100.0 + rmse_diff * 100.0, 'tic': 0.12, 'r2': 0.97}}
       }
       
       selector = ModelSelector(SelectionMode.AUTO)
       fx_model, _ = selector.select_models(results)
       
       # Should select model with lower RMSE
       if rmse_diff > 0.05:  # More than 5% difference
           assert fx_model == 'tmeasy'
       elif rmse_diff < -0.05:
           assert fx_model == 'magic_formula'
       else:  # Within 5%, should use TIC tiebreaker
           assert fx_model == 'magic_formula'  # Has lower TIC
   ```

### Test Coverage Goals

- **Line coverage**: > 90% for all production code
- **Branch coverage**: > 85% for conditional logic
- **Property test iterations**: Minimum 100 per property
- **Integration test scenarios**: All major user workflows covered

### Continuous Integration

All tests run automatically on:
- Every commit to main branch
- Every pull request
- Nightly builds with extended property test iterations (1000+)

Test execution order:
1. Unit tests (fast feedback)
2. Property tests (comprehensive coverage)
3. Integration tests (end-to-end workflows)
4. Performance benchmarks (regression detection)
