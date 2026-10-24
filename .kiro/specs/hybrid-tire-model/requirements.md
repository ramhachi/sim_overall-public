# Requirements Document

## Introduction

This document specifies the requirements for a hybrid tire model fitting system that combines TMeasy and Pacejka Magic Formula models to achieve superior fitting accuracy for both longitudinal and lateral tire forces. The system addresses the structural limitations discovered in TMeasy for longitudinal force prediction while maintaining its excellent lateral force performance.

## Glossary

- **TMeasy_Model**: The existing simplified tire model using hyperbolic tangent functions for force calculation
- **Magic_Formula**: The Pacejka Magic Formula tire model (MF5.2 or MF6.1) using trigonometric functions for force calculation
- **Hybrid_System**: The combined system that supports both TMeasy and Magic Formula models with independent selection per force direction
- **Fitting_Pipeline**: The optimization and parameter estimation system that fits tire models to experimental data
- **Force_Direction**: Either longitudinal (Fx) or lateral (Fy) tire force component
- **Model_Selector**: The component that chooses the best model based on fitting metrics or user preferences
- **Unified_Interface**: The common API that abstracts model-specific implementations
- **Fitting_Metrics**: Quantitative measures of model accuracy including RMSE, TIC, and R²

## Requirements

### Requirement 1: Magic Formula Model Implementation

**User Story:** As a tire dynamics engineer, I want to use the Pacejka Magic Formula model for tire force calculation, so that I can accurately represent complex longitudinal force characteristics that TMeasy cannot capture.

#### Acceptance Criteria

1. THE Magic_Formula SHALL implement the MF5.2 or MF6.1 equations for pure longitudinal force (Fx0)
2. THE Magic_Formula SHALL implement the MF5.2 or MF6.1 equations for pure lateral force (Fy0)
3. WHEN calculating forces, THE Magic_Formula SHALL use parameters B (stiffness factor), C (shape factor), D (peak value), and E (curvature factor)
4. THE Magic_Formula SHALL include load dependency parameters for vertical load variations
5. WHEN fitting parameters, THE Magic_Formula SHALL enforce physically meaningful parameter bounds based on tire dynamics literature
6. FOR ALL valid parameter sets, THE Magic_Formula SHALL produce continuous and differentiable force outputs

### Requirement 2: Unified Model Interface

**User Story:** As a vehicle simulator developer, I want a unified interface for all tire models, so that I can switch between models without changing simulation code.

#### Acceptance Criteria

1. THE Unified_Interface SHALL provide a common method signature for force calculation across all tire models
2. WHEN a model is instantiated, THE Unified_Interface SHALL accept model type specification (TMeasy, Magic_Formula, or Hybrid)
3. THE Unified_Interface SHALL expose methods for calculating longitudinal force given slip ratio and vertical load
4. THE Unified_Interface SHALL expose methods for calculating lateral force given slip angle and vertical load
5. WHEN queried, THE Unified_Interface SHALL return the active model type for each Force_Direction
6. THE Unified_Interface SHALL maintain backward compatibility with existing TMeasy-only code

### Requirement 3: Independent Model Selection

**User Story:** As a tire fitting specialist, I want to select different models for longitudinal and lateral forces independently, so that I can use the best model for each force direction.

#### Acceptance Criteria

1. THE Model_Selector SHALL support independent model selection for longitudinal and lateral Force_Direction
2. WHEN configured in hybrid mode, THE Hybrid_System SHALL allow TMeasy for one Force_Direction and Magic_Formula for the other
3. THE Model_Selector SHALL support three selection modes: Auto, Manual, and Hybrid
4. WHEN in Auto mode, THE Model_Selector SHALL automatically choose the model with the best Fitting_Metrics for each Force_Direction
5. WHEN in Manual mode, THE Model_Selector SHALL use user-specified models for each Force_Direction
6. THE Model_Selector SHALL validate that at least one model is available for each Force_Direction

### Requirement 4: Automatic Model Selection

**User Story:** As a researcher, I want the system to automatically select the best model based on fitting accuracy, so that I can achieve optimal results without manual model comparison.

#### Acceptance Criteria

1. WHEN in Auto mode, THE Model_Selector SHALL fit both TMeasy and Magic_Formula to the experimental data
2. THE Model_Selector SHALL compare models using RMSE, TIC, and R² metrics
3. WHEN comparing models, THE Model_Selector SHALL select the model with the lowest RMSE as the primary criterion
4. IF RMSE values differ by less than 5 percent, THEN THE Model_Selector SHALL use TIC as the tiebreaker
5. THE Model_Selector SHALL generate a comparison report showing Fitting_Metrics for all fitted models
6. THE Model_Selector SHALL log the selection decision with justification based on Fitting_Metrics

### Requirement 5: Parameter Fitting Enhancement

**User Story:** As a tire model calibrator, I want the fitting pipeline to support multiple tire models, so that I can fit and compare different models efficiently.

#### Acceptance Criteria

1. THE Fitting_Pipeline SHALL support parameter optimization for both TMeasy_Model and Magic_Formula
2. WHEN fitting Magic_Formula, THE Fitting_Pipeline SHALL use model-specific objective functions
3. THE Fitting_Pipeline SHALL support parallel fitting of multiple models for comparison
4. WHEN fitting completes, THE Fitting_Pipeline SHALL cache results for each model to avoid redundant computation
5. THE Fitting_Pipeline SHALL enforce model-specific parameter bounds during optimization
6. WHEN optimization fails to converge, THE Fitting_Pipeline SHALL report the failure with diagnostic information

### Requirement 6: Output Format Extension

**User Story:** As a simulation engineer, I want fitted parameters exported in formats compatible with my simulation tools, so that I can directly use the fitted models.

#### Acceptance Criteria

1. WHEN exporting to YAML, THE Hybrid_System SHALL include a model_type field indicating the active model for each Force_Direction
2. THE Hybrid_System SHALL store parameters for all successfully fitted models in the YAML output
3. WHEN exporting to C++ header format, THE Hybrid_System SHALL generate model-specific parameter structs
4. THE Hybrid_System SHALL include model selection metadata in all output files
5. THE Hybrid_System SHALL maintain backward compatibility with existing YAML and C++ export formats
6. WHEN reading configuration files, THE Hybrid_System SHALL parse model type specifications and parameters correctly

### Requirement 7: Fitting Quality Validation

**User Story:** As a quality assurance engineer, I want the system to validate that fitted models meet accuracy requirements, so that I can ensure reliable simulation results.

#### Acceptance Criteria

1. WHEN fitting completes, THE Hybrid_System SHALL validate that longitudinal force RMSE is less than 100 N
2. WHEN fitting completes, THE Hybrid_System SHALL validate that lateral force RMSE is less than 100 N
3. THE Hybrid_System SHALL validate that TIC is less than 0.15 for both Force_Direction components
4. THE Hybrid_System SHALL validate that R² is greater than 0.95 for both Force_Direction components
5. IF validation fails, THEN THE Hybrid_System SHALL report which metrics failed and by how much
6. THE Hybrid_System SHALL generate validation reports comparing fitted model predictions against experimental data

### Requirement 8: Command-Line Interface

**User Story:** As a command-line user, I want to control model selection through command-line arguments, so that I can easily run different fitting configurations.

#### Acceptance Criteria

1. THE Hybrid_System SHALL accept a --model-long flag with values [tmeasy, magic_formula, auto]
2. THE Hybrid_System SHALL accept a --model-lat flag with values [tmeasy, magic_formula, auto]
3. WHEN --model-long is set to auto, THE Model_Selector SHALL automatically select the best longitudinal model
4. WHEN --model-lat is set to auto, THE Model_Selector SHALL automatically select the best lateral model
5. THE Hybrid_System SHALL support configuration file input for model preferences as an alternative to command-line flags
6. WHEN both command-line flags and configuration file are provided, THE Hybrid_System SHALL prioritize command-line flags

### Requirement 9: Backward Compatibility

**User Story:** As a maintainer of existing simulation code, I want the hybrid system to maintain compatibility with existing TMeasy implementations, so that I don't need to modify working code.

#### Acceptance Criteria

1. WHEN no model type is specified, THE Hybrid_System SHALL default to TMeasy_Model for both Force_Direction components
2. THE Hybrid_System SHALL support loading existing TMeasy parameter files without modification
3. WHEN using TMeasy_Model only, THE Hybrid_System SHALL produce identical results to the previous implementation
4. THE Hybrid_System SHALL maintain the existing data loading and preprocessing pipeline without changes
5. THE Hybrid_System SHALL pass all existing unit tests and integration tests for TMeasy functionality
6. THE Hybrid_System SHALL support Python 3.8 and later versions

### Requirement 10: Logging and Diagnostics

**User Story:** As a debugging engineer, I want detailed logging of model selection and fitting processes, so that I can diagnose issues and understand system behavior.

#### Acceptance Criteria

1. WHEN fitting begins, THE Hybrid_System SHALL log which models are being fitted for each Force_Direction
2. THE Hybrid_System SHALL log Fitting_Metrics for each model after fitting completes
3. WHEN Model_Selector makes a selection decision, THE Hybrid_System SHALL log the decision rationale
4. THE Hybrid_System SHALL log parameter values and bounds during optimization
5. WHEN errors occur, THE Hybrid_System SHALL log detailed error messages with context
6. THE Hybrid_System SHALL support configurable logging levels (DEBUG, INFO, WARNING, ERROR)

### Requirement 11: Magic Formula Parameter Serialization

**User Story:** As a data analyst, I want Magic Formula parameters stored in a structured format, so that I can analyze and compare parameters across different tires and conditions.

#### Acceptance Criteria

1. WHEN serializing Magic Formula parameters to YAML, THE Hybrid_System SHALL store B, C, D, E parameters for longitudinal force
2. WHEN serializing Magic Formula parameters to YAML, THE Hybrid_System SHALL store B, C, D, E parameters for lateral force
3. THE Hybrid_System SHALL store load dependency parameters in the YAML output
4. WHEN deserializing from YAML, THE Hybrid_System SHALL validate that all required Magic Formula parameters are present
5. THE Hybrid_System SHALL include parameter units and descriptions in YAML comments
6. FOR ALL serialized parameters, deserializing then serializing SHALL produce equivalent YAML structure

### Requirement 12: Model Comparison Reporting

**User Story:** As a research engineer, I want detailed comparison reports between TMeasy and Magic Formula, so that I can understand the performance differences and make informed model choices.

#### Acceptance Criteria

1. WHEN Auto mode completes, THE Hybrid_System SHALL generate a comparison report showing RMSE for both models
2. THE comparison report SHALL include TIC and R² values for both models
3. THE comparison report SHALL show peak force error for both models
4. THE comparison report SHALL include force-slip curve visualizations for both models
5. THE comparison report SHALL highlight which model performs better for each Force_Direction
6. THE Hybrid_System SHALL save comparison reports in a structured format (Markdown or HTML)
