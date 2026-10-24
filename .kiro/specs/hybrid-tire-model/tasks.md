# Implementation Plan: Hybrid Tire Model Fitting System

## Overview

This implementation plan breaks down the hybrid tire model system into incremental coding steps. Each task builds on previous work, with property-based tests integrated throughout to catch errors early. The plan follows a bottom-up approach: core models → interfaces → selection logic → fitting pipeline → I/O → CLI integration.

## Tasks

- [x] 1. Create unified tire model interface and refactor TMeasy
  - [x] 1.1 Define TireModel abstract base class with force calculation methods
    - Create `tire_fitting/models/base.py` with TireModel ABC
    - Define abstract methods: calculate_fx, calculate_fy, get_model_type, get_parameters, set_parameters
    - Add TireForces dataclass for force outputs
    - _Requirements: 2.1, 2.3, 2.4_
  
  - [x] 1.2 Refactor existing TMeasy implementation to use TireModel interface
    - Move TMeasy code to `tire_fitting/models/tmeasy.py`
    - Implement TireModel interface in TMeasyModel class
    - Preserve existing force calculation logic exactly
    - _Requirements: 2.6, 9.3_
  
  - [x]* 1.3 Write property test for TMeasy backward compatibility
    - **Property 7: TMeasy Backward Compatibility**
    - **Validates: Requirements 2.6, 9.3**
    - Generate random slip ratios, slip angles, and vertical loads
    - Compare new TMeasyModel outputs with original implementation
    - Assert outputs are identical within numerical precision (1e-10)
  
  - [ ]* 1.4 Write property test for unified interface consistency
    - **Property 5: Unified Interface Consistency**
    - **Validates: Requirements 2.1, 2.3, 2.4**
    - Generate random model types
    - Verify all required methods exist and are callable
    - Test that methods accept correct parameter types

- [x] 2. Implement Magic Formula tire model
  - [x] 2.1 Implement MagicFormulaModel class with MF5.2 equations
    - Create `tire_fitting/models/magic_formula.py`
    - Implement calculate_fx with pure longitudinal force equation
    - Implement calculate_fy with pure lateral force equation
    - Include load dependency parameters (pKx2, pKx3, pKy2, pKy3)
    - _Requirements: 1.1, 1.2, 1.3, 1.4_
  
  - [ ]* 2.2 Write property test for Magic Formula continuity
    - **Property 1: Magic Formula Force Continuity**
    - **Validates: Requirements 1.1, 1.2, 1.6**
    - Generate random valid parameters and inputs
    - Test that small input changes produce proportionally small output changes
    - Verify continuity for both fx and fy calculations
  
  - [ ]* 2.3 Write property test for parameter sensitivity
    - **Property 2: Magic Formula Parameter Sensitivity**
    - **Validates: Requirements 1.3**
    - Generate random base parameters
    - Vary each parameter (B, C, D, E) individually
    - Verify expected effects: D affects peak, C affects shape, B affects stiffness, E affects curvature
  
  - [ ]* 2.4 Write property test for load dependency
    - **Property 3: Load Dependency Behavior**
    - **Validates: Requirements 1.4**
    - Generate random parameters and varying vertical loads
    - Verify that increasing load increases peak force and stiffness
    - Test for both longitudinal and lateral directions

- [x] 3. Implement hybrid model wrapper
  - [x] 3.1 Create HybridTireModel class
    - Create `tire_fitting/models/hybrid.py`
    - Accept separate models for fx and fy in constructor
    - Delegate calculate_fx to fx_model, calculate_fy to fy_model
    - Implement get_model_type to return hybrid description
    - Implement get_parameters and set_parameters for both sub-models
    - _Requirements: 3.1, 3.2_
  
  - [x]* 3.2 Write unit tests for hybrid model delegation
    - Test that fx calls go to fx_model
    - Test that fy calls go to fy_model
    - Test get_model_type returns correct hybrid description
    - _Requirements: 3.1, 3.2_
  
  - [x]* 3.3 Write property test for model type query accuracy
    - **Property 6: Model Type Query Accuracy**
    - **Validates: Requirements 2.5**
    - Generate random model type combinations
    - Instantiate models and query their types
    - Verify returned type matches instantiation type

- [x] 4. Checkpoint - Ensure all model tests pass
  - Ensure all tests pass, ask the user if questions arise.

- [x] 5. Implement model selector with auto/manual/hybrid modes
  - [x] 5.1 Create ModelSelector class with selection logic
    - Create `tire_fitting/model_selector.py`
    - Implement SelectionMode enum (AUTO, MANUAL, HYBRID)
    - Implement select_models method with mode branching
    - Implement _auto_selection using RMSE-first, TIC-tiebreaker logic
    - Implement _manual_selection for user-specified models
    - Implement _hybrid_selection combining auto and manual
    - Add logging for all selection decisions
    - _Requirements: 3.3, 3.4, 3.5, 4.1, 4.2, 4.3, 4.4_
  
  - [ ]* 5.2 Write property test for auto mode selection optimality
    - **Property 8: Auto Mode Selection Optimality**
    - **Validates: Requirements 3.4, 4.3, 4.4**
    - Generate random fitting results with varying RMSE/TIC values
    - Test that lowest RMSE model is selected
    - Test that TIC is used as tiebreaker when RMSE within 5%
  
  - [ ]* 5.3 Write property test for model availability validation
    - **Property 9: Model Availability Validation**
    - **Validates: Requirements 3.6**
    - Generate scenarios with missing models for directions
    - Verify appropriate errors are raised
    - Check error messages indicate which direction lacks models
  
  - [ ]* 5.4 Write unit tests for manual and hybrid modes
    - Test manual mode respects user specification
    - Test hybrid mode combines auto and manual correctly
    - Test error handling for invalid mode configurations
    - _Requirements: 3.5, 4.1_

- [x] 6. Enhance fitting pipeline for multiple models
  - [x] 6.1 Extend ModelFitter to support multiple model types
    - Modify `tire_fitting/optimizer.py` to add ModelFitter class
    - Implement fit_multiple_models with parallel execution using ProcessPoolExecutor
    - Implement _fit_single_model for individual model fitting
    - Add _get_parameter_bounds for TMeasy and Magic Formula
    - Implement _calculate_metrics for RMSE, TIC, R², peak error
    - Add result caching to avoid redundant optimization
    - _Requirements: 5.1, 5.2, 5.3, 5.4, 5.5_
  
  - [ ]* 6.2 Write property test for parameter bounds enforcement
    - **Property 4: Parameter Bounds Enforcement**
    - **Validates: Requirements 1.5, 5.5**
    - Generate random model types
    - Run fitting (or mock fitting results)
    - Verify all fitted parameters are within specified bounds
  
  - [ ]* 6.3 Write property test for fitting result caching
    - **Property 10: Fitting Result Caching**
    - **Validates: Requirements 5.4**
    - Fit a model twice with same data and parameters
    - Verify second call returns cached results
    - Verify optimization is not re-run (check execution time or call counts)
  
  - [ ]* 6.4 Write unit tests for convergence failure handling
    - Test that non-convergence is logged with diagnostics
    - Test that best result found is returned despite non-convergence
    - Test error handling for numerical instability (NaN/Inf)
    - _Requirements: 5.6_

- [x] 7. Checkpoint - Ensure fitting pipeline tests pass
  - Ensure all tests pass, ask the user if questions arise.

- [x] 8. Implement enhanced YAML export with model metadata
  - [x] 8.1 Create YAML exporter for hybrid configurations
    - Create `tire_fitting/exporters/yaml_exporter.py`
    - Implement export_hybrid_config function
    - Include model_type field for each force direction
    - Store parameters for all fitted models
    - Add fitting metrics (RMSE, TIC, R², peak error)
    - Include model selection metadata and rationale
    - Add parameter units and descriptions as YAML comments
    - _Requirements: 6.1, 6.2, 6.4, 11.1, 11.2, 11.3, 11.5_
  
  - [x] 8.2 Create YAML importer with validation
    - Implement import_hybrid_config function in same file
    - Validate all required fields are present
    - Support loading legacy TMeasy-only files
    - Default to TMeasy model type when not specified
    - _Requirements: 6.6, 9.1, 9.2, 11.4_
  
  - [ ]* 8.3 Write property test for output format completeness
    - **Property 11: Output Format Completeness**
    - **Validates: Requirements 6.1, 6.2, 6.3, 6.4, 11.1, 11.2, 11.3**
    - Generate random hybrid configurations
    - Export to YAML
    - Verify all required fields are present (model_type, parameters, metrics, metadata)
    - Verify parameter units and descriptions are in comments
  
  - [ ]* 8.4 Write property test for configuration round-trip consistency
    - **Property 12: Configuration Round-Trip Consistency**
    - **Validates: Requirements 6.6, 11.6**
    - Generate random hybrid configurations
    - Serialize to YAML then deserialize
    - Verify deserialized config is equivalent to original
  
  - [ ]* 8.5 Write property test for backward compatible file loading
    - **Property 13: Backward Compatible File Loading**
    - **Validates: Requirements 6.5, 9.2**
    - Load existing TMeasy parameter files from v6/v8
    - Verify successful parsing
    - Verify default to TMeasy model type
    - Verify parameters are correctly loaded

- [x] 9. Implement C++ header export
  - [x] 9.1 Create C++ header exporter for hybrid models
    - Create `tire_fitting/exporters/cpp_exporter.py`
    - Generate ModelType enum
    - Generate model-specific parameter structs (TMeasyFxParams, MagicFormulaFxParams, etc.)
    - Generate HybridTireModel struct with model type fields and parameter structs
    - Add header guards and namespace
    - _Requirements: 6.3_
  
  - [x]* 9.2 Write unit tests for C++ header generation
    - Test that generated code has valid C++ syntax
    - Test that all parameter structs are present
    - Test that model type enum is correct
    - _Requirements: 6.3_

- [x] 10. Implement comparison report generation
  - [x] 10.1 Create comparison report generator
    - Create `tire_fitting/reporting/comparison_report.py`
    - Implement generate_comparison_report function
    - Include RMSE, TIC, R², peak error for all models
    - Highlight which model performs better for each direction
    - Generate force-slip curve visualizations using matplotlib
    - Save report in Markdown format with embedded plots
    - _Requirements: 4.5, 12.1, 12.2, 12.3, 12.4, 12.5, 12.6_
  
  - [ ]* 10.2 Write property test for comparison report completeness
    - **Property 14: Comparison Report Completeness**
    - **Validates: Requirements 4.5, 12.1, 12.2, 12.3, 12.5**
    - Generate mock fitting results for multiple models
    - Generate comparison report
    - Verify report contains all required metrics
    - Verify better model is indicated for each direction
  
  - [ ]* 10.3 Write unit tests for report visualization
    - Test that force-slip curve plots are generated
    - Test that plots are saved correctly
    - Test report formatting and structure
    - _Requirements: 12.4, 12.6_

- [x] 11. Checkpoint - Ensure I/O and reporting tests pass
  - Ensure all tests pass, ask the user if questions arise.

- [x] 12. Implement CLI enhancements for model selection
  - [x] 12.1 Add command-line flags to main.py
    - Add --model-long flag with choices [tmeasy, magic_formula, auto]
    - Add --model-lat flag with choices [tmeasy, magic_formula, auto]
    - Add --config flag for configuration file path
    - Implement flag parsing with argparse
    - _Requirements: 8.1, 8.2, 8.5_
  
  - [x] 12.2 Implement configuration file support
    - Support YAML configuration files for model preferences
    - Implement CLI flag precedence over config file
    - Add validation for configuration values
    - _Requirements: 8.5, 8.6_
  
  - [ ]* 12.3 Write property test for CLI flag precedence
    - **Property 15: CLI Flag Precedence**
    - **Validates: Requirements 8.6**
    - Generate random model preferences in config file and CLI flags
    - Verify CLI flags override config file values
  
  - [ ]* 12.4 Write unit tests for CLI parsing
    - Test flag parsing for all valid values
    - Test error handling for invalid flag values
    - Test help text generation
    - _Requirements: 8.1, 8.2, 8.3, 8.4_

- [x] 13. Implement comprehensive logging
  - [x] 13.1 Add logging throughout the system
    - Configure logging with levels (DEBUG, INFO, WARNING, ERROR)
    - Log model fitting start with model types
    - Log fitting metrics after completion
    - Log model selection decisions with rationale
    - Log parameter values and bounds during optimization
    - Log detailed error messages with context
    - _Requirements: 10.1, 10.2, 10.3, 10.4, 10.5, 10.6_
  
  - [ ]* 13.2 Write property test for logging completeness
    - **Property 16: Logging Completeness**
    - **Validates: Requirements 10.1, 10.2, 10.3, 10.4, 10.5**
    - Run fitting execution with log capture
    - Verify logs contain: models being fitted, metrics, selection decisions, parameters, errors
  
  - [ ]* 13.3 Write unit tests for logging configuration
    - Test that different log levels work correctly
    - Test log output formatting
    - Test log file creation
    - _Requirements: 10.6_

- [x] 14. Implement validation and quality checks
  - [x] 14.1 Add fitting quality validation
    - Create `tire_fitting/validation.py`
    - Implement validate_fitting_quality function
    - Check RMSE < 100 N for both directions
    - Check TIC < 0.15 for both directions
    - Check R² > 0.95 for both directions
    - Generate validation reports with pass/fail status
    - Log warnings for threshold violations
    - _Requirements: 7.1, 7.2, 7.3, 7.4, 7.5, 7.6_
  
  - [ ]* 14.2 Write unit tests for validation thresholds
    - Test validation passes with good metrics
    - Test validation fails with poor metrics
    - Test validation report generation
    - Test warning messages for threshold violations
    - _Requirements: 7.1, 7.2, 7.3, 7.4, 7.5_
  
  - [ ]* 14.3 Write property test for data pipeline preservation
    - **Property 17: Data Pipeline Preservation**
    - **Validates: Requirements 9.4**
    - Load experimental data using new system
    - Load same data using original TMeasy system
    - Verify filtered datasets are identical

- [x] 15. Integration and end-to-end wiring
  - [x] 15.1 Wire all components together in main.py
    - Integrate CLI parsing with model selector
    - Connect model selector with fitting pipeline
    - Wire fitting pipeline with exporters
    - Connect validation with reporting
    - Add error handling for all integration points
    - _Requirements: All requirements_
  
  - [ ]* 15.2 Write integration tests for complete workflows
    - Test auto mode end-to-end: data load → fit → select → export
    - Test manual mode end-to-end with specific model choices
    - Test hybrid mode end-to-end with mixed auto/manual
    - Test backward compatibility: TMeasy-only workflow
    - _Requirements: 9.5_
  
  - [ ]* 15.3 Write integration test for Python version compatibility
    - Test on Python 3.8, 3.9, 3.10, 3.11
    - Verify all functionality works across versions
    - _Requirements: 9.6_

- [x] 16. Final checkpoint - Comprehensive testing and validation
  - Run complete test suite (unit + property + integration)
  - Verify all 17 correctness properties pass
  - Run fitting on v6/v8 data and verify quality thresholds
  - Generate comparison reports and review results
  - Ensure all tests pass, ask the user if questions arise.
  - 実際のデータサンプル（抽出）と予想値の比較をして検証すること

## Notes

- Tasks marked with `*` are optional and can be skipped for faster MVP
- Each property test references its design document property number
- Checkpoints ensure incremental validation at logical breakpoints
- Integration tests verify end-to-end workflows work correctly
- All tasks reference specific requirements for traceability
