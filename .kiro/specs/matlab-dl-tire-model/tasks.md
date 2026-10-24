# Implementation Plan: MATLAB Deep Learning Tire Model

## Overview

This implementation plan breaks down the development of a MATLAB deep learning tire model into discrete, incremental tasks. The model will predict tire forces (Fx, Fy) and moments (Mx, Mz) from tire state variables using a feedforward neural network trained on ~140,000 TTC tire test samples.

The implementation follows this sequence:
1. Set up project structure and data loading
2. Implement data preprocessing and normalization
3. Define neural network architecture
4. Implement training engine with GPU support
5. Implement validation and metrics computation
6. Create visualization functions
7. Implement model export functionality
8. Add hyperparameter tuning support
9. Create complete workflow scripts

## Tasks

- [x] 1. Set up project structure and data loading
  - Create directory structure for MATLAB project
  - Implement CSV data loading function for TTC tire test data
  - Add validation for required columns (SR, SA, Fz, IA, TSTC, Fx, Fy, Mx, Mz)
  - Implement error handling for missing or corrupted files
  - _Requirements: 1.1_

- [ ]* 1.1 Write unit test for data loading with valid CSV
  - Test loading DriveBrake and Cornering CSV files
  - Verify correct column extraction
  - _Requirements: 1.1_

- [ ]* 1.2 Write unit test for data loading error handling
  - Test missing file error
  - Test missing columns error
  - _Requirements: 1.1_

- [ ] 2. Implement data preprocessing and normalization
  - [x] 2.1 Implement feature extraction (SR, SA, Fz, IA, TSTC) and target extraction (Fx, Fy, Mx, Mz)
    - Extract input features as N×5 matrix
    - Extract output targets as N×4 matrix
    - _Requirements: 1.2_

  - [ ]* 2.2 Write property test for feature extraction
    - **Property 2: Data extraction preserves column mapping**
    - **Validates: Requirements 1.2**

  - [x] 2.3 Implement missing data handling
    - Detect NaN and Inf values
    - Remove rows with missing data
    - Log statistics on removed samples
    - _Requirements: 1.6_

  - [x] 2.4 Implement normalization functions
    - Z-score normalization for inputs (zero mean, unit variance)
    - Z-score normalization for outputs
    - Store normalization parameters (mean, std)
    - _Requirements: 1.3, 1.4, 1.7_

  - [ ]* 2.5 Write property test for normalization
    - **Property 1: Normalization produces zero mean and unit variance**
    - **Validates: Requirements 1.3, 1.4**

  - [ ]* 2.6 Write property test for normalization parameter storage
    - **Property 4: Normalization parameters are stored and retrievable**
    - **Validates: Requirements 1.7**

  - [ ]* 2.7 Write property test for normalization round-trip
    - **Property 8: Normalization round-trip preserves values**
    - **Validates: Requirements 4.5**

  - [x] 2.8 Implement data splitting function
    - Split data into training and validation sets (80/20 default)
    - Support configurable split ratio
    - _Requirements: 1.5_

  - [ ]* 2.9 Write property test for data splitting
    - **Property 3: Data splitting maintains specified ratio**
    - **Validates: Requirements 1.5**

  - [x] 2.10 Create complete preprocessing pipeline function
    - Combine all preprocessing steps into single function
    - Return training data, validation data, and normalization parameters
    - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7_

- [x] 3. Checkpoint - Verify data preprocessing
  - Ensure all preprocessing tests pass
  - Verify preprocessing works on actual TTC data files
  - Ask the user if questions arise

- [x] 4. Define neural network architecture
  - [x] 4.1 Implement feedforward network creation function
    - Input layer: 5 neurons (SR, SA, Fz, IA, TSTC)
    - Hidden layer 1: 128 neurons, ReLU activation, 0.2 dropout
    - Hidden layer 2: 64 neurons, ReLU activation, 0.2 dropout
    - Hidden layer 3: 32 neurons, ReLU activation
    - Output layer: 4 neurons (Fx, Fy, Mx, Mz), linear activation
    - _Requirements: 2.1, 2.2, 2.3, 2.5, 2.7_

  - [ ]* 4.2 Write unit test for model architecture
    - Verify input layer size is 5
    - Verify output layer size is 4
    - Verify at least 2 hidden layers exist
    - Verify dropout layers are present
    - _Requirements: 2.1, 2.2, 2.3, 2.7_

  - [x] 4.3 Implement optional LSTM architecture creation function
    - LSTM-based architecture for temporal dependencies
    - Support both feedforward and LSTM via configuration
    - _Requirements: 2.6_

- [x] 5. Implement training engine with GPU support
  - [x] 5.1 Implement training options configuration function
    - Adam optimizer
    - Learning rate: 0.001 with decay schedule
    - Batch size: 256 (configurable)
    - Max epochs: 200 (configurable)
    - Early stopping with patience: 10 epochs
    - Validation frequency configuration
    - Execution environment: 'auto' (GPU if available)
    - _Requirements: 3.2, 3.3, 3.4, 3.5, 3.8, 9.4, 9.5_

  - [x] 5.2 Implement GPU detection and fallback logic
    - Check GPU availability using gpuDeviceCount
    - Display GPU information if available
    - Warn if no GPU detected
    - Fall back to CPU training if GPU fails
    - _Requirements: 9.4, 9.5, 9.7_

  - [ ]* 5.3 Write unit test for GPU detection
    - Test GPU detection when available
    - Test CPU fallback when GPU unavailable
    - _Requirements: 9.4, 9.5, 9.7_

  - [x] 5.4 Implement main training function
    - Set random seed for reproducibility
    - Create training options
    - Train network using trainNetwork
    - Handle GPU out-of-memory errors
    - Detect and handle training divergence (NaN loss)
    - Return trained network and training info
    - _Requirements: 3.1, 3.6, 3.7, 10.3_

  - [ ]* 5.5 Write property test for reproducibility
    - **Property 12: Reproducibility with fixed random seed**
    - **Validates: Requirements 10.3**

  - [ ]* 5.6 Write unit test for training error handling
    - Test GPU out-of-memory handling
    - Test NaN loss detection
    - _Requirements: 3.1, 3.6_

- [x] 6. Checkpoint - Verify training works
  - Train a small model on subset of data
  - Verify GPU is being used (if available)
  - Ensure training completes without errors
  - Ask the user if questions arise

- [x] 7. Implement validation and metrics computation
  - [x] 7.1 Implement RMSE computation function
    - Compute RMSE for each output (Fx, Fy, Mx, Mz)
    - _Requirements: 4.1_

  - [ ]* 7.2 Write property test for RMSE computation
    - **Property 5: RMSE computation is mathematically correct**
    - **Validates: Requirements 4.1**

  - [x] 7.3 Implement R² computation function
    - Compute R² for each output
    - _Requirements: 4.2_

  - [ ]* 7.4 Write property test for R² computation
    - **Property 6: R² computation is mathematically correct**
    - **Validates: Requirements 4.2**

  - [x] 7.5 Implement TIC computation function
    - Compute TIC for each output
    - _Requirements: 4.3_

  - [ ]* 7.6 Write property test for TIC computation
    - **Property 7: TIC computation is mathematically correct**
    - **Validates: Requirements 4.3**

  - [x] 7.7 Implement model evaluation function
    - Make predictions on validation data
    - Denormalize predictions and targets
    - Compute all metrics (RMSE, R², TIC, MAE)
    - Check performance thresholds (RMSE < 100N, R² > 0.95)
    - Return metrics structure
    - _Requirements: 4.1, 4.2, 4.3, 4.4, 4.5, 4.7, 4.8_

  - [ ]* 7.8 Write unit test for performance threshold checking
    - Test RMSE threshold flagging (> 100N)
    - Test R² threshold flagging (< 0.95)
    - _Requirements: 4.7, 4.8_

  - [x] 7.9 Implement out-of-range input detection
    - Check if inputs are within training data range
    - Generate warnings for out-of-range inputs
    - _Requirements: 8.3, 8.5_

  - [ ]* 7.10 Write property test for out-of-range detection
    - **Property 13: Out-of-range input detection**
    - **Validates: Requirements 8.3**

- [x] 8. Create visualization functions
  - [x] 8.1 Implement scatter plot generation (predicted vs actual)
    - Create scatter plots for each output (Fx, Fy, Mx, Mz)
    - Add perfect prediction reference line
    - Save plots to output directory
    - _Requirements: 5.1, 5.5_

  - [x] 8.2 Implement residual plot generation
    - Create residual plots for each output
    - Add zero reference line
    - Save plots to output directory
    - _Requirements: 5.2, 5.5_

  - [x] 8.3 Implement time series plot generation
    - Create time series plots for sequential data
    - Save plots to output directory
    - _Requirements: 5.3, 5.5_

  - [x] 8.4 Implement performance by operating condition plots
    - Create plots showing performance across different Fz, SA, SR ranges
    - Save plots to output directory
    - _Requirements: 5.4, 5.5_

  - [ ]* 8.5 Write unit test for visualization file creation
    - Verify all plot files are created
    - Verify plots are saved to correct directory
    - _Requirements: 5.6_

- [x] 9. Implement model export functionality
  - [x] 9.1 Implement basic model export to .mat file
    - Save trained network and normalization parameters
    - _Requirements: 6.1, 6.2_

  - [x] 9.2 Implement prediction function for exported model
    - Create function that accepts raw inputs
    - Normalize inputs, predict, denormalize outputs
    - Save prediction function alongside model
    - _Requirements: 6.3_

  - [ ]* 9.3 Write property test for prediction function
    - **Property 9: Prediction function applies correct transformations**
    - **Validates: Requirements 6.3**

  - [x] 9.4 Implement export validation
    - Reload exported model
    - Verify predictions match original model
    - Abort export if validation fails
    - _Requirements: 6.6_

  - [ ]* 9.5 Write property test for export-reload
    - **Property 10: Model export-reload preserves predictions**
    - **Validates: Requirements 6.6**

  - [x] 9.6 Implement lookup table generation (optional)
    - Generate multi-dimensional grid of inputs
    - Compute predictions on grid
    - Save lookup table for C++ integration
    - _Requirements: 6.4_

  - [ ]* 9.7 Write unit test for lookup table generation
    - Verify lookup table is created
    - Verify grid dimensions are correct
    - _Requirements: 6.4_

- [x] 10. Implement batch prediction support
  - [x] 10.1 Verify batch prediction works correctly
    - Ensure predict() handles multiple samples
    - _Requirements: 9.2_

  - [ ]* 10.2 Write property test for batch predictions
    - **Property 11: Batch predictions match individual predictions**
    - **Validates: Requirements 9.2**

- [x] 11. Add hyperparameter tuning support
  - [x] 11.1 Implement configuration structure for hyperparameters
    - Support learning rate, batch size, epochs, patience, etc.
    - _Requirements: 7.1_

  - [x] 11.2 Implement experiment logging
    - Log hyperparameter configurations
    - Log performance metrics for each experiment
    - Save logs to file
    - _Requirements: 7.2_

  - [x] 11.3 Implement grid search or random search function
    - Define hyperparameter search space
    - Train models with different configurations
    - Track best configuration
    - _Requirements: 7.3_

  - [x] 11.4 Implement experiment comparison function
    - Generate comparison tables across experiments
    - Save comparison results
    - _Requirements: 7.4_

  - [x] 11.5 Implement unique model saving for experiments
    - Save each experiment's model with unique identifier
    - _Requirements: 7.5_

- [x] 12. Checkpoint - Verify complete system
  - Run full workflow on actual TTC data
  - Verify all metrics are computed correctly
  - Verify visualizations are generated
  - Verify model can be exported and reloaded
  - Ask the user if questions arise

- [x] 13. Create complete workflow scripts
  - [x] 13.1 Create main training script
    - Load data
    - Preprocess data
    - Create model
    - Train model
    - Evaluate model
    - Generate visualizations
    - Export model
    - _Requirements: 10.2_

  - [x] 13.2 Create prediction script for using trained model
    - Load exported model
    - Accept new input data
    - Make predictions
    - Display results
    - _Requirements: 10.5_

  - [x] 13.3 Create hyperparameter tuning script
    - Define search space
    - Run grid/random search
    - Generate comparison report
    - _Requirements: 7.3, 7.4_

  - [ ]* 13.4 Write integration test for full workflow
    - Test complete pipeline from data loading to export
    - Verify all components work together
    - _Requirements: 10.2_

- [x] 14. Create documentation
  - [x] 14.1 Write README with usage instructions
    - Document model architecture
    - Document training procedure
    - Document how to use trained model
    - Document dependencies (Deep Learning Toolbox, Parallel Computing Toolbox)
    - _Requirements: 10.1, 10.4_

  - [x] 14.2 Create example scripts
    - Example: Train model on TTC data
    - Example: Use trained model for prediction
    - Example: Export model for C++ integration
    - _Requirements: 10.5_

  - [x] 14.3 Document data format requirements
    - Document expected CSV structure
    - Document preprocessing steps
    - _Requirements: 10.6_

- [x] 15. Final checkpoint - Complete system validation
  - Run all unit tests and property tests
  - Train model on full TTC dataset (~140k samples)
  - Verify performance meets requirements (RMSE < 100N, R² > 0.95)
  - Verify training time < 30 minutes with GPU
  - Verify prediction time < 1ms per sample
  - Generate final performance report
  - Ask the user if questions arise

## Notes

- Tasks marked with `*` are optional and can be skipped for faster MVP
- Each task references specific requirements for traceability
- Checkpoints ensure incremental validation throughout development
- Property tests validate universal correctness properties (minimum 100 iterations each)
- Unit tests validate specific examples and edge cases
- GPU acceleration is automatically used when available (RTX 4070 Ti)
- All code should be MATLAB-compatible (R2023a or later)
