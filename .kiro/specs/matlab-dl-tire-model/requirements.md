# Requirements Document

## Introduction

This document specifies the requirements for a deep learning-based tire model implemented in MATLAB that predicts tire forces and moments from tire test data. The model serves as an alternative to traditional physics-based models (TMeasy, Magic Formula) by leveraging neural networks to learn complex tire behavior patterns from approximately 140,000 data points collected from TTC tire testing.

## Glossary

- **DL_Model**: The deep learning neural network model that predicts tire forces and moments
- **Training_Engine**: The MATLAB component responsible for training the neural network
- **Data_Preprocessor**: The component that prepares raw tire test data for model training
- **Model_Exporter**: The component that exports the trained model for integration with simulators
- **Validation_Engine**: The component that evaluates model performance against test data
- **Input_Features**: The set of tire state variables used as model inputs (SR, SA, Fz, IA, TSTC)
- **Output_Predictions**: The set of tire forces and moments predicted by the model (Fx, Fy, Mx, Mz)
- **TTC_Dataset**: The tire test consortium dataset containing approximately 140,000 samples
- **RMSE**: Root Mean Square Error, a metric for prediction accuracy
- **R²**: Coefficient of determination, a metric for model fit quality
- **TIC**: Theil Inequality Coefficient, a normalized error metric

## Requirements

### Requirement 1: Data Loading and Preprocessing

**User Story:** As a tire model developer, I want to load and preprocess TTC tire test data, so that I can prepare training and validation datasets for the neural network.

#### Acceptance Criteria

1. WHEN the system loads TTC tire test data, THE Data_Preprocessor SHALL read CSV files containing DriveBrake and Cornering test data
2. WHEN preprocessing data, THE Data_Preprocessor SHALL extract input features (SR, SA, Fz, IA, TSTC) and output targets (Fx, Fy, Mx, Mz)
3. WHEN preprocessing data, THE Data_Preprocessor SHALL normalize input features to zero mean and unit variance
4. WHEN preprocessing data, THE Data_Preprocessor SHALL normalize output targets to improve training stability
5. WHEN splitting data, THE Data_Preprocessor SHALL partition the dataset into 80% training and 20% validation sets
6. WHEN handling missing values, THE Data_Preprocessor SHALL remove or impute missing data points appropriately
7. THE Data_Preprocessor SHALL store normalization parameters for use during inference

### Requirement 2: Neural Network Architecture Definition

**User Story:** As a tire model developer, I want to define a neural network architecture, so that the model can learn complex relationships between tire inputs and outputs.

#### Acceptance Criteria

1. THE DL_Model SHALL accept five input features (SR, SA, Fz, IA, TSTC)
2. THE DL_Model SHALL produce four output predictions (Fx, Fy, Mx, Mz)
3. THE DL_Model SHALL use a feedforward neural network architecture with at least two hidden layers
4. WHEN defining layer sizes, THE DL_Model SHALL use hidden layers with sufficient neurons to capture tire behavior complexity
5. THE DL_Model SHALL use activation functions appropriate for regression tasks
6. WHERE temporal dependencies are important, THE DL_Model SHALL support LSTM or recurrent architectures as an optional configuration
7. THE DL_Model SHALL use dropout or regularization techniques to prevent overfitting

### Requirement 3: Model Training

**User Story:** As a tire model developer, I want to train the neural network on tire test data, so that the model learns to predict tire forces and moments accurately.

#### Acceptance Criteria

1. WHEN training begins, THE Training_Engine SHALL use the training dataset partition
2. THE Training_Engine SHALL use an appropriate loss function for multi-output regression (mean squared error or similar)
3. THE Training_Engine SHALL use an optimization algorithm suitable for deep learning (Adam, SGD with momentum, or similar)
4. WHEN training, THE Training_Engine SHALL monitor validation loss to detect overfitting
5. THE Training_Engine SHALL implement early stopping when validation loss stops improving
6. THE Training_Engine SHALL save the best model based on validation performance
7. WHEN training completes, THE Training_Engine SHALL report final training and validation losses
8. THE Training_Engine SHALL support configurable hyperparameters (learning rate, batch size, epochs)

### Requirement 4: Model Validation and Performance Metrics

**User Story:** As a tire model developer, I want to evaluate model performance using standard metrics, so that I can assess prediction accuracy and compare with existing models.

#### Acceptance Criteria

1. WHEN validating the model, THE Validation_Engine SHALL compute RMSE for each output (Fx, Fy, Mx, Mz)
2. WHEN validating the model, THE Validation_Engine SHALL compute R² for each output
3. WHEN validating the model, THE Validation_Engine SHALL compute TIC for each output
4. THE Validation_Engine SHALL evaluate performance on the validation dataset
5. WHEN computing metrics, THE Validation_Engine SHALL denormalize predictions and targets to physical units
6. THE Validation_Engine SHALL generate performance comparison reports against TMeasy and Magic Formula models
7. WHEN RMSE for force predictions exceeds 100N, THE Validation_Engine SHALL flag the model as not meeting accuracy requirements
8. WHEN R² for any output is below 0.95, THE Validation_Engine SHALL flag the model as not meeting fit quality requirements

### Requirement 5: Prediction Visualization

**User Story:** As a tire model developer, I want to visualize model predictions against actual data, so that I can identify patterns, errors, and areas for improvement.

#### Acceptance Criteria

1. WHEN generating visualizations, THE Validation_Engine SHALL create scatter plots of predicted vs actual values for each output
2. WHEN generating visualizations, THE Validation_Engine SHALL create residual plots showing prediction errors
3. WHEN generating visualizations, THE Validation_Engine SHALL create time series plots for sequential test data
4. THE Validation_Engine SHALL generate plots showing prediction performance across different operating conditions (varying Fz, SA, SR)
5. WHEN creating plots, THE Validation_Engine SHALL include reference lines for perfect prediction
6. THE Validation_Engine SHALL save all visualizations to output files for documentation

### Requirement 6: Model Export and Integration

**User Story:** As a vehicle dynamics engineer, I want to export the trained model for use in vehicle simulators, so that I can integrate the deep learning tire model into simulation workflows.

#### Acceptance Criteria

1. WHEN exporting the model, THE Model_Exporter SHALL save the trained neural network in MATLAB format (.mat file)
2. THE Model_Exporter SHALL save normalization parameters alongside the model
3. THE Model_Exporter SHALL provide a prediction function that accepts raw inputs and returns denormalized outputs
4. WHERE C++ integration is required, THE Model_Exporter SHALL generate MATLAB Coder-compatible code or lookup tables
5. THE Model_Exporter SHALL include documentation on how to use the exported model
6. WHEN exporting, THE Model_Exporter SHALL validate that the exported model produces identical predictions to the trained model

### Requirement 7: Hyperparameter Tuning and Experimentation

**User Story:** As a tire model developer, I want to experiment with different hyperparameters and architectures, so that I can optimize model performance.

#### Acceptance Criteria

1. THE Training_Engine SHALL support configuration files or scripts for specifying hyperparameters
2. WHEN experimenting, THE Training_Engine SHALL log all hyperparameter configurations and corresponding performance metrics
3. THE Training_Engine SHALL support grid search or random search for hyperparameter optimization
4. WHEN comparing experiments, THE Validation_Engine SHALL generate comparison tables showing performance across different configurations
5. THE Training_Engine SHALL save models from different experiments with unique identifiers

### Requirement 8: Model Robustness and Generalization

**User Story:** As a tire model developer, I want to ensure the model generalizes well to unseen data, so that predictions remain accurate across different operating conditions.

#### Acceptance Criteria

1. WHEN evaluating generalization, THE Validation_Engine SHALL test the model on data from different test conditions than training data
2. THE DL_Model SHALL maintain prediction accuracy when inputs are within the training data range
3. WHEN inputs are near the boundaries of the training data range, THE DL_Model SHALL provide reasonable extrapolation or flag out-of-range inputs
4. THE Validation_Engine SHALL report performance metrics separately for different operating regimes (low/medium/high Fz, different SA ranges)
5. WHEN the model encounters out-of-distribution inputs, THE DL_Model SHALL provide uncertainty estimates or warnings

### Requirement 9: Computational Performance and GPU Acceleration

**User Story:** As a vehicle dynamics engineer, I want the model to make predictions efficiently and leverage GPU acceleration during training, so that training is fast and the model can be used in real-time or near-real-time simulations.

#### Acceptance Criteria

1. WHEN making predictions, THE DL_Model SHALL compute outputs for a single input sample in less than 1 millisecond on standard hardware
2. THE DL_Model SHALL support batch prediction for multiple input samples simultaneously
3. WHEN used in simulation, THE DL_Model SHALL have minimal memory footprint to allow integration with larger vehicle models
4. WHERE MATLAB Parallel Computing Toolbox is available, THE Training_Engine SHALL utilize GPU acceleration for training
5. WHEN GPU acceleration is enabled, THE Training_Engine SHALL automatically detect and use available NVIDIA GPUs (such as RTX 4070 Ti)
6. THE Training_Engine SHALL complete training on the full dataset in less than 30 minutes when using GPU acceleration
7. WHEN GPU is not available, THE Training_Engine SHALL fall back to CPU training with appropriate warnings

### Requirement 10: Documentation and Reproducibility

**User Story:** As a tire model developer, I want comprehensive documentation and reproducible workflows, so that others can understand, validate, and extend the model.

#### Acceptance Criteria

1. THE system SHALL include documentation describing the model architecture, training procedure, and usage instructions
2. THE system SHALL include scripts that reproduce the complete workflow from data loading to model export
3. WHEN training the model, THE Training_Engine SHALL set random seeds for reproducibility
4. THE system SHALL document all dependencies and MATLAB toolbox requirements (Deep Learning Toolbox, Parallel Computing Toolbox for GPU support)
5. THE system SHALL include example code demonstrating how to use the trained model for prediction
6. THE system SHALL document the expected data format and preprocessing steps
