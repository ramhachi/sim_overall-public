% WORKFLOW QUICK REFERENCE - MATLAB Deep Learning Tire Model
%
% This file provides a quick reference for using the complete workflow scripts.
% It includes examples and common use cases.
%
% ========================================================================
% AVAILABLE WORKFLOW SCRIPTS
% ========================================================================
%
% 1. main_training_workflow.m
%    - Complete training pipeline from data to exported model
%    - Use this for: Training a new model with default or custom settings
%
% 2. main_prediction_workflow.m
%    - Load trained model and make predictions on new data
%    - Use this for: Using a trained model for inference
%
% 3. main_hyperparameter_tuning.m
%    - Automated hyperparameter search (grid or random)
%    - Use this for: Finding optimal model configuration
%
% ========================================================================
% QUICK START GUIDE
% ========================================================================
%
% STEP 1: Train your first model
% -------------------------------
% >> cd matlab/dl_tire_model
% >> main_training_workflow
%
% This will:
%   - Load TTC tire data
%   - Train a model with default settings
%   - Save model to output/tire_model_latest.mat
%   - Generate visualizations in output/visualizations/
%
% STEP 2: Make predictions with trained model
% --------------------------------------------
% >> main_prediction_workflow
%
% This will:
%   - Load the trained model
%   - Make predictions on sample inputs
%   - Save results to output/predictions/
%
% STEP 3: Optimize model performance (optional)
% ----------------------------------------------
% >> main_hyperparameter_tuning
%
% This will:
%   - Try different hyperparameter combinations
%   - Save best model to experiments/best_model_*.mat
%   - Generate comparison report
%
% ========================================================================
% CUSTOMIZATION EXAMPLES
% ========================================================================
%
% Example 1: Train with custom learning rate
% -------------------------------------------
% Edit main_training_workflow.m, line ~40:
%   config.learning_rate = 0.0001;  % Reduce learning rate
%
% Example 2: Use different data files
% ------------------------------------
% Edit main_training_workflow.m, line ~25:
%   data_paths = {
%       'path/to/your/data1.csv'
%       'path/to/your/data2.csv'
%   };
%
% Example 3: Train for more epochs
% ---------------------------------
% Edit main_training_workflow.m, line ~42:
%   config.max_epochs = 300;  % Train longer
%
% Example 4: Predict on custom inputs
% ------------------------------------
% Edit main_prediction_workflow.m, line ~60:
%   X_raw = [
%       0.05, 3.0, 1500, -1.0, 30.0;  % Your custom input
%       0.10, 5.0, 1800, -2.0, 35.0;  % Another input
%   ];
%
% Example 5: Predict from CSV file
% ---------------------------------
% Edit main_prediction_workflow.m, uncomment lines ~80-88:
%   input_file = 'my_inputs.csv';
%   input_table = readtable(input_file);
%   X_raw = [input_table.SR, input_table.SA, ...];
%
% Example 6: Custom hyperparameter search
% ----------------------------------------
% Edit main_hyperparameter_tuning.m, line ~50:
%   search_space.learning_rate = {0.01, 0.001, 0.0001};
%   search_space.batch_size = {64, 128, 256, 512};
%
% ========================================================================
% COMMON WORKFLOWS
% ========================================================================
%
% Workflow A: Quick model training and testing
% ---------------------------------------------
% 1. >> main_training_workflow
% 2. >> main_prediction_workflow
% 3. Review results in output/ directory
%
% Workflow B: Optimize then deploy
% ---------------------------------
% 1. >> main_hyperparameter_tuning
% 2. Copy best model to output/tire_model_latest.mat
% 3. >> main_prediction_workflow
% 4. Use model in production
%
% Workflow C: Iterative development
% ----------------------------------
% 1. >> main_training_workflow  (baseline model)
% 2. Review metrics and visualizations
% 3. Adjust configuration in script
% 4. >> main_training_workflow  (improved model)
% 5. Compare results
% 6. Repeat until satisfied
%
% Workflow D: Batch prediction on large dataset
% ----------------------------------------------
% 1. Prepare input CSV with columns: SR, SA, Fz, IA, TSTC
% 2. Edit main_prediction_workflow.m to load from CSV
% 3. >> main_prediction_workflow
% 4. Results saved to output/predictions/
%
% ========================================================================
% TROUBLESHOOTING
% ========================================================================
%
% Problem: "Model file not found"
% Solution: Run main_training_workflow first to create a model
%
% Problem: "Data file not found"
% Solution: Check data_paths in script points to correct CSV files
%
% Problem: "GPU out of memory"
% Solution: Training will automatically reduce batch size or use CPU
%
% Problem: "Model doesn't meet requirements"
% Solution: Run main_hyperparameter_tuning to find better configuration
%
% Problem: Training is very slow
% Solution: 
%   - Check if GPU is being used (should see GPU info during training)
%   - Reduce max_epochs or use smaller dataset for testing
%   - Reduce batch_size if GPU memory is limited
%
% Problem: Predictions seem incorrect
% Solution:
%   - Check input units match training data (SR: ratio, SA: degrees, etc.)
%   - Verify model was trained successfully (check metrics)
%   - Ensure inputs are within training data range
%
% ========================================================================
% OUTPUT FILES REFERENCE
% ========================================================================
%
% After main_training_workflow:
%   output/tire_model_YYYYMMDD_HHMMSS.mat  - Trained model (timestamped)
%   output/tire_model_latest.mat           - Trained model (latest)
%   output/training_summary_*.txt          - Training summary report
%   output/visualizations/*.png            - Performance plots
%
% After main_prediction_workflow:
%   output/predictions/predictions_*.csv   - Prediction results (CSV)
%   output/predictions/predictions_*.mat   - Prediction results (MAT)
%   output/predictions/predictions_plot_*.png - Visualization
%
% After main_hyperparameter_tuning:
%   experiments/best_model_*.mat           - Best model found
%   experiments/hyperparameter_comparison.txt - Comparison report
%   experiments/tuning_summary_*.txt       - Tuning summary
%   experiments/logs/exp_*.mat             - Individual experiment logs
%   experiments/models/exp_*_model.mat     - Individual experiment models
%   experiments/visualizations_best_*/*.png - Best model visualizations
%
% ========================================================================
% INTEGRATION WITH SIMULATORS
% ========================================================================
%
% To use trained model in vehicle simulator:
%
% 1. Load model:
%    model = load('output/tire_model_latest.mat');
%
% 2. In simulation loop, call prediction function:
%    X_raw = [SR, SA, Fz, IA, TSTC];  % Current tire state
%    Y = predictTireForces(model.trained_net, X_raw, model.norm_params);
%    Fx = Y(1); Fy = Y(2); Mx = Y(3); Mz = Y(4);
%
% 3. Use forces in vehicle dynamics equations
%
% For C++ integration:
%    - Use exportModelWithValidation.m to create lookup table
%    - Implement interpolation in C++ code
%    - See MATLAB Coder documentation for code generation
%
% ========================================================================
% PERFORMANCE BENCHMARKS
% ========================================================================
%
% Typical performance on RTX 4070 Ti GPU:
%   - Training time: 10-20 minutes (200 epochs, 140k samples)
%   - Prediction time: < 1 ms per sample
%   - Model size: ~5-10 MB
%   - Expected RMSE: < 50 N for forces, < 5 Nm for moments
%   - Expected R²: > 0.98 for all outputs
%
% ========================================================================
% ADDITIONAL RESOURCES
% ========================================================================
%
% Documentation:
%   - README.md - Project overview and setup instructions
%   - design.md - Detailed design document
%   - requirements.md - Requirements specification
%
% Example scripts:
%   - example_hyperparameter_tuning.m - Hyperparameter tuning examples
%   - test_complete_system.m - System integration test
%
% Helper functions:
%   - All functions in matlab/dl_tire_model/ directory
%   - See function headers for detailed documentation
%
% ========================================================================
% CONTACT AND SUPPORT
% ========================================================================
%
% For questions or issues:
%   1. Check this quick reference
%   2. Review function documentation (help function_name)
%   3. Check README.md and design documents
%   4. Review example scripts
%
% ========================================================================

% This is a documentation file - no executable code
fprintf('MATLAB Deep Learning Tire Model - Workflow Quick Reference\n');
fprintf('See comments in this file for usage instructions.\n\n');
fprintf('Available workflow scripts:\n');
fprintf('  1. main_training_workflow.m - Train new model\n');
fprintf('  2. main_prediction_workflow.m - Make predictions\n');
fprintf('  3. main_hyperparameter_tuning.m - Optimize hyperparameters\n\n');
fprintf('For detailed instructions, open this file in MATLAB editor.\n');
