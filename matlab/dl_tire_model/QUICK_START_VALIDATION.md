# Quick Start: Final System Validation

## Run the Validation in 3 Steps

### Step 1: Open MATLAB
Navigate to the project directory:
```matlab
cd matlab/dl_tire_model
```

### Step 2: Run the Validation
Execute the validation script:
```matlab
run_final_validation
```

### Step 3: Review Results
Check the output:
- Console: Real-time progress and summary
- Report: `output/final_validation_report.txt`
- Results: `output/final_validation_results.mat`

## What Gets Validated

✓ **Unit Tests** - All component tests  
✓ **Property Tests** - Mathematical correctness (100 iterations each)  
✓ **Full Training** - Complete TTC dataset (~140k samples)  
✓ **Performance** - RMSE, R², training time, prediction speed  

## Expected Duration

- **With GPU**: 15-30 minutes
- **Without GPU**: 1-2 hours

## Success Indicators

Look for these in the final output:

```
=================================================================
OVERALL STATUS: PASS
=================================================================
```

## Requirements Checked

1. RMSE < 100N for forces (Fx, Fy)
2. R² > 0.95 for all outputs
3. Training time < 30 minutes (GPU)
4. Prediction time < 1ms per sample

## Troubleshooting

**GPU not detected?**
- Training will use CPU (slower but works)
- Install Parallel Computing Toolbox for GPU support

**Missing toolbox error?**
- Install Deep Learning Toolbox
- Install Statistics and Machine Learning Toolbox

**Data file not found?**
- Verify files exist in `data/tiredata/`
- Check file names match exactly

## Output Files

After completion, you'll find:

```
output/
├── final_validation_report.txt      # Human-readable report
├── final_validation_results.mat     # MATLAB results structure
└── final_trained_model.mat          # Trained model
```

## Next Steps

If validation passes:
1. Review metrics in the report
2. Use the trained model for predictions
3. Export model for C++ integration

If validation fails:
1. Check error messages in console
2. Review failed tests in report
3. Verify data quality and preprocessing
4. Consider hyperparameter tuning

## Quick Commands

```matlab
% Run validation
run_final_validation

% Load results
load('output/final_validation_results.mat')

% View report
type('output/final_validation_report.txt')

% Load trained model
load('output/final_trained_model.mat')
```

## Need Help?

See `TASK_15_FINAL_VALIDATION.md` for detailed documentation.
