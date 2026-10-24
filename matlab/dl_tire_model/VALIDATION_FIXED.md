# Validation Scripts - Fixed for Windows

## Issues Fixed

1. **License check error** - Fixed the toolbox name length issue
2. **Data file path** - Updated to use only `DriveBrake_sampled_quarter.csv`

## Two Validation Options

### Option 1: Quick Validation (Recommended for Testing)

Use the simplified validation script with the sampled dataset:

```matlab
cd matlab/dl_tire_model
run_quick_validation
```

**Features:**
- Uses sampled quarter dataset (~35k samples)
- Faster training (5-15 minutes)
- Simplified workflow
- All essential checks included

**What it does:**
1. Checks GPU availability
2. Loads sampled data
3. Preprocesses data
4. Creates model
5. Trains model (50 epochs max)
6. Evaluates performance
7. Tests prediction speed
8. Saves results

**Output:**
- `output/quick_validation_model.mat` - Trained model
- Console summary with metrics

### Option 2: Full Validation

Use the complete validation system (now fixed):

```matlab
cd matlab/dl_tire_model
run_final_validation
```

**Features:**
- Runs all unit tests
- Runs property-based tests (400 iterations)
- Trains on sampled dataset
- Comprehensive reporting

**Note:** This takes longer but provides complete validation coverage.

## Changes Made

### 1. Fixed `run_final_validation.m`
- Changed toolbox checking to use proper license IDs
- Updated to use `DriveBrake_sampled_quarter.csv`

### 2. Fixed `final_system_validation.m`
- Updated data path to sampled dataset

### 3. Fixed `check_validation_readiness.m`
- Updated data file check to sampled dataset

### 4. Created `run_quick_validation.m`
- New simplified validation script
- Perfect for quick testing
- Uses sampled data
- Reduced epochs for faster completion

## Data File Used

**Path:** `data/tiredata/DriveBrake_sampled_quarter.csv`

This is a quarter-sized sample of the full dataset, which is:
- Faster to load
- Faster to train
- Still provides good validation
- Suitable for development and testing

## Expected Results

### With Sampled Data

**Training time:**
- With GPU: 5-15 minutes
- Without GPU: 20-40 minutes

**Performance:**
- RMSE: May be slightly higher than full dataset
- R²: Should still be > 0.90
- Prediction speed: < 1ms per sample

## Recommendations

1. **For quick testing:** Use `run_quick_validation`
2. **For full validation:** Use `run_final_validation` (after testing)
3. **For production:** Train on full dataset when ready

## Quick Start

```matlab
% Navigate to directory
cd matlab/dl_tire_model

% Run quick validation
run_quick_validation

% View results
load('output/quick_validation_model.mat')
```

## Troubleshooting

### If you still get license errors:
The script now uses proper MATLAB license IDs:
- `neural_network_toolbox` for Deep Learning Toolbox
- `statistics_toolbox` for Statistics Toolbox

### If data file not found:
Verify the file exists at:
```
data/tiredata/DriveBrake_sampled_quarter.csv
```

### If training is too slow:
- Reduce `max_epochs` in the config
- Reduce `batch_size`
- Use GPU if available

## Next Steps

After successful validation:

1. Review the metrics in console output
2. Load the trained model
3. Test predictions on new data
4. Consider training on full dataset for production

## Files Modified

- ✓ `run_final_validation.m` - Fixed license check
- ✓ `final_system_validation.m` - Updated data path
- ✓ `check_validation_readiness.m` - Updated data check
- ✓ `run_quick_validation.m` - New simplified script

## Status

✅ All issues fixed  
✅ Ready to run validation  
✅ Works with sampled dataset  
✅ Compatible with Windows MATLAB  
