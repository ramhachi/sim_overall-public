# Data Format Requirements

This document specifies the data format requirements for the MATLAB Deep Learning Tire Model system.

**Requirements: 10.6**

## Table of Contents

- [Overview](#overview)
- [CSV File Format](#csv-file-format)
- [Column Specifications](#column-specifications)
- [Data Quality Requirements](#data-quality-requirements)
- [Preprocessing Pipeline](#preprocessing-pipeline)
- [Example Data](#example-data)
- [Validation](#validation)
- [Common Issues](#common-issues)

## Overview

The MATLAB Deep Learning Tire Model requires tire test data in CSV (Comma-Separated Values) format. The system expects specific columns representing tire state variables (inputs) and tire forces/moments (outputs).

### Input Features (5 columns)

The model uses five input features that describe the tire's operating condition:

1. **SR** - Slip Ratio
2. **SA** - Slip Angle
3. **Fz** - Vertical Load
4. **IA** - Inclination Angle
5. **TSTC** - Tire Surface Temperature

### Output Targets (4 columns)

The model predicts four output values representing tire forces and moments:

1. **Fx** - Longitudinal Force
2. **Fy** - Lateral Force
3. **Mx** - Overturning Moment
4. **Mz** - Aligning Moment

## CSV File Format

### File Structure

CSV files must follow this structure:

```
Column1,Column2,Column3,...,ColumnN
value1,value2,value3,...,valueN
value1,value2,value3,...,valueN
...
```

### Requirements

- **File extension**: `.csv`
- **Delimiter**: Comma (`,`)
- **Header row**: Required (first row contains column names)
- **Encoding**: UTF-8 or ASCII
- **Line endings**: Any (Windows CRLF, Unix LF, or Mac CR)
- **Decimal separator**: Period (`.`)
- **Missing values**: Not allowed (will be removed during preprocessing)

### Multiple Files

The system supports loading multiple CSV files simultaneously. Files are automatically combined during the loading process:

```matlab
data_paths = {
    'path/to/file1.csv',
    'path/to/file2.csv',
    'path/to/file3.csv'
};
data = loadTireData(data_paths);
```

## Column Specifications

### Required Columns

All nine columns must be present in the CSV file:

| Column | Name | Type | Unit | Description |
|--------|------|------|------|-------------|
| 1 | SR | Float | - | Slip Ratio (dimensionless) |
| 2 | SA | Float | deg | Slip Angle (degrees) |
| 3 | Fz | Float | N | Vertical Load (Newtons) |
| 4 | IA | Float | deg | Inclination Angle / Camber (degrees) |
| 5 | TSTC | Float | °C | Tire Surface Temperature (Celsius) |
| 6 | Fx | Float | N | Longitudinal Force (Newtons) |
| 7 | Fy | Float | N | Lateral Force (Newtons) |
| 8 | Mx | Float | Nm | Overturning Moment (Newton-meters) |
| 9 | Mz | Float | Nm | Aligning Moment (Newton-meters) |

**Note**: Column order does not matter. The system identifies columns by name, not position.

### Column Name Variations

The system accepts these column names (case-insensitive):

- **SR**: `SR`, `SlipRatio`, `Slip_Ratio`
- **SA**: `SA`, `SlipAngle`, `Slip_Angle`
- **Fz**: `Fz`, `FZ`, `VerticalLoad`, `Vertical_Load`, `NormalForce`
- **IA**: `IA`, `InclinationAngle`, `Inclination_Angle`, `Camber`, `CamberAngle`
- **TSTC**: `TSTC`, `TireTemp`, `Tire_Temperature`, `SurfaceTemp`
- **Fx**: `Fx`, `FX`, `LongitudinalForce`, `Longitudinal_Force`
- **Fy**: `Fy`, `FY`, `LateralForce`, `Lateral_Force`
- **Mx**: `Mx`, `MX`, `OverturningMoment`, `Overturning_Moment`
- **Mz**: `Mz`, `MZ`, `AligningMoment`, `Aligning_Moment`, `SelfAligningMoment`

### Data Types and Ranges

#### Input Features

| Feature | Type | Typical Range | Physical Limits | Description |
|---------|------|---------------|-----------------|-------------|
| SR | Float | [-0.2, 0.2] | [-1.0, 1.0] | Negative = acceleration, Positive = braking |
| SA | Float | [-15, 15] | [-90, 90] | Lateral slip angle |
| Fz | Float | [500, 2500] | [0, 5000] | Normal force on tire |
| IA | Float | [-5, 5] | [-45, 45] | Camber angle (negative = top tilted inward) |
| TSTC | Float | [20, 80] | [-20, 150] | Tire surface temperature |

#### Output Targets

| Target | Type | Typical Range | Physical Limits | Description |
|--------|------|---------------|-----------------|-------------|
| Fx | Float | [-2000, 2000] | [-5000, 5000] | Longitudinal force (x-direction) |
| Fy | Float | [-2000, 2000] | [-5000, 5000] | Lateral force (y-direction) |
| Mx | Float | [-100, 100] | [-500, 500] | Overturning moment (about x-axis) |
| Mz | Float | [-50, 50] | [-200, 200] | Aligning moment (about z-axis) |

**Note**: Values outside typical ranges are allowed but may indicate measurement errors or unusual operating conditions.

## Data Quality Requirements

### Minimum Requirements

- **Sample count**: Minimum 1,000 samples recommended
  - For production models: 10,000+ samples recommended
  - For research models: 100,000+ samples ideal
- **Coverage**: Data should cover the full operating range of all input features
- **Balance**: Avoid over-representation of specific operating conditions
- **Consistency**: Units must be consistent across all samples

### Missing Values

The system automatically handles missing values:

- **Detection**: NaN (Not a Number) and Inf (Infinity) values are detected
- **Action**: Rows containing missing values are removed
- **Reporting**: Statistics on removed samples are logged

```matlab
% Example output
Warning: Removed 150 samples (1.2%) with missing or infinite values
```

### Data Validation

Use the validation function to check data quality:

```matlab
data = loadTireData(data_paths);
stats = validateTireData(data);
```

The validation function reports:

- Total number of samples
- Number of clean samples (no missing values)
- Number of rows with missing values
- Missing value count per column
- Data ranges (min, max, mean, std) for each column

### Outlier Detection

While the system does not automatically remove outliers, you should manually inspect data for:

- **Physically impossible values**: e.g., negative Fz, extreme temperatures
- **Measurement errors**: e.g., sudden spikes, sensor failures
- **Duplicate rows**: Identical samples that may skew training

## Preprocessing Pipeline

The preprocessing pipeline automatically transforms raw data for model training:

### Step 1: Data Loading

```matlab
data = loadTireData(data_paths);
```

- Reads CSV files
- Combines multiple files into single table
- Validates column presence

### Step 2: Feature and Target Extraction

```matlab
features = extractFeatures(data);  % Extracts SR, SA, Fz, IA, TSTC
targets = extractTargets(data);    % Extracts Fx, Fy, Mx, Mz
```

- Separates inputs from outputs
- Creates N×5 feature matrix
- Creates N×4 target matrix

### Step 3: Missing Data Removal

```matlab
[X_clean, Y_clean, stats] = removeMissingData(X, Y);
```

- Detects NaN and Inf values
- Removes affected rows
- Logs removal statistics

### Step 4: Normalization

```matlab
[X_norm, mu_X, sigma_X] = normalizeData(X_clean);
[Y_norm, mu_Y, sigma_Y] = normalizeData(Y_clean);
```

- Applies z-score normalization: `X_norm = (X - mean(X)) / std(X)`
- Produces zero mean and unit variance
- Stores normalization parameters for inference

### Step 5: Data Splitting

```matlab
[X_train, Y_train, X_val, Y_val] = splitData(X_norm, Y_norm, 0.8);
```

- Splits data into training (80%) and validation (20%) sets
- Random shuffling before split
- Maintains input-output correspondence

### Complete Pipeline

```matlab
[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config);
```

This single function performs all preprocessing steps and returns:

- `X_train`: Training features (normalized)
- `Y_train`: Training targets (normalized)
- `X_val`: Validation features (normalized)
- `Y_val`: Validation targets (normalized)
- `norm_params`: Structure containing normalization parameters

## Example Data

### Minimal Example

```csv
SR,SA,Fz,IA,TSTC,Fx,Fy,Mx,Mz
0.0,0.0,1000.0,0.0,25.0,0.0,0.0,0.0,0.0
0.05,2.0,1200.0,-1.0,28.0,450.2,180.5,12.3,5.8
0.1,5.0,1500.0,-2.0,30.0,820.5,520.8,25.6,15.2
-0.05,-3.0,1800.0,1.0,32.0,-380.1,-280.3,-15.4,-8.9
```

### Comprehensive Example

```csv
SR,SA,Fz,IA,TSTC,Fx,Fy,Mx,Mz
0.0,0.0,1000.0,0.0,25.0,0.0,0.0,0.0,0.0
0.01,0.0,1000.0,0.0,25.0,85.3,0.0,0.0,0.0
0.02,0.0,1000.0,0.0,25.0,168.2,0.0,0.0,0.0
0.05,0.0,1000.0,0.0,25.0,402.5,0.0,0.0,0.0
0.1,0.0,1000.0,0.0,25.0,745.8,0.0,0.0,0.0
0.0,1.0,1000.0,0.0,25.0,0.0,95.2,5.1,2.3
0.0,2.0,1000.0,0.0,25.0,0.0,188.4,10.2,4.5
0.0,5.0,1000.0,0.0,25.0,0.0,452.3,24.8,10.8
0.0,10.0,1000.0,0.0,25.0,0.0,812.5,45.2,18.5
0.05,2.0,1200.0,-1.0,28.0,450.2,180.5,12.3,5.8
0.1,5.0,1500.0,-2.0,30.0,820.5,520.8,25.6,15.2
-0.05,-3.0,1800.0,1.0,32.0,-380.1,-280.3,-15.4,-8.9
```

### TTC Data Format

The system is designed for TTC (Tire Test Consortium) data format:

**DriveBrake Data** (longitudinal slip):
- Focuses on SR variation
- SA typically near zero
- Measures Fx primarily

**Cornering Data** (lateral slip):
- Focuses on SA variation
- SR typically near zero
- Measures Fy primarily

Both datasets include all nine required columns.

## Validation

### Automatic Validation

The `loadTireData` function automatically validates:

1. **File existence**: Checks if files exist before loading
2. **File readability**: Verifies CSV format is valid
3. **Column presence**: Ensures all required columns are present
4. **Data types**: Confirms columns contain numeric data

### Manual Validation

Use `validateTireData` for detailed quality checks:

```matlab
data = loadTireData(data_paths);
stats = validateTireData(data);

% Check if data quality is acceptable
if stats.clean_samples / stats.total_samples < 0.95
    warning('More than 5%% of data has missing values');
end

% Check data coverage
fprintf('SR range: [%.3f, %.3f]\n', stats.ranges.SR.min, stats.ranges.SR.max);
fprintf('SA range: [%.1f, %.1f]\n', stats.ranges.SA.min, stats.ranges.SA.max);
fprintf('Fz range: [%.0f, %.0f]\n', stats.ranges.Fz.min, stats.ranges.Fz.max);
```

### Validation Checklist

Before training a model, verify:

- [ ] All required columns are present
- [ ] Data types are numeric (float/double)
- [ ] No more than 5% missing values
- [ ] Input ranges cover expected operating conditions
- [ ] Output values are physically reasonable
- [ ] Sufficient sample count (>1,000 minimum)
- [ ] Data is balanced across operating conditions

## Common Issues

### Issue 1: Missing Columns

**Error**: `Missing required columns: SA, Fz`

**Cause**: CSV file does not contain all required columns

**Solution**:
- Check column names in CSV file
- Ensure column names match expected names (case-insensitive)
- Add missing columns to CSV file

### Issue 2: File Not Found

**Error**: `Data file not found: path/to/file.csv`

**Cause**: File path is incorrect or file does not exist

**Solution**:
- Verify file path is correct
- Use absolute paths if relative paths fail
- Check file extension is `.csv`

### Issue 3: Invalid Data Format

**Error**: `Failed to read CSV file: Invalid format`

**Cause**: CSV file is corrupted or not in valid CSV format

**Solution**:
- Open file in text editor to check format
- Verify delimiter is comma
- Check for special characters or encoding issues
- Re-export data from source

### Issue 4: Too Many Missing Values

**Warning**: `Removed 5000 samples (50%) with missing or infinite values`

**Cause**: Data contains many NaN or Inf values

**Solution**:
- Investigate source of missing values
- Check measurement equipment
- Consider data imputation if appropriate
- Collect more data if necessary

### Issue 5: Insufficient Data

**Warning**: `Only 500 samples available. Minimum 1000 recommended.`

**Cause**: Dataset is too small for effective training

**Solution**:
- Collect more data
- Combine multiple datasets
- Use data augmentation techniques (with caution)
- Reduce model complexity

### Issue 6: Unbalanced Data

**Issue**: Model performs poorly in certain operating regions

**Cause**: Data is concentrated in specific operating conditions

**Solution**:
- Visualize data distribution across input ranges
- Collect additional data in underrepresented regions
- Use stratified sampling during train/validation split
- Consider weighted loss functions

### Issue 7: Unit Inconsistencies

**Issue**: Model predictions are orders of magnitude wrong

**Cause**: Input or output units are inconsistent

**Solution**:
- Verify all forces are in Newtons (N)
- Verify all moments are in Newton-meters (Nm)
- Verify angles are in degrees (not radians)
- Verify temperature is in Celsius (not Fahrenheit or Kelvin)
- Check slip ratio is dimensionless (not percentage)

## Best Practices

### Data Collection

1. **Cover full operating range**: Ensure data spans all expected input values
2. **Include edge cases**: Test at boundaries of operating range
3. **Maintain consistency**: Use same measurement equipment and procedures
4. **Document metadata**: Record test conditions, tire specifications, etc.
5. **Quality control**: Regularly calibrate sensors and validate measurements

### Data Preparation

1. **Clean data first**: Remove obvious errors before training
2. **Visualize distributions**: Plot histograms of all features and targets
3. **Check correlations**: Verify expected relationships (e.g., Fx vs SR)
4. **Normalize consistently**: Use same normalization for training and inference
5. **Split randomly**: Shuffle data before train/validation split

### Data Storage

1. **Use version control**: Track changes to datasets
2. **Document preprocessing**: Record all transformations applied
3. **Backup original data**: Keep raw data separate from processed data
4. **Use descriptive names**: Include date, tire type, test conditions in filename
5. **Compress large files**: Use compression for storage efficiency

## References

- Main README: `README.md`
- Design Document: `../../.kiro/specs/matlab-dl-tire-model/design.md`
- Requirements: `../../.kiro/specs/matlab-dl-tire-model/requirements.md`
- Example Scripts: `example_train_model.m`, `example_use_trained_model.m`
