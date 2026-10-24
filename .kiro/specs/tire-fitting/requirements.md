# Requirements Document

## Introduction

このドキュメントは、FSAE車両運動シミュレータで使用するTMeasyタイヤモデルのパラメータを、FSAE Tire Test Consortium (TTC)の実験データからフィッティングする機能の要件を定義します。本システムは、DriveBrake試験データ（縦力）とCornering試験データ（横力）を読み込み、データ前処理、TMeasyモデル実装、パラメータ最適化、評価・検証、結果出力を行います。

## Glossary

- **System**: タイヤモデルフィッティングシステム
- **Data_Loader**: CSVファイルからタイヤ試験データを読み込むコンポーネント
- **Preprocessor**: データのフィルタリング、グループ化、外れ値除去を行うコンポーネント
- **TMeasy_Model**: 定常状態のTMeasyタイヤモデル方程式を実装するコンポーネント
- **Optimizer**: パラメータフィッティングを実行する最適化コンポーネント
- **Evaluator**: フィッティング結果を評価するコンポーネント
- **Exporter**: フィッティング結果をYAMLおよびC++ヘッダファイルとして出力するコンポーネント
- **DriveBrake_Data**: 縦力試験データ（スリップ比と縦力の関係）
- **Cornering_Data**: 横力試験データ（スリップ角と横力の関係）
- **TTC**: FSAE Tire Test Consortium（タイヤ試験データの提供元）

## Requirements

### Requirement 1: データ読み込み

**User Story:** As a developer, I want to load tire test data from CSV files, so that I can use experimental data for model fitting.

#### Acceptance Criteria

1. WHEN a DriveBrake CSV file path is provided, THE Data_Loader SHALL read the file and extract columns ET, V, N, SA, IA, RL, RE, P, FX, FY, FZ, MX, MZ, NFX, NFY, RST, TSTI, TSTC, TSTO, AmbTmp, SR, and SL
2. WHEN a Cornering CSV file path is provided, THE Data_Loader SHALL read the file and extract columns SA, IA, FY, FZ, NFY, MZ, and CP
3. WHEN reading FZ values from CSV files, THE Data_Loader SHALL convert negative values to positive values to match the TMeasy model convention
4. WHEN a CSV file does not exist at the specified path, THE Data_Loader SHALL return an error indicating the file was not found
5. WHEN a CSV file is missing required columns, THE Data_Loader SHALL return an error listing the missing columns

### Requirement 2: データ前処理とフィルタリング

**User Story:** As a developer, I want to filter and preprocess tire test data, so that I can remove noise and invalid measurements before fitting.

#### Acceptance Criteria

1. WHEN filtering DriveBrake data, THE Preprocessor SHALL exclude rows where vehicle speed V is less than 30 kph
2. WHEN filtering data, THE Preprocessor SHALL exclude rows where vertical load magnitude is less than 500 N
3. WHEN filtering DriveBrake data for pure longitudinal conditions, THE Preprocessor SHALL exclude rows where slip angle magnitude exceeds 0.5 degrees
4. WHEN filtering Cornering data for pure lateral conditions, THE Preprocessor SHALL exclude rows where slip ratio magnitude exceeds 0.01
5. WHEN filtering data, THE Preprocessor SHALL exclude rows where tire temperature TSTC is outside the range 40°C to 80°C

### Requirement 3: 荷重グループ化

**User Story:** As a developer, I want to group tire data by vertical load levels, so that I can fit load-dependent parameters accurately.

#### Acceptance Criteria

1. WHEN grouping data by load, THE Preprocessor SHALL classify rows with FZ between 600N and 900N as Low load group
2. WHEN grouping data by load, THE Preprocessor SHALL classify rows with FZ between 900N and 1100N as Medium load group
3. WHEN grouping data by load, THE Preprocessor SHALL classify rows with FZ between 1100N and 1400N as High load group
4. WHEN a data row has FZ outside all defined load ranges, THE Preprocessor SHALL exclude that row from load-grouped analysis

### Requirement 4: 外れ値検出と除去

**User Story:** As a developer, I want to detect and remove outliers from tire data, so that measurement errors do not corrupt the fitting results.

#### Acceptance Criteria

1. WHEN detecting outliers in a data column, THE Preprocessor SHALL calculate the first quartile Q1, third quartile Q3, and interquartile range IQR
2. WHEN detecting outliers, THE Preprocessor SHALL mark values below Q1 minus 1.5 times IQR as outliers
3. WHEN detecting outliers, THE Preprocessor SHALL mark values above Q3 plus 1.5 times IQR as outliers
4. WHEN removing outliers, THE Preprocessor SHALL exclude all rows marked as outliers from the filtered dataset

### Requirement 5: TMeasyモデル実装（縦力）

**User Story:** As a developer, I want to implement the TMeasy longitudinal force model, so that I can predict tire forces from slip ratio and vertical load.

#### Acceptance Criteria

1. WHEN computing longitudinal force, THE TMeasy_Model SHALL calculate normalized deformation as Kx times slip_ratio divided by mu_x times vertical_load
2. WHEN normalized deformation magnitude s is less than 1, THE TMeasy_Model SHALL calculate effective friction as s times (2 minus s)
3. WHEN normalized deformation magnitude s is greater than or equal to 1, THE TMeasy_Model SHALL set effective friction to 1
4. WHEN computing longitudinal force, THE TMeasy_Model SHALL calculate force as negative effective_friction times mu_x times vertical_load times normalized_deformation divided by s
5. WHEN computing mu_x with load dependency, THE TMeasy_Model SHALL calculate mu_x as mu_x_peak times (1 minus k_mu_x times (Fz minus Fz_nominal) divided by Fz_nominal)

### Requirement 6: TMeasyモデル実装（横力）

**User Story:** As a developer, I want to implement the TMeasy lateral force model, so that I can predict tire forces from slip angle and vertical load.

#### Acceptance Criteria

1. WHEN computing lateral force, THE TMeasy_Model SHALL calculate normalized deformation as Ky times tangent of slip_angle divided by mu_y times vertical_load
2. WHEN computing lateral force with camber, THE TMeasy_Model SHALL add camber thrust as C_gamma times camber_angle times vertical_load
3. WHEN computing mu_y with load dependency, THE TMeasy_Model SHALL calculate mu_y as mu_y_peak times (1 minus k_mu_y times (Fz minus Fz_nominal) divided by Fz_nominal)
4. WHEN computing lateral force, THE TMeasy_Model SHALL use the same saturation function as longitudinal force for effective friction calculation

### Requirement 7: パラメータフィッティング（縦力）

**User Story:** As a developer, I want to fit longitudinal tire parameters from DriveBrake data, so that the model accurately predicts longitudinal forces.

#### Acceptance Criteria

1. WHEN fitting longitudinal parameters, THE Optimizer SHALL optimize mu_x_peak, k_mu_x, and Kx to minimize the residual between measured and predicted longitudinal forces
2. WHEN optimizing parameters, THE Optimizer SHALL constrain mu_x_peak between 1.0 and 2.5
3. WHEN optimizing parameters, THE Optimizer SHALL constrain k_mu_x between 0 and 0.5
4. WHEN optimizing parameters, THE Optimizer SHALL constrain Kx between 5000 and 50000
5. WHEN fitting longitudinal parameters, THE Optimizer SHALL use differential evolution for global search followed by Levenberg-Marquardt for local refinement

### Requirement 8: パラメータフィッティング（横力）

**User Story:** As a developer, I want to fit lateral tire parameters from Cornering data, so that the model accurately predicts lateral forces.

#### Acceptance Criteria

1. WHEN fitting lateral parameters, THE Optimizer SHALL optimize mu_y_peak, k_mu_y, Ky, and C_gamma to minimize the residual between measured and predicted lateral forces
2. WHEN optimizing parameters, THE Optimizer SHALL constrain mu_y_peak between 1.0 and 2.5
3. WHEN optimizing parameters, THE Optimizer SHALL constrain k_mu_y between 0 and 0.5
4. WHEN optimizing parameters, THE Optimizer SHALL constrain Ky between 5000 and 50000
5. WHEN fitting lateral parameters, THE Optimizer SHALL perform fitting after longitudinal parameters are determined

### Requirement 9: 評価メトリクス計算

**User Story:** As a developer, I want to calculate evaluation metrics for fitted parameters, so that I can assess the quality of the fit.

#### Acceptance Criteria

1. WHEN evaluating fit quality, THE Evaluator SHALL calculate RMSE as the square root of the mean squared error between measured and predicted forces
2. WHEN evaluating fit quality, THE Evaluator SHALL calculate NRMSE as RMSE divided by the range of measured forces
3. WHEN evaluating fit quality, THE Evaluator SHALL calculate R-squared as 1 minus the ratio of residual sum of squares to total sum of squares
4. WHEN evaluating fit quality, THE Evaluator SHALL calculate Theil inequality coefficient as the ratio of RMSE to the sum of RMS of measured and predicted forces
5. WHEN evaluating peak friction, THE Evaluator SHALL calculate peak friction error as the absolute difference between measured and predicted peak friction divided by measured peak friction

### Requirement 10: 結果の可視化

**User Story:** As a developer, I want to visualize fitting results, so that I can graphically verify model accuracy.

#### Acceptance Criteria

1. WHEN visualizing longitudinal results, THE System SHALL generate plots of Fx versus SR for each load group showing both measured and predicted forces
2. WHEN visualizing lateral results, THE System SHALL generate plots of Fy versus SA for each load group showing both measured and predicted forces
3. WHEN visualizing load dependency, THE System SHALL generate plots of peak friction coefficient versus vertical load for both longitudinal and lateral directions
4. WHEN visualizing fit quality, THE System SHALL generate residual plots showing the difference between measured and predicted forces
5. WHEN generating plots, THE System SHALL save figures to the output directory in PNG format

### Requirement 11: YAMLパラメータファイル出力

**User Story:** As a developer, I want to export fitted parameters to a YAML file, so that I can use the parameters in other tools and simulations.

#### Acceptance Criteria

1. WHEN exporting to YAML, THE Exporter SHALL write a file containing tire_model, tire_name, effective_radius, and nominal_load fields
2. WHEN exporting to YAML, THE Exporter SHALL write longitudinal section containing mu_peak, mu_sensitivity, Kx, and relaxation_length fields
3. WHEN exporting to YAML, THE Exporter SHALL write lateral section containing mu_peak, mu_sensitivity, Ky, and relaxation_length fields
4. WHEN exporting to YAML, THE Exporter SHALL write camber section containing thrust_coefficient field
5. WHEN exporting to YAML, THE Exporter SHALL write fitting_info section containing data_source, fit_date, rmse_fx, rmse_fy, r2_fx, and r2_fy fields

### Requirement 12: C++ヘッダファイル出力

**User Story:** As a developer, I want to export fitted parameters to a C++ header file, so that I can directly use the parameters in the vehicle simulator.

#### Acceptance Criteria

1. WHEN exporting to C++ header, THE Exporter SHALL generate a header file with include guards
2. WHEN exporting to C++ header, THE Exporter SHALL define a struct containing all fitted parameters as static constexpr double members
3. WHEN exporting to C++ header, THE Exporter SHALL include comments indicating the tire name, generation date, and parameter units
4. WHEN exporting to C++ header, THE Exporter SHALL place the struct within fsae::tire namespace
5. WHEN exporting to C++ header, THE Exporter SHALL write the file with .h or .hpp extension to the output directory

### Requirement 13: 目標精度の達成

**User Story:** As a user, I want the fitted model to meet accuracy targets, so that the simulation results are reliable.

#### Acceptance Criteria

1. WHEN fitting is complete, THE System SHALL achieve longitudinal force RMSE less than 100 N
2. WHEN fitting is complete, THE System SHALL achieve lateral force RMSE less than 100 N
3. WHEN fitting is complete, THE System SHALL achieve Theil inequality coefficient less than 0.15 for both longitudinal and lateral forces
4. WHEN fitting is complete, THE System SHALL achieve peak friction coefficient error less than 5 percent for both longitudinal and lateral directions
5. IF accuracy targets are not met, THEN THE System SHALL report which metrics failed to meet targets
