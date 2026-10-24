# Design Document: Tire Model Fitting

## Overview

本設計は、FSAE Tire Test Consortium (TTC)の実験データからTMeasyタイヤモデルのパラメータをフィッティングするシステムを定義します。システムは、DriveBrake試験データ（縦力）とCornering試験データ（横力）を読み込み、データ前処理、TMeasyモデル実装、2段階最適化（縦力→横力）、評価・検証、結果出力（YAML、C++ヘッダ）を実行します。

TMeasyモデルは、Pacejka Magic Formulaと比較して低速特異点がなく、パラメータ数が少なく（~15個）、物理的解釈が可能な半物理モデルです。本システムは定常状態のTMeasyモデルを使用し、荷重依存性を考慮したパラメータフィッティングを行います。

## Architecture

システムは以下のコンポーネントで構成されます：

```
┌─────────────────────────────────────────────────────────────┐
│                    Tire Fitting System                       │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐      ┌──────────────┐                     │
│  │ Data Loader  │─────>│ Preprocessor │                     │
│  └──────────────┘      └──────┬───────┘                     │
│         │                      │                              │
│         │                      v                              │
│         │              ┌──────────────┐                      │
│         │              │ Load Grouper │                      │
│         │              └──────┬───────┘                      │
│         │                      │                              │
│         v                      v                              │
│  ┌──────────────────────────────────┐                       │
│  │      TMeasy Model Engine         │                       │
│  │  - Longitudinal Force Model      │                       │
│  │  - Lateral Force Model           │                       │
│  │  - Load Dependency Model         │                       │
│  └──────────┬───────────────────────┘                       │
│             │                                                 │
│             v                                                 │
│  ┌──────────────────────────────────┐                       │
│  │         Optimizer                │                       │
│  │  Phase 1: Longitudinal Fitting   │                       │
│  │  Phase 2: Lateral Fitting        │                       │
│  └──────────┬───────────────────────┘                       │
│             │                                                 │
│             v                                                 │
│  ┌──────────────────────────────────┐                       │
│  │         Evaluator                │                       │
│  │  - Metrics Calculation           │                       │
│  │  - Visualization                 │                       │
│  └──────────┬───────────────────────┘                       │
│             │                                                 │
│             v                                                 │
│  ┌──────────────────────────────────┐                       │
│  │         Exporter                 │                       │
│  │  - YAML Output                   │                       │
│  │  - C++ Header Output             │                       │
│  └──────────────────────────────────┘                       │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

**データフロー**:
1. Data Loader: CSVファイルを読み込み、DataFrameに変換
2. Preprocessor: フィルタリング、FZ符号変換、外れ値除去
3. Load Grouper: 荷重レベルでデータをグループ化
4. TMeasy Model: 定常状態のタイヤ力を計算
5. Optimizer: 差分進化法 + Levenberg-Marquardt法でパラメータ最適化
6. Evaluator: RMSE、R²、TIC等のメトリクスを計算し、プロット生成
7. Exporter: YAMLとC++ヘッダファイルを出力

## Components and Interfaces

### 1. Data Loader

**責務**: CSVファイルからタイヤ試験データを読み込む

**インターフェース**:
```python
class TireDataLoader:
    def load_drivebrake_data(self, filepath: str) -> pd.DataFrame:
        """
        DriveBrake試験データを読み込む
        
        Args:
            filepath: CSVファイルパス
            
        Returns:
            DataFrame with columns: ET, V, N, SA, IA, RL, RE, P, FX, FY, FZ, 
            MX, MZ, NFX, NFY, RST, TSTI, TSTC, TSTO, AmbTmp, SR, SL
            
        Raises:
            FileNotFoundError: ファイルが存在しない
            ValueError: 必須カラムが欠落している
        """
        
    def load_cornering_data(self, filepath: str) -> pd.DataFrame:
        """
        Cornering試験データを読み込む
        
        Args:
            filepath: CSVファイルパス
            
        Returns:
            DataFrame with columns: SA, IA, FY, FZ, NFY, MZ, CP
            
        Raises:
            FileNotFoundError: ファイルが存在しない
            ValueError: 必須カラムが欠落している
        """
```

**実装詳細**:
- pandas.read_csv()を使用
- FZ列の符号を反転（データは負値、モデルは正値）
- 必須カラムの存在チェック

### 2. Preprocessor

**責務**: データのフィルタリング、外れ値除去、荷重グループ化

**インターフェース**:
```python
class TireDataPreprocessor:
    def filter_data(self, df: pd.DataFrame, 
                   min_speed: float = 30.0,
                   min_fz: float = 500.0,
                   max_sa_for_longitudinal: float = 0.5,
                   max_sr_for_lateral: float = 0.01,
                   temp_range: tuple = (40.0, 80.0)) -> pd.DataFrame:
        """
        データをフィルタリング
        
        Args:
            df: 入力DataFrame
            min_speed: 最小車速 [kph]
            min_fz: 最小垂直荷重 [N]
            max_sa_for_longitudinal: 縦力データの最大スリップ角 [deg]
            max_sr_for_lateral: 横力データの最大スリップ比 [-]
            temp_range: タイヤ温度範囲 [°C]
            
        Returns:
            フィルタリング済みDataFrame
        """
        
    def remove_outliers_iqr(self, df: pd.DataFrame, 
                           column: str, 
                           multiplier: float = 1.5) -> pd.DataFrame:
        """
        IQR法で外れ値を除去
        
        Args:
            df: 入力DataFrame
            column: 対象カラム名
            multiplier: IQR乗数（通常1.5）
            
        Returns:
            外れ値除去後のDataFrame
        """
        
    def group_by_load(self, df: pd.DataFrame) -> dict:
        """
        垂直荷重でデータをグループ化
        
        Args:
            df: 入力DataFrame
            
        Returns:
            {'Low': DataFrame, 'Medium': DataFrame, 'High': DataFrame}
            Low: 600-900N, Medium: 900-1100N, High: 1100-1400N
        """
```

**実装詳細**:
- フィルタリング: boolean indexingで条件適用
- IQR法: Q1 - 1.5*IQR < x < Q3 + 1.5*IQR
- 荷重グループ化: FZ値で3グループに分類

### 3. TMeasy Model Engine

**責務**: 定常状態のTMeasyモデルで縦力・横力を計算

**インターフェース**:
```python
class TMeasyModel:
    def __init__(self, fz_nominal: float = 1000.0):
        """
        Args:
            fz_nominal: 基準垂直荷重 [N]
        """
        
    def compute_longitudinal_force(self, 
                                   slip_ratio: np.ndarray,
                                   fz: np.ndarray,
                                   mu_x_peak: float,
                                   k_mu_x: float,
                                   Kx: float) -> np.ndarray:
        """
        縦力を計算（定常状態）
        
        Args:
            slip_ratio: スリップ比 [-]
            fz: 垂直荷重 [N]
            mu_x_peak: ピーク縦摩擦係数 [-]
            k_mu_x: 荷重感度 [-]
            Kx: 複合剛性パラメータ Cx*Lx [N]
            
        Returns:
            縦力 Fx [N]
        """
        
    def compute_lateral_force(self,
                             slip_angle: np.ndarray,
                             fz: np.ndarray,
                             camber_angle: np.ndarray,
                             mu_y_peak: float,
                             k_mu_y: float,
                             Ky: float,
                             C_gamma: float = 0.0) -> np.ndarray:
        """
        横力を計算（定常状態）
        
        Args:
            slip_angle: スリップ角 [rad]
            fz: 垂直荷重 [N]
            camber_angle: キャンバー角 [rad]
            mu_y_peak: ピーク横摩擦係数 [-]
            k_mu_y: 荷重感度 [-]
            Ky: 複合剛性パラメータ Cy*Ly [N]
            C_gamma: キャンバースラスト係数 [1/rad]
            
        Returns:
            横力 Fy [N]
        """
```

**モデル方程式**:

縦力計算:
```
mu_x(Fz) = mu_x_peak * (1 - k_mu_x * (Fz - Fz_nom) / Fz_nom)
xi_bar = Kx * SR / (mu_x * Fz)
s = |xi_bar|
mu_eff = s * (2 - s)  if s < 1 else 1
Fx = -mu_eff * mu_x * Fz * sign(xi_bar)
```

横力計算:
```
mu_y(Fz) = mu_y_peak * (1 - k_mu_y * (Fz - Fz_nom) / Fz_nom)
eta_bar = Ky * tan(SA) / (mu_y * Fz)
s = |eta_bar|
mu_eff = s * (2 - s)  if s < 1 else 1
Fy = -mu_eff * mu_y * Fz * sign(eta_bar) + C_gamma * camber * Fz
```

### 4. Optimizer

**責務**: 2段階最適化でパラメータをフィッティング

**インターフェース**:
```python
class TireParameterOptimizer:
    def __init__(self, model: TMeasyModel):
        """
        Args:
            model: TMeasyModelインスタンス
        """
        
    def fit_longitudinal(self, 
                        sr_data: np.ndarray,
                        fx_data: np.ndarray,
                        fz_data: np.ndarray,
                        bounds: dict = None) -> dict:
        """
        縦力パラメータをフィッティング
        
        Args:
            sr_data: スリップ比データ [-]
            fx_data: 縦力データ [N]
            fz_data: 垂直荷重データ [N]
            bounds: パラメータ境界 {'mu_x_peak': (1.0, 2.5), ...}
            
        Returns:
            {'mu_x_peak': float, 'k_mu_x': float, 'Kx': float,
             'rmse': float, 'r2': float}
        """
        
    def fit_lateral(self,
                   sa_data: np.ndarray,
                   fy_data: np.ndarray,
                   fz_data: np.ndarray,
                   camber_data: np.ndarray,
                   bounds: dict = None) -> dict:
        """
        横力パラメータをフィッティング
        
        Args:
            sa_data: スリップ角データ [rad]
            fy_data: 横力データ [N]
            fz_data: 垂直荷重データ [N]
            camber_data: キャンバー角データ [rad]
            bounds: パラメータ境界
            
        Returns:
            {'mu_y_peak': float, 'k_mu_y': float, 'Ky': float, 
             'C_gamma': float, 'rmse': float, 'r2': float}
        """
```

**最適化戦略**:
1. 差分進化法（scipy.optimize.differential_evolution）でグローバル探索
2. Levenberg-Marquardt法（scipy.optimize.least_squares）で局所最適化
3. 目的関数: 正規化残差二乗和 `sum((F_data - F_model) / Fz)^2`

**パラメータ境界**:
- mu_x_peak, mu_y_peak: [1.0, 2.5]
- k_mu_x, k_mu_y: [0.0, 0.5]
- Kx, Ky: [5000, 50000]
- C_gamma: [0.0, 0.1]

### 5. Evaluator

**責務**: フィッティング結果の評価とプロット生成

**インターフェース**:
```python
class FitEvaluator:
    def calculate_metrics(self, 
                         y_true: np.ndarray, 
                         y_pred: np.ndarray) -> dict:
        """
        評価メトリクスを計算
        
        Args:
            y_true: 実測値
            y_pred: 予測値
            
        Returns:
            {'rmse': float, 'nrmse': float, 'r2': float, 'tic': float}
        """
        
    def calculate_peak_friction_error(self,
                                     force_data: np.ndarray,
                                     fz_data: np.ndarray,
                                     force_pred: np.ndarray) -> float:
        """
        ピーク摩擦係数の誤差を計算
        
        Args:
            force_data: 実測力 [N]
            fz_data: 垂直荷重 [N]
            force_pred: 予測力 [N]
            
        Returns:
            ピーク摩擦係数誤差 [%]
        """
        
    def plot_force_vs_slip(self, 
                          slip_data: np.ndarray,
                          force_data: np.ndarray,
                          force_pred: np.ndarray,
                          load_groups: dict,
                          xlabel: str,
                          ylabel: str,
                          title: str,
                          save_path: str):
        """
        力 vs スリップのプロットを生成
        
        Args:
            slip_data: スリップ比またはスリップ角
            force_data: 実測力
            force_pred: 予測力
            load_groups: 荷重グループ辞書
            xlabel, ylabel, title: プロットラベル
            save_path: 保存先パス
        """
        
    def plot_residuals(self,
                      y_true: np.ndarray,
                      y_pred: np.ndarray,
                      save_path: str):
        """
        残差プロットを生成
        
        Args:
            y_true: 実測値
            y_pred: 予測値
            save_path: 保存先パス
        """
```

**評価メトリクス**:
- RMSE: `sqrt(mean((y_true - y_pred)^2))`
- NRMSE: `RMSE / (max(y_true) - min(y_true))`
- R²: `1 - sum((y_true - y_pred)^2) / sum((y_true - mean(y_true))^2)`
- TIC: `sqrt(sum((y_true - y_pred)^2)) / (sqrt(sum(y_true^2)) + sqrt(sum(y_pred^2)))`

### 6. Exporter

**責務**: フィッティング結果をYAMLとC++ヘッダファイルに出力

**インターフェース**:
```python
class ParameterExporter:
    def export_yaml(self, 
                   params: dict, 
                   metrics: dict,
                   filepath: str):
        """
        YAMLファイルに出力
        
        Args:
            params: フィッティングパラメータ辞書
            metrics: 評価メトリクス辞書
            filepath: 出力ファイルパス
        """
        
    def export_cpp_header(self,
                         params: dict,
                         tire_name: str,
                         filepath: str):
        """
        C++ヘッダファイルに出力
        
        Args:
            params: フィッティングパラメータ辞書
            tire_name: タイヤ名
            filepath: 出力ファイルパス
        """
```

## Data Models

### TireData

```python
@dataclass
class TireData:
    """タイヤ試験データ"""
    # 共通
    fz: np.ndarray          # 垂直荷重 [N]
    v: np.ndarray           # 車速 [kph]
    temp: np.ndarray        # タイヤ温度 [°C]
    
    # DriveBrake専用
    sr: np.ndarray          # スリップ比 [-]
    fx: np.ndarray          # 縦力 [N]
    
    # Cornering専用
    sa: np.ndarray          # スリップ角 [deg]
    fy: np.ndarray          # 横力 [N]
    camber: np.ndarray      # キャンバー角 [deg]
```

### FittedParameters

```python
@dataclass
class FittedParameters:
    """フィッティング結果パラメータ"""
    # 基本
    tire_name: str
    effective_radius: float     # [m]
    nominal_load: float         # [N]
    
    # 縦方向
    mu_x_peak: float           # [-]
    k_mu_x: float              # [-]
    Kx: float                  # [N]
    Lx: float                  # [m] (参考値)
    
    # 横方向
    mu_y_peak: float           # [-]
    k_mu_y: float              # [-]
    Ky: float                  # [N]
    Ly: float                  # [m] (参考値)
    
    # キャンバー
    C_gamma: float             # [1/rad]
    
    # 低速安定化
    virtual_velocity: float    # [m/s]
    
    # メタデータ
    fit_date: str
    data_source: str
```

### EvaluationMetrics

```python
@dataclass
class EvaluationMetrics:
    """評価メトリクス"""
    # 縦力
    rmse_fx: float
    nrmse_fx: float
    r2_fx: float
    tic_fx: float
    peak_friction_error_x: float  # [%]
    
    # 横力
    rmse_fy: float
    nrmse_fy: float
    r2_fy: float
    tic_fy: float
    peak_friction_error_y: float  # [%]
```


## Correctness Properties

A property is a characteristic or behavior that should hold true across all valid executions of a system—essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.

### Property 1: CSV Data Loading Preserves All Required Columns

*For any* valid DriveBrake CSV file, loading the file should return a DataFrame containing all 22 required columns (ET, V, N, SA, IA, RL, RE, P, FX, FY, FZ, MX, MZ, NFX, NFY, RST, TSTI, TSTC, TSTO, AmbTmp, SR, SL).

**Validates: Requirements 1.1**

### Property 2: FZ Sign Conversion Round Trip

*For any* CSV file with FZ values, loading the data and then converting FZ signs should result in all FZ values being positive when the original data contained negative values.

**Validates: Requirements 1.3**

### Property 3: Missing Column Error Detection

*For any* CSV file missing one or more required columns, the Data_Loader should raise an error that lists all missing columns.

**Validates: Requirements 1.5**

### Property 4: Data Filtering Preserves Only Valid Rows

*For any* DataFrame with mixed valid and invalid rows, applying all filter conditions (speed >= 30 kph, |FZ| >= 500 N, temperature in range, slip angle/ratio constraints) should result in a DataFrame where every row satisfies all filter conditions.

**Validates: Requirements 2.1, 2.2, 2.3, 2.4, 2.5**

### Property 5: Load Grouping Classification Correctness

*For any* DataFrame with FZ values, grouping by load should result in three groups where every row in the Low group has 600 <= FZ <= 900, every row in Medium has 900 <= FZ <= 1100, every row in High has 1100 <= FZ <= 1400, and rows outside these ranges are excluded.

**Validates: Requirements 3.1, 3.2, 3.3, 3.4**

### Property 6: IQR Outlier Removal Completeness

*For any* DataFrame column, applying IQR-based outlier removal should result in a DataFrame where no values fall outside the range [Q1 - 1.5*IQR, Q3 + 1.5*IQR].

**Validates: Requirements 4.1, 4.2, 4.3, 4.4**

### Property 7: Longitudinal Normalized Deformation Formula

*For any* slip ratio SR, vertical load Fz, and parameters (Kx, mu_x), the TMeasy model should calculate normalized deformation xi_bar as (Kx * SR) / (mu_x * Fz).

**Validates: Requirements 5.1**

### Property 8: Saturation Function Piecewise Correctness

*For any* normalized deformation magnitude s, the effective friction should equal s * (2 - s) when s < 1, and should equal 1 when s >= 1.

**Validates: Requirements 5.2, 5.3**

### Property 9: Longitudinal Force Calculation Formula

*For any* effective friction mu_eff, friction coefficient mu_x, vertical load Fz, and normalized deformation xi_bar with magnitude s, the longitudinal force should equal -mu_eff * mu_x * Fz * (xi_bar / s).

**Validates: Requirements 5.4**

### Property 10: Load-Dependent Friction Coefficient Formula

*For any* vertical load Fz, peak friction mu_peak, load sensitivity k_mu, and nominal load Fz_nom, the load-dependent friction coefficient should equal mu_peak * (1 - k_mu * (Fz - Fz_nom) / Fz_nom).

**Validates: Requirements 5.5, 6.3**

### Property 11: Lateral Normalized Deformation Formula

*For any* slip angle SA, vertical load Fz, and parameters (Ky, mu_y), the TMeasy model should calculate normalized deformation eta_bar as (Ky * tan(SA)) / (mu_y * Fz).

**Validates: Requirements 6.1**

### Property 12: Camber Thrust Addition

*For any* lateral force calculation with camber angle gamma, vertical load Fz, and camber coefficient C_gamma, the camber contribution should equal C_gamma * gamma * Fz and should be added to the base lateral force.

**Validates: Requirements 6.2**

### Property 13: Saturation Function Consistency

*For any* normalized deformation (longitudinal or lateral), both longitudinal and lateral force calculations should use the identical saturation function mu_eff = s * (2 - s) for s < 1, else 1.

**Validates: Requirements 6.4**

### Property 14: Longitudinal Optimizer Returns Valid Parameters

*For any* longitudinal fitting operation, the optimizer should return parameters mu_x_peak, k_mu_x, and Kx where 1.0 <= mu_x_peak <= 2.5, 0 <= k_mu_x <= 0.5, and 5000 <= Kx <= 50000.

**Validates: Requirements 7.1, 7.2, 7.3, 7.4**

### Property 15: Lateral Optimizer Returns Valid Parameters

*For any* lateral fitting operation, the optimizer should return parameters mu_y_peak, k_mu_y, Ky, and C_gamma where 1.0 <= mu_y_peak <= 2.5, 0 <= k_mu_y <= 0.5, and 5000 <= Ky <= 50000.

**Validates: Requirements 8.1, 8.2, 8.3, 8.4**

### Property 16: RMSE Calculation Formula

*For any* arrays of measured forces y_true and predicted forces y_pred, the RMSE should equal sqrt(mean((y_true - y_pred)^2)).

**Validates: Requirements 9.1**

### Property 17: NRMSE Calculation Formula

*For any* RMSE value and measured forces y_true, the NRMSE should equal RMSE / (max(y_true) - min(y_true)).

**Validates: Requirements 9.2**

### Property 18: R-Squared Calculation Formula

*For any* measured forces y_true and predicted forces y_pred, R² should equal 1 - sum((y_true - y_pred)^2) / sum((y_true - mean(y_true))^2).

**Validates: Requirements 9.3**

### Property 19: Theil Inequality Coefficient Formula

*For any* measured forces y_true and predicted forces y_pred, TIC should equal sqrt(sum((y_true - y_pred)^2)) / (sqrt(sum(y_true^2)) + sqrt(sum(y_pred^2))).

**Validates: Requirements 9.4**

### Property 20: Peak Friction Error Calculation

*For any* measured peak friction mu_peak_data and predicted peak friction mu_peak_pred, the peak friction error should equal |mu_peak_data - mu_peak_pred| / mu_peak_data * 100.

**Validates: Requirements 9.5**

### Property 21: YAML Parameter Serialization Round Trip

*For any* fitted parameters object, exporting to YAML and then parsing the YAML file should produce a data structure containing all required fields (tire_model, tire_name, effective_radius, nominal_load, longitudinal section, lateral section, camber section, fitting_info section) with their correct values.

**Validates: Requirements 11.1, 11.2, 11.3, 11.4, 11.5**

### Property 22: C++ Header Structure Correctness

*For any* fitted parameters, the generated C++ header should contain include guards, a struct with all parameters as static constexpr double members, comments with tire name and units, and the struct should be within the fsae::tire namespace.

**Validates: Requirements 12.2, 12.3, 12.4**

## Error Handling

### Data Loading Errors

**File Not Found**:
- Raise `FileNotFoundError` with descriptive message including the attempted file path
- Example: "CSV file not found: data/tiredata/missing_file.csv"

**Missing Columns**:
- Raise `ValueError` listing all missing required columns
- Example: "Missing required columns: ['FX', 'FZ', 'SR']"

**Invalid Data Format**:
- Raise `ValueError` if CSV cannot be parsed
- Example: "Invalid CSV format: unable to parse file"

### Preprocessing Errors

**Empty DataFrame After Filtering**:
- Raise `ValueError` if all rows are filtered out
- Example: "No data remains after applying filters. Check filter thresholds."

**Invalid Load Range**:
- Log warning if no data falls into a load group
- Example: "Warning: No data in High load group (1100-1400N)"

### Model Computation Errors

**Division by Zero**:
- Handle Fz = 0 by returning zero force
- Log warning: "Zero vertical load encountered, returning zero force"

**Invalid Slip Values**:
- Handle NaN or infinite slip values by filtering them out
- Log warning with count of invalid values removed

### Optimization Errors

**Convergence Failure**:
- If optimizer fails to converge, return best parameters found
- Log warning: "Optimizer did not converge. Results may be suboptimal."
- Include convergence status in output metrics

**Bounds Violation**:
- Optimizer should enforce bounds automatically
- If bounds are violated, raise `ValueError`

**Insufficient Data**:
- Raise `ValueError` if fewer than 10 data points available for fitting
- Example: "Insufficient data for fitting: only 5 points available, need at least 10"

### Export Errors

**File Write Permission**:
- Raise `PermissionError` if output directory is not writable
- Example: "Cannot write to output directory: permission denied"

**Invalid Parameters**:
- Raise `ValueError` if parameters contain NaN or infinite values
- Example: "Cannot export invalid parameters: mu_x_peak is NaN"

## Testing Strategy

本システムのテストは、ユニットテストとプロパティベーステストの両方を使用します。これらは相補的であり、包括的なカバレッジを実現するために両方が必要です。

### Unit Testing Approach

ユニットテストは、特定の例、エッジケース、エラー条件に焦点を当てます：

**Data Loading Tests**:
- 既知のCSVファイルで正しいカラムが読み込まれることを確認
- 存在しないファイルでFileNotFoundErrorが発生することを確認
- 欠損カラムでValueErrorが発生することを確認

**Preprocessing Tests**:
- 既知のデータセットでフィルタリング結果を確認
- 荷重グループ化の境界値（600N, 900N, 1100N, 1400N）をテスト
- IQR法で既知の外れ値が除去されることを確認

**Model Tests**:
- 既知のパラメータで既知の力が計算されることを確認
- s = 0, s = 1, s > 1 の境界条件をテスト
- Fz = 0 でゼロ除算が適切に処理されることを確認

**Optimizer Tests**:
- 合成データ（既知のパラメータで生成）でパラメータが回復されることを確認
- パラメータ境界が尊重されることを確認

**Export Tests**:
- YAMLファイルが正しい構造で出力されることを確認
- C++ヘッダファイルが有効な構文を持つことを確認

### Property-Based Testing Approach

プロパティベーステストは、多数のランダム入力にわたって普遍的な性質を検証します。Pythonでは`hypothesis`ライブラリを使用します。

**テスト設定**:
- 各プロパティテストは最低100回の反復を実行
- 各テストは設計ドキュメントのプロパティを参照するタグを含む
- タグ形式: `# Feature: tire-fitting, Property N: [property text]`

**Property Test Examples**:

```python
from hypothesis import given, strategies as st
import hypothesis.extra.numpy as npst

# Property 4: Data Filtering
@given(
    df=st.dataframes(
        columns={
            'V': npst.arrays(float, shape=100, elements=st.floats(0, 100)),
            'FZ': npst.arrays(float, shape=100, elements=st.floats(-2000, 2000)),
            'TSTC': npst.arrays(float, shape=100, elements=st.floats(20, 100)),
            'SA': npst.arrays(float, shape=100, elements=st.floats(-5, 5)),
        }
    )
)
def test_filtering_preserves_only_valid_rows(df):
    # Feature: tire-fitting, Property 4: Data Filtering Preserves Only Valid Rows
    preprocessor = TireDataPreprocessor()
    filtered = preprocessor.filter_data(df)
    
    assert all(filtered['V'] >= 30.0)
    assert all(abs(filtered['FZ']) >= 500.0)
    assert all((filtered['TSTC'] >= 40.0) & (filtered['TSTC'] <= 80.0))
    assert all(abs(filtered['SA']) <= 0.5)

# Property 7: Longitudinal Normalized Deformation
@given(
    sr=st.floats(-0.3, 0.3),
    fz=st.floats(500, 2000),
    kx=st.floats(5000, 50000),
    mu_x=st.floats(1.0, 2.5)
)
def test_longitudinal_normalized_deformation_formula(sr, fz, kx, mu_x):
    # Feature: tire-fitting, Property 7: Longitudinal Normalized Deformation Formula
    model = TMeasyModel()
    xi_bar = model._compute_normalized_deformation_long(sr, fz, kx, mu_x)
    
    expected = (kx * sr) / (mu_x * fz)
    assert abs(xi_bar - expected) < 1e-10

# Property 21: YAML Round Trip
@given(
    params=st.fixed_dictionaries({
        'mu_x_peak': st.floats(1.0, 2.5),
        'k_mu_x': st.floats(0.0, 0.5),
        'Kx': st.floats(5000, 50000),
        'mu_y_peak': st.floats(1.0, 2.5),
        'k_mu_y': st.floats(0.0, 0.5),
        'Ky': st.floats(5000, 50000),
    })
)
def test_yaml_serialization_round_trip(params, tmp_path):
    # Feature: tire-fitting, Property 21: YAML Parameter Serialization Round Trip
    exporter = ParameterExporter()
    filepath = tmp_path / "test_params.yaml"
    
    exporter.export_yaml(params, {}, str(filepath))
    
    import yaml
    with open(filepath) as f:
        loaded = yaml.safe_load(f)
    
    assert 'tire_model' in loaded
    assert 'longitudinal' in loaded
    assert 'lateral' in loaded
    assert loaded['longitudinal']['mu_peak'] == params['mu_x_peak']
    assert loaded['lateral']['Ky'] == params['Ky']
```

**Property Test Coverage**:
- Properties 1-6: データ処理の正確性
- Properties 7-13: TMeasyモデルの数学的正確性
- Properties 14-15: 最適化の制約遵守
- Properties 16-20: 評価メトリクスの計算正確性
- Properties 21-22: 出力フォーマットの正確性

### Integration Testing

**End-to-End Test**:
- 実際のTTCデータファイルを使用
- 完全なフィッティングパイプラインを実行
- 目標精度（RMSE < 100N, TIC < 0.15, ピーク誤差 < 5%）を確認
- YAML と C++ ヘッダファイルが生成されることを確認

### Test Organization

```
tests/
├── unit/
│   ├── test_data_loader.py
│   ├── test_preprocessor.py
│   ├── test_tmeasy_model.py
│   ├── test_optimizer.py
│   ├── test_evaluator.py
│   └── test_exporter.py
├── property/
│   ├── test_properties_data.py
│   ├── test_properties_model.py
│   ├── test_properties_metrics.py
│   └── test_properties_export.py
└── integration/
    └── test_end_to_end_fitting.py
```

### Test Execution

```bash
# Run all tests
pytest tests/

# Run only unit tests
pytest tests/unit/

# Run only property tests
pytest tests/property/

# Run with coverage
pytest --cov=tire_fitting tests/

# Run property tests with more iterations
pytest tests/property/ --hypothesis-profile=thorough
```
