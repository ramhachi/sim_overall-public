# `tire_fitting/optimizer.py`

## 役割

フィッティングの最適化本体を集めたモジュールです。  
旧 TMeasy 専用の `TireParameterOptimizer` と、hybrid 系で複数モデルを並列比較する `ModelFitter` の 2 系統を同居させています。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - スリップ比 / スリップ角
  - 実測 `FX` / `FY`
  - 垂直荷重 `FZ`
  - キャンバー角、温度、モデル種別
- 主な出力:
  - 推定パラメータ辞書
  - `rmse`, `tic`, `r2`, `peak_error` を含む指標辞書
- 副作用:
  - `ProcessPoolExecutor` による並列実行
  - `self.cache` への結果保存

## 主要クラス

### `TireParameterOptimizer`
- 旧 TMeasy 単独用です。
- `longitudinal_bounds`, `lateral_bounds` に探索範囲を持ちます。

#### `fit_longitudinal(sr_data, fx_data, fz_data, bounds=None)`
- 差分進化で大域探索し、その後 `least_squares(method='trf')` で局所改善します。
- 現在は正規化残差ではなく **絶対残差二乗和** を最小化します。
- 戻り値は `mu_x_peak`, `k_mu_x`, `Kx`, `rmse`, `r2`, `success`, `message` です。

#### `fit_lateral(sa_data, fy_data, fz_data, camber_data, bounds=None)`
- 横力版で、追加パラメータ `C_gamma` を含みます。

### `ModelFitter`
- hybrid パイプライン向けです。
- `tmeasy` / `magic_formula` を方向別に同時最適化できます。

#### `fit_multiple_models(model_types, directions, drivebrake_file, cornering_file, max_workers=None)`
- 各 `(model_type, direction)` をプロセスプールへ投げ、成功結果だけを `results` に格納します。
- 既存キャッシュがあれば再計算を省略します。

#### `_fit_single_model(model_type, direction, data_file)`
- データ読込、最小前処理、探索境界設定、目的関数定義、差分進化、最終指標計算を一括実行します。
- `fx` では平均タイヤ温度をモデルへ渡せます。

#### `_get_parameter_bounds(...)`
- モデル種別と方向に応じた境界を返します。

#### `_params_array_to_dict(...)`
- SciPy 最適化用配列をモデル別パラメータ辞書へ戻します。

#### `_calculate_metrics(...)`
- 予測再計算後に RMSE / TIC / R² / peak error を返します。

## データフロー

1. 上流から配列またはファイルパスを受けます。
2. 目的関数でモデル予測と実測の差を評価します。
3. 最適化器が候補パラメータを探索します。
4. 最良解を辞書へ戻し、評価指標とセットで返します。

## システム内での位置づけ

`main.py` と `main_hybrid.py` の中核です。  
単独 TMeasy 適合と、hybrid 比較用の多モデル適合の両方をここで提供しています。
