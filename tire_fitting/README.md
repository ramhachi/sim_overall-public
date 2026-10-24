# Tire Model Fitting Tool

TMeasyタイヤモデルのパラメータを、FSAE Tire Test Consortium (TTC)の実験データからフィッティングするツール。

## 概要

このツールは、DriveBrake試験データ（縦力）とCornering試験データ（横力）を読み込み、TMeasyモデルのパラメータを最適化します。

### 主要機能

- **データ読み込み**: TTC形式のCSVファイルを読み込み
- **データ前処理**: フィルタリング、荷重グループ化、外れ値除去
- **TMeasyモデル**: 定常状態の縦力・横力計算
- **2段階最適化**: 差分進化法 + Levenberg-Marquardt法
- **評価メトリクス**: RMSE, R², TIC, ピーク摩擦係数誤差
- **結果出力**: YAML、C++ヘッダファイル、プロット

## インストール

### 必要なパッケージ

```bash
pip install pandas numpy scipy matplotlib pyyaml hypothesis pytest
```

または、プロジェクトルートから：

```bash
pip install -r requirements.txt
```

## 使用方法

### 基本的な使い方

```bash
python3 tire_fitting/main.py \
    --drivebrake data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv \
    --cornering data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv \
    --output output/tire_params \
    --tire-name Hoosier_18x6_R20
```

### コマンドライン引数

| 引数 | 説明 | デフォルト |
|:---|:---|:---|
| `--drivebrake` | DriveBrake CSVファイルパス | 必須 |
| `--cornering` | Cornering CSVファイルパス | 必須 |
| `--output` | 出力ディレクトリ | `output/tire_params` |
| `--tire-name` | タイヤ名 | `tire` |
| `--radius` | 有効半径 [m] | `0.2286` |
| `--nominal-load` | 基準荷重 [N] | `1000.0` |

### 出力ファイル

実行後、以下のファイルが生成されます：

```
output/tire_params/
├── hoosier_18x6_r20.yaml          # YAMLパラメータファイル
├── hoosier_18x6_r20.hpp           # C++ヘッダファイル
├── fx_vs_sr.png                   # 縦力 vs スリップ比プロット
├── fy_vs_sa.png                   # 横力 vs スリップ角プロット
├── residuals_fx.png               # 縦力残差プロット
└── residuals_fy.png               # 横力残差プロット
```

## YAMLファイル形式

```yaml
tire_model: TMeasy
tire_name: Hoosier_18x6_R20
effective_radius: 0.2286
nominal_load: 1000.0

longitudinal:
  mu_peak: 1.5           # ピーク縦摩擦係数 [-]
  mu_sensitivity: 0.1    # 荷重感度 [-]
  Kx: 15000.0           # 複合剛性 [N]
  relaxation_length: 0.05  # 緩和長 [m]

lateral:
  mu_peak: 1.4           # ピーク横摩擦係数 [-]
  mu_sensitivity: 0.12   # 荷重感度 [-]
  Ky: 12000.0           # 複合剛性 [N]
  relaxation_length: 0.06  # 緩和長 [m]

camber:
  thrust_coefficient: 0.05  # キャンバースラスト係数 [1/rad]

fitting_info:
  data_source: FSAE TTC
  fit_date: '2026-01-24'
  rmse_fx: 45.2          # 縦力RMSE [N]
  rmse_fy: 38.7          # 横力RMSE [N]
  r2_fx: 0.98            # 縦力R²
  r2_fy: 0.97            # 横力R²
  nrmse_fx: 0.023        # 縦力正規化RMSE
  nrmse_fy: 0.019        # 横力正規化RMSE
  tic_fx: 0.012          # 縦力TIC
  tic_fy: 0.010          # 横力TIC
```

## C++での使用例

生成されたヘッダファイルをインクルードして使用：

```cpp
#include "hoosier_18x6_r20.hpp"

using namespace fsae::tire;

// パラメータにアクセス
double mu_x = Hoosier_18x6_R20::mu_x_peak;
double Kx = Hoosier_18x6_R20::Kx;
double radius = Hoosier_18x6_R20::effective_radius;

// TMeasyモデルで使用
TMeasyTire tire(
    Hoosier_18x6_R20::mu_x_peak,
    Hoosier_18x6_R20::k_mu_x,
    Hoosier_18x6_R20::Kx,
    Hoosier_18x6_R20::mu_y_peak,
    Hoosier_18x6_R20::k_mu_y,
    Hoosier_18x6_R20::Ky,
    Hoosier_18x6_R20::C_gamma
);
```

## テスト

### 全テスト実行

```bash
pytest tests/ -v
```

### ユニットテストのみ

```bash
pytest tests/unit/ -v
```

### プロパティベーステストのみ

```bash
pytest tests/property/ -v
```

### 統合テスト

```bash
pytest tests/test_end_to_end_integration.py -v
```

### カバレッジ付き

```bash
pytest --cov=tire_fitting tests/
```

## TMeasyモデルについて

TMeasyは、Pacejka Magic Formulaと比較して以下の利点があります：

- **低速特異点なし**: ゼロ速度でも安定
- **パラメータ数が少ない**: ~15個（Magic Formulaは~30個）
- **物理的解釈可能**: 各パラメータが物理的意味を持つ
- **計算効率**: シンプルな数式で高速計算

### モデル方程式

#### 縦力

```
mu_x(Fz) = mu_x_peak * (1 - k_mu_x * (Fz - Fz_nom) / Fz_nom)
xi_bar = Kx * SR / (mu_x * Fz)
s = |xi_bar|
mu_eff = s * (2 - s)  if s < 1 else 1
Fx = -mu_eff * mu_x * Fz * sign(xi_bar)
```

#### 横力

```
mu_y(Fz) = mu_y_peak * (1 - k_mu_y * (Fz - Fz_nom) / Fz_nom)
eta_bar = Ky * tan(SA) / (mu_y * Fz)
s = |eta_bar|
mu_eff = s * (2 - s)  if s < 1 else 1
Fy = -mu_eff * mu_y * Fz * sign(eta_bar) + C_gamma * camber * Fz
```

## データ要件

### DriveBrake CSVファイル

必須カラム：
- `ET`: 経過時間 [s]
- `V`: 車速 [kph]
- `FZ`: 垂直荷重 [N]
- `FX`: 縦力 [N]
- `SR`: スリップ比 [-]
- `SA`: スリップ角 [deg]
- `TSTC`: タイヤ温度 [°C]

### Cornering CSVファイル

必須カラム：
- `SA`: スリップ角 [deg]
- `IA`: キャンバー角 [deg]
- `FY`: 横力 [N]
- `FZ`: 垂直荷重 [N]

## フィルタリング条件

データ前処理で以下の条件が適用されます：

- 車速: V >= 30 kph
- 垂直荷重: |FZ| >= 500 N
- タイヤ温度: 40°C <= TSTC <= 80°C
- 純縦力条件: |SA| <= 0.5 deg
- 純横力条件: |SR| <= 0.01

## 荷重グループ

データは3つの荷重グループに分類されます：

- **Low**: 600-900 N
- **Medium**: 900-1100 N
- **High**: 1100-1400 N

## 目標精度

フィッティングは以下の目標精度を達成します：

- 縦力RMSE < 100 N
- 横力RMSE < 100 N
- TIC < 0.15
- ピーク摩擦係数誤差 < 5%

## トラブルシューティング

### 最適化が収束しない

- データ品質を確認（外れ値、ノイズ）
- フィルタリング条件を調整
- パラメータ境界を調整

### 精度が目標に達しない

- データ量を増やす
- 温度範囲を狭める
- 荷重範囲を調整

### メモリエラー

- データサイズを確認
- 大きなデータセットは分割処理

## 参考文献

1. Hirschberg, W., Rill, G., & Weinfurter, H. (2007). "Tire model TMeasy"
2. FSAE Tire Test Consortium (TTC) Data Format Specification
3. Pacejka, H. B. (2012). "Tire and Vehicle Dynamics"

## ライセンス

MIT License

## 作成者

FSAE Vehicle Dynamics Team

## 更新履歴

- 2026-01-24: 初版リリース
  - データ読み込み、前処理、最適化、評価、出力機能を実装
  - 全92テストが成功
  - プロパティベーステスト22個を含む
