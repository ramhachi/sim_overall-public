# `cpp/bindings/py_sim.cpp`

## 1. ファイルの目的
`fsae_sim_py` という pybind11 モジュールを定義し、C++ 側のタイヤ・熱・サスペンション・回転計算を Python から直接呼べるようにするためのバインディング層です。

## 2. 実行上の役割
- CMake で Python 拡張モジュールとしてビルドされる入口です。
- 学習、同定、可視化、ノートブック検証などを Python 側で実施する際に、C++ 実装の計算ルーチンを再利用する窓口になります。

## 3. 依存関係
- `fsae/physics/tire/tmeasy.hpp`
- `fsae/physics/tire/thermal.hpp`
- `fsae/physics/suspension/bouc_wen.hpp`
- `fsae/physics/suspension/lugre.hpp`
- `fsae/math/rotation.hpp`
- pybind11 本体 (`pybind11/pybind11.h`, `stl.h`, `eigen.h`)

## 4. 入力/出力
- 入力: Python から渡されるパラメータ構造体、状態、スカラー値。
- 出力:
  - Python クラス `TMeasyParams`, `TMeasyState`, `TMeasyOutput`, `TireThermalParams`, `BoucWenParams`, `LuGreParams`, `EulerAngles`
  - Python 関数 `compute_tmeasy`, `compute_mu_temperature_factor`, `compute_bouc_wen_force`, `compute_bouc_wen_dz`, `euler_to_rotation_matrix`
  - 属性 `__version__ = "0.1.0"`

## 5. 関数・メソッド・クラスの網羅的な説明

### `PYBIND11_MODULE(fsae_sim_py, m)`
- このファイルの唯一のエントリポイントです。
- Python モジュール名を `fsae_sim_py` に固定し、以下の型・関数を公開します。

### `physics::tire::TMeasyParams<double>` の公開
- Python 名: `TMeasyParams`
- 役割: TMeasy タイヤモデルの主要パラメータ入れ物。
- 公開フィールド:
  - `mu_x_max`, `mu_y_max`: 縦横最大摩擦係数
  - `Fz_nom`: 基準鉛直荷重
  - `Lx`, `Ly`: 縦横の緩和長
  - `v_N`: 低速時の仮想速度

### `physics::tire::TMeasyState<double>` の公開
- Python 名: `TMeasyState`
- 役割: TMeasy の内部状態。
- 公開フィールド:
  - `xi`, `eta`: 縦・横の内部状態

### `physics::tire::TMeasyOutput<double>` の公開
- Python 名: `TMeasyOutput`
- 役割: TMeasy 計算結果の読み出し専用コンテナ。
- 公開フィールド:
  - `Fx`, `Fy`, `Mz`
  - `dxi_dt`, `deta_dt`

### `compute_tmeasy`
- C++ 実体: `physics::tire::compute_tmeasy<double>`
- 入力:
  - `params`, `state`
  - `Fz`: 鉛直荷重
  - `vsx`, `vsy`: スリップ関連速度
  - `vcx`: 接地速度
- 出力: `TMeasyOutput`
- 役割: Python から TMeasy を 1 回評価する主要関数です。

### `physics::tire::TireThermalParams<double>` の公開
- Python 名: `TireThermalParams`
- 役割: 温度依存摩擦補正の簡易パラメータ。
- 公開フィールド:
  - `T_opt`: 最適温度
  - `T_width`: 有効温度幅
  - `mu_max`: 最大摩擦係数

### `compute_mu_temperature_factor`
- C++ 実体: `physics::tire::compute_mu_temperature_factor<double>`
- 役割: タイヤ温度に応じた摩擦係数スケーリングを返します。

### `physics::suspension::BoucWenParams<double>` の公開
- Python 名: `BoucWenParams`
- 役割: Bouc-Wen ダンパ/ヒステリシスモデルの設定値。
- 公開フィールド:
  - `c0`, `alpha`, `beta`, `gamma`, `n`

### `compute_bouc_wen_force`
- C++ 実体: `physics::suspension::compute_bouc_wen_force<double>`
- 役割: 相対変位・相対速度・内部変数からダンパ力を返します。

### `compute_bouc_wen_dz`
- C++ 実体: `physics::suspension::compute_bouc_wen_dz<double>`
- 役割: Bouc-Wen 内部状態の時間微分を返します。

### `physics::suspension::LuGreParams<double>` の公開
- Python 名: `LuGreParams`
- 役割: LuGre 摩擦モデル用パラメータ。
- 公開フィールド:
  - `sigma_0`, `sigma_1`, `Fc`, `Fs`
- 備考: このファイルでは型公開のみで、対応する計算関数は公開していません。

### `math::EulerAngles<double>` の公開
- Python 名: `EulerAngles`
- 役割: Z-Y-X オイラー角の受け渡し構造体。
- 公開フィールド:
  - `phi`, `theta`, `psi`

### `euler_to_rotation_matrix`
- C++ 実体: `math::euler_to_rotation_matrix<double>`
- 役割: オイラー角から 3x3 回転行列を返します。
- 出力は `pybind11/eigen.h` により Python 側では Eigen/NumPy 相当の行列として扱われます。

