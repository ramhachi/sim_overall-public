# `cpp/src/physics/aero/aero_map_4d.cpp`

## 1. ファイルの目的
4 次元エアロマップ `AeroMap4D` テンプレートの明示的インスタンス化を行い、`double`/`float` 版をリンク可能にするファイルです。

## 2. 実行上の役割
- 実装本体は `aero_map_4d.hpp` にあります。
- ラップタイム計算や 14DOF 車両で用いる空力補間器の実体をまとめて生成します。

## 3. 依存関係
- `fsae/physics/aero/aero_map_4d.hpp`
- `nlohmann/json`

## 4. 入力/出力
- 実行入口ではありません。
- `AeroMap4D<double>` と `AeroMap4D<float>` のテンプレート実体を生成します。

## 5. 関数・メソッド・クラスの網羅的な説明

### この `.cpp` 自体に独自関数はない
- `template class AeroMap4D<double>;`
- `template class AeroMap4D<float>;`
のみを持ちます。

### `AeroOutput<Scalar>`
- 係数空力モデル出力です。
- `CL`, `CD`, `CY`, `Cm`, `CoP_x`, `CoP_dist` を保持します。

### `AeroForces<Scalar>`
- 実力/モーメント出力です。
- `Fx`, `Fy`, `Fz`, `Mx`, `My`, `Mz` と、前後配分 `Fz_front`, `Fz_rear` を保持します。

### `AeroMapParams<Scalar>`
- 空力マップのパラメータです。
- 空気密度、基準面積、ホイールベース、CG、参照速度/密度、直接荷重マップ使用可否、既定係数/既定荷重を持ちます。

### `AeroMapGrid<Scalar>`
- 補間軸です。
- `rh_f`, `rh_r`, `roll`, `yaw` の 4 軸を保持します。

### `AeroMap4D<Scalar>`
- `double` 版と `float` 版がこの `.cpp` で実体化されます。
- `aero_map_4d.hpp` で定義される主な公開メソッド:
  - コンストラクタ群: 既定グリッドとサンプルデータ初期化
  - `compute(...)`: RH 前後、ロール、ヨー、DRS 状態から係数を 4D 線形補間
  - `compute_forces(...)`: 車速込みで力・モーメントへ変換。直接荷重マップにも対応
  - `compute_ride_height(...)`: サスペンション変位からライドハイトを算出
  - `params()`, `grid()`, `uses_direct_load_map()`: アクセサ
  - `set_reference_conditions(...)`
  - `set_direct_load_defaults(...)`
  - `set_coefficient_defaults(...)`
  - `load_from_json(...)`: JSON から軸・マップデータをロード
- 主な private helper:
  - `idx(...)`: 4D 配列の一次元化インデックス
  - `allocate_data()`
  - `init_default_grid()`
  - `init_sample_data()`
  - `init_direct_load_data()`
  - `expected_data_size()`
  - `has_valid_interpolation_grid()`
  - `clamp(...)`
  - `find_index_weight(...)`
  - `interpolate_4d(...)`

