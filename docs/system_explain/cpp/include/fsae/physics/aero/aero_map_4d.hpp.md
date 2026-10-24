# aero_map_4d.hpp

## ファイルの目的
- フロント/リアライドハイト、ロール、ヨーを入力にした 4 次元エアロマップを表現します。

## 実行上の役割
- ラップタイム計算や車両運動計算で、空力係数または直接荷重マップから `Fx/Fy/Fz/M*` を生成します。
- JSON 読み込み、4 次元補間、DRS 補正、前後ダウンフォース配分までを 1 つのヘッダで担います。

## 依存関係
- 外部: `<vector>`, `<array>`, `<cmath>`, `<algorithm>`, `<fstream>`, `<string>`, `<utility>`, `nlohmann/json.hpp`
- リポジトリ内依存: `velocity_profile.hpp` などから利用されます

## 入力/出力
- 入力: `rh_front_mm`, `rh_rear_mm`, `roll_rad`, `yaw_rad`, `velocity`, `drs_active`
- 出力:
  - `compute()` は係数系 `AeroOutput`
  - `compute_forces()` は荷重系 `AeroForces`

## 関数・メソッド・クラスの網羅的な説明
- `template<typename Scalar> struct AeroOutput`
  - `CL`, `CD`, `CY`, `Cm`, `CoP_x`, `CoP_dist` を保持します。
  - `Cm` は補間された `CL` と空力中心位置から計算されるピッチングモーメント係数です。
- `template<typename Scalar> struct AeroForces`
  - `Fx`, `Fy`, `Fz`, `Mx`, `My`, `Mz` と、前後配分済み `Fz_front`, `Fz_rear` を保持します。
- `template<typename Scalar> struct AeroMapParams`
  - 基本物理量 `rho`, `A_ref`, `L_ref`, `x_cg`、参照条件 `reference_speed`, `reference_density` を保持します。
  - `use_direct_load_map` により「係数マップ」か「参照速度基準の直接荷重マップ」かを切り替えます。
  - `CL_default/CD_default/CoP_default` は係数マップ未設定時の既定値です。
  - `Fx_default`〜`Mz_default` は直接荷重モードの既定値です。
- `template<typename Scalar> struct AeroMapGrid`
  - 補間軸 `rh_f`, `rh_r`, `roll`, `yaw` を保持します。
- `template<typename Scalar> class AeroMap4D`
  - `AeroMap4D()` / `explicit AeroMap4D(const AeroMapParams<Scalar>& params)`
    - デフォルトグリッドを初期化し、サンプルデータを生成します。
  - `compute(...) const`
    - 4 軸をクランプし、`find_index_weight()` で補間位置を求め、`CL/CD/CY/CoP_x/CoP_dist` を 4D 線形補間します。
    - `Cm` は `CL * (CoP_x - x_cg) / L_ref` で計算されます。
    - `drs_active=true` なら `CL` を 0.54 倍、`CD` を 0.74 倍に補正します。
  - `compute_forces(...) const`
    - `use_direct_load_map=true` なら `Fx_ref_data_` などを補間し、参照速度・参照密度から速度二乗比例でスケーリングします。
    - direct-load モードの DRS は `Fx/Fz/My/Mx/Mz` に別係数を掛けます。
    - 係数モードでは動圧 `q` と `compute()` の結果から `Fx/Fy/Fz/My/Mz` を算出し、`CoP_dist` で前後配分します。
  - `static compute_ride_height(...)`
    - ばね上/ばね下変位差からライドハイト低下量を mm で計算します。
    - `wheel_radius` は現在未使用です。
  - `params()` / `params() const`
    - パラメータ参照を返します。
  - `grid() const`
    - 補間グリッドを返します。
  - `uses_direct_load_map() const`
    - direct-load モードの有効状態を返します。
  - `set_reference_conditions(...)`
    - 参照速度と参照密度を下限付きで更新します。
  - `set_direct_load_defaults(...)`
    - 直接荷重既定値を設定し、`use_direct_load_map=true` にして `init_direct_load_data()` を呼びます。
  - `set_coefficient_defaults(...)`
    - 係数既定値を設定し、係数モードに戻して `init_sample_data()` を再生成します。
  - `load_from_json(const std::string& filepath)`
    - `aero_map` ルートまたは直下 JSON を読み、軸配列とデータ配列を検証して本体へ反映します。
    - `roll_deg/yaw_deg` も受理し、rad に変換します。
    - 直接荷重データ `Fx_N`〜`Mz_Nm` が揃えば direct-load、そうでなければ係数データ `CL/CD/CY/CoP_x/CoP_dist` を期待します。
  - private `idx(i,j,k,l) const`
    - 4 次元添字を一次元ベクタ添字に変換します。
  - private `allocate_data()`
    - グリッドサイズに合わせて全データ配列を確保・初期化します。
  - private `init_default_grid()`
    - RH、ロール、ヨーの既定軸を設定します。
  - private `init_sample_data()`
    - 地面効果、ロール損失、ヨー影響、空力中心移動、前後配分の簡易式で係数マップを埋めます。
  - private `init_direct_load_data()`
    - 直接荷重モード用配列を既定値で埋めます。
  - private `expected_data_size() const`
    - JSON 配列長の期待値を返します。
  - private `has_valid_interpolation_grid() const`
    - 各軸が 2 点以上あるか検証します。
  - private `clamp(...)`
    - 値を範囲内へ収める内部関数です。
  - private `find_index_weight(...)`
    - 1 軸分の補間区間始点と重みを返します。
  - private `interpolate_4d(...) const`
    - 16 頂点の重み付き和を取る 4D 線形補間本体です。
