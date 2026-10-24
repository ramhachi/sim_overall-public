# tire_thermal_integrator.hpp

## ファイルの目的
- 4 輪分の表面温度を時間積分する簡易熱積分器を提供します。

## 実行上の役割
- `TireForces` とスリップ速度を受け取り、オイラー法で各輪温度を更新します。
- 熱モデルを状態保持付きコンポーネントとして使いたい経路向けの実装です。

## 依存関係
- 外部: `<array>`, `<cmath>`, `<algorithm>`
- リポジトリ内: `TireForces` 型を前提にしているため、利用側では `nn_lookup_tire.hpp` 等の先行インクルードが必要です

## 入力/出力
- 入力: `dt`, `std::array<TireForces,4>`, `std::array<TireSlipInfo,4>`, `v_vehicle`, `T_air`
- 出力: 内部保持温度、`get_temperature()`/`get_temperatures()` による取得値

## 関数・メソッド・クラスの網羅的な説明
- `struct TireThermalParams`
  - 熱容量 `m_tread`, `c_p`
  - 対流冷却 `h_conv_base`, `A_tread`, `v_ref`
  - 路面伝導 `h_road`, `A_contact`, `T_road`
  - 初期温度 `T_initial`
  - を持ちます。
- `struct TireSlipInfo`
  - `vsx`, `vsy` に縦横スリップ速度を保持します。
- `class TireThermalIntegrator`
  - `explicit TireThermalIntegrator(const TireThermalParams& params = TireThermalParams())`
    - パラメータを保存し、4 輪温度を初期温度で埋めます。
  - `update(double dt, const std::array<TireForces,4>& forces, const std::array<TireSlipInfo,4>& slip_info, double v_vehicle, double T_air)`
    - 各輪について摩擦発熱、速度依存対流、路面伝導を計算し、`dT_dt` を Euler 積分します。
    - 温度は `[-10,150]` ℃ にクランプされます。
  - `get_temperature(int wheel_index) const`
    - 単輪の表面温度を返します。
  - `get_temperatures() const`
    - 4 輪温度配列への const 参照を返します。
  - `reset(double T = -1.0)`
    - 負値なら `T_initial`、それ以外なら指定温度で全輪を再初期化します。
  - `set_params(const TireThermalParams& params)`
    - パラメータを差し替えます。
  - private `params_`, `T_surf_`
    - パラメータと 4 輪温度状態です。
