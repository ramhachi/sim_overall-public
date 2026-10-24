# thermal.hpp

## ファイルの目的
- 表面温度 1 状態の簡易タイヤ熱モデルを提供します。

## 実行上の役割
- 発熱、対流冷却、路面伝導、温度依存摩擦係数スケールを純関数で計算します。
- 別ソルバや積分器から呼びやすいよう、状態更新そのものは外部に委ねています。

## 依存関係
- 外部: `<cmath>`, `<algorithm>`
- リポジトリ内依存: なし

## 入力/出力
- 入力: `Fx`, `Fy`, `vsx`, `vsy`, `vehicle_speed`, `TireThermalParams`, `TireThermalState`
- 出力: `dT_dt`, `mu_factor`

## 関数・メソッド・クラスの網羅的な説明
- `template<typename Scalar> struct TireThermalParams`
  - 熱物性 `mass_tread`, `Cp`, `contact_area`
  - 最適温度特性 `T_opt`, `T_width`, `mu_max`, `mu_min`
  - 熱伝達係数 `h_conv`, `h_cond`, 境界温度 `T_ambient`, `T_road`
  - をまとめます。
- `template<typename Scalar> struct TireThermalState`
  - 状態量は `T_surface` のみです。
- `template<typename Scalar> struct TireThermalOutput`
  - `dT_dt` と `mu_factor` を返します。
- `compute_friction_power(Scalar Fx, Scalar Fy, Scalar vsx, Scalar vsy)`
  - `|Fx*vsx| + |Fy*vsy|` により摩擦発熱量を返します。
- `compute_mu_temperature_factor(const TireThermalParams<Scalar>& params, Scalar T_surface)`
  - 最適温度を中心としたガウス形で摩擦係数スケールを返します。
  - 下限は `mu_min/mu_max`、頂点は 1.0 です。
- `compute_tire_thermal(...)`
  - 発熱 `Q_gen`、速度依存対流 `Q_conv`、路面伝導 `Q_cond` を求め、熱収支から `dT_dt` を返します。
  - 同時に現在温度に対する `mu_factor` を返します。
