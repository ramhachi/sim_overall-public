# track_distance.hpp

## ファイルの目的
- グローバル座標からトラック最近点を高速取得する距離クエリを提供します。

## 実行上の役割
- Frenet の代替または前段として、中心線上の最近 `s`、横偏差 `n`、曲率、参照軌道列を返します。
- 事前サンプルと `hint_s` による局所探索で MPC 向けの軽量クエリを実現します。

## 依存関係
- リポジトリ内: `track_model.hpp`
- 外部: `<vector>`, `<cmath>`, `<limits>`

## 入力/出力
- 入力: `x`, `y`, `hint_s`, `velocity_sv`, `N`, `dt`
- 出力: `NearestResult`、`std::vector<ReferencePoint>`

## 関数・メソッド・クラスの網羅的な説明
- `struct NearestResult`
  - `s`, `n`, `theta`, `kappa`, `distance` を保持します。
- `struct ReferencePoint`
  - `x`, `y`, `psi`, `vx`, `s`, `kappa`, `width_left`, `width_right` を持つ MPC 用参照点です。
- `class TrackDistanceQuery`
  - `explicit TrackDistanceQuery(const CurvatureBasedTrack& track, double ds = 0.5)`
    - トラック全長に沿って等間隔サンプル `samples_` を事前生成します。
  - `nearest(double x, double y, double hint_s = -1.0) const`
    - `hint_s` があれば ±10m のサンプルだけを局所探索し、なければ全探索します。
    - 最良サンプル周辺を 10 分割して `track_.get_point()` で精密化し、最終 `s/n/theta/kappa/distance` を返します。
  - `get_reference_trajectory(double s_start, const std::vector<std::pair<double,double>>& velocity_sv, int N, double dt) const`
    - 開始 `s` から `N+1` 点の参照列を生成し、各点の速度は `interpolate_velocity()` で補間します。
    - 閉ループ前提で `fmod` により `s` を正規化します。
  - `track_length() const`
    - トラック全長を返します。
  - private `struct SamplePoint`
    - 事前サンプルされた中心線情報です。
  - private `interpolate_velocity(double s, const std::vector<std::pair<double,double>>& sv) const`
    - `(s,v)` 表を線形補間し、空なら 10 m/s を返します。
  - private メンバ `track_`, `ds_`, `track_length_`, `samples_`
    - 参照トラック、サンプル間隔、全長、事前計算サンプル列です。
