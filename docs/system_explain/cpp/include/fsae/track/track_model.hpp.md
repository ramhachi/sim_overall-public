# track_model.hpp

## ファイルの目的
- 曲率ベースの中心線トラック表現と、代表的な解析トラック生成関数群を提供します。

## 実行上の役割
- シミュレーション、MPC、ラップタイム計算の共通基盤として `s` と `kappa(s)` からトラック点列を生成します。
- 円、スキッドパッド、直線+コーナー、加速コースなどの試験トラックもここで生成します。

## 依存関係
- 外部: `<vector>`, `<cmath>`, `<algorithm>`, `<stdexcept>`
- リポジトリ内依存: なし

## 入力/出力
- 入力: 曲率データ、`TrackPoint` 列、各種トラック寸法
- 出力: `TrackPoint`, `CurvatureBasedTrack`

## 関数・メソッド・クラスの網羅的な説明
- `struct TrackPoint`
  - `s`, `x`, `y`, `theta`, `kappa`, `width_left`, `width_right` を持つ中心線点です。
- `class CurvatureBasedTrack`
  - `CurvatureBasedTrack(std::vector<std::pair<double,double>> curvature_data, double x0=0, double y0=0, double theta0=0, double track_width=3.0, bool force_loop=false)`
    - 曲率データを `s` でソートし、`generate_track_points()` で座標列を積分生成します。
  - `CurvatureBasedTrack(std::vector<TrackPoint> track_points, bool force_loop=false)`
    - 既成 `TrackPoint` 列から直接構築し、互換性のため `curvature_data_` も埋めます。
  - `get_curvature(double s) const`
    - `curvature_data_` を線形補間して曲率を返します。
  - `get_point(double s) const`
    - `track_points_` を線形補間して位置・姿勢・幅を返します。
  - `length() const`
    - 最終点の `s` を返します。
  - `points() const`
    - 全点列への const 参照です。
  - `mutable_points()`
    - 解析解で上書きしたい用途向けの可変参照です。
  - `is_closed_loop() const`
    - `force_loop`、始終点距離、総曲率積分が 2π の整数倍か、の順で閉ループ判定します。
  - private `generate_track_points()`
    - `ds=0.1m` のオイラー積分で `theta/x/y` を進めて `track_points_` を作ります。
    - 閉ループ判定時は終点位置誤差を全体へ線形配分し、最後の `theta` も開始方向に整合させます。
  - private メンバ `curvature_data_`, `track_points_`, `x0_`, `y0_`, `theta0_`, `track_width_`, `is_loop_forced_`
    - 曲率入力、生成点列、初期条件、既定幅、ループ強制フラグです。
- `create_circle_track(double radius, double track_width = 3.0)`
  - 反時計回り円を解析式で生成し、`mutable_points()` を使って積分誤差を避けた点列へ上書きします。
- `create_circle_cw_track(double radius, double track_width = 3.0)`
  - 負曲率の時計回り円を同様に生成します。
- `create_figure_eight_track(double radius, double track_width = 3.0)`
  - 縦並び 2 円のレガシー 8 の字トラックを解析式で生成します。
- `create_skidpad_track(double radius = 9.125, double track_width = 3.0)`
  - SAE スキッドパッドを右円 CW → 左円 CCW の水平 figure-8 として生成します。
  - 始終点とヘディングを一致させる補正を入れます。
- `create_straight_corner_track(double straight_length, double corner_radius, double track_width = 3.0)`
  - 直線→90 度コーナー→直線の曲率列を作って返します。
- `create_acceleration_track(double length = 75.0, double track_width = 5.0)`
  - SAE Acceleration Event 向けの完全直線トラックを返します。
