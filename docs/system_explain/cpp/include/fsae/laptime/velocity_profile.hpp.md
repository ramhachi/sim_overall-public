# velocity_profile.hpp

## ファイルの目的
- トラック曲率と車両性能から速度プロファイルを生成します。

## 実行上の役割
- 旋回限界、前向き加速積分、後向き減速積分、空力荷重、摩擦円制約をまとめて扱い、ラップタイム計算の基礎となる目標速度列を作ります。

## 依存関係
- リポジトリ内: `../track/track_model.hpp`, `../physics/aero/aero_map_4d.hpp`
- 外部: `<vector>`, `<cmath>`, `<algorithm>`, `<optional>`

## 入力/出力
- 入力: `CurvatureBasedTrack`, `VehiclePerformance`, `ds`, `v_start`
- 出力: `std::vector<VelocityPoint>`, ラップタイム、g-g 図サンプル

## 関数・メソッド・クラスの網羅的な説明
- `struct VelocityPoint`
  - `s`, `v`, `ax`, `ay`, `kappa` を保持します。
- `struct VehiclePerformance`
  - 固定制限 `max_lat_accel`, `max_lon_accel`, `max_lon_decel`, `max_speed`, `min_speed`, `safety_factor`
  - 物理パラメータ `use_physics_model`, `mass`, `mu_lat`, `mu_lon`, `cl_a`, `cd_a`, `rho`
  - 空力マップ系 `use_aero_map`, `aero_map`, `aero_nominal_front_ride_height_mm`, `aero_nominal_rear_ride_height_mm`, `aero_nominal_roll_rad`, `aero_nominal_yaw_rad`, `aero_drs_active`
  - 立上がり制限 `warmup_distance`, `warmup_accel_limit`
  - をまとめます。
- `class VelocityProfileGenerator`
  - `explicit VelocityProfileGenerator(const VehiclePerformance& perf)`
    - 性能設定を保持します。
  - `generate(const track::CurvatureBasedTrack& track, double ds = 1.0, std::optional<double> v_start = std::nullopt)`
    - 旋回限界速度を作り、必要なら開始速度を上書きし、前方積分→後方積分→境界処理→加速度更新の順で最終プロファイルを返します。
    - 閉ループでは始終端速度を揃え、開ループでは終端 10m で安全減速します。
  - private `compute_corner_limited_speed(...)`
    - `s` 走査し、曲率がある区間では横加速度限界から速度上限を計算します。
    - `use_aero_map` 時は固定点反復で空力依存横加速度を解きます。
  - private `forward_integration(std::vector<VelocityPoint> profile)`
    - 先行点速度・曲率から横加速度消費分を引いた縦加速度余裕を使い、加速側の速度上限を伝播させます。
    - ドラッグ損失とウォームアップ制限も加味します。
  - private `backward_integration(std::vector<VelocityPoint> profile)`
    - 次点速度から必要減速を伝播し、ブレーキ側の速度上限を前方へ戻します。
  - private `update_accelerations(std::vector<VelocityPoint>& profile)`
    - `ay=v^2*kappa`、`ax=Δ(v^2)/(2Δs)` を更新します。
  - private `get_max_lat_accel(double v) const`
    - 空力ダウンフォースを考慮した横加速度上限を返します。
    - `use_aero_map` なら `aero_map.compute_forces()` を使います。
  - private `get_drag_accel(double v) const`
    - 空力マップまたは `cd_a` からドラッグ減速度を返します。
  - private `perf_`
    - 性能設定です。
- `compute_lap_time(const std::vector<VelocityPoint>& velocity_profile)`
  - 区間平均速度で `ds/v_avg` を積算し、総所要時間を返します。
- `generate_gg_diagram(const VehiclePerformance& perf, int n_points = 50)`
  - 前後非対称の加減速限界を楕円近似し、`(ax,ay)` 点群を返します。
