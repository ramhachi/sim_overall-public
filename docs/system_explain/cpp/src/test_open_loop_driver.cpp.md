# `cpp/src/test_open_loop_driver.cpp`

## 1. ファイルの目的
オープンループ系ドライバーモデル群（ステップ舵、正弦舵、ダブルレーンチェンジ、加減速、時系列入力）が `Vehicle14DOF_New` に対して期待通りの入力を生成できるかを確認するデモ集です。

## 2. 実行上の役割
- ドライバーモジュールのサンプル実行ファイルです。
- 各ドライバーが返す `DriverInput` を 14DOF 車両へ与え、結果 CSV を生成します。
- 後段の可視化やログ解析用データを作る役割もあります。

## 3. 依存関係
- `fsae/driver/open_loop_driver.hpp`
- `StepSteerDriver`, `SineSteerDriver`, `DoubleLaneChangeDriver`, `AccelBrakeDriver`, `TimeSeriesDriver`, `TimeSeriesPoint`
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `Vehicle14DOF_New`, `VehicleParams`
- 標準ライブラリ `iostream`, `fstream`, `iomanip`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - 各テストで固定ドライバー設定と固定初速を使用
  - 全ケース `dt=0.001 s`
- 出力:
  - `step_steer_result.csv`
  - `sine_steer_result.csv`
  - `double_lane_change_result.csv`
  - `accel_brake_result.csv`
  - `custom_timeseries_result.csv`
  - 標準出力に各 CSV 保存メッセージ
  - 明示的 pass/fail 判定なし

## 5. 関数・メソッド・クラスの網羅的な説明

### `test_step_steer()`
- `StepSteerDriver(1.0, 0.1, 10.0)` を使います。
- 初速 `15 m/s` の 14DOF 車両を 10 秒積分します。
- 10 ms ごとに `time,x,y,psi,vx,vy,r,steering,lateral_accel` を保存します。
- 確認内容:
  - 1 秒後のステップ舵角入力が想定通り出るか
  - そのときの横応答が取得できるか

### `test_sine_steer()`
- `SineSteerDriver(0.5, 0.08, 10.0)` を使います。
- 初速 `20 m/s` で 10 秒積分します。
- 出力列は `test_step_steer()` と同じです。
- 確認内容:
  - 周期操舵入力の生成と応答記録

### `test_double_lane_change()`
- `DoubleLaneChangeDriver(2.0, 0.12, 12.0)` を使います。
- 初速 `15 m/s`、12 秒試験です。
- 10 ms ごとに位置・姿勢・速度・操舵・横加速度を保存します。
- 確認内容:
  - 大きめ操舵マニューバの時系列生成

### `test_accel_brake()`
- `AccelBrakeDriver(4.0, 5.0, 10.0)` を使います。
- 初速 `5 m/s` で、加速からブレーキへ切り替わるシナリオを実行します。
- `prev_vx` との差分で `ax` を近似計算します。
- 10 ms ごとに `time,x,vx,ax,throttle,brake` を保存します。
- 確認内容:
  - スロットル・ブレーキ時系列生成と縦応答取得

### `test_custom_timeseries()`
- `std::vector<driver::TimeSeriesPoint>` をファイル内で定義します。
- 直進 → 左旋回 → 反対旋回 → 直進復帰 → 減速の流れです。
- `TimeSeriesDriver` で再生し、初速 `15 m/s` で積分します。
- 10 ms ごとに `time,x,y,psi,vx,vy,r,steering` を保存します。
- 確認内容:
  - 任意時系列マニューバの再生

### `main()`
- このファイルの入口です。
- 以下を順に実行します。
  1. `test_step_steer()`
  2. `test_sine_steer()`
  3. `test_double_lane_change()`
  4. `test_accel_brake()`
  5. `test_custom_timeseries()`

### このファイル内で新規定義される型・補助関数
- 関数:
  - `test_step_steer`
  - `test_sine_steer`
  - `test_double_lane_change`
  - `test_accel_brake`
  - `test_custom_timeseries`
  - `main`
- 独自 `class` / `struct`: なし
