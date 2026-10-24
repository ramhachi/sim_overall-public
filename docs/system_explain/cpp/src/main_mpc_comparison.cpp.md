# `cpp/src/main_mpc_comparison.cpp`

## 1. ファイルの目的
同一トラック・同一速度プロファイル上で PID ドライバーと MPC ドライバーの性能を比較する検証用プログラムです。

## 2. 実行上の役割
- Phase 8b の比較実験入口です。
- ラップタイム、横偏差、制御平滑性、MPC 求解時間をまとめて比較します。

## 3. 依存関係
- `fsae/driver/pid_driver.hpp`
- `fsae/driver/mpc_driver.hpp`（`HAS_OSQP` 時のみ）
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `fsae/track/track_model.hpp`
- `fsae/laptime/velocity_profile.hpp`

## 4. 入力/出力
- 入力: 第1引数でトラック名（既定は `circle`）
- 出力:
  - 標準出力に比較表
  - `mpc_comparison_<track>.csv`
- `HAS_OSQP` 無効時はエラーメッセージを出して終了コード `1`

## 5. 関数・メソッド・クラスの網羅的な説明

### `to_vehicle_state(const vehicle::State14DOF& state14)`
- 14DOF 状態を、MPC ドライバーが要求する `VehicleState` へ転写します。

### `struct SimResult`
- 1 回の比較走行結果を保持します。
- 主な指標:
  - ラップ関連: `driver_name`, `lap_time`, `distance_traveled`, `lap_completed`
  - 横偏差: `max_lateral_error`, `rms_lateral_error`, `avg_lateral_error`
  - 速度: `max_speed`, `avg_speed`, `rms_speed_error`
  - 制御平滑性: `max_steering_rate`, `rms_steering_rate`, `max_throttle_rate`, `rms_throttle_rate`
  - MPC 専用: `avg_solve_time_ms`, `max_solve_time_ms`, `solve_fail_count`
  - ログ列群: `time_log` ほか

### `struct ComparisonConfig`
- 比較実験の共通設定。
- トラック形状、時間刻み、出力周期、最大横/縦加速度、最大速度、初速を保持します。

### `create_track(const ComparisonConfig& config)`
- `circle` と `figure_eight` を切り替えてトラックを生成します。
- 未知名は `circle` にフォールバックします。

### `template<typename T> compute_rms(const std::vector<T>& data)`
- RMS を返す汎用ヘルパーです。
- 空配列なら `0.0` を返します。

### `template<typename T> compute_max_abs(const std::vector<T>& data)`
- 最大絶対値を返す汎用ヘルパーです。

### `run_with_pid(...)`
- PID ドライバーで 14DOF 車両を走らせ、比較指標を集計します。
- ゴールテープ判定、走行距離積算、Frenet による横偏差評価、入力レート計算を行います。

### `run_with_mpc(...)`
- `HAS_OSQP` 有効時のみ存在します。
- `control::MPCConfig` を固定値で構成し、40 Hz で `driver.compute_control(vs, t)` を実行します。
- `Vehicle14DOF_New` は 1 kHz で積分し、求解時間統計は `driver.stats()` から回収します。

### `print_comparison(const SimResult& pid_result, const SimResult& mpc_result)`
- ラップタイム、横偏差、操舵レート、MPC 求解時間を整形表示します。
- PID 比の改善率も計算します。

### `save_results_csv(...)`
- PID と MPC の時系列ログを同じ時刻軸で横並び保存します。
- 列:
  `time,pid_lateral_error,mpc_lateral_error,pid_steering,mpc_steering,pid_throttle,mpc_throttle`

### `main(int argc, char** argv)`
- トラック生成、速度プロファイル生成、PID 実行、MPC 実行、比較表示、CSV 保存を順に行います。
- `HAS_OSQP` なし構成では代替 `main()` がエラー終了します。

