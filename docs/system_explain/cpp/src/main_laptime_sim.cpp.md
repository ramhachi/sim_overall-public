# `cpp/src/main_laptime_sim.cpp`

## 1. ファイルの目的
トラック生成、速度プロファイル生成、PID/RL ドライバー、14DOF 車両モデルを統合し、1 周または複数周のフル過渡ラップタイムを評価する実運用寄りの入口です。

## 2. 実行上の役割
- ラップタイム評価の標準 C++ エントリポイントです。
- PID ベースの既存経路追従と RL アクタ推論の双方を同一車両モデル上で実行できます。
- CSV ログ生成、JSON 出力、車両パラメータ注入までこの 1 本で担います。

## 3. 依存関係
- `fsae/driver/pid_driver.hpp`
- `fsae/driver/rl_driver.hpp`
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `fsae/track/track_model.hpp`
- `fsae/track/waypoint_loader.hpp`
- `fsae/laptime/velocity_profile.hpp`
- `VehicleParams::from_json(...)`, `from_json_string(...)`
- `<filesystem>`, `<memory>`, `<algorithm>`, `<stdexcept>`

## 4. 入力/出力
- 主な CLI 引数:
  - `--track`, `--radius`, `--time`, `--laps`
  - `--gain-preset`
  - `--driver pid|rl`
  - `--rl-actor-dir`
  - `--params_file`, `--params-json`
  - `--output`
  - `--output_json`
  - `--initial-speed`
  - 個別ゲイン: `--kp-steer`, `--kd-steer`, `--ki-steer`, `--kh-steer`, `--kff-steer`, `--kp-speed`, `--ki-speed`, `--max-throttle`, `--max-steer`, `--brake-gain-ratio`, `--corner-throttle-reduction`
- 出力:
  - デフォルト CSV: `laptime_sim_<track>.csv`
  - `--output` 指定時はそのファイル
  - `--output_json` 指定時は JSON を標準出力
  - 通常モードではラップ完了可否・ラップタイム・最大横偏差を標準出力

## 5. 関数・メソッド・クラスの網羅的な説明

### `struct SimulationConfig`
- ラップタイムシミュレーション全体の設定を保持します。
- 主要メンバ:
  - トラック関連: `track_name`, `track_radius`, `track_width`
  - 時間関連: `simulation_time`, `simulation_time_explicit`, `dt`, `output_rate`, `laps`
  - 車両性能: `max_lat_accel`, `max_lon_accel`, `max_lon_decel`, `max_speed`, `initial_speed`
  - ドライバー関連: `gain_preset`, `driver_mode`, `rl_actor_dir`
  - 個別 PID ゲイン上書き群
  - 車両パラメータ注入: `params_file`, `params_json_str`
  - 出力制御: `output_json_result`, `output_file`

### `resolve_autocross_csv()`
- オートクロス CSV の探索関数です。
- 優先順:
  1. 環境変数 `FOPT_AUTOCROSS_CSV`
  2. 既知候補パス群
  3. 既定の `/workspace/sim/data/ASE_autocross_sim.csv`

### `struct LapResult`
- シミュレーション結果の要約。
- メンバ:
  - `lap_time`, `distance_traveled`
  - `max_lateral_error`, `avg_lateral_error`
  - `max_speed`, `avg_speed`
  - `lap_completed`

### `create_track(const SimulationConfig& config)`
- `config.track_name` からトラックを構築します。
- 対応:
  - `circle`, `circle_cw`, `figure_eight`, `skidpad`, `straight_corner`, `acceleration/accel`, `autocross/ASE_autocross`
  - `.csv` を末尾に持つファイル名
- CSV/autocross の読み込み失敗は `DiagnosticFailure` で fail-fast します。未知のトラック名のみ円形トラックへフォールバックします。

### `select_gains(const SimulationConfig& config)`
- PID ゲインを選定・上書きする関数です。
- プリセット:
  - `conservative`
  - `default`
  - `aggressive`
  - `optimized_figure_eight`
  - `optimized_skidpad`
  - `optimized_autocross`（CSV/autocross 系の自動選択でも使用）
- `use_custom_gains` が真なら個別数値で上書きします。
- 車両パラメータのトルク操舵モード反映は `run_laptime_simulation()` 側で追加実施されます。

### `run_laptime_simulation(const SimulationConfig& config)`
- このファイルの中心処理です。
- 主な処理:
  1. トラック生成
  2. 車両パラメータ読込
  3. `VehiclePerformance` を組み立て速度プロファイル生成
  4. エアロマップ設定・スキッドパッド用制限を反映
  5. PID または RL ドライバーを初期化
  6. `Vehicle14DOF_New` を生成し座標系をモード別に初期配置
  7. CSV ヘッダ出力
  8. `vehicle.step_adaptive(config.dt, input)` で時間発展
  9. Frenet 座標でラップ完了判定
  10. 横偏差・速度・走行距離を集計
- PID モードでは MAP 座標と ISO-8855/NED 座標を相互変換します。
- RL モードでは学習サーバーと同じ MAP 座標をそのまま使用します。

#### ローカルラムダ `safe_json`
- `main()` 内で使われる NaN/Inf ガードです。
- JSON に非数値を出さないため、異常値を `0.0` に置換します。

### `main(int argc, char** argv)`
- すべての CLI 引数を逐次パースして `SimulationConfig` を構築します。
- プリセット名、`driver_mode`、`laps` の妥当性を検証します。
- `run_laptime_simulation(config)` 実行後、
  - JSON モードなら JSON を出力
  - 通常モードなら要約を表示

