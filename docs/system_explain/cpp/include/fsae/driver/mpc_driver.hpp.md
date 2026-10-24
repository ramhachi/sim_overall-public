# mpc_driver.hpp

## 1. ファイルの目的
sampling-based preview MPC 経路追従ドライバーを定義するヘッダです。

## 2. 実行上の役割
最近接点探索、操舵角・縦加速度候補の有限ホライズン予測、コスト評価、`DriverInput` 変換を 1 周期で実行します。

## 3. 依存関係
- `vehicle/vehicle_state.hpp`
- `track/track_model.hpp`, `track/track_distance.hpp`
- `laptime/velocity_profile.hpp`
- `control/mpc_problem.hpp`

## 4. 入力/出力
- 入力: `VehicleState`, 現在時刻
- 出力: `DriverInput`
- 内部保持: `hint_s_`, `prev_delta_`, `prev_ax_`, `stats_`

## 5. 関数・メソッド・クラスの網羅的な説明
### `struct MPCStats`
予測評価時間、失敗回数、評価 rollout 数を保持します。`avg_solve_time_ms()` で平均値を返します。

### `class MPCPathFollowingDriver`
- `MPCPathFollowingDriver(...)`: トラック、速度プロファイル、MPC 設定を保持
- `compute_control(...)`: 現状態から候補入力を予測評価し、最良の初期入力を `DriverInput` へ変換
- `stats()`: 統計取得
- `current_s()`: 最近接弧長取得
- `reset()`: 内部状態リセット
- `convert_to_driver_input(...)`: `(delta, ax)` を `DriverInput` へ変換
