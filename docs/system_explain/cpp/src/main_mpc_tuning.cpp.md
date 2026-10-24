# `cpp/src/main_mpc_tuning.cpp`

## 1. ファイルの目的
MPC の重みと自転車モデル剛性パラメータをグリッドサーチし、横偏差性能の良い設定を探す自動チューニング用プログラムです。

## 2. 実行上の役割
- Phase 8c の実験入口です。
- 比較プログラムに埋め込む MPC 重みの探索元として機能します。

## 3. 依存関係
- `fsae/driver/pid_driver.hpp`
- `fsae/driver/mpc_driver.hpp`（`HAS_OSQP` 時）
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `fsae/track/track_model.hpp`
- `fsae/laptime/velocity_profile.hpp`
- `<algorithm>`, `<fstream>`, `<iomanip>`

## 4. 入力/出力
- 入力: CLI 引数なし。
- 出力:
  - 標準出力に進捗とトップ 10 設定
  - `mpc_best_config.txt` に最良設定を書き出し
- `HAS_OSQP` 無効時は即エラー終了

## 5. 関数・メソッド・クラスの網羅的な説明

### `to_vehicle_state(const vehicle::State14DOF& state14)`
- 14DOF 状態を MPC ドライバー用 `VehicleState` に変換します。

### `struct TuningResult`
- 1 設定の探索結果です。
- パラメータ:
  - `w_x`, `w_y`, `w_psi`, `w_delta_rate`
  - `Cf`, `Cr`
- 性能:
  - `lateral_error_rms`, `lateral_error_max`
  - `steering_rate_rms`
  - `lap_time`
  - `avg_solve_time_ms`
  - `solve_failures`
  - `converged`

### `run_mpc_tuning(...)`
- `HAS_OSQP` 時のみ存在する、1 設定を評価する関数です。
- 主な処理:
  1. `MPCPathFollowingDriver` を生成
  2. 14DOF 車両を円形トラック上に配置
  3. 30 秒以内でラップ成立を判定
  4. 10 ms ごとに横偏差と操舵レートを集計
  5. `driver.stats()` から平均求解時間と失敗回数を取得
- 例外発生時は `converged=false` のまま返します。

### `main()`
- 円形トラックと速度プロファイルを生成します。
- PID を目標性能の目安として表示します。
- `w_xy_candidates`, `w_delta_rate_candidates`, `Cf_candidates` の 3 重ループで探索します。
- 成功設定のみ保持し、横偏差 RMS 昇順でソート後、上位 10 件を表示します。
- 最良設定を `mpc_best_config.txt` に C++ 記法で保存します。

