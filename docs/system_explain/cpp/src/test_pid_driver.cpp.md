# `cpp/src/test_pid_driver.cpp`

## 1. ファイルの目的
PID 経路追従ドライバー `PIDPathFollowingDriver` を、複数トラック・複数ゲイン・速度制御条件で動かし、ステア・スロットル・ブレーキの閉ループ制御が成立するかを確認するデモ集です。

## 2. 実行上の役割
- ドライバー、トラック、速度プロファイル、14DOF 車両をまとめてつなぐ結合テストです。
- 使用例としても回帰確認としても使えます。
- CSV 出力により、追従誤差や速度誤差を後解析できます。

## 3. 依存関係
- `fsae/driver/pid_driver.hpp`
- `PIDPathFollowingDriver`, `PIDGains`, `conservative_gains()`, `aggressive_gains()`
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `fsae/track/track_model.hpp`
- `fsae/laptime/velocity_profile.hpp`
- `VehiclePerformance`, `VelocityProfileGenerator`
- 標準ライブラリ `iostream`, `fstream`, `iomanip`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - 円追従、8 の字追従、ゲイン比較、速度制御の 4 ケース
  - いずれも固定トラック・固定性能パラメータ
  - 全ケース `dt=0.001 s`
- 出力:
  - `pid_circle_tracking.csv`
  - `pid_figure_eight_tracking.csv`
  - `pid_speed_control.csv`
  - 標準出力に最終誤差、走行距離、ゲイン比較結果
  - 明示的 pass/fail 判定なし

## 5. 関数・メソッド・クラスの網羅的な説明

### `test_pid_circle_tracking()`
- 円トラック `create_circle_track(30.0, 3.0)` を使います。
- `VehiclePerformance` から速度プロファイルを生成します。
- `PIDPathFollowingDriver(track, velocity_profile)` を生成します。
- 車両初期位置を円の開始点 `(30,0)`、方位 `π/2` に設定します。
- 5 秒積分し、10 ms ごとに
  - `x,y,psi`
  - `vx,vy,r`
  - `steering,throttle,brake`
  - `s,n,d_heading`
  - `target_v,actual_v`
  を CSV 保存します。
- 確認内容:
  - 基本的な円追従

### `test_pid_figure_eight_tracking()`
- `create_figure_eight_track(25.0, 3.0)` を使用します。
- 初期位置は原点・東向きです。
- 10 秒積分し、10 ms ごとに `s,n,d_heading,steering,speed,target_speed` を保存します。
- 確認内容:
  - 曲率が入れ替わる複雑コースでの追従

### `test_pid_gain_tuning()`
- 円トラックを用い、3 種のゲインセットを比較します。
- 比較対象:
  - `Conservative`
  - `Default`
  - `Aggressive`
- 各ゲインで 3 秒積分し、最初の 1 秒を除いた区間の
  - 最大横偏差
  - 平均横偏差
  を集計表示します。
- 確認内容:
  - ゲインによる追従性の違い

### `test_pid_speed_control()`
- `create_straight_corner_track(50.0, 15.0, 3.0)` を使用します。
- 0.5 m 間隔の速度プロファイルを作り、初速 `10 m/s` から 10 秒積分します。
- 10 ms ごとに `s,speed,target_speed,throttle,brake,speed_error` を保存します。
- 確認内容:
  - 速度目標に対する縦制御の挙動

### `main()`
- このファイルの入口です。
- 以下を順に実行します。
  1. `test_pid_circle_tracking()`
  2. `test_pid_figure_eight_tracking()`
  3. `test_pid_gain_tuning()`
  4. `test_pid_speed_control()`

### このファイル内で新規定義される型・補助関数
- 関数:
  - `test_pid_circle_tracking`
  - `test_pid_figure_eight_tracking`
  - `test_pid_gain_tuning`
  - `test_pid_speed_control`
  - `main`
- 独自 `class` / `struct`: なし
