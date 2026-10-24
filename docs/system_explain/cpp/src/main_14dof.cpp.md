# `cpp/src/main_14dof.cpp`

## 1. ファイルの目的
14DOF フルビークルモデル `Vehicle14DOF_New` を用いた代表試験（ステップステア、定常円旋回、加速、旋回加速）を切り替えて実行する総合デモです。

## 2. 実行上の役割
- 14DOF 実装の代表的な挙動確認用エントリポイントです。
- エンジントルク、ギア、後輪差速、タイヤ力の挙動を含む統合確認に使われます。

## 3. 依存関係
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `VehicleParams::default_fsae()`
- `Vehicle14DOF_New`, `State14DOF`, `DriverInput`
- `spdlog`, `<filesystem>`, `<chrono>`, `<fstream>`, `<cmath>`

## 4. 入力/出力
- 入力: 第1引数で試験種別を選択
  - `step_steer`
  - `circle`
  - `acceleration`（デフォルト）
  - `cornering`
- 出力:
  - `output/14dof/step_steer_result.csv`
  - `output/14dof/steady_circle_result.csv`
  - `output/14dof/acceleration_result.csv`
  - `output/14dof/cornering_acceleration_result.csv`
  - 標準ログに試験サマリ、加速試験ではシフトアップや 75 m 到達時刻

## 5. 関数・メソッド・クラスの網羅的な説明

### `ensure_output_path(const std::string& filename)`
- `output/14dof` ディレクトリを作成し、その配下の完全パス文字列を返します。

### `struct SimData`
- 14DOF 試験の記録用コンテナです。
- 主メンバ:
  - 基本時系列: `time`, `x`, `y`, `psi`, `vx`, `vy`, `r`, `ax`, `ay`
  - 入力: `steering`, `throttle`, `brake`, `gear`
  - パワートレイン: `engine_rpm`, `engine_torque`, `omega_RL`, `omega_RR`, `omega_diff`
  - 各輪補助: `Fz`, `slip_angle`
- 備考: 現状 `Fz` と `slip_angle` には実値ではなくゼロ配列を入れている箇所があります。

#### `SimData::reserve(size_t n)`
- 全ベクタに事前容量を確保し、長時間試験の push_back コストを下げます。

#### `SimData::save_csv(const std::string& filename)`
- 収集済みデータを 1 行 1 サンプルで CSV 出力します。
- 列には姿勢・速度・制御入力・エンジン情報・後輪回転差・各輪補助データが含まれます。

### `run_step_steer_test()`
- 20 m/s、3 速固定、1 秒後 0.1 rad のステップ舵角を与える試験です。
- 初期区間と操舵入力直後では `get_tire_Fx/Fy/Fz()` をログ出力してデバッグします。
- 毎ステップ記録し、`step_steer_result.csv` を生成します。

### `run_steady_state_circle_test()`
- 10 m/s から開始し、目標半径 15 m を維持する定常円旋回試験です。
- `atan(wheelbase / target_radius)` による一定操舵角を使います。
- 100 ms ごとに間引いて記録し、`steady_circle_result.csv` を生成します。

### `run_acceleration_test()`
- 2 速・5 m/s 開始の全開直進加速試験です。
- `shift_rpm = 13000 rpm` を超えると 6 速まで自動シフトアップします。
- 10 ms ごとに記録し、`acceleration_result.csv` を保存します。
- ログでは最終ギア、最終速度、走行距離、最終回転数、75 m 加速タイムを表示します。

### `run_cornering_acceleration_test()`
- 初速 10 m/s、初期ヨーレートありの状態から、一定舵角 `0.15 rad` を維持しつつ 1 秒で全開へ増やす旋回加速試験です。
- LSD 効果や左右後輪速度差の観察を主目的とします。
- 10 ms ごとに記録し、`cornering_acceleration_result.csv` を出力します。

### `main(int argc, char** argv)`
- 試験種別を引数から選択して対応関数を呼び分けます。
- 引数未指定時は `acceleration` を実行します。
- 不正な引数では利用可能な試験名をログに表示するだけで終了します。

