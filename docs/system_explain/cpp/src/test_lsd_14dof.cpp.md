# `cpp/src/test_lsd_14dof.cpp`

## 1. ファイルの目的
14DOF 車両モデルに組み込まれた LSD パラメータを用い、旋回加速中の左右後輪速度差・エンジン回転数・車速推移を確認する統合デモです。

## 2. 実行上の役割
- LSD 単体ではなく、車両運動と結合した状態での挙動確認に使います。
- 低速コーナー立ち上がりを模した条件で、LSD 設定が後輪挙動へどう現れるかを見ます。
- CSV を残すため、後解析の入口にもなります。

## 3. 依存関係
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `VehicleParams`, `Vehicle14DOF_New`, `State14DOF`, `DriverInput`
- `vehicle.step()`, `vehicle.state()`
- `vehicle.get_engine_rpm()`, `vehicle.get_engine_torque()`
- 標準ライブラリ `iostream`, `iomanip`, `fstream`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - 車両諸元と LSD 条件はファイル内で明示設定
  - 初期状態:
    - `vx=10 m/s`
    - `vy=0`
    - `r=0.5 rad/s`
    - 全輪 `omega[i]=vx/tire_radius`
  - 入力:
    - `gear=2`
    - `steering=0.1 rad`
    - `brake=0`
    - `throttle` を 0.5 秒で 0→1 にランプ入力
  - `dt=0.001 s`
  - 総時間 `1.0 s`
- 出力:
  - `lsd_test_result.csv`
  - 標準出力に 0.1 秒ごとの状態表
  - 最終速度、左右輪速度差、ヨーレートを表示
  - 明示的 pass/fail 判定なし

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 主処理:
  1. `VehicleParams` に質量・寸法・慣性・LSD 条件を設定
  2. `Vehicle14DOF_New vehicle(params)` を生成
  3. `state().vx`, `state().vy`, `state().r` を直接設定
  4. 4 輪 `omega[i]` をタイヤ半径で初期速度に合わせる
  5. `lsd_test_result.csv` を開く
  6. `t=0`〜`1.0 s` を 1 ms 刻みで積分
  7. `throttle = min(t*2.0, 1.0)` でランプ入力を作る
  8. 0.1 秒ごとに後輪左右回転数差、エンジン回転数、車速を出力
  9. 最終状態を表示

#### `main()` が確認するシナリオ
- 低速コーナー立ち上がり加速
- 後輪左右角速度差が時間とともにどう変わるか
- エンジン回転数・トルク取得 API が車両状態と整合するか

#### CSV 列
- `time,throttle,vx,vy,r,omega_RL,omega_RR,omega_diff,engine_rpm,engine_torque`

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - `main()` 内の LSD 統合試験 1 本
