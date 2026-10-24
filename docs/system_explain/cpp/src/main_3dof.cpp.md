# `cpp/src/main_3dof.cpp`

## 1. ファイルの目的
3DOF 平面車両モデルのステップステア応答を確認するデバッグ用デモです。

## 2. 実行上の役割
- `Vehicle3DOF` の最小運動モデルが破綻せず走るかを確認します。
- 1 ms ごとの状態 CSV を残し、横運動の基礎挙動確認に使われます。

## 3. 依存関係
- `fsae/vehicle/vehicle_3dof.hpp`
- `VehicleParams::default_fsae()`
- `Vehicle3DOF`, `DriverInput`
- `spdlog`, `<fstream>`, `<chrono>`, `<cmath>`

## 4. 入力/出力
- 入力: CLI 引数は受け取るが未使用。
- 出力:
  - `3dof_step_steer_result.csv`
  - 標準ログに終了時の位置、ヨー角、速度、ヨーレート

## 5. 関数・メソッド・クラスの網羅的な説明

### `main(int argc, char** argv)`
- このファイルの唯一の関数です。
- 処理内容:
  1. デフォルト FSAE パラメータで `Vehicle3DOF` を構築
  2. 初速 15 m/s を設定
  3. 5 秒間、1 秒後に 0.1 rad のステップ舵角を付与
  4. 毎ステップで `vx`, `vy` の差分から `ax`, `ay` を近似計算
  5. `vehicle.step(dt, input)` で積分
  6. `NaN` や異常速度で発散判定
- CSV 列:
  `time,x,y,psi,vx,vy,r,ax,ay,steering`
- `argc`, `argv` は将来拡張用で、現状は未使用です。

