# `cpp/src/main_6dof.cpp`

## 1. ファイルの目的
6DOF 車両モデルのステップステア応答を上下・姿勢自由度込みで確認するデモです。

## 2. 実行上の役割
- `Vehicle6DOF` の平面運動に加えて `z`, `phi`, `theta` を含む安定性確認を行います。
- 3DOF より高次なモデルの発散検知にも使われます。

## 3. 依存関係
- `fsae/vehicle/vehicle_6dof.hpp`
- `VehicleParams::default_fsae()`
- `Vehicle6DOF`, `DriverInput`
- `spdlog`, `<fstream>`, `<chrono>`, `<cmath>`

## 4. 入力/出力
- 入力: CLI 引数は受け取るが未使用。
- 出力:
  - `6dof_step_steer_result.csv`
  - 標準ログに平衡車高、最終位置、姿勢角、速度、ヨーレート
- 異常時は終了コード `1`、正常時は `0`

## 5. 関数・メソッド・クラスの網羅的な説明

### `main(int argc, char** argv)`
- このファイルの唯一の関数です。
- 主処理:
  1. `Vehicle6DOF` を生成し `z_equilibrium()` を表示
  2. 初速 15 m/s を設定
  3. 5 秒間、1 秒後から 0.1 rad のステップ舵角を印加
  4. `ax`, `ay` を有限差分で算出して CSV 出力
  5. `vehicle.step(dt, input)` で積分
  6. `vx` 異常、`z` 異常、`phi` 異常で発散判定
- CSV 列:
  `time,x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,ax,ay`

