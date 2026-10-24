# `cpp/src/main_8dof.cpp`

## 1. ファイルの目的
8DOF モデルのステップステア試験を行い、ばね下 4 輪自由度を含む挙動を CSV に保存するデモです。

## 2. 実行上の役割
- `Vehicle8DOF` の鉛直/姿勢/車輪上下挙動をまとめて検証します。
- 6DOF から 8DOF への拡張で、各輪の `z_u`・`vz_u` が安定しているかを見るための入口です。

## 3. 依存関係
- `fsae/vehicle/vehicle_8dof.hpp`
- `VehicleParams::default_fsae()`
- `Vehicle8DOF`, `DriverInput`
- `spdlog`, `<fstream>`, `<iomanip>`, `<cmath>`

## 4. 入力/出力
- 入力: CLI 引数は無視されます。
- 出力:
  - `8dof_step_steer_result.csv`
  - 標準ログに平衡ばね上高さ、最終位置、速度、ヨーレート
- 発散時の戻り値は `1`

## 5. 関数・メソッド・クラスの網羅的な説明

### `main(int, char**)`
- このファイルの唯一の関数です。
- 主処理:
  1. デフォルト FSAE パラメータで `Vehicle8DOF` を生成
  2. 平衡 `z_sprung` をログ表示
  3. 初速 15 m/s を設定
  4. 5 秒間、1 秒後から 0.1 rad を与えるステップステア
  5. 1 ms ごとに車体 6 状態と各輪 `z_u`, `vz_u` を CSV 化
  6. `z` や `theta` の異常値で停止
- CSV 列:
  `time,x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,z_u0..3,vz_u0..3`

