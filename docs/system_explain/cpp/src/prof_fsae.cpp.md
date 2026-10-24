# `cpp/src/prof_fsae.cpp`

## 1. ファイルの目的
14DOF シミュレータの 1 ms 物理ステップを大量実行し、RL 1 ステップ相当の計算時間を計測する簡易プロファイラです。

## 2. 実行上の役割
- 学習サーバーや RL 推論時のスループット見積もりに使います。
- 20 サブステップを 1 RL ステップとみなす前提の性能確認を担います。

## 3. 依存関係
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `fsae/vehicle/vehicle_params.hpp`
- `<chrono>`, `<iostream>`

## 4. 入力/出力
- 入力: CLI 引数なし。
- 出力:
  - 総物理ステップ時間
  - `20` サブステップあたりの平均時間

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 処理:
  1. 既定 `VehicleParams` で `Vehicle14DOF_New` を生成
  2. 初速 5 m/s を設定
  3. `steering=0`, `throttle=0.5`, `brake=0` の一定入力を設定
  4. `num_steps = 1000 * 20` 回、`vehicle.step(0.001, input)` を実行
  5. 総計測時間と、1000 RL ステップ換算の平均時間を表示

