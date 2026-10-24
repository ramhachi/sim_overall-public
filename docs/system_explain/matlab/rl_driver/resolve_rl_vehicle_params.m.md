# `matlab/rl_driver/resolve_rl_vehicle_params.m`

## 1. 役割

`resolve_rl_vehicle_params` は、RL 用の基準車両テンプレートを作り、必要なら呼び出し側から渡された上書き設定をマージする関数です。  
RL 環境で使う車両パラメータの「標準初期値」を一箇所で定めています。

## 2. `resolve_rl_vehicle_params(vehicleParams)` 詳細

処理の流れは非常に明快です。

1. `create_vehicle_baseline_template()` でベース車両を作る
2. RL 用の既定上書きとして `params.steering.torque_mode = true` を設定
3. `params.suspension.K_chassis = 2500.0` を設定
4. 引数が無い、または空ならそのまま返す
5. 引数があれば `merge_nested_structs(params, vehicleParams)` でネスト構造ごと上書き

## 3. この関数が担う RL 固有化

通常の車両テンプレートに対して、少なくとも以下を RL 用前提へ寄せています。

- **操舵入力は torque mode を既定にする**
- **フレームねじり剛性 `K_chassis` を 2500 に設定する**

つまり、RL 系コードが何も渡さなくても、学習実験向けの既定車両が得られます。

## 4. 依存関数との関係

- `create_vehicle_baseline_template()` : ベース構造体の供給元
- `merge_nested_structs()` : 呼び出し側 overrides の深いマージ

## 5. 実装上の要点

- 単なる pass-through ではなく、RL 系の既定値を差し込む関数です。  
- `get_rl_observation_bounds()`、`FSAEEnvironment` などはこの関数を前提にしているため、既定値変更の影響は広いです。
