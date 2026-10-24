# `matlab/optimize_concept_design.m`

## 1. ファイルの目的

車両コンセプト段階の共有変数を GA で最適化し、スキッドパッドと円旋回評価を通じて速い概念仕様を artifact 化するスクリプトです。

## 2. 主な入出力または依存関係

- 入力: スクリプト内で定義する設計変数境界と GA 設定
- 出力:
  - `optimized_concept.json`
  - 標準出力への最適化結果
- 依存関係:
  - `VehicleSimRunner`
  - `build_extended_vehicle_tunable_specs`
  - `create_vehicle_baseline_template`
  - `compute_aero_cd_a`
  - `apply_vehicle_extra_tunables`
  - `build_vehicle_artifact`
  - `save_vehicle_artifact`

## 3. 処理の流れ / アルゴリズム

1. `VehicleSimRunner` を起動し、共有追加 spec を構築して設計変数境界を定義します。
2. `ga()` で `concept_cost_function()` を最小化します。
3. 最適ベクトル `x_opt` を `apply_vehicle_tradeoffs()` で物理車両パラメータへ変換します。
4. 最適パラメータで skidpad と circle を再実行し、イベント結果を正規化します。
5. `build_vehicle_artifact()` を複数回呼んで、段階情報とイベント結果を artifact に追記します。
6. `optimized_concept.json` として保存します。

## 4. ファイル内の全定義

### スクリプト本体

- 共有変数は総質量、重量配分、`CL_A`、エンジントルク倍率、空力バランス、および拡張 spec です。
- `EngineTorqueScale` は専用変数として持つため、共有追加 spec から一度除外しています。
- artifact には `stage_entry` と skidpad / circle 結果を別々に積み上げます。

### `concept_cost_function(x, base_params, runner, shared_extra_specs)`

- 設計ベクトルを車両パラメータへ変換し、skidpad と circle の 2 本を実行してラップタイム和を cost にします。
- クラッシュや DNF は 1000 の大きなペナルティへ置き換えます。

### `create_concept_base_params()`

- 現状は `create_vehicle_baseline_template()` をそのまま返す薄いラッパーです。

### `apply_vehicle_tradeoffs(base_params, x, shared_extra_specs)`

- 質量、前後重量配分、慣性、空力、エンジントルク倍率を概念レベルで反映します。
- 追加 spec があれば `apply_vehicle_extra_tunables()` でさらに上書きします。

### `normalize_event_result(raw_result, success, event_name)`

- event struct が空でも最低限 `name`, `success`, `lap_completed` を持つ形へ正規化します。

## 5. 注意点・前提条件

- 実行すると Docker ベースのシミュレーションを繰り返すため重い処理です。
- `apply_vehicle_tradeoffs()` は概念設計用の簡易慣性スケーリングを使っており、詳細サスペンション最適化とは別レイヤです。
