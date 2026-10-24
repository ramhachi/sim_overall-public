# `matlab/optimize_multi_event.m`

## 1. ファイルの目的

スキッドパッドとアクセラレーションを同時に考慮し、共有コンセプト変数とイベント別セットアップ変数を GA で最適化するスクリプトです。

## 2. 主な入出力または依存関係

- 入力: スクリプト内で定義する基準タイム、重み、境界、GA 設定
- 出力:
  - `optimized_accel_new_engine.json`
  - 標準出力への選択空力パッケージと結果
- 依存関係:
  - `VehicleSimRunner`
  - `ConceptTradeOffs`
  - `build_extended_vehicle_tunable_specs`
  - `create_vehicle_baseline_template`
  - `apply_tradeoff_design`
  - `build_vehicle_artifact`
  - `save_vehicle_artifact`

## 3. 処理の流れ / アルゴリズム

1. 共有変数、共有追加 spec、イベント別 15 変数×2 の設計空間を定義します。
2. 空力パッケージ ID だけ整数制約 `int_con=8` を与えて GA を実行します。
3. `blend_cost()` が共有部とイベント別部を分割し、skid / accel 用の 2 台分の `params` を作ります。
4. 重みが 0 でないイベントだけ実シミュレーションし、失敗時は大きなペナルティを返します。
5. 最適解を再評価し、artifact に最適化履歴とイベント別 params / result を保存します。

## 4. ファイル内の全定義

### スクリプト本体

- 共有部には質量、前後重量配分、重心高、ホイールベース、前後トレッド、ファイナル、空力パッケージ ID が含まれます。
- `w_skid=0`, `w_accel=1` に設定されており、現状は実質アクセラレーション重視です。
- 保存 artifact の `primary_params_event` は `acceleration` に固定されています。

### `blend_cost(x, base_params, runner, config, ref_skid, ref_accel, w_skid, w_accel, design_context)`

- 共有変数長を `design_context.shared_extra_specs` に応じて可変にし、残りを 2 イベントへ等分します。
- `apply_tradeoff_design()` で `params_skid` と `params_accel` を生成し、必要なイベントだけ `runner.run()` します。
- 失敗時は `make_failure_result()` / `normalize_failure_result()` で結果 struct を整えつつ cost=1000 を返します。
- 成功時は `res_skid/ref_skid` と `res_accel/ref_accel` の重み付き和を返します。

### `make_failure_result(event_name, lap_time, reason)`

- シミュレーション済みだが失敗したイベントを表す最小 struct を返します。

### `make_reference_result(event_name, lap_time, reason)`

- 重み 0 などで未実行だったイベントを「参照値のみ」の struct として返します。

### `normalize_failure_result(raw_result, event_name, default_lap_time, reason)`

- 既存の失敗結果へ最低限の `name`, `lap_time`, `simulated`, `success`, `lap_completed`, `reason` を補います。

## 5. 注意点・前提条件

- `n_tunable = (length(x) - n_shared) / 2` 前提なので、設計変数総数は共有部を除いて偶数でなければなりません。
- 現状重み設定では skidpad はほぼ監視対象で、目的関数への寄与はありません。
- `save_vehicle_artifact()` に保存される主 params は accel 側です。
