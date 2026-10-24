# `frame_optimize/examples/run_frame_optimization.m`

## ファイル目的

- フレーム最適化サブシステムの **interactive wrapper**。
- 実処理は持たず、対話入力を受けて `run_canonical_frame_optimization.m` を呼ぶ。

## スクリプトの流れ

- `vehicle_params_file` を受け取る
- 目標重量を対話入力で受ける
- SolidWorks マクロ出力の有無を聞く
- Onshape 3D スケッチ用 FeatureScript 出力の有無を聞く
- `run_canonical_frame_optimization(...)` を呼ぶ
- 返ってきた `output_path` を `frame_optimization_output_path` として base workspace に戻す

## 入力

- 任意で `vehicle_params_file`
- 目標重量
- SolidWorks マクロ出力フラグ
- Onshape FeatureScript 出力フラグ

## 出力

- `frame_optimization_output_path`
- canonical API が生成する図・artifact・任意マクロ

## 依存関係

- `run_canonical_frame_optimization`

## 前提・注意点

- 対話入力つきの wrapper なので、自動化や EasyStart 以外の利用では canonical API を直接呼ぶ方が適しています。
- 旧 launcher 互換のため、結果パスだけ base workspace に戻します。
