# `matlab/run_full_pipeline.m`

## 1. 役割

MATLAB 側の通常ラップタイム運用における実行エンジンです。EasyStart のシミュレーションメニューは最終的にこの関数を呼び、車両 artifact 読込、runtime params 生成、必要なら POU 最適化、`laptime_sim` 実行、レポート生成、可視化までを一括で扱います。

## 2. 今回重要になった変更点

- runtime 用 `params_run.json` / `params_drs_run.json` も **vehicle artifact** として保存するようになりました。
- `pou_optimize=true` のとき、`matlab/pou_hardpoint_design/` を呼んで **同じ車両 artifact を更新**します。
- 可視化呼び出しは `visualize_laptime_sim(output_csv_host, track_type, radius)` に統一され、トラック種別の取り違えを防ぎます。

## 3. ベース車両の決め方

`custom_params_file` が指定されればそれを優先し、未指定時は `optimized_<track>.json` 系を見ます。存在しなければ baseline template にフォールバックしますが、存在する場合は `load_vehicle_artifact()` で読んで `base_artifact.params` を使います。

重要なのは、ここで選ばれるのが「単なる params struct」ではなく、**stage 履歴を持つ artifact 全体**になった点です。

## 4. runtime params 生成

### 4.1 DRS あり

DRS 有効時は

1. `base` を複製
2. 空力係数と重量を補正
3. `drs.enabled = true` を付ける
4. `build_vehicle_artifact(..., 'base_artifact', base_artifact, 'stage_entry', drs_stage)` を呼ぶ
5. `params_drs_run.json` へ保存

という流れです。

### 4.2 DRS なし

DRS 無しでも、既存 artifact を使う場合や PINN を使う場合は runtime 用 JSON を生成します。これも raw `jsonencode(base)` ではなく `build_vehicle_artifact()` を経由します。

## 5. POU 最適化統合

`opt.pou_optimize` が真なら、Phase 0.5 として `run_all_wheels(pou_params_file, pou_opts)` を呼びます。

ここでのポイント:

- `pou_params_file = opt_file`
- `export_json = false`
- `update_params = true`

となっており、POU の結果は sidecar JSON ではなく **元 artifact へ追記**されます。

## 6. RL 実行との関係

RL モードでは:

1. actor export directory を host / container 両方で解決
2. 必要ファイル (`rl_actor_weights.txt` など) を検証
3. `--driver rl --rl-actor-dir ... --params_file ...` を付けて `laptime_sim` を実行

します。つまりこの関数は、MATLAB 学習済み方策を C++ runtime に接続する運用側入口でもあります。

## 7. 出力

- `output/runtime/params_run.json` または `params_drs_run.json`
- `output/laptime/laptime_sim_<track>.csv`
- `output/reports/report_<track>_<timestamp>.txt`

## 8. システム上の位置づけ

EasyStart が UI ハブだとすると、この関数は MATLAB 側のオーケストレーション本体です。SSOT の観点では、**設計 artifact を読み、runtime 派生 artifact を生成し、C++ 実行へ渡す橋**になっています。
