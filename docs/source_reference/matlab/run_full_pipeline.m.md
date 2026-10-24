# `matlab/run_full_pipeline.m`

## 概要

MATLAB 側の統合実行パイプラインです。トラック条件、車両 artifact、タイヤモデル、PID / RL ドライバー、DRS、POU 最適化有無を受け取り、最終的に `laptime_sim` 実行・結果保存・可視化までを一括で回します。

## 主な入力

- `track_type`, `radius`
- `custom_params_file`
- `driver_mode` (`pid` / `rl`)
- `rl_actor_dir`
- `drs_enabled`
- `tire_model_family`, `tire_model_name`
- `optimize`
- `pou_optimize`
- `visualize`

## 主な出力

- `output/runtime/params_run.json` / `params_drs_run.json`
- `output/laptime/laptime_sim_<track>.csv`
- `output/reports/report_<track>_*.txt`

## 重要な挙動

- runtime 用 params JSON も raw struct ではなく **vehicle artifact** として保存します。
- `pou_optimize=true` の場合、POU 最適化を先行実行し、同じ車両 artifact を更新します。
- RL モードでは `--rl-actor-dir` と `--params_file` を付けて Docker 内 `laptime_sim` を実行します。
- 可視化時は `visualize_laptime_sim(output_csv_host, track_type, radius)` を呼びます。

## システム上の役割

EasyStart のシミュレーション実行メニューが最終的に到達する MATLAB 実行エンジンです。
