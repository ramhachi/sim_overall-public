# `matlab/find_vehicle_json_candidates.m`

## 概要

利用可能な車両 JSON / artifact を探索し、EasyStart や各最適化フローがそのまま使える候補一覧へ正規化する関数です。`matlab/`、`matlab/parameter_presets/`、`output/runtime/` を再帰探索し、artifact 読み込み・候補判定・表示用 summary 生成までを一括で行います。

## 主な入力

- `projectRoot`
- `RequireExportReady`
- `RequireSuspensionHardpoints`
- `RequiredHardpointCorners`

## 主な出力

- `candidates`
  - `path`
  - `label`
  - `group`
  - `is_optimized`
  - `has_suspension_hardpoints`
  - `summary`
  - `params`

## 重要な挙動

- `load_vehicle_artifact()` で legacy / artifact 差を吸収します。
- `params_run.json` / `params_drs_run.json` のような一時 runtime JSON は候補から除外します。
- `RequiredHardpointCorners` により、frame 用の `FL` / `RL` 必須選別や steering 用の `FL` 必須選別ができます。

## システム上の役割

車両 artifact 選択の SSOT レイヤです。EasyStart、frame、steering、suspension、AC export の候補選定をこの関数に集約します。
