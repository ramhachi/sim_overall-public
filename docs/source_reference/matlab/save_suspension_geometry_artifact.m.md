# `matlab/save_suspension_geometry_artifact.m`

## 概要

サスペンション逆解析・ステアリング最適化の結果を、**同じ車両 artifact JSON** へ追記保存する helper です。`params`、`params.suspension_geometry`、`optimization.stages[]` をまとめて更新します。

## 主な入力

- `BaseArtifactPath`
- `OutputPath`
- `CurrentParams`
- `SuspensionGeometry`
- `StageName`
- `StageOptimizer`
- `StageResult`

## 主な出力

- `result.output_path`
- `result.updated_existing_artifact`
- `result.params`

## システム上の役割

サスペンション系を artifact-first / SSOT へ揃える保存窓口です。
