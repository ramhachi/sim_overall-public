# `matlab/save_suspension_geometry_artifact.m`

## 役割

サスペンション関連ステージの保存処理を 1 箇所へ寄せる helper です。front / rear A-arm や steering の更新結果を、既存 vehicle artifact を壊さずに merge して同じ JSON へ保存します。

## 入力

- `BaseArtifactPath`
- `OutputPath`
- `CurrentParams`
- `SuspensionGeometry`
- `StageName`
- `StageOptimizer`
- `StageResult`

## 処理の流れ

1. `OutputPath` 必須チェック。
2. baseline template を作る。
3. `BaseArtifactPath` があれば `load_vehicle_artifact()` で既存 artifact を読む。
4. baseline + 既存 params + `CurrentParams` を `merge_nested_structs()` で統合。
5. `SuspensionGeometry` があれば `params.suspension_geometry` 配下へ merge。
6. `stageEntry` を組み、`build_vehicle_artifact(..., 'stage_entry', stageEntry)` を呼ぶ。
7. 出力ディレクトリを必要なら作り、`save_vehicle_artifact()` で保存。

## 重要点

- 保存対象は sidecar ではなく **同じ車両 artifact** です。
- `optimization.stages[]` に append されるので、設計履歴をステージ単位で追えます。

## システム上の位置づけ

artifact-first サスペンション設計の保存規約を強制する共通窓口です。
