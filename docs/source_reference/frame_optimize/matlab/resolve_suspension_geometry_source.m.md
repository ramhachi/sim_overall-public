# `frame_optimize/matlab/resolve_suspension_geometry_source.m`

## 概要

frame 側が参照すべき authoritative artifact を決める helper です。指定ファイルに `FL` / `RL` hardpoints があればそれを使い、なければ shared candidate selector から条件一致の artifact を探します。

## 主な入力 / 出力

- 入力: `project_root`, `preferred_file`, 任意で `ErrorIfMissing`
- 出力: `suspension_file`

## システム上の役割

frame 最適化を artifact-first に揃える source resolver です。

## 注意点

- `ErrorIfMissing=true` を渡すと、warning で空文字を返す代わりに `MissingGeometry` エラーを投げます。
- canonical flow と compatibility wrapper はこの fail-fast モードを使い、hardpoint 不足のまま最適化を始めないようにしています。
