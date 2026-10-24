# filterPipeCatalog.m
## ファイル目的
- 管材カタログを入手可否・材質・断面形状・サイズ区分・価格で絞り込む。

## 実行フロー
- 入力型を検証する。
- `valid` フラグを全件 true で初期化する。
- 利用可能性、材質、形状、最小サイズ、最大価格を順に適用する。
- 最終的なインデックスを返す。

## 関数詳細
### `filtered_indices = filterPipeCatalog(catalog, options)`
- シグネチャ: `function filtered_indices = filterPipeCatalog(catalog, options)`
- 入力:
  - `catalog`: 管材 struct 配列
  - `options`: `min_size_class`, `material_id`, `shape`, `available_only`, `max_cost`
- 出力:
  - `filtered_indices`: 条件一致した catalog の添字
- 責務:
  - 設計候補の事前スクリーニング
- 主要ロジック:
  - サイズ階層は `None < D < C < B < A`
  - 未知サイズは不合格扱い
- 依存関係:
  - `containers.Map`
- 前提条件・注意事項:
  - 一致候補が無い場合は warning
  - `available_only` は true のときのみ適用
