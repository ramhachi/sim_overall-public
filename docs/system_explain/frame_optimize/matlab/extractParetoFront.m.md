# extractParetoFront.m
## ファイル目的
- 多目的最小化問題の非支配解集合を抽出する。

## 実行フロー
- 全解を一旦パレート候補とする。
- 各解について他解からの支配関係を総当たり判定する。
- 非支配解を抽出し、第 1 目的でソートする。
- 同一行を `unique(...,'rows')` で除去する。

## 関数詳細
### `[pareto_front, pareto_indices] = extractParetoFront(objectives)`
- シグネチャ: `function [pareto_front, pareto_indices] = extractParetoFront(objectives)`
- 入力:
  - `objectives`: `N x M` 最小化目的行列
- 出力:
  - `pareto_front`: 非支配解
  - `pareto_indices`: 元配列内インデックス
- 責務:
  - パレート前線抽出
- 主要ロジック:
  - `j <= i` が全目的で成り立ち、かつ一つ以上厳密改善なら支配とみなす
- 依存関係:
  - `sortrows`, `unique`
- 前提条件・注意事項:
  - 最大化目的は事前に符号反転して渡す前提
  - 計算量は O(N^2)
