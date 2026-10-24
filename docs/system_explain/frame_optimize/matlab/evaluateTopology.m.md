# evaluateTopology.m
## ファイル目的
- 2 値位相変数から活性要素を選択し、接続性・FEM・干渉・複雑度を含む目的関数を返す。

## 実行フロー
- 必要に応じて対称グループ変数を全要素へ展開する。
- mandatory 要素を強制 ON にする。
- 選択要素数と接続性を事前評価する。
- 固定管材・材料で FEM 配列を作る。
- `beam_fem_solver` と `checkExclusionZones` を実行する。
- 質量、剛性、複雑度、接続性、長尺要素罰則を合算する。

## 関数詳細
### `[obj_val, results] = evaluateTopology(x, nodes, candidate_elements, is_mandatory, pipe, material, bcs, loads, config)`
- シグネチャ: `function [obj_val, results] = evaluateTopology(...)`
- 入力:
  - `x`: 要素または対称グループごとの 2 値変数
  - `candidate_elements`, `is_mandatory`
  - `pipe`, `material`
  - `bcs`, `loads`, `config`
- 出力:
  - `obj_val`: 位相評価値
  - `results`: FEM と各ペナルティ詳細
- 責務:
  - ground structure 上の 2 値位相評価
- 主要ロジック:
  - 要素数 8 未満は即棄却
  - 接続性ペナルティ > 50 は FEM を省略
  - 剛性は reward 的に扱い、最終式では `-W_stiff * f_stiff`
- 依存関係:
  - `beam_fem_solver`
  - `checkExclusionZones`
  - ローカル関数 `computeConnectivityPenalty`, `countEdgeDisjointPaths`
- 前提条件・注意事項:
  - `pipe/material` は全要素共通
  - 位相評価式は `frameObjectiveFunction` と異なり reward を含む

### `penalty = computeConnectivityPenalty(elements, n_structural)`
- シグネチャ: `function penalty = computeConnectivityPenalty(elements, n_structural)`
- 入力:
  - `elements`: 選択接続
  - `n_structural`: 重要節点数
- 出力:
  - `penalty`
- 責務:
  - 連結性と前後パス冗長度の評価
- 主要ロジック:
  - BFS で node 1 から到達不能な重要節点に `+100`
  - 前側 `{1..4}` から後側 `{5..8}` への edge-disjoint path 数不足に `+5`
- 依存関係:
  - `countEdgeDisjointPaths`
- 前提条件・注意事項:
  - 重要節点番号が固定

### `n_paths = countEdgeDisjointPaths(elements, sources, sinks, max_node)`
- シグネチャ: `function n_paths = countEdgeDisjointPaths(elements, sources, sinks, max_node)`
- 入力:
  - `elements`, `sources`, `sinks`, `max_node`
- 出力:
  - `n_paths`
- 責務:
  - 単位容量 max-flow による辺素パス数算出
- 主要ロジック:
  - super source/sink 付き有向残余ネットワークを構築
  - Edmonds-Karp 反復で最大流を計算
- 依存関係:
  - なし
- 前提条件・注意事項:
  - 無向辺は 2 組の有向辺へ展開される
