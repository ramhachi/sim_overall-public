# test_week3_functions.m
## ファイル目的
- Week 3 実装の ground structure・GA 設定・パレート抽出をスモークテストする。

## 実行フロー
- 材料・管カタログと簡易ノード群を準備する。
- 3 個のテストを順に実行し、合格数を集計する。

## テスト詳細
### スクリプト初期化
- シグネチャ: スクリプト
- 入力: `frame_optimize/data`, `frame_optimize/matlab`
- 出力: 集計変数
- 責務: テスト環境準備
- 依存関係: `addpath`

### Test 1: `generateGroundStructure`
- シグネチャ: `elems = generateGroundStructure(test_nodes, gs_config)`
- 入力: 正方形 4 ノード、`max_length_mm=1200`
- 出力: 候補辺
- 責務: 長さ制限フィルタ確認
- 主要ロジック: 対角 1414 を落とし 4 辺のみ残ることを確認

### Test 2: `configureGA`
- シグネチャ: `opts = configureGA(ga_conf)`
- 入力: `pop_size=20`, `max_generations=5`
- 出力: `optim.options.SolverOptions`
- 責務: GA オプション生成確認
- 注意: 実行はせず型のみ確認

### Test 3: `extractParetoFront`
- シグネチャ: `[p_front, p_idx] = extractParetoFront(data)`
- 入力: 6 点の 2 目的行列
- 出力: パレート前線
- 責務: 非支配解抽出確認
- 主要ロジック: 3 点のみが非支配解であることを確認
